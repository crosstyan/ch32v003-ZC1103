//#define TX
#include "clock.h"
#include "ch32v003fun.h"
#include "system_tick.h"
#include "gpio.h"
#include "instant.h"
#include "exti.h"
#include <etl/vector.h>
#include "rfsystem.h"
#include "message_wrapper.h"
#include "utils.h"
#include <printf.h>
#include <pb_common.h>
#include <pb_decode.h>
#include "simple.pb.h"

#ifdef TX
#include <pb_encode.h>
#endif

static const pin_size_t PKT_FLAG_PIN = GPIO::C3;
static const pin_size_t SDN_PIN = GPIO::C2;
static const pin_size_t CS_PIN = GPIO::C4;
static const pin_size_t RST_PIN = GPIO::C1;

int main() {
  putchar('a');
  SystemInit48HSI();
  SysTick_init();
  SetupDebugPrintf();
  printf("[INFO] booting\n");

  pin_size_t LED_pin = GPIO::D6;
  pinMode(LED_pin, OUTPUT);
  configureEXTI();

  auto &rf = RfSystem::get();
  auto success = rf.setPins(RST_PIN, CS_PIN, PKT_FLAG_PIN, SDN_PIN);
  if (!success) {
    printf("[ERROR] failed to set pins\n");
  }
  rf.begin();

  // expect to be 0x03
  auto version = rf.version();
  printf("[INFO] version=%d\n", version);
  printf("[DEBUG] HEADER_SIZE=%d\n", MessageWrapper::HEADER_SIZE);
  #ifdef TX
  printf("[INFO] TX mode\n");
  #else
  printf("RX mode\n");
  rf.rx();
  rf.setWorEn(true);
  #endif

  char src[3] = {0x01, 0x02, 0x03};
  char dst[3] = {0x04, 0x05, 0x06};
  uint8_t pkt_id = 0;
  uint32_t counter = 0;
  auto instant = Instant();
  #ifdef TX
  auto encoder = MessageWrapper::Encoder(src, dst, pkt_id);
  #else
  auto decoder = MessageWrapper::Decoder();
  #endif
  while (true) {
    #ifdef TX
    auto d = std::chrono::duration<uint16_t, std::milli>(1000);
    if (instant.elapsed() >= d) {
      uint8_t buf[256];
      Simple message = Simple_init_zero;
      pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
      message.counter = counter;
      message.message.funcs.encode = [](pb_ostream_t *stream, const pb_field_t *field, void *const *arg) {
        auto message = static_cast<char *>(*arg);
        // zero terminated string (zero sentinel is not included)
        if (!pb_encode_tag_for_field(stream, field)) {
          return false;
        } else {
          bool res = pb_encode_string(stream, reinterpret_cast<uint8_t *>(message), strlen(message));
          if (!res) {
            return false;
          }
        }
        return true;
      };
      uint8_t payload[] = "hello world!";
      message.message.arg = payload;
      bool status = pb_encode(&stream, Simple_fields, &message);
      if (status) {
        encoder.reset(src, dst, pkt_id);
        encoder.setPayload(buf, stream.bytes_written);
        auto res = encoder.next();
        while (res.has_value()) {
          auto &v = res.value();
          printf("size=%d; payload_size=%d\n", v.size(), stream.bytes_written);
          rf.send(v.data(), v.size());
          digitalWrite(GPIO::D6, HIGH);
          Delay_Ms(10);
          digitalWrite(GPIO::D6, LOW);
          auto state = rf.pollState();
          RF::printState(state);
          res = encoder.next();
        }
      } else {
        printf("[ERROR] failed to encode\n");
      }
      pkt_id++;
      counter++;
      instant.reset();
    }
    #else // RX
    // See also `exti.cpp`
    if (RF::rxFlag()) {
      rf.setWorEn(false);
      digitalWrite(GPIO::D6, HIGH);
      auto state = rf.pollState();
      if (state.crc_error) {
        printf("[ERROR] CRC error\n");
      }
      etl::vector<char, 256> rx_buf;
      // when a valid packet is received the state should be 0xc0
      // (at least the rx_pkt_state would be 0x00)
      // (sync_word_rev = 1, preamble_rev = 1) but the pkg_flag is useless
      // one should only use interrupt to detect the packet
      if (state.rx_pkt_state != RF::NO_PACKET_RECEIVED) {
        if (auto maybe = rf.recv(rx_buf)) {
          auto h = decoder.decodeHeader(rx_buf.data(), rx_buf.size());
          if (h.has_value()) {
            decoder.printHeader(h.value());
          }
          auto res = decoder.decode(rx_buf.data(), rx_buf.size());
          if (res == MessageWrapper::WrapperDecodeResult::Finished) {
            auto payload = decoder.getOutput();
            etl::vector<char, 32> string_payload;
            pb_istream_t istream = pb_istream_from_buffer(reinterpret_cast<uint8_t*>(payload.data()) , payload.size());
            Simple message = Simple_init_zero;
            message.message.arg = &string_payload;
            message.message.funcs.decode = [](pb_istream_t *stream, const pb_field_t *field, void **arg) {
              auto &payload = *(static_cast<etl::ivector<char> *>(*arg));
              if (stream->bytes_left > payload.max_size() - 1) {
                return false;
              }
              payload.resize(stream->bytes_left);
              if (!pb_read(stream, reinterpret_cast<uint8_t *>(payload.data()), stream->bytes_left)) {
                return false;
              }
              payload.push_back('\0');
              return true;
            };
            bool status = pb_decode(&istream, Simple_fields, &message);
            if (status) {
              printf("[INFO] counter=%d, message=%s\n", message.counter, string_payload.data());
            } else {
              printf("[ERROR] failed to decode\n");
            }
          } else if (res == MessageWrapper::WrapperDecodeResult::Unfinished) {
            printf("[INFO] unfinished\n");
          } else {
            printf("[ERROR] WrapperDecodeError:%s\n", MessageWrapper::decodeResultToString(res));
            decoder.reset();
          }
          rf.clrRxFifo();
          rf.wor();
          RF::setRxFlag(false);
        }
      }
      digitalWrite(GPIO::D6, LOW);
    }
    #endif
  }
}
