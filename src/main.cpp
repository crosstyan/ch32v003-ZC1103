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
#ifdef TX
#include <etl/string.h>
#include <etl/to_string.h>
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
  rf.printRegisters();
  #ifdef TX
  printf("[INFO] TX mode\n");
  #else
  printf("RX mode\n");
  rf.wor();
  rf.rx();
  #endif

  char src[3] = {0x01, 0x02, 0x03};
  char dst[3] = {0x04, 0x05, 0x06};
  uint8_t counter = 0;
  auto instant = Instant();
  #ifdef TX
  auto encoder = MessageWrapper::Encoder(src, dst, counter);
  #else
  auto decoder = MessageWrapper::Decoder();
  #endif
  while (true) {
    #ifdef TX
    auto d = std::chrono::duration<uint16_t , std::milli>(1000);
    if (instant.elapsed() >= d) {
      // construct a payload
      etl::string<32> payload = "hello world:";
      auto r = utils::rand_range(0, 65535);
      etl::to_string(r, payload, true);
      payload.append("\r\n");
      utils::printWithSize(payload.data(), payload.size());
      encoder.reset(src, dst, counter);
      encoder.setPayload(payload.c_str(), payload.length());
      auto res = encoder.next();
      while(res.has_value()){
        auto &v = res.value();
        printf("size=%d\n", v.size());
        rf.send(v.data(), v.size());
        digitalWrite(GPIO::D6, HIGH);
        Delay_Ms(10);
        digitalWrite(GPIO::D6, LOW);
        auto state = rf.pollState();
        RF::printState(state);
        res = encoder.next();
      }
      counter++;
      instant.reset();
    }
    #else // RX
    // See also `exti.cpp`
    if (RF::rxFlag()) {
      digitalWrite(GPIO::D6, HIGH);
      auto state = rf.pollState();
      if (state.crc_error) {
        printf("[ERROR] CRC error\n");
      }
      etl::vector<char, 256> buf;
      // when a valid packet is received the state should be 0xc0
      // (at least the rx_pkt_state would be 0x00)
      // (sync_word_rev = 1, preamble_rev = 1) but the pkg_flag is useless
      // one should only use interrupt to detect the packet
      if (state.rx_pkt_state != RF::NO_PACKET_RECEIVED) {
        if (auto maybe = rf.recv(buf)) {
          auto h = decoder.decodeHeader(buf.data(), buf.size());
          if (h.has_value()) {
            decoder.printHeader(h.value());
          }
          auto res = decoder.decode(buf.data(), buf.size());
          if (res == MessageWrapper::WrapperDecodeResult::Finished) {
            auto payload = decoder.getOutput();
            printf("[INFO] payload=");
            utils::printWithSize(payload);
            if (*(buf.end() - 1) != '\n') {
              printf("\n");
            }
          } else if (res == MessageWrapper::WrapperDecodeResult::Unfinished) {
            printf("[INFO] unfinished\n");
          } else {
            printf("[ERROR] WrapperDecodeError:%s\n", MessageWrapper::decodeResultToString(res));
            decoder.reset();
          }
          rf.clrRxFifo();
          RF::setRxFlag(false);
          rf.wor();
        }
      }
      digitalWrite(GPIO::D6, LOW);
    }
    #endif
  }
}
