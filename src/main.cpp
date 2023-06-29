//#define TX
//#define DISABLE_LED

#include "clock.h"
#include "ch32v003fun.h"
#include "system_tick.h"
#include "gpio.h"
#include "instant.h"
#include "exti.h"
#include "llcc68.h"
#include "message_wrapper.h"
#include "utils.h"
#include <printf.h>
#include "led.h"
#include <etl/random.h>
#include "flags.h"
#include "flash.h"
#include "spot.h"

#ifdef TX
#include <pb_encode.h>
#endif

// TODO: what's the IRQ pin?
// DIO2 is connected to IRQ (at PD1)
static const pin_size_t IRQ_PIN   = GPIO::D1;
static const pin_size_t BUSY_PIN  = GPIO::C3;
static const pin_size_t CS_PIN    = GPIO::C4;
static const pin_size_t RST_PIN   = GPIO::C0;
static const pin_size_t RX_EN_PIN = GPIO::C1;
static const pin_size_t TX_EN_PIN = GPIO::C2;

int main() {
  SystemInit48HSI();
  SysTick_init();
  SetupDebugPrintf();
  printf("[INFO] booting\n");

  pin_size_t LED_pin = GPIO::D6;
  pinMode(LED_pin, GPIO::OUTPUT);
  configureEXTI();

  auto hal = RadioLibHal();
  hal.spiBegin();
  auto mod = Module(&hal, CS_PIN, IRQ_PIN, RST_PIN, BUSY_PIN);
  auto rf  = LLCC68(&mod);
  rf.setDio2AsRfSwitch(false);
  //  rf.setRfSwitchPins(RX_EN_PIN, TX_EN_PIN);
  // TODO: ...
  auto res = rf.begin(434);
  if (res != RADIOLIB_ERR_NONE) {
    printf("[ERROR] failed to initialize radio, code %d\n", res);
  }
  // Won't change as long as startTransmit() is not called
  // expect to be 0x03
  printf("[DEBUG] HEADER_SIZE=%d\n", MessageWrapper::HEADER_SIZE);
#ifdef TX
  printf("[INFO] TX mode\n");
#else
  printf("RX mode\n");
#endif

  char src[3]      = {0x01, 0x02, 0x03};
  char dst[3]      = {0x04, 0x05, 0x06};
  uint8_t pkt_id   = 0;
  uint32_t counter = 0;
  auto instant     = Instant();
  auto d           = std::chrono::duration<uint16_t, std::milli>(250);
  LED::begin();
  auto rng = etl::random_xorshift();
  rng.initialise(0);
  uint8_t rgb;
#ifdef TX
  auto encoder = MessageWrapper::Encoder(src, dst, pkt_id);
#else
  auto decoder = MessageWrapper::Decoder();
  auto instant_rx = Instant();
  auto d_rx           = std::chrono::duration<uint16_t, std::milli>(1);

#endif
  while (true) {
#ifdef TX
    if (instant.elapsed() >= d) {
      uint8_t buf[256];
      Simple message = Simple_init_zero;
      bool random_r;
      bool random_g;
      bool random_b;
      uint8_t temp;
      do {
        random_r = rng.range(0, 1);
        random_g = rng.range(0, 1);
        random_b = rng.range(0, 1);
        temp     = random_r | (random_g << 1) | (random_b << 2);
      } while (!(temp != 0) || !(temp != rgb)); // refresh until at least one color is on and the color is different from the previous one
      rgb                          = temp;
      message.is_red               = random_r;
      message.is_green             = random_g;
      message.is_blue              = random_b;
      pb_ostream_t stream          = pb_ostream_from_buffer(buf, sizeof(buf));
      message.counter              = counter;
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
      const char *payload = "test";
      message.message.arg = const_cast<char *>(payload);
      bool status         = pb_encode(&stream, Simple_fields, &message);
      if (status) {
        encoder.reset(src, dst, pkt_id);
        encoder.setPayload(buf, stream.bytes_written);
        auto res = encoder.next();
        while (res.has_value()) {
          auto &v = res.value();
          printf("counter:%d; size=%d; payload_size=%d; \n", message.counter, v.size(), stream.bytes_written);
          auto r = rf.transmit(reinterpret_cast<uint8_t *>(v.data()), v.size());
          if (r != RADIOLIB_ERR_NONE) {
            printf("[ERROR] failed to transmit, code %d\n", r);
          }
          digitalWrite(GPIO::D6, GPIO::HIGH);
          Delay_Ms(10);
          digitalWrite(GPIO::D6, GPIO::LOW);
          res = encoder.next();
        }
      } else {
        printf("[ERROR] failed to encode\n");
      }
#ifndef DISABLE_LED
      LED::setColor(random_r, random_g, random_b);
#else
      LED::setColor(false, false, false);
#endif
      pkt_id++;
      counter++;
      instant.reset();
      if (Flags::getFlag()) {
        printf("[INFO] TX flag set\n");
        Flags::setFlag(false);
      }
    }
#else // RX
    // I guess some reorder magic is happening here
    // I'm not sure if standby is working...
    // See also `exti.cpp`
    while (instant_rx.elapsed() < d_rx) {
      printf("*");
      rf.rx();
      if (instant_rx.elapsed() >= d_rx) {
        printf("\n");
      }
    }
    if (Flags::getFlag()) {
      printf("[INFO] RX flag set\n");
      char rx_buf[256];
      uint16_t rx_size;

      auto t1 = Track();
      t1.color = 0b00000011;
      t1.addSpeed(0, 0);
      t1.addSpeed(50, 1.1);
      t1.addSpeed(100, 1.5);
      t1.addSpeed(200, 1.3);
      t1.addSpeed(300, 1.1);
      t1.addSpeed(400, 1.0);
      auto scfg = SpotConfig {
          .circleLength = 400,
          .lineLength = 18,
          .total = 400,
          .current = -1,
          .updateInterval = 100,
      };
      auto s = Spot(std::move(scfg));
      s.addTrack(std::move(t1));
      s.setColorCallback = [](uint8_t c) {
        auto r = c & 0b00000001;
        auto g = (c & 0b00000010) >> 1;
        auto b = (c & 0b00000100) >> 2;
        LED::setColor(r, g, b);
      };
      s.start();
      s.update();
      // when a valid packet is received the state should be 0xc0
      // (at least the rx_pkt_state would be 0x00)
      // (sync_word_rev = 1, preamble_rev = 1) but the pkg_flag is useless
      // one should only use interrupt to detect the packet

      // TODO: sleep and duty cycle (see `startReceiveDutyCycleAuto`)
      // polling for now... interrupt is not working
      // check `setDioIrqParams`
      if (auto maybe_len = rf.tryReceive(reinterpret_cast<uint8_t *>(rx_buf))) {
        digitalWrite(GPIO::D6, GPIO::HIGH);
        rx_size = maybe_len.value();
        auto h  = decoder.decodeHeader(rx_buf, rx_size);
        if (h.has_value()) {
          decoder.printHeader(h.value());
        }
        auto end_padding = rx_buf + rx_size - 3;
        if (memcmp(end_padding, "\x00\x00\x00", 3) != 0) {
          printf("[ERROR] end padding is not correct. gets \"");
          utils::printWithSize(rx_buf + rx_size - 3, 3, true);
          printf("\"\n");
        }
        auto res = decoder.decode(rx_buf, rx_size);
        if (res == MessageWrapper::WrapperDecodeResult::Finished) {
          auto payload = decoder.getOutput();
          etl::vector<char, 32> string_payload;
        } else if (res == MessageWrapper::WrapperDecodeResult::Unfinished) {
          printf("[INFO] unfinished\n");
        } else {
          printf("[ERROR] WrapperDecodeError:%s\n", MessageWrapper::decodeResultToString(res));
          decoder.reset();
        }
      }
      // define ATTR_PACKED __attribute__ ((__packed__))
      // https://linux.die.net/man/3/htonl
      // https://stackoverflow.com/questions/8568432/is-gccs-attribute-packed-pragma-pack-unsafe
      // https://www.quora.com/What-is-the-attribute__-packed-variable-attribute-in-C-and-why-and-how-is-it-used
      // https://dev.to/agudpp/packing-unpacking-data-in-c-3gmh
      // https://stackoverflow.com/questions/45116212/are-packed-structs-portable
      // https://capnproto.org
      // https://github.com/dloss/binary-parsing
      // https://construct.readthedocs.io/en/latest/
      // https://news.ycombinator.com/item?id=21874949
      auto a = __htonl(1);
      auto b = __ntohl(a);
      digitalWrite(GPIO::D6, GPIO::LOW);
      Flags::setFlag(false);
    }
#endif
  }
}
