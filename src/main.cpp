// #define TX
//  #define DISABLE_LED

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
#include "boring.h"

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
static const uint8_t PING_MAGIC   = 0x06;

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

  uint8_t src[3] = {0x01, 0x02, 0x03};
  uint8_t dst[3] = {0xff, 0xff, 0xff};
  uint8_t pkt_id = 0;
  auto instant   = Instant();
  auto d         = std::chrono::duration<uint16_t, std::milli>(500);
  LED::begin();
  auto rng = etl::random_xorshift();
  rng.initialise(0);
  uint8_t rgb;
#ifdef TX
  auto encoder = MessageWrapper::Encoder(src, dst, pkt_id);
#else
  auto decoder    = MessageWrapper::Decoder();
  auto instant_rx = Instant();
  auto d_rx       = std::chrono::duration<uint16_t, std::milli>(1);

#endif
  while (true) {
#ifdef TX
    if (instant.elapsed() >= d) {
      // Channel Activity Detection
      // https://github.com/jgromes/RadioLib/blob/bea5e70d0ad4f6df74a4eb2a3d1bdb683014d6c1/examples/SX126x/SX126x_Channel_Activity_Detection_Interrupt/SX126x_Channel_Activity_Detection_Interrupt.ino#L4
      // https://github.com/jgromes/RadioLib/blob/bea5e70d0ad4f6df74a4eb2a3d1bdb683014d6c1/examples/SX128x/SX128x_Channel_Activity_Detection_Blocking/SX128x_Channel_Activity_Detection_Blocking.ino#L54
      // basic do an IRQ status check
      bool random_r;
      bool random_g;
      bool random_b;
      uint8_t temp;
      do {
        random_r = rng.range(0, 1);
        random_g = rng.range(0, 1);
        random_b = rng.range(0, 1);
        temp     = random_b | (random_g << 1) | (random_r << 2);
      }
      // refresh until at least one color is on and the color is different from the previous one
      while (!(temp != 0) || !(temp != rgb));
      rgb                 = temp;
      auto b              = boring::Boring();
      b.led               = rgb;
      const char *payload = "test";
      for (auto i = 0; i < strlen(payload); i++) {
        b.comments.emplace_back(payload[i]);
      }
      uint8_t buf[256];

      auto st = rf.scanChannel();
      while (st != RADIOLIB_CHANNEL_FREE) {
        st = rf.scanChannel();
      }
      auto sz = boring::toBytes(b, buf);
      encoder.setPayload(buf, sz);
      auto maybe = encoder.next();
      while (maybe.has_value()) {
        auto &v = maybe.value();
        auto r  = rf.transmit(reinterpret_cast<uint8_t *>(v.data()), v.size());
        if (r != RADIOLIB_ERR_NONE) {
          printf("[ERROR] failed to transmit, code %d\n", r);
        }
        maybe = encoder.next();
      }
      printf("[INFO] Boring rgb=%d\n", rgb);
      LED::setColor(rgb);
      if (Flags::getFlag()) {
        //        printf("[INFO] TX flag set\n");
        Flags::setFlag(false);
      }
      instant.reset();
      pkt_id++;
      encoder.reset(src, dst, pkt_id);
    }
#else // RX
    // I guess some reorder magic is happening here
    // I'm not sure if standby is working...
    // See also `exti.cpp`
    while (instant_rx.elapsed() < d_rx) {
      printf("*");
      rf.startReceive();
      if (instant_rx.elapsed() >= d_rx) {
        printf("\n");
      }
    }
    if (Flags::getFlag()) {
      printf("[INFO] RX flag set\n");
      char rx_buf[256];
      uint16_t rx_size;

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
          printf("[INFO] ");
          decoder.printHeader(h.value());
        } else {
          printf("[ERROR] dump: ");
          utils::printWithSize(rx_buf, rx_size, true);
          printf("\n");
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
          auto b       = boring::fromBytes(reinterpret_cast<const uint8_t *>(payload.data()));
          if (b.has_value()) {
            auto &v = b.value();
            printf("[INFO] boring: led=%d, comments=\"", v.led);
            for (auto c : v.comments) {
              printf("%c", c);
            }
            printf("\"\n");
#ifdef DISABLE_LED
            LED::setColr(false, false, false);
#else
            LED::setColor(b->led);
#endif
          } else {
            printf("[ERROR] failed to decode boring\n");
          }
        } else if (res == MessageWrapper::WrapperDecodeResult::Unfinished) {
          printf("[INFO] unfinished\n");
        } else {
          printf("[ERROR] WrapperDecodeError:%s\n", MessageWrapper::decodeResultToString(res));
          decoder.reset();
        }
      }
      digitalWrite(GPIO::D6, GPIO::LOW);
      Flags::setFlag(false);
    }
#endif
  }
}
