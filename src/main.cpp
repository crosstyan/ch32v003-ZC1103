// #define TX
//  #define DISABLE_LED
#define DISABLE_STANDBY

#include "funconfig.h"
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
#include "adc.h"
#include <etl/random.h>
#include "flags.h"
#include "flash.h"
#include "spot.h"
#include <etl/map.h>
#include "boring.h"

// TODO: what's the IRQ pin?
// DIO2 is connected to IRQ (at PD1)
static const pin_size_t IRQ_PIN   = GPIO::D1;
static const pin_size_t BUSY_PIN  = GPIO::C3;
static const pin_size_t CS_PIN    = GPIO::C4;
static const pin_size_t RST_PIN   = GPIO::C0;
static const pin_size_t RX_EN_PIN = GPIO::C1;
static const pin_size_t TX_EN_PIN = GPIO::C2;

static const auto ADDR_BYTES                     = 3;
static const uint8_t MY_ADDR[ADDR_BYTES]         = {0x01, 0x02, 0x03};
static const uint8_t BROAD_CAST_ADDR[ADDR_BYTES] = {0xFF, 0xFF, 0xFF};

static const auto DECODE_BUFFER_SIZE = 512;

bool isValidAddr(const uint8_t *addr) {
  auto is_bc_addr = memcmp(addr, BROAD_CAST_ADDR, ADDR_BYTES) == 0;
  if (is_bc_addr) {
    return true;
  }
  auto is_my_addr = memcmp(addr, MY_ADDR, ADDR_BYTES) == 0;
  if (is_my_addr) {
    return true;
  }
  return false;
}

int main() {
restart:
  SystemInit();
  SysTick_init();
  adc_init();
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
    Delay_Ms(1000);
    goto restart;
  }

  // Won't change as long as startTransmit() is not called
  // expect to be 0x03
  printf("[DEBUG] HEADER_SIZE=%d\n", MessageWrapper::HEADER_SIZE);
#ifdef TX
  printf("[INFO] TX mode\n");
#else
  printf("RX mode\n");
#endif
  LED::begin();
#ifdef TX
  uint8_t src[3] = {0x01, 0x02, 0x03};
  uint8_t dst[3] = {0xff, 0xff, 0xff};
  uint8_t pkt_id = 0;
  auto instant   = Instant();
  auto d         = std::chrono::duration<uint16_t, std::milli>(500);
  auto rng       = etl::random_xorshift();
  rng.initialise(0);
  uint8_t rgb;
  auto encoder = MessageWrapper::Encoder<MessageWrapper::MAX_ENCODER_OUTPUT_SIZE>(src, dst, pkt_id);
#else
  auto decoder      = MessageWrapper::Decoder<DECODE_BUFFER_SIZE>();
  auto instant_rx   = Instant();
  auto d_rx         = std::chrono::duration<uint16_t, std::milli>(1);
  auto instant_spot = Instant();
  auto counter = 0;
  // pay attention to the stack size!
  auto spot = RfMessage::Spot();
  res    = rf.startReceive();
  if (res != RADIOLIB_ERR_NONE) {
    printf("[ERROR] failed to start receiving, code %d\n", res);
    Delay_Ms(1000);
    goto restart;
  }
#endif

#ifndef DISABLE_STANDBY
  // enable power interface module clock
  RCC->APB1PCENR |= RCC_APB1Periph_PWR;

  // enable low speed oscillator (LSI)
  RCC->RSTSCKR |= RCC_LSION;
  while ((RCC->RSTSCKR & RCC_LSIRDY) == 0) {}

  // enable AutoWakeUp event
  EXTI->EVENR |= EXTI_Line9;
  EXTI->FTENR |= EXTI_Line9;

  // configure AWU prescaler
  PWR->AWUPSC |= PWR_AWU_Prescaler_4096;

  // configure AWU window comparison value
  PWR->AWUWR &= ~0x3f;
  PWR->AWUWR |= 63;

  // enable AWU
  PWR->AWUCSR |= (1 << 1);

  // select standby on power-down
  PWR->CTLR |= PWR_CTLR_PDDS;

  // peripheral interrupt controller send to deep sleep
  PFIC->SCTLR |= (1 << 2);
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
#ifndef DISABLE_STANDBY
    __WFE();
    // restore clock to full speed
    SystemInit48HSI();
    printf("[INFO] wake up with %d\n", counter++);
#endif
    if (Flags::getFlag()) {
      printf("[INFO] RX flag set\n");
      auto span = decoder.getSpan();
      uint16_t rx_size;

      if (span.has_value()) {
        auto rx_buf = span.value();
        // when a valid packet is received the state should be 0xc0
        // (at least the rx_pkt_state would be 0x00)
        // (sync_word_rev = 1, preamble_rev = 1) but the pkg_flag is useless
        // one should only use interrupt to detect the packet

        // TODO: sleep and duty cycle (see `startReceiveDutyCycleAuto`)
        // polling for now... interrupt is not working
        // check `setDioIrqParams`
        if (auto maybe_len = rf.tryReceive(reinterpret_cast<uint8_t *>(rx_buf.begin()))) {
          digitalWrite(GPIO::D6, GPIO::HIGH);
          rx_size          = maybe_len.value();
          auto end_padding = rx_buf.begin() + rx_size - 3;
          if (memcmp(end_padding, "\x00\x00\x00", 3) != 0) {
            printf("[ERROR] end padding is not correct. gets \"");
            utils::printWithSize(end_padding, 3, true);
            printf("\"\n");
          }
          auto [res, header] = decoder.decode(rx_size);
          printf("[INFO] ");
          if (res == MessageWrapper::WrapperDecodeResult::Finished) {
            MessageWrapper::printHeader(header);
            auto payload_ = decoder.getOutput();
            if (payload_.has_value()) {
              auto payload = payload_.value();
              auto magic   = payload[0];
              switch (magic) {
                case RfMessage::BORING_MAGIC: {
                  auto b = RfMessage::boring::fromBytes(reinterpret_cast<const uint8_t *>(payload.data()));
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
                  break;
                }
                case RfMessage::COMMAND_MAGIC: {
                  auto c = RfMessage::CommandMessage::fromBytes(payload.data());
                  if (c.has_value()) {
                    auto command = c.value();
                    switch (command) {
                      case RfMessage::Command::START: {
                        spot.start();
                      }
                      case RfMessage::Command::STOP: {
                        spot.stop();
                      }
                      case RfMessage::Command::Ping: {
                        auto val = adc_get();
                        printf("[INFO] ping: %d\n", val);
                        // TODO: write a encode function to encode the value (pong)
                        // directly into the buffer to avoid the creation of Encoder object
                        uint8_t b[4] = {0};
                        // would switch to RX mode after transmission
                        auto st = rf.transmit(b, 4);
                        if (st != RADIOLIB_ERR_NONE) {
                          printf("[ERROR] failed to transmit, code %d\n", st);
                        }
                      }
                    }
                  } else {
                    printf("[ERROR] failed to decode command\n");
                  }
                  break;
                }
                case RfMessage::SPOT_CONFIG_MAGIC: {
                  auto maybe = RfMessage::SpotConfig::fromBytes(payload.data());
                  if (maybe.has_value()) {
                    auto config = maybe.value();
                    spot.setConfig(config);
                    printf("[INFO] spot config set\n");
                  }
                  break;
                }
                case RfMessage::SPOT_MAGIC: {
                  spot.fromBytes(payload.data());
                  printf("[INFO] spot set\n");
                  break;
                }
                case RfMessage::SET_CURRENT_MAGIC: {
                  auto maybe = RfMessage::SetCurrent::fromBytes(payload.data());
                  if (maybe.has_value()) {
                    auto current = maybe.value().current_id;
                    printf("[INFO] set current to %d\n", current);
                    Current::set(current);
                  }
                  break;
                }
              }
            } else {
              printf("[ERROR] failed to decode boring\n");
            }
          } else if (res == MessageWrapper::WrapperDecodeResult::Unfinished) {
            MessageWrapper::printHeader(header);
            printf("[INFO] unfinished\n");
          } else {
            printf("[ERROR] WrapperDecodeError:%s\n", MessageWrapper::decodeResultToString(res));
            decoder.reset();
          }
        }
      }
      digitalWrite(GPIO::D6, GPIO::LOW);
      Flags::setFlag(false);
    }
   // update spot task
    if (spot.state() == RfMessage::SpotState::START) {
      auto &cfg     = spot.config();
      auto interval = std::chrono::duration<decltype(cfg.updateInterval), std::milli>(cfg.updateInterval);
      if (instant_spot.elapsed() >= interval) {
        spot.update();
        instant_spot.reset();
      }
    }
#endif
  }
}
