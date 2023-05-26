#include "spi.h"
#include "clock.h"
#include "ch32v003fun.h"
#include "system_tick.h"
#include "gpio.h"
#include "instant.h"
#include <etl/string.h>
#include <etl/to_string.h>
#include "rfsystem.h"
#include <printf.h>
#include "utils.h"

void printWithSize(const char *str, size_t size, bool hex = false) {
  for (size_t i = 0; i < size; i++) {
    if (hex) {
      printf("%02x", str[i]);
    } else {
      printf("%c", str[i]);
    }
  }
};

static const pin_size_t PKT_FLAG_PIN = GPIO::C3;
static const pin_size_t SDN_PIN = GPIO::C2;
static const pin_size_t CS_PIN = GPIO::C4;
static const pin_size_t RST_PIN = GPIO::C1;

int main() {
  SystemInit48HSI();
  SysTick_init();
  SetupDebugPrintf();
  printf("booting\n");

  pin_size_t LED_pin = GPIO::D6;
  pinMode(LED_pin, OUTPUT);

  auto &rf = RfSystem::get();
  auto success = rf.setPins(RST_PIN, CS_PIN, PKT_FLAG_PIN, SDN_PIN);
  if (!success) {
    printf("[ERROR] failed to set pins\n");
  }
  rf.begin();

  // expect to be 0x03
  auto version = rf.version();
  printf("version=%d\n", version);

  auto instant = Instant();
  auto rx_instant = Instant();
  rf.printRegisters();
  #define TX
  #ifdef TX
  printf("TX mode\n");
  #else
  printf("RX mode\n");
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
      auto status = rf.pollStatus();
      if (!status.tx) {
        rf.tx();
      }
      auto res = rf.send(payload.c_str(), payload.length());
      if (!res.has_value()){
        printf("TX timeout\n");
      }
      printWithSize(payload.c_str(), payload.length());
      digitalWrite(GPIO::D6, HIGH);
      Delay_Ms(10);
      digitalWrite(GPIO::D6, LOW);
//      auto state = rf.pollState();
//      RF::printState(state);
      instant.reset();
    }
    #else // RX
    auto d = std::chrono::duration<uint32_t , std::milli>(3000);
    if (instant.elapsed() > d) {
      auto status = rf.pollStatus();
      auto state = rf.pollState();
      if (!status.rx) {
        rf.rx();
      }
      auto rssi = rf.rssi();
      printf("rssi=%u\n", rssi);
      RF::printStatus(status);
      RF::printState(state);

      etl::string<256> buf;
      // magic number 0x03 means no packet received
      // when a valid packet is received the state will be 0xc0
      // (sync_word_rev = 1, preamble_rev = 1) but the pkg_flag is useless
      if (state.rx_pkt_state != 0x03) {
        if (auto maybe = rf.recv(buf.data())) {
          buf.resize(maybe.value());
          printf("len=%d\n", buf.length());
          printWithSize(buf.c_str(), buf.length(), true);
          printf("\n");
          rf.clrRxFifo();
          rf.resetRxFlag();
        }
      }

      instant.reset();
    }
    #endif
  }
}
