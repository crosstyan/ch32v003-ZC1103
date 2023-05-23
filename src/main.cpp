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

static const pin_size_t IRQ_PIN = GPIO::C3;
static const pin_size_t SDN_PIN = GPIO::C2;
static const pin_size_t CS_PIN = GPIO::C4;
static const pin_size_t RST_PIN = GPIO::C0;

int main() {
  SystemInit48HSI();
  SysTick_init();
  SetupDebugPrintf();
  printf("restart\n");

  pin_size_t LED_pin = GPIO::D6;
  pinMode(LED_pin, OUTPUT);

  auto &rf = RfSystem::get();
  auto success = rf.setPins(RST_PIN, CS_PIN, IRQ_PIN, SDN_PIN);
  if (!success) {
    printf("[ERROR] failed to set pins\n");
  }
  rf.begin();

  // expect 0x00
  // if the value is 0x03 (0b11) then shit goes wrong
  auto version = rf.version();
  printf("version=%d\n", version);

  auto instant = Instant();
  rf.printRegisters();
  while (true) {
    #ifdef TX
    auto d = std::chrono::duration<uint64_t, std::milli>(500);
    if (instant.elapsed() >= d) {
      // construct a payload
      etl::string<32> payload = "hello world:";
      auto r = utils::rand_range(0, 100);
      etl::to_string(r, payload, true);
      payload.append("\n");
      rf.send(payload.c_str(), payload.length());
      digitalWrite(GPIO::D6, HIGH);
      Delay_Ms(10);
      digitalWrite(GPIO::D6, LOW);
      auto status = rf.pollStatus();
      auto state = rf.pollState();
      RF::printStatus(status);
      RF::printState(state);
      instant.reset();
    }
    #else // RX
    auto d = std::chrono::duration<uint64_t, std::milli>(1000);
    if (instant.elapsed() > d) {
      auto status = rf.pollStatus();
      auto s = rf.pollState();
      RF::printStatus(status);
      RF::printState(s);
      instant.reset();
    }
//    etl::string<256> buf;
//    auto s = rf.pollState();
//    if (s.fifo_flag) {
//      if (auto maybe = rf.recv(buf.data())) {
//        buf.resize(maybe.value());
//        printf("len=%d\n", buf.length());
//        printWithSize(buf.c_str(), buf.length(), true);
//        printf("\n");
//        rf.resetRxFlag();
//      }
//    }
    #endif
  }
}
