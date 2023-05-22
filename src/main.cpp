#include "spi.h"
#include "clock.h"
#include "ch32v003fun.h"
#include "ch32v003_SPI.h"
#include "system_tick.h"
#include "gpio.h"
#include <cstdlib>
#include "instant.h"
#include <etl/string.h>
#include "rfsystem.h"
#include <printf.h>

static const pin_size_t IRQ_PIN = GPIO::C3;
static const pin_size_t SDN_PIN = GPIO::C2;
static const pin_size_t CS_PIN = GPIO::C4;
static const pin_size_t RST_PIN = GPIO::C0;

int rand_range(int min, int max) {
  return min + (std::rand() % (max - min + 1));
}

int main() {
  SystemInit48HSI();
  SysTick_init();
  SetupDebugPrintf();
  printf("start\n");

  pin_size_t LED_pin = GPIO::D6;
  pinMode(LED_pin, OUTPUT);
  auto rf  = RfSystem(RST_PIN, CS_PIN, IRQ_PIN, SDN_PIN);
  rf.begin();
  auto version = rf.version();
  printf("version=%d\n", version);

  auto instant = Instant();
  auto d = std::chrono::duration<uint64_t, std::milli>(500);

  while (true) {
    if (instant.elapsed() > d) {
      printf("elapsed=%d\n", instant.elapsed().count());
      etl::string<32> payload = "hello world";
      rf.dataPackageSend(payload.c_str(), payload.length());
      instant.reset();
    }
  }
}
