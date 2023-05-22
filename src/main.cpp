#include "clock.h"
#include "ch32v003fun.h"
#include "system_tick.h"
#include "gpio.h"
#include <printf.h>
#include <cstdlib>
#include "instant.h"
#include "etl/string.h"

int rand_range(int min, int max) {
  return min + (std::rand() % (max - min + 1));
}

int main() {
  SystemInit48HSI();
  SysTick_init();
  SetupDebugPrintf();
  // test ETL
  auto hello = etl::string<32>("hello");
  hello.append(" world");
  printf("%s\n", hello.c_str());

  pin_size_t LED_pin = GPIO::D6;
  pinMode(LED_pin, OUTPUT);

  bool i = false;
  auto instant = Instant();
  auto d = std::chrono::duration<uint64_t, std::milli>(500);

  while (true) {
    if (instant.elapsed() > d) {
      digitalWrite(LED_pin, boolToStatus(i));
      i = !i;
      printf("millis=%d;\n", instant.count());
      instant.reset();
    }
  }
}
