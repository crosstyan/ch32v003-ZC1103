#include "clock.h"
#include "ch32v003fun.h"
#include "system_tick.h"
#include "gpio.h"
#include <printf.h>
#include "instant.h"

int rand_range(int min, int max) {
  return min + (std::rand() % (max - min + 1));
}

int main() {
  SystemInit48HSI();
//  SystemInitHSE(0);
  SysTick_init();
  SetupUART(115200);
  SetupDebugPrintf();

  pin_size_t LED_pin = GPIO::D6;
  pinMode(LED_pin, OUTPUT);

  bool i = false;
  auto instant = Instant();
  auto d = std::chrono::duration<uint64_t, std::milli>(500);

// Don't have enough memory to run this
//  while (true) {
//    instant.try_run<void(void)>([&]() {
//      digitalWrite(LED_pin, boolToStatus(i));
//      i = !i;
//      printf("delayed %d ms; count: %d;\n", d.count(), instant.count());
//    }, d);
//  }

  while (true) {
    if (instant.elapsed() > d) {
      digitalWrite(LED_pin, boolToStatus(i));
      i = !i;
      printf("delayed=%dms;\n", d.count());
      printf("count=%dms;\n", instant.count());
      instant.reset();
    }
  }
}
