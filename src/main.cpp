#include "clock.h"
#include "ch32v003fun.h"
#include "system_tick.h"
#include "gpio.h"
#include <cstdlib>
#include <printf.h>

int rand_range(int min, int max) {
  return min + (std::rand() % (max - min + 1));
}

int main() {
  SystemInit48HSI();
  SysTick_init();
  SetupUART(115200);

  pin_size_t LED_pin = GPIO::D6;
  pinMode(LED_pin, OUTPUT);

  bool i = false;
  auto last = millis();
  uint32_t d = rand_range(10, 100);
  while (true) {
    if (millis() - last > d) {
      digitalWrite(LED_pin, boolToStatus(i));
      i = !i;
      printf("delayed %d ms; count: %d;\n", d, millis());
      last = millis();
      d = rand_range(10, 100);
    }
  }
}
