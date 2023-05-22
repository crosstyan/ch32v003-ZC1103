#include "clock.h"
#include "ch32v003fun.h"
#include "gpio.h"
#include "rfsystem.h"
#include <cstdlib>
#include <printf.h>

int rand_range(int min, int max) {
  return min + (std::rand() % (max - min + 1));
}

int main() {
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//  Delay_Init();
//  Delay_Ms(100);
//  USART_Printf_Init(115200);

//  SystemCoreClockUpdate();
//  printf("SystemClk:%lu\r\n", SystemCoreClock);
  SystemInit48HSI();
  SetupUART(115200);

  pin_size_t LED_pin = GPIO::D6;
  pinMode(LED_pin, OUTPUT);

  bool i = false;
  while (true) {
    uint32_t d = rand_range(50, 500);
    Delay_Ms(d);
    digitalWrite(LED_pin, boolToStatus(i));
    i = !i;
  }
}
