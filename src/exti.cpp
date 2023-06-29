//
// Created by Kurosu Chan on 2023/5/23.
//
#include "exti.h"
#include "flags.h"

constexpr uint8_t EXTICR_PC = 0b10;
constexpr uint8_t EXTICR_PD = 0b11;
constexpr uint8_t EXTICR_PA = 0x00;
constexpr uint8_t EXTICR_PX = EXTICR_PD;
constexpr uint8_t PIN_NUM = 0;

extern "C" void EXTI7_0_IRQHandler(){
  // Acknowledge the interrupt
  Flags::setFlag(true);
  EXTI->INTFR = 1<<PIN_NUM;
}

/*
 * @brief Configure the EXTI for GPIO pin C3
 * @see https://github.com/cnlohr/ch32v003fun/blob/master/examples/exti_pin_change_isr/exti_pin_change_isr.c
 */
void configureEXTI(){
  constexpr uint8_t CNF_AND_MODE_WIDTH = 4; // 4 bytes for each pin.
  constexpr uint8_t EXTICR_EXTIx_WIDTH = 2;

  asm volatile(
#if __GNUC__ > 10
      ".option arch, +zicsr\n"
      #endif
      "addi t1, x0, 3\n"
      "csrrw x0, 0x804, t1\n"
      : : :  "t1" );
  // Enable GPIOs
  // AFIOEN: I/O auxiliary function module clock enable bit.
  RCC->APB2PCENR = RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO;
  // GPIO C3 for input pin change.
  GPIOC->CFGLR |= (GPIO_SPEED_IN | GPIO_CNF_IN_PUPD)<<(CNF_AND_MODE_WIDTH * PIN_NUM);
  GPIOC->CFGLR |= (GPIO_CNF_IN_PUPD)<<(CNF_AND_MODE_WIDTH * 1);  // Keep SWIO enabled.
  // GPIO and Alternate function (AFIO)
  // Configure the IO as an interrupt.
  // (x=0-7), external interrupt input pin configuration bit.
  // Used to determine to which port pins the external interrupt pins are mapped.
  // 00: xth pin of the PA pin.
  // 10: xth pin of the PC pin.
  // 11: xth pin of the PD pin.
  AFIO->EXTICR = EXTICR_PX<<(EXTICR_EXTIx_WIDTH * PIN_NUM);
  EXTI->INTENR = 1<<PIN_NUM; // Enable EXT3
  EXTI->RTENR = 1<<PIN_NUM;  // Rising edge trigger

  NVIC_EnableIRQ( EXTI7_0_IRQn );
}
