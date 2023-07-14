//
// Created by Kurosu Chan on 2023/7/7.
//

#ifndef SIMPLE_ADC_H
#define SIMPLE_ADC_H

#include <ch32v003fun.h>
#include "gpio.h"
// A1 is analog channel 1
static const pin_size_t PWD_PIN      = GPIO::A1;
static const auto PWD_PIN_GROUP      = GPIOA;
static const uint8_t PWD_PIN_NUM     = 1;
static const uint8_t PWD_PIN_ADC_CHN = 1;

// used in CFGLR
// 4 bits for each pin. two bits for CNF and two bits for MODE
static constexpr uint8_t CNF_AND_MODE_WIDTH = 4;
static constexpr uint8_t EXTICR_EXTIx_WIDTH = 2;
static constexpr uint8_t SMPx_WIDTH         = 3;

void adc_init(void) {
  // ADCCLK = 24 MHz => RCC_ADCPRE = 0: divide by 2
  RCC->CFGR0 &= ~(0x1F << 11);

  // Enable GPIOD and ADC
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_ADC1;

  // PA1 is analog input chl 1
  // 0xf = 0b1111
  // CNF = 00: Analog, MODE = 00: Input
  PWD_PIN_GROUP->CFGLR &= ~(0xf << (PWD_PIN_NUM * CNF_AND_MODE_WIDTH));

  // Reset the ADC to init all regs
  RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
  RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;

  // Set up single conversion on chl 1
  ADC1->RSQR1 = 0;
  ADC1->RSQR2 = 0;
  // The number of the 1st conversion channel in the rule sequence (0-9).
  ADC1->RSQR3 = PWD_PIN_ADC_CHN; // 0-9 for 8 ext inputs and two internals

  // sample time configuration for channel x
  ADC1->SAMPTR2 &= ~(ADC_SMP0 << (SMPx_WIDTH * PWD_PIN_ADC_CHN));
  ADC1->SAMPTR2 |= 7 << (SMPx_WIDTH * PWD_PIN_ADC_CHN); // 0:7 => 3/9/15/30/43/57/73/241 cycles

  // turn on ADC and set rule group to sw trig
  ADC1->CTLR2 |= ADC_ADON | ADC_EXTSEL;

  // Reset calibration
  ADC1->CTLR2 |= ADC_RSTCAL;
  while (ADC1->CTLR2 & ADC_RSTCAL) {
    // wait for calibration reset to complete
  }

  // Calibrate
  ADC1->CTLR2 |= ADC_CAL;
  while (ADC1->CTLR2 & ADC_CAL) {
    // wait for calibration to complete
  }

  // should be ready for SW conversion now
}

/*
 * start conversion, wait and return result
 */
uint16_t adc_get() {
  // start sw conversion (auto clears)
  ADC1->CTLR2 |= ADC_SWSTART;

  while (!(ADC1->STATR & ADC_EOC)) {
    // wait for conversion to complete
  }

  // get result
  // ADC regular data register
  return ADC1->RDATAR;
}

#endif // SIMPLE_ADC_H
