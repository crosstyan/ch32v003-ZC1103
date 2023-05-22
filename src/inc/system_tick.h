//
// Created by Kurosu Chan on 2023/5/22.
// Basically copy and paste of
// https://github.com/crosstyan/ch32v003fun/blob/2aa519b999e90d2de7cc1589ee867941b4284522/examples/systick_irq/systick_irq.c
//

#ifndef SIMPLE_SYSTEM_TICK_H
#define SIMPLE_SYSTEM_TICK_H

#include "clock.h"
#include "ch32v003fun.h"

void SysTick_init();

/*
 * SysTick ISR just counts ticks
 * note - the __attribute__((interrupt)) syntax is crucial!
 */
void SysTick_Handler() __attribute__((interrupt));

uint32_t millis();

#endif //SIMPLE_SYSTEM_TICK_H
