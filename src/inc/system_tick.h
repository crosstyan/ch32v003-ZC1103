//
// Created by Kurosu Chan on 2023/5/22.
// Basically copy and paste of
// https://github.com/crosstyan/ch32v003fun/blob/2aa519b999e90d2de7cc1589ee867941b4284522/examples/systick_irq/systick_irq.c
//

#ifndef SIMPLE_SYSTEM_TICK_H
#define SIMPLE_SYSTEM_TICK_H

#include "clock.h"
#include "ch32v003fun.h"

// NOT usable...
// #define SYSTEM_TICK_US

extern "C" void SysTick_init();

/*
 * SysTick ISR just counts ticks
 * note - the __attribute__((interrupt)) syntax is crucial!
 * extern "C" is also important for C++ linkage
 */
extern "C" void SysTick_Handler() __attribute__((interrupt));

uint64_t millis();

/// should only be used when `SYSTEM_TICK_US` is defined
uint64_t micros();

#endif //SIMPLE_SYSTEM_TICK_H
