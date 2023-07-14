//
// Created by Kurosu Chan on 2023/5/22.
// Basically copy and paste of
// https://github.com/crosstyan/ch32v003fun/blob/2aa519b999e90d2de7cc1589ee867941b4284522/examples/systick_irq/systick_irq.c
//

#ifndef SIMPLE_SYSTEM_TICK_H
#define SIMPLE_SYSTEM_TICK_H

#include "clock.h"
#include "ch32v003fun.h"

static const auto SYSTICK_SR_CNTIF   = (1u << 0);
static const auto SYSTICK_CTLR_STE   = (1u << 0);
static const auto SYSTICK_CTLR_STIE  = (1u << 1);
static const auto SYSTICK_CTLR_STCLK = (1u << 2);
static const auto SYSTICK_CTLR_STRE  = (1u << 3);
static const auto SYSTICK_CTLR_SWIE  = (1u << 31);

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

/**
 * @brief disable the system tick interrupt
 * @warning disabling the system tick would cause the millis() and micros() to stop working like expected.
 *          You would lose some time.
 * @note You have to disable this if you want to use standby or the system tick would wake up the MCU every tick.
 * @see https://stackoverflow.com/questions/47981/how-to-set-clear-and-toggle-a-single-bit
 */
inline void disableSystemTickExti() {
  SysTick->CTLR &= ~SYSTICK_CTLR_STIE;
};

/**
 * @brief enable the system tick interrupt
 * @note already called in SysTick_init()
 */
inline void enableSystemTickExti(){
  SysTick->CTLR |= SYSTICK_CTLR_STIE;
};

#endif // SIMPLE_SYSTEM_TICK_H
