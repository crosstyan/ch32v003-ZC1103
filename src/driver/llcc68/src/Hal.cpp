#include "Hal.h"

RadioLibHal::RadioLibHal(const uint32_t input, const uint32_t output, const uint32_t low, const uint32_t high, const uint32_t rising, const uint32_t falling)
    : GpioModeInput(input),
      GpioModeOutput(output),
      GpioLevelLow(low),
      GpioLevelHigh(high),
      GpioInterruptRising(rising),
      GpioInterruptFalling(falling) {}

void RadioLibHal::init() {

}

void RadioLibHal::term() {

}
