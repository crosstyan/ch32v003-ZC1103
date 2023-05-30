//
// Created by Kurosu Chan on 2023/5/30.
//

#ifndef SIMPLE_LED_H
#define SIMPLE_LED_H
#include "ch32v003fun.h"
#include "gpio.h"

// PWM should be used...
// too lazy to implement PWM
// https://github.com/crosstyan/ch32v003fun/tree/5e7eddc2ff74e7dfec97e5655ec3ac9420c53ec7/examples/tim1_pwm

// TODO: use PWM
class LED {
  // T1CH1
  pin_size_t R_PIN = GPIO::D2;
  // T2CH2
  pin_size_t G_PIN = GPIO::D3;
  // T2CH1ETR (the same as T2CH1?)
  pin_size_t B_PIN = GPIO::D4;
public:
  void begin();
  void setColor(bool r, bool g, bool b);
};

#endif //SIMPLE_LED_H
