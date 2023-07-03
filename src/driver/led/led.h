//
// Created by Kurosu Chan on 2023/5/30.
//

#ifndef SIMPLE_LED_H
#define SIMPLE_LED_H

#include "ch32v003fun.h"

// PWM should be used...
// too lazy to implement PWM
// https://github.com/crosstyan/ch32v003fun/tree/5e7eddc2ff74e7dfec97e5655ec3ac9420c53ec7/examples/tim1_pwm

// TODO: use PWM
namespace LED {
  void begin();

  void setColor(bool r, bool g, bool b);
  /// low 3 bits are used. (rgb)
  void setColor(uint8_t rgb);
}

#endif //SIMPLE_LED_H
