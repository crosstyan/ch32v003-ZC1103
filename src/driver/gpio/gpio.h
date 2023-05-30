//
// Created by Kurosu Chan on 2023/5/19.
// copy and paste from
// https://github.com/AlexanderMandera/arduino-wch32v003/blob/6e0af1483712150cce910ba3f22ce9bb208fb584/cores/arduino/wiring_digital.cpp
// to provide Arduino-like API for GPIO. Nothing fancy.
//

#ifndef SIMPLE_GPIO_H
#define SIMPLE_GPIO_H

#include "ch32v003fun.h"

// defined in ch32v003fun.h
// https://github.com/cnlohr/ch32v003fun/blob/2698222cf674c546cac75e4a1db99c4b97473f08/ch32v003fun/ch32v003fun.h#L3239
// I'm not sure the difference between this header and official one.
#define GPIO_SPEED_IN 0

#define GPIO_CNF_IN_ANALOG   0
#define GPIO_CNF_IN_FLOATING 4
#define GPIO_CNF_IN_PUPD     8
#define GPIO_CNF_OUT_PP      0
#define GPIO_CNF_OUT_OD      4
#define GPIO_CNF_OUT_PP_AF   8
#define GPIO_CNF_OUT_OD_AF   12

typedef uint8_t pin_size_t;

// https://github.com/AlexanderMandera/arduino-wch32v003/blob/6e0af1483712150cce910ba3f22ce9bb208fb584/variants/WCH32V003/pins_arduino.h#L20
namespace GPIO {
// GPIO A1~A2
  const pin_size_t A1 = 0u;
  const pin_size_t A2 = 1u;
// GPIO C0~C7
  const pin_size_t C0 = 2u;
  const pin_size_t C1 = 3u;
  const pin_size_t C2 = 4u;
  const pin_size_t C3 = 5u;
  const pin_size_t C4 = 6u;
  const pin_size_t C5 = 7u;
  const pin_size_t C6 = 8u;
  const pin_size_t C7 = 9u;
// GPIO D0~D7
  const pin_size_t D0 = 10u;
  const pin_size_t D1 = 11u;
  const pin_size_t D2 = 12u;
  const pin_size_t D3 = 13u;
  const pin_size_t D4 = 14u;
  const pin_size_t D5 = 15u;
  const pin_size_t D6 = 16u;
  const pin_size_t D7 = 17u;
}

enum PinMode {
  INPUT,
  OUTPUT,
  INPUT_PULLUP,
  INPUT_PULLDOWN,
};

enum PinStatus {
  LOW = 0,
  HIGH = 1,
};


bool statusToBool(PinStatus status);

PinStatus boolToStatus(bool val);

uint8_t gpioForPin(pin_size_t pin);

GPIO_TypeDef *gpioRegister(uint8_t gpio);

uint8_t gpioPin(uint8_t gpio, pin_size_t pin);

/**
 * @note cost 102 bytes in Flash
 */
void pinMode(pin_size_t pin, PinMode mode);

/**
 * @note cost 66 bytes in Flash
 */
void digitalWrite(pin_size_t pin, PinStatus val);

PinStatus digitalRead(pin_size_t pin);

#endif //SIMPLE_GPIO_H
