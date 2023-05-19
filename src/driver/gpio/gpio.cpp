//
// Created by Kurosu Chan on 2023/5/19.
//
#include "gpio.h"

bool statusToBool(PinStatus status) {
  return status == HIGH;
};

PinStatus boolToStatus(bool val) {
  return val ? HIGH : LOW;
};

uint8_t gpioForPin(pin_size_t pin) {
  if (pin < 2) {
    return 0;
  } else if (pin < 10) {
    return 2;
  } else {
    return 3;
  }
}

GPIO_TypeDef *gpioRegister(uint8_t gpio) {
  if (gpio == 0) {
    return GPIOA;
  } else if (gpio == 2) {
    return GPIOC;
  } else {
    return GPIOD;
  }
}

uint8_t gpioPin(uint8_t gpio, pin_size_t pin) {
  if (gpio == 0) {
    return pin;
  } else if (gpio == 2) {
    return pin - 2;
  } else {
    return pin - 10;
  }
}

void pinMode(pin_size_t pin, PinMode mode) {
  uint8_t gpio = gpioForPin(pin);
  GPIO_TypeDef *port = gpioRegister(gpio);
  uint8_t p = gpioPin(gpio, pin);

  // Enable GPIO
  RCC->APB2PCENR |= (0x04 << gpio);

  // Configure pin
  uint8_t pinConfig = 0;
  switch (mode) {
    case INPUT:
      pinConfig = GPIO_CNF_IN_FLOATING;
      break;
    case OUTPUT:
      pinConfig = GPIO_Speed_50MHz | GPIO_CNF_OUT_PP;
      break;
    case INPUT_PULLUP:
      pinConfig = GPIO_CNF_IN_PUPD;
      port->BSHR = (((uint32_t) 0x01) << p);
      break;
    case INPUT_PULLDOWN:
    default:
      pinConfig = GPIO_CNF_IN_PUPD;
      port->BCR = (((uint32_t) 0x01) << p);
      break;
  }

  port->CFGLR &= ~(0xf << (p * 4));
  port->CFGLR |= (pinConfig << (p * 4));
}

void digitalWrite(pin_size_t pin, PinStatus val) {
  uint8_t gpio = gpioForPin(pin);
  GPIO_TypeDef *port = gpioRegister(gpio);
  uint8_t p = gpioPin(gpio, pin);

  if (val == HIGH) {
    port->BSHR = ((uint32_t) 1 << p);
  } else {
    port->BCR = ((uint32_t) 1 << p);
  }
}

PinStatus digitalRead(pin_size_t pin) {
  uint8_t gpio = gpioForPin(pin);
  GPIO_TypeDef *port = gpioRegister(gpio);
  uint8_t p = gpioPin(gpio, pin);

  return (port->INDR & (1 << p)) != 0 ? HIGH : LOW;
}
