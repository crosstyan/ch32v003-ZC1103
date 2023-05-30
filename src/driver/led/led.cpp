//
// Created by Kurosu Chan on 2023/5/30.
//

#include "led.h"

// just a reminder. won't be used.
//static const pin_size_t LED_B_PIN = GPIO::D4;
//static const pin_size_t LED_G_PIN = GPIO::D3;
//static const pin_size_t LED_R_PIN = GPIO::D2;

static constexpr uint8_t CNF_AND_MODE_WIDTH = 4; // 4 bytes for each pin.

void LED::begin() {
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOD;

  // D2
  GPIOD->CFGLR &= ~(0xf << (CNF_AND_MODE_WIDTH * 2));
  GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP) << (CNF_AND_MODE_WIDTH * 2);

  // D3
  GPIOD->CFGLR &= ~(0xf << (CNF_AND_MODE_WIDTH * 3));
  GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP) << (CNF_AND_MODE_WIDTH * 3);

  // D4
  GPIOD->CFGLR &= ~(0xf << (CNF_AND_MODE_WIDTH * 4));
  GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP) << (CNF_AND_MODE_WIDTH * 4);
}

void LED::setColor(bool r, bool g, bool b) {
  uint32_t colorBits = 0;

  if (r) {
    colorBits |= (1u << 2);
  }
  if (g) {
    colorBits |= (1u << 3);
  }
  if (b) {
    colorBits |= (1u << 4);
  }

  GPIOD->BSHR = colorBits;
  GPIOD->BCR = ~colorBits;
}
