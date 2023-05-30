//
// Created by Kurosu Chan on 2023/5/30.
//

#include "led.h"


static constexpr uint8_t CNF_AND_MODE_WIDTH = 4; // 4 bytes for each pin.

void LED::begin() {
  pinMode(R_PIN, OUTPUT);
  pinMode(G_PIN, OUTPUT);
  pinMode(B_PIN, OUTPUT);
}

void LED::setColor(bool r, bool g, bool b) {
  digitalWrite(R_PIN, r ? HIGH : LOW);
  digitalWrite(G_PIN, g ? HIGH : LOW);
  digitalWrite(B_PIN, b ? HIGH : LOW);
}
