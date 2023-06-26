#include "spi.h"
#include "clock.h"
#include "ch32v003fun.h"
#include "ch32v003_SPI.h"
#include "system_tick.h"
#include "gpio.h"
#include "llcc68.h"

void LLCC68::halInit() {
  this->spiBegin();
}

void LLCC68::halTerm() {
  this->spiEnd();
}

void LLCC68::pinMode(uint32_t pin, uint32_t mode) {
  GPIO::pinMode(pin, static_cast<GPIO::PinMode>(mode));
}

void LLCC68::digitalWrite(uint32_t pin, uint32_t value) {
  GPIO::digitalWrite(pin, static_cast<GPIO::PinStatus>(value));
}

uint32_t LLCC68::digitalRead(uint32_t pin) {
  return GPIO::digitalRead(pin);
}
void LLCC68::delay(unsigned long ms) {
  Delay_Ms(ms);
}
unsigned long LLCC68::millis() {
  return ::millis();
}

void LLCC68::spiBegin() {
  SPI_init();
  SPI_begin_8();
}

uint8_t LLCC68::spiTransfer(uint8_t b) {
  return SPI_transfer_8(b);
}

// not sure... leave it for now
void LLCC68::spiBeginTransaction() {}
void LLCC68::spiEndTransaction() {}

void LLCC68::spiEnd() {}
// do nothing since no OS
void LLCC68::yield() {}
