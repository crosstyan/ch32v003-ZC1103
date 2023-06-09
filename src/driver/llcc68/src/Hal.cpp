#include "spi.h"
#include "clock.h"
#include "ch32v003fun.h"
#include "ch32v003_SPI.h"
#include "system_tick.h"
#include "gpio.h"
#include "Hal.h"

void RadioLibHal::init() {
  this->spiBegin();
}

void RadioLibHal::term() {
  this->spiEnd();
}

void RadioLibHal::pinMode(uint32_t pin, uint32_t mode) {
  GPIO::pinMode(pin, static_cast<GPIO::PinMode>(mode));
}

void RadioLibHal::digitalWrite(uint32_t pin, uint32_t value) {
  GPIO::digitalWrite(pin, static_cast<GPIO::PinStatus>(value));
}

uint32_t RadioLibHal::digitalRead(uint32_t pin) {
  return GPIO::digitalRead(pin);
}
void RadioLibHal::delay(unsigned long ms) {
  Delay_Ms(ms);
}
unsigned long RadioLibHal::millis() {
  return ::millis();
}

void RadioLibHal::spiBegin() {
  SPI_init();
  SPI_begin_8();
}

uint8_t RadioLibHal::spiTransfer(uint8_t b) {
  return SPI_transfer_8(b);
}

// not sure... leave it for now
void RadioLibHal::spiBeginTransaction() {}
void RadioLibHal::spiEndTransaction() {}

void RadioLibHal::spiEnd() {}
// do nothing since no OS
void RadioLibHal::yield() {}
