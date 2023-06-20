#include "spi.h"
#include "clock.h"
#include "ch32v003fun.h"
#include "ch32v003_SPI.h"
#include "system_tick.h"
#include "printf.h"
#include "gpio.h"
#include "Hal.h"

void RadioLibHal::init() {}

void RadioLibHal::term() {}

inline void RadioLibHal::pinMode(uint32_t pin, uint32_t mode) {
  GPIO::pinMode(pin, static_cast<GPIO::PinMode>(mode));
}

inline void RadioLibHal::digitalWrite(uint32_t pin, uint32_t value) {
  GPIO::digitalWrite(pin, static_cast<GPIO::PinStatus>(value));
}

inline uint32_t RadioLibHal::digitalRead(uint32_t pin) {
  return GPIO::digitalRead(pin);
}
inline void RadioLibHal::delay(unsigned long ms) {
  Delay_Ms(ms);
}
inline unsigned long RadioLibHal::millis() {
  return ::millis();
}

inline void RadioLibHal::spiBegin() {
  SPI_init();
  SPI_begin_8();
}



inline uint8_t RadioLibHal::spiTransfer(uint8_t b) {
  return SPI_transfer_8(b);
}

// not sure... leave it for now
void RadioLibHal::spiBeginTransaction() {}
// not sure... leave it for now
void RadioLibHal::spiEndTransaction() {}

void RadioLibHal::spiEnd() {}
// do nothing since no OS
void RadioLibHal::yield() {}
