#if !defined(_RADIOLIB_HAL_H)
#define _RADIOLIB_HAL_H

#include <stdint.h>
#include <stddef.h>
#include "gpio.h"

// https://www.youtube.com/watch?v=gTNJXVmuRRA
// I do those to eliminate the unnecessary virtual function.
// function pointer/delegate call is okay...

/*!
  \class Hal
  \brief Hardware abstraction library base interface.
*/
class RadioLibHal {
  public:

    // values for pin modes, levels and change directions
    // these tell RadioLib how are different logic states represented on a given platform

    /*!
      \brief Value to be used as the "input" GPIO direction.
    */
    const uint32_t GpioModeInput = GPIO::INPUT;

    /*!
      \brief Value to be used as the "output" GPIO direction.
    */
    const uint32_t GpioModeOutput = GPIO::OUTPUT;
    
    /*!
      \brief Value to be used as the "low" GPIO level.
    */
    const uint32_t GpioLevelLow = GPIO::LOW;
    
    /*!
      \brief Value to be used as the "high" GPIO level.
    */
    const uint32_t GpioLevelHigh = GPIO::HIGH;

    /*!
      \brief GPIO pin mode (input/output/...) configuration method.
      Must be implemented by the platform-specific hardware abstraction!
      \param pin Pin to be changed (platform-specific).
      \param mode Mode to be set (platform-specific).
    */
     void pinMode(uint32_t pin, uint32_t mode);

    /*!
      \brief Digital write method.
      Must be implemented by the platform-specific hardware abstraction!
      \param pin Pin to be changed (platform-specific).
      \param value Value to set (platform-specific).
    */
    void digitalWrite(uint32_t pin, uint32_t value);

    /*!
      \brief Digital read method.
      Must be implemented by the platform-specific hardware abstraction!
      \param pin Pin to be changed (platform-specific).
      \returns Value read on the pin (platform-specific).
    */
    uint32_t digitalRead(uint32_t pin);
    
    /*!
      \brief Blocking wait function.
      Must be implemented by the platform-specific hardware abstraction!
      \param ms Number of milliseconds to wait.
    */
    void delay(unsigned long ms);

    /*!
      \brief Get number of milliseconds since start.
      Must be implemented by the platform-specific hardware abstraction!
      \returns Number of milliseconds since start.
    */
    unsigned long millis();
    
    /*!
      \brief SPI initialization method.
    */
    void spiBegin();

    /*!
      \brief Method to start SPI transaction.
    */
    void spiBeginTransaction();

    /*!
      \brief Method to transfer one byte over SPI.
      \param b Byte to send.
      \returns Received byte.
    */
    uint8_t spiTransfer(uint8_t b);

    /*!
      \brief Method to end SPI transaction.
    */
    void spiEndTransaction();

    /*!
      \brief SPI termination method.
    */
    void spiEnd();

    /*!
      \brief Module initialization method.
      This will be called by all radio modules at the beginning of startup.
      Can be used to e.g., initalize SPI interface.
    */
    void init();

    /*!
      \brief Module termination method.
      This will be called by all radio modules when the desctructor is called.
      Can be used to e.g., stop SPI interface.
    */
    void term();

    void yield();
};

#endif
