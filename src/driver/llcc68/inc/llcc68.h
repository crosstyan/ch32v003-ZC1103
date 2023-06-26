//
// Created by Kurosu Chan on 2023/6/20.
//

#ifndef SIMPLE_LLCC68_H
#define SIMPLE_LLCC68_H

#include "spi.h"
#include "clock.h"
#include "printf.h"
#include "ch32v003fun.h"
#include "ch32v003_SPI.h"
#include "gpio.h"
#include "unit.h"
#include <etl/optional.h>
#include <etl/vector.h>
#include <etl/delegate.h>
#include "radio_sx126x.h"
#include "TypeDef.h"
#include "cnl_def.h"
#include <etl/optional.h>

// basically LLCC68 is a SX126x
// LLCC68芯片引脚兼容SX1262,且在设计、驱动代码及应用上与SX1262完全相同
#define RADIOLIB_LLCC68_CHIP_TYPE                               "LLCC68"

/*!
* Value to use as the last element in a mode table to indicate the
* end of the table.
*
* See setRfSwitchTable() for details.
 */
#define END_OF_MODE_TABLE    { MODE_END_OF_TABLE, {} }

// default timeout for SPI transfers
#define RADIOLIB_MODULE_SPI_TIMEOUT                             (1000)

class LLCC68 {
public:

  /*!
     * \brief The maximum number of pins supported by the RF switch
     * code.
     *
     * Note: It is not recommended to use this constant in your sketch
     * when defining a rfswitch pins array, to prevent issues when this
     * value is ever increased and such an array gets extra zero
     * elements (that will be interpreted as pin 0).
   */
  static const size_t RFSWITCH_MAX_PINS = 3;

  /*!
     * Description of RF switch pin states for a single mode.
     *
     * See setRfSwitchTable() for details.
   */
  struct RfSwitchMode_t {
    uint8_t mode;
    uint32_t values[RFSWITCH_MAX_PINS];
  };

  /*!
     * Constants to use in a mode table set be setRfSwitchTable. These
     * constants work for most radios, but some radios define their own
     * constants to be used instead.
     *
     * See setRfSwitchTable() for details.
   */
  enum OpMode_t {
    /*! End of table marker, use \ref END_OF_MODE_TABLE constant
       * instead. Value is zero to ensure zero-initialized mode ends the
       * table */
    MODE_END_OF_TABLE = 0,
    /*! Idle mode */
    MODE_IDLE,
    /*! Receive mode */
    MODE_RX,
    /*! Transmission mode */
    MODE_TX,
  };

  /*!
    \brief Module constructor.
    \param cs Pin to be used as chip select.
    \param irq Pin to be used as interrupt/GPIO.
    \param rst Pin to be used as hardware reset for the module.
    \param gpio Pin to be used as additional interrupt/GPIO.
  */
  void setModule(uint32_t cs, uint32_t irq, uint32_t rst, uint32_t gpio = RADIOLIB_NC);

  /*!
    \brief Basic SPI read command. Defaults to 0x00.
  */
  uint8_t SPIreadCommand = 0b00000000;

  /*!
    \brief Basic SPI write command. Defaults to 0x80.
  */
  uint8_t SPIwriteCommand = 0b10000000;

  /*!
    \brief Basic SPI no-operation command. Defaults to 0x00.
  */
  uint8_t SPInopCommand = 0x00;

  /*!
    \brief Basic SPI status read command. Defaults to 0x00.
  */
  uint8_t SPIstatusCommand = 0x00;

  /*!
    \brief SPI address width. Defaults to 8, currently only supports 8 and 16-bit addresses.
  */
  uint8_t SPIaddrWidth = 8;

  /*!
    \brief Whether the SPI interface is stream-type (e.g. SX126x) or register-type (e.g. SX127x).
    Defaults to register-type SPI interfaces.
  */
  bool SPIstreamType = false;

  /*!
    \brief The last recorded SPI stream error.
  */
  int16_t SPIstreamError = RADIOLIB_ERR_UNKNOWN;

  /*!
    \brief SPI status parsing callback typedef.
  */
  typedef int16_t (*SPIparseStatusCb_t)(uint8_t in);

  /*!
    \brief Callback to function that will parse the module-specific status codes to RadioLib status codes.
    Typically used for modules with SPI stream-type interface (e.g. SX126x/SX128x).
  */
  SPIparseStatusCb_t SPIparseStatusCb = nullptr;

  // basic methods

  /*!
    \brief Initialize low-level module control.
  */
  void init();

  /*!
    \brief Terminate low-level module control.
  */
  void term();

  // SPI methods

  /*!
    \brief SPI read method that automatically masks unused bits. This method is the preferred SPI read mechanism.
    \param reg Address of SPI register to read.
    \param msb Most significant bit of the register variable. Bits above this one will be masked out.
    \param lsb Least significant bit of the register variable. Bits below this one will be masked out.
    \returns Masked register value or status code.
  */
  int16_t SPIgetRegValue(uint16_t reg, uint8_t msb = 7, uint8_t lsb = 0);

  /*!
    \brief Overwrite-safe SPI write method with verification. This method is the preferred SPI write mechanism.
    \param reg Address of SPI register to write.
    \param value Single byte value that will be written to the SPI register.
    \param msb Most significant bit of the register variable. Bits above this one will not be affected by the write operation.
    \param lsb Least significant bit of the register variable. Bits below this one will not be affected by the write operation.
    \param checkInterval Number of milliseconds between register writing and verification reading. Some registers need up to 10ms to process the change.
    \param checkMask Mask of bits to check, only bits set to 1 will be verified.
    \returns \ref status_codes
  */
  int16_t SPIsetRegValue(uint16_t reg, uint8_t value, uint8_t msb = 7, uint8_t lsb = 0, uint8_t checkInterval = 2, uint8_t checkMask = 0xFF);

  /*!
    \brief SPI burst read method.
    \param reg Address of SPI register to read.
    \param numBytes Number of bytes that will be read.
    \param inBytes Pointer to array that will hold the read data.
  */
  void SPIreadRegisterBurst(uint16_t reg, size_t numBytes, uint8_t* inBytes);

  /*!
    \brief SPI basic read method. Use of this method is reserved for special cases, SPIgetRegValue should be used instead.
    \param reg Address of SPI register to read.
    \returns Value that was read from register.
  */
  uint8_t SPIreadRegister(uint16_t reg);

  /*!
    \brief SPI burst write method.
    \param reg Address of SPI register to write.
    \param data Pointer to array that holds the data that will be written.
    \param numBytes Number of bytes that will be written.
  */
  void SPIwriteRegisterBurst(uint16_t reg, uint8_t* data, size_t numBytes);

  /*!
    \brief SPI basic write method. Use of this method is reserved for special cases, SPIsetRegValue should be used instead.
    \param reg Address of SPI register to write.
    \param data Value that will be written to the register.
  */
  void SPIwriteRegister(uint16_t reg, uint8_t data);

  /*!
    \brief SPI single transfer method.
    \param cmd SPI access command (read/write/burst/...).
    \param reg Address of SPI register to transfer to/from.
    \param dataOut Data that will be transfered from master to slave.
    \param dataIn Data that was transfered from slave to master.
    \param numBytes Number of bytes to transfer.
  */
  void SPItransfer(uint8_t cmd, uint16_t reg, uint8_t* dataOut, uint8_t* dataIn, size_t numBytes);

  /*!
    \brief Method to check the result of last SPI stream transfer.
    \returns \ref status_codes
  */
  int16_t SPIcheckStream();

  /*!
    \brief Method to perform a read transaction with SPI stream.
    \param cmd SPI operation command.
    \param data Data that will be transferred from slave to master.
    \param numBytes Number of bytes to transfer.
    \param waitForGpio Whether to wait for some GPIO at the end of transfer (e.g. BUSY line on SX126x/SX128x).
    \param verify Whether to verify the result of the transaction after it is finished.
    \returns \ref status_codes
  */
  int16_t SPIreadStream(uint8_t cmd, uint8_t* data, size_t numBytes, bool waitForGpio = true, bool verify = true);

  /*!
    \brief Method to perform a read transaction with SPI stream.
    \param cmd SPI operation command.
    \param cmdLen SPI command length in bytes.
    \param data Data that will be transferred from slave to master.
    \param numBytes Number of bytes to transfer.
    \param waitForGpio Whether to wait for some GPIO at the end of transfer (e.g. BUSY line on SX126x/SX128x).
    \param verify Whether to verify the result of the transaction after it is finished.
    \returns \ref status_codes
  */
  int16_t SPIreadStream(uint8_t* cmd, uint8_t cmdLen, uint8_t* data, size_t numBytes, bool waitForGpio = true, bool verify = true);

  /*!
    \brief Method to perform a write transaction with SPI stream.
    \param cmd SPI operation command.
    \param data Data that will be transferred from master to slave.
    \param numBytes Number of bytes to transfer.
    \param waitForGpio Whether to wait for some GPIO at the end of transfer (e.g. BUSY line on SX126x/SX128x).
    \param verify Whether to verify the result of the transaction after it is finished.
    \returns \ref status_codes
  */
  int16_t SPIwriteStream(uint8_t cmd, uint8_t* data, size_t numBytes, bool waitForGpio = true, bool verify = true);

  /*!
    \brief Method to perform a write transaction with SPI stream.
    \param cmd SPI operation command.
    \param cmdLen SPI command length in bytes.
    \param data Data that will be transferred from master to slave.
    \param numBytes Number of bytes to transfer.
    \param waitForGpio Whether to wait for some GPIO at the end of transfer (e.g. BUSY line on SX126x/SX128x).
    \param verify Whether to verify the result of the transaction after it is finished.
    \returns \ref status_codes
  */
  int16_t SPIwriteStream(uint8_t* cmd, uint8_t cmdLen, uint8_t* data, size_t numBytes, bool waitForGpio = true, bool verify = true);

  /*!
    \brief SPI single transfer method for modules with stream-type SPI interface (SX126x, SX128x etc.).
    \param cmd SPI operation command.
    \param cmdLen SPI command length in bytes.
    \param write Set to true for write commands, false for read commands.
    \param dataOut Data that will be transfered from master to slave.
    \param dataIn Data that was transfered from slave to master.
    \param numBytes Number of bytes to transfer.
    \param waitForGpio Whether to wait for some GPIO at the end of transfer (e.g. BUSY line on SX126x/SX128x).
    \param timeout GPIO wait period timeout in milliseconds.
    \returns \ref status_codes
  */
  int16_t SPItransferStream(uint8_t* cmd, uint8_t cmdLen, bool write, uint8_t* dataOut, uint8_t* dataIn, size_t numBytes, bool waitForGpio, uint32_t timeout);

  // pin number access methods

  /*!
    \brief Access method to get the pin number of SPI chip select.
    \returns Pin number of SPI chip select configured in the constructor.
  */
  uint32_t getCs() const { return(csPin); }

  /*!
    \brief Access method to get the pin number of interrupt/GPIO.
    \returns Pin number of interrupt/GPIO configured in the constructor.
  */
  uint32_t getIrq() const { return(irqPin); }

  /*!
    \brief Access method to get the pin number of hardware reset pin.
    \returns Pin number of hardware reset pin configured in the constructor.
  */
  uint32_t getRst() const { return(rstPin); }

  /*!
    \brief Access method to get the pin number of second interrupt/GPIO.
    \returns Pin number of second interrupt/GPIO configured in the constructor.
  */
  uint32_t getGpio() const { return(gpioPin); }

  /*!
    \brief Some modules contain external RF switch controlled by pins.
    This function gives RadioLib control over those pins to
    automatically switch between various modes: When idle both pins
    will be LOW, during TX the `txEn` pin will be HIGH, during RX the
    `rxPin` will be HIGH.

    Radiolib will automatically set the pin mode and value of these
    pins, so do not control them from the sketch.

    When more than two pins or more control over the output values are
    needed, use the setRfSwitchTable() function.

    \param rxEn RX enable pin.
    \param txEn TX enable pin.
  */
  void setRfSwitchPins(uint32_t rxEn, uint32_t txEn);

  /*!
    \brief Some modules contain external RF switch controlled by pins.
    This function gives RadioLib control over those pins to
    automatically switch between various modes.

    Radiolib will automatically set the pin mode and value of these
    pins, so do not control them from the sketch.


    \param pins A reference to an array of pins to control. This
    should always be an array of 3 elements. If you need less pins,
    use RADIOLIB_NC for the unused elements.

    \param table A reference to an array of pin values to use for each
    supported mode. Each element is an RfSwitchMode_T struct that
    lists the mode for which it applies and the values for each of the
    pins passed in the pins argument respectively.

    The `pins` array will be copied into the Module object, so the
    original array can be deallocated after this call. However,
    a reference to the `table` array will be stored, so that array
    must remain valid as long RadioLib is being used.

    The `mode` field in each table row should normally use any of the
    `MODE_*` constants from the Module::OpMode_t enum. However, some
    radios support additional modes and will define their own OpMode_t
    enum.

    The length of the table is variable (to support radios that add
    additional modes), so the table must always be terminated with the
    special END_OF_MODE_TABLE value.

    Normally all modes should be listed in the table, but for some
    radios, modes can be omitted to indicate they are not supported
    (e.g. when a radio has a high power and low power TX mode but
    external circuitry only supports low power). If applicable, this
    is documented in the radio class itself.

    #### Example
    For example, on a board that has an RF switch with an enable pin
    connected to PA0 and a TX/RX select pin connected to PA1:

    \code
    // In global scope, define the pin array and mode table
    static const uint32_t rfswitch_pins[] =
                           {PA0,  PA1,  RADIOLIB_NC};
    static const Module::RfSwitchMode_t rfswitch_table[] = {
      {Module::MODE_IDLE,  {LOW,  LOW}},
      {Module::MODE_RX,    {HIGH, LOW}},
      {Module::MODE_TX,    {HIGH, HIGH}},
       Module::END_OF_MODE_TABLE,
    };

    void setup() {
      ...
      // Then somewhere in setup, pass them to radiolib
      radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
      ...
    }
    \endcode
  */

  void setRfSwitchTable(const uint32_t (&pins)[RFSWITCH_MAX_PINS], const RfSwitchMode_t table[]);

  /*!
    \brief Find a mode in the RfSwitchTable.
    \param The mode to find.
    \returns A pointer to the RfSwitchMode_t struct in the table that
    matches the passed mode. Returns nullptr if no rfswitch pins are
    configured, or the passed mode is not listed in the table.
  */
  const RfSwitchMode_t *findRfSwitchMode(uint8_t mode) const;

  /*!
    \brief Set RF switch state.
    \param mode The mode to set. This must be one of the MODE_ constants, or a radio-specific constant.
  */
  void setRfSwitchState(uint8_t mode);

  /*!
    \brief Wait for time to elapse, either using the microsecond timer, or the TimerFlag.
    Note that in interrupt timing mode, it is up to the user to set up the timing interrupt!

    \param start Waiting start timestamp, in microseconds.
    \param len Waiting duration, in microseconds;
  */
  void waitForMicroseconds(uint32_t start, uint32_t len);

  /*!
    \brief Function to reflect bits within a byte.
    \param in The input to reflect.
    \param bits Number of bits to reflect.
    \return The reflected input.
  */
  static uint32_t reflect(uint32_t in, uint8_t bits);

#if !defined(RADIOLIB_GODMODE)
  /*!
      \brief SPI initialization method.
    */
  void spiBegin();

private:
#endif
  uint32_t csPin = RADIOLIB_NC;
  uint32_t irqPin = RADIOLIB_NC;
  uint32_t rstPin = RADIOLIB_NC;
  uint32_t gpioPin = RADIOLIB_NC;

  // RF switch pins and table
  uint32_t rfSwitchPins[RFSWITCH_MAX_PINS] = { RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC };
  const RfSwitchMode_t *rfSwitchTable = nullptr;

  // from HAL
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
  void halInit();

  /*!
    \brief Module termination method.
    This will be called by all radio modules when the desctructor is called.
    Can be used to e.g., stop SPI interface.
  */
  void halTerm();

  void yield();


public:

  /*!
    \brief Default constructor.
    \param mod Instance of Module that will be used to communicate with the radio.
  */
  LLCC68();

//  Module* getMod() const;

  /*!
    \brief Whether the module has an XTAL (true) or TCXO (false). Defaults to false.
  */
  bool XTAL;

  // basic methods

  /*!
    \brief Initialization method for LoRa modem.
    \param cr LoRa coding rate denominator. Allowed values range from 5 to 8.
    \param syncWord 1-byte LoRa sync word.
    \param preambleLength LoRa preamble length in symbols. Allowed values range from 1 to 65535.
    \param tcxoVoltage TCXO reference voltage to be set on DIO3. Defaults to 1.6 V, set to 0 to skip.
    \param useRegulatorLDO Whether to use only LDO regulator (true) or DC-DC regulator (false). Defaults to false.
    \returns \ref status_codes
  */
  int16_t begin(uint8_t cr, uint8_t syncWord, uint16_t preambleLength, float tcxoVoltage, bool useRegulatorLDO = false);


  /*!
    \brief Initialization method for LoRa modem.
    \param freq Carrier frequency in MHz. Defaults to 434.0 MHz.
    \param bw LoRa bandwidth in kHz. Defaults to 125.0 kHz.
    \param sf LoRa spreading factor. Defaults to 9.
    \param cr LoRa coding rate denominator. Defaults to 7 (coding rate 4/7).
    \param syncWord 1-byte LoRa sync word. Defaults to RADIOLIB_SX126X_SYNC_WORD_PRIVATE (0x12).
    \param pwr Output power in dBm. Defaults to 10 dBm.
    \param preambleLength LoRa preamble length in symbols. Defaults to 8 symbols.
    \param tcxoVoltage TCXO reference voltage to be set on DIO3. Defaults to 1.6 V, set to 0 to skip.
    \param useRegulatorLDO Whether to use only LDO regulator (true) or DC-DC regulator (false). Defaults to false.
    \returns \ref status_codes
  */
  int16_t begin(float freq = 434.0, float bw = 125.0, uint8_t sf = 9, uint8_t cr = 7, uint8_t syncWord = RADIOLIB_SX126X_SYNC_WORD_PRIVATE, int8_t pwr = 10, uint16_t preambleLength = 8, float tcxoVoltage = 1.6, bool useRegulatorLDO = false);

  /*!
    \brief Reset method. Will reset the chip to the default state using RST pin.
    \param verify Whether correct module startup should be verified. When set to true, RadioLib will attempt to verify the module has started correctly
    by repeatedly issuing setStandby command. Enabled by default.
    \returns \ref status_codes
  */
  int16_t reset(bool verify = true);

  etl::optional<size_t> tryReceive(uint8_t* data);

  /*!
    \brief Blocking binary transmit method.
    Overloads for string-based transmissions are implemented in PhysicalLayer.
    \param data Binary data to be sent.
    \param len Number of bytes to send.
    \param addr Address to send the data to. Will only be added if address filtering was enabled.
    \returns \ref status_codes
  */
  int16_t transmit(uint8_t* data, size_t len, uint8_t addr = 0);

  /*!
    \brief Starts direct mode transmission.
    \param frf Raw RF frequency value. Defaults to 0, required for quick frequency shifts in RTTY.
    \returns \ref status_codes
  */
  int16_t transmitDirect(uint32_t frf = 0) ;

  /*!
    \brief Starts direct mode reception. Only implemented for PhysicalLayer compatibility, as %SX126x series does not support direct mode reception.
    Will always return RADIOLIB_ERR_UNKNOWN.
    \returns \ref status_codes
  */
  int16_t receiveDirect() ;

  /*!
    \brief Performs scan for LoRa transmission in the current channel. Detects both preamble and payload.
    \param symbolNum Number of symbols for CAD detection. Defaults to the value recommended by AN1200.48.
    \param detPeak Peak value for CAD detection. Defaults to the value recommended by AN1200.48.
    \param detMin Minimum value for CAD detection. Defaults to the value recommended by AN1200.48.
    \returns \ref status_codes
  */
  int16_t scanChannel(uint8_t symbolNum = RADIOLIB_SX126X_CAD_PARAM_DEFAULT, uint8_t detPeak = RADIOLIB_SX126X_CAD_PARAM_DEFAULT, uint8_t detMin = RADIOLIB_SX126X_CAD_PARAM_DEFAULT);

  /*!
    \brief Sets the module to sleep mode. To wake the device up, call standby().
    \param retainConfig Set to true to retain configuration of the currently active modem ("warm start")
    or to false to discard current configuration ("cold start"). Defaults to true.
    \returns \ref status_codes
  */
  int16_t sleep(bool retainConfig = true);

  /*!
    \brief Sets the module to standby mode (overload for PhysicalLayer compatibility, uses 13 MHz RC oscillator).
    \returns \ref status_codes
  */
  int16_t standby() ;

  /*!
    \brief Sets the module to standby mode.
    \param mode Oscillator to be used in standby mode. Can be set to RADIOLIB_SX126X_STANDBY_RC (13 MHz RC oscillator)
    or RADIOLIB_SX126X_STANDBY_XOSC (32 MHz external crystal oscillator).
    \param wakeup Whether to force the module to wake up. Setting to true will immediately attempt to wake up the module.
    \returns \ref status_codes
  */
  int16_t standby(uint8_t mode, bool wakeup = true);

  // interrupt methods
  // ISR is managed by my self

  /*!
    \brief Interrupt-driven binary transmit method.
    Overloads for string-based transmissions are implemented in PhysicalLayer.
    \param data Binary data to be sent.
    \param len Number of bytes to send.
    \param addr Address to send the data to. Will only be added if address filtering was enabled.
    \returns \ref status_codes
  */
  int16_t startTransmit(uint8_t* data, size_t len, uint8_t addr = 0) ;

  /*!
    \brief Clean up after transmission is done.
    \returns \ref status_codes
  */
  int16_t finishTransmit() ;

  /*!
    \brief Interrupt-driven receive method with default parameters.
    Implemented for compatibility with PhysicalLayer.

    \returns \ref status_codes
  */
  int16_t startReceive();

  /*!
    \brief Interrupt-driven receive method. DIO1 will be activated when full packet is received.
    \param timeout Receive mode type and/or raw timeout value, expressed as multiples of 15.625 us.
    When set to RADIOLIB_SX126X_RX_TIMEOUT_INF, the timeout will be infinite and the device will remain
    in Rx mode until explicitly commanded to stop (Rx continuous mode).
    When set to RADIOLIB_SX126X_RX_TIMEOUT_NONE, there will be no timeout and the device will return
    to standby when a packet is received (Rx single mode).
    For any other value, timeout will be applied and signal will be generated on DIO1 for conditions
    defined by irqFlags and irqMask.

    \param irqFlags Sets the IRQ flags, defaults to RADIOLIB_SX126X_IRQ_RX_DEFAULT.
    \param irqMask Sets the mask of IRQ flags that will trigger DIO1, defaults to RADIOLIB_SX126X_IRQ_RX_DONE.
    \param len Only for PhysicalLayer compatibility, not used.
    \returns \ref status_codes
  */
  int16_t startReceive(uint32_t timeout);

  /*!
    \brief Interrupt-driven receive method where the device mostly sleeps and periodically wakes to listen.
    Note that this function assumes the unit will take 500us + TCXO_delay to change state.
    See datasheet section 13.1.7, version 1.2.
    \param rxPeriod The duration the receiver will be in Rx mode, in microseconds.
    \param sleepPeriod The duration the receiver will not be in Rx mode, in microseconds.
    \param irqFlags Sets the IRQ flags, defaults to RADIOLIB_SX126X_IRQ_RX_DEFAULT.
    \param irqMask Sets the mask of IRQ flags that will trigger DIO1, defaults to RADIOLIB_SX126X_IRQ_RX_DONE.
    \returns \ref status_codes
  */
  int16_t startReceiveDutyCycle(uint32_t rxPeriod, uint32_t sleepPeriod, uint16_t irqFlags = RADIOLIB_SX126X_IRQ_RX_DEFAULT, uint16_t irqMask = RADIOLIB_SX126X_IRQ_RX_DONE);

  /*!
    \brief Calls \ref startReceiveDutyCycle with rxPeriod and sleepPeriod set so the unit shouldn't miss any messages.
    \param senderPreambleLength Expected preamble length of the messages to receive.
    If set to zero, the currently configured preamble length will be used. Defaults to zero.

    \param minSymbols Parameters will be chosen to ensure that the unit will catch at least this many symbols
    of any preamble of the specified length. Defaults to 8.
    According to Semtech, receiver requires 8 symbols to reliably latch a preamble.
    This makes this method redundant when transmitter preamble length is less than 17 (2*minSymbols + 1).

    \param irqFlags Sets the IRQ flags, defaults to RADIOLIB_SX126X_IRQ_RX_DEFAULT.
    \param irqMask Sets the mask of IRQ flags that will trigger DIO1, defaults to RADIOLIB_SX126X_IRQ_RX_DONE.
    \returns \ref status_codes
  */
  int16_t startReceiveDutyCycleAuto(uint16_t senderPreambleLength = 0, uint16_t minSymbols = 8, uint16_t irqFlags = RADIOLIB_SX126X_IRQ_RX_DEFAULT, uint16_t irqMask = RADIOLIB_SX126X_IRQ_RX_DONE);

  /*!
    \brief Reads the current IRQ status.
    \returns IRQ status bits
  */
  uint16_t getIrqStatus();

  /*!
    \brief Reads data received after calling startReceive method.
    \param data Pointer to array to save the received binary data.
    \param len Number of bytes that will be read. When set to 0, the packet length will be retreived automatically.
    When more bytes than received are requested, only the number of bytes requested will be returned.
    \returns \ref status_codes
  */
  int16_t readData(uint8_t* data, size_t len) ;

  /*!
    \brief Interrupt-driven channel activity detection method. DIO0 will be activated
    when LoRa preamble is detected, or upon timeout.
    \param symbolNum Number of symbols for CAD detection. Defaults to the value recommended by AN1200.48.
    \param detPeak Peak value for CAD detection. Defaults to the value recommended by AN1200.48.
    \param detMin Minimum value for CAD detection. Defaults to the value recommended by AN1200.48.
    \returns \ref status_codes
  */
  int16_t startChannelScan(uint8_t symbolNum = RADIOLIB_SX126X_CAD_PARAM_DEFAULT, uint8_t detPeak = RADIOLIB_SX126X_CAD_PARAM_DEFAULT, uint8_t detMin = RADIOLIB_SX126X_CAD_PARAM_DEFAULT);

  /*!
    \brief Read the channel scan result
    \returns \ref status_codes
  */
  int16_t getChannelScanResult();

  // configuration methods

  /*!
    \brief Sets LoRa bandwidth. Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
    \param bw LoRa bandwidth to be set in kHz.
    \returns \ref status_codes
  */
  int16_t setBandwidth(float bw);

  /*!
    \brief Sets LoRa spreading factor. Allowed values range from 5 to 12.
    \param sf LoRa spreading factor to be set.
    \returns \ref status_codes
  */
  int16_t setSpreadingFactor(uint8_t sf);

  /*!
    \brief Sets LoRa coding rate denominator. Allowed values range from 5 to 8.
    \param cr LoRa coding rate denominator to be set.
    \returns \ref status_codes
  */
  int16_t setCodingRate(uint8_t cr);

  /*!
    \brief Sets LoRa sync word.
    \param syncWord LoRa sync word to be set.
    \param controlBits Undocumented control bits, required for compatibility purposes.
    \returns \ref status_codes
  */
  int16_t setSyncWord(uint8_t syncWord, uint8_t controlBits = 0x44);

  /*!
    \brief Sets current protection limit. Can be set in 2.5 mA steps.
    \param currentLimit current protection limit to be set in mA. Allowed values range from 0 to 140.
    \returns \ref status_codes
  */
  int16_t setCurrentLimit(float currentLimit);

  /*!
    \brief Reads current protection limit.
    \returns Currently configured overcurrent protection limit in mA.
  */
  float getCurrentLimit();

  /*!
    \brief Sets carrier frequency. Allowed values are in range from 150.0 to 960.0 MHz.
    \param freq Carrier frequency to be set in MHz.
    \returns \ref status_codes
  */
  int16_t setFrequency(float freq);

  /*!
    \brief Sets carrier frequency. Allowed values are in range from 150.0 to 960.0 MHz.
    \param freq Carrier frequency to be set in MHz.
    \param calibrate Run image calibration.
    \returns \ref status_codes
  */
  int16_t setFrequency(float freq, bool calibrate);

  /*!
    \brief Sets preamble length for LoRa or FSK modem. Allowed values range from 1 to 65535.
    \param preambleLength Preamble length to be set in symbols (LoRa) or bits (FSK).
    \returns \ref status_codes
  */
  int16_t setPreambleLength(uint16_t preambleLength);

  /*!
    \brief Sets FSK frequency deviation. Allowed values range from 0.0 to 200.0 kHz.
    \param freqDev FSK frequency deviation to be set in kHz.
    \returns \ref status_codes
  */
  int16_t setFrequencyDeviation(float freqDev) ;

  /*!
    \brief Sets FSK bit rate. Allowed values range from 0.6 to 300.0 kbps.
    \param br FSK bit rate to be set in kbps.
    \returns \ref status_codes
  */
  int16_t setBitRate(float br);

  /*!
    \brief Sets FSK receiver bandwidth. Allowed values are 4.8, 5.8, 7.3, 9.7, 11.7, 14.6, 19.5,
    23.4, 29.3, 39.0, 46.9, 58.6, 78.2, 93.8, 117.3, 156.2, 187.2, 234.3, 312.0, 373.6 and 467.0 kHz.
    \param rxBw FSK receiver bandwidth to be set in kHz.
    \returns \ref status_codes
  */
  int16_t setRxBandwidth(float rxBw);

  /*!
    \brief Enables or disables Rx Boosted Gain mode as described in SX126x datasheet
    section 9.6 (SX1261/2 v2.1, SX1268 v1.1)
    \param rxbgm True for Rx Boosted Gain, false for Rx Power Saving Gain
    \param persist True to persist Rx gain setting when waking up from warm-start mode
    (e.g. when using Rx duty cycle mode).
    \returns \ref status_codes
  */
  int16_t setRxBoostedGainMode(bool rxbgm, bool persist = true);

  /*!
    \brief Sets time-bandwidth product of Gaussian filter applied for shaping.
    Allowed values are RADIOLIB_SHAPING_0_3, RADIOLIB_SHAPING_0_5, RADIOLIB_SHAPING_0_7 or RADIOLIB_SHAPING_1_0.
    Set to RADIOLIB_SHAPING_NONE to disable data shaping.
    \param sh Time-bandwidth product of Gaussian filter to be set.
    \returns \ref status_codes
  */
  int16_t setDataShaping(uint8_t sh) ;

  /*!
    \brief Sets FSK sync word in the form of array of up to 8 bytes.
    \param syncWord FSK sync word to be set.
    \param len FSK sync word length in bytes.
    \returns \ref status_codes
  */
  int16_t setSyncWord(uint8_t* syncWord, uint8_t len);

  /*!
    \brief Sets FSK sync word in the form of array of up to 8 bytes.
    \param syncWord FSK sync word to be set.
    \param bitsLen FSK sync word length in bits. If length is not divisible by 8,
    least significant bits of syncWord will be ignored.
    \returns \ref status_codes
  */
  int16_t setSyncBits(uint8_t *syncWord, uint8_t bitsLen);

  /*!
    \brief Sets node address. Calling this method will also enable address filtering for node address only.
    \param nodeAddr Node address to be set.
    \returns \ref status_codes
  */
  int16_t setNodeAddress(uint8_t nodeAddr);

  /*!
    \brief Sets broadcast address. Calling this method will also enable address
    filtering for node and broadcast address.
    \param broadAddr Node address to be set.
    \returns \ref status_codes
  */
  int16_t setBroadcastAddress(uint8_t broadAddr);

  /*!
    \brief Disables address filtering. Calling this method will also erase previously set addresses.
    \returns \ref status_codes
  */
  int16_t disableAddressFiltering();

  /*!
    \brief Sets CRC configuration.
    \param len CRC length in bytes, Allowed values are 1 or 2, set to 0 to disable CRC.
    \param initial Initial CRC value. FSK only. Defaults to 0x1D0F (CCIT CRC).
    \param polynomial Polynomial for CRC calculation. FSK only. Defaults to 0x1021 (CCIT CRC).
    \param inverted Invert CRC bytes. FSK only. Defaults to true (CCIT CRC).
    \returns \ref status_codes
  */
  int16_t setCRC(uint8_t len, uint16_t initial = 0x1D0F, uint16_t polynomial = 0x1021, bool inverted = true);

  /*!
    \brief Sets FSK whitening parameters.
    \param enabled True = Whitening enabled
    \param initial Initial value used for the whitening LFSR in FSK mode. Defaults to 0x0100,
    use 0x01FF for SX127x compatibility.
    \returns \ref status_codes
  */
  int16_t setWhitening(bool enabled, uint16_t initial = 0x0100);

  /*!
    \brief Sets TCXO (Temperature Compensated Crystal Oscillator) configuration.
    \param voltage TCXO reference voltage in volts. Allowed values are 1.6, 1.7, 1.8, 2.2. 2.4, 2.7, 3.0 and 3.3 V.
    Set to 0 to disable TCXO.
    NOTE: After setting this parameter to 0, the module will be reset (since there's no other way to disable TCXO).

    \param delay TCXO timeout in us. Defaults to 5000 us.
    \returns \ref status_codes
  */
  int16_t setTCXO(float voltage, uint32_t delay = 5000);

  /*!
    \brief Set DIO2 to function as RF switch (default in Semtech example designs).
    \returns \ref status_codes
  */
  int16_t setDio2AsRfSwitch(bool enable = true);

  /*!
    \brief Gets effective data rate for the last transmitted packet. The value is calculated only for payload bytes.
    \returns Effective data rate in bps.
  */
  float getDataRate() const;

  /*!
    \brief GetsRSSI (Recorded Signal Strength Indicator).
    \param packet Whether to read last packet RSSI, or the current value.
    \returns RSSI value in dBm.
  */
  float getRSSI(bool packet = true);

  /*!
    \brief Gets SNR (Signal to Noise Ratio) of the last received packet. Only available for LoRa modem.
    \returns SNR of the last received packet in dB.
  */
  float getSNR();

  /*!
    \brief Gets frequency error of the latest received packet.
    WARNING: This functionality is based on SX128x implementation and not documented on SX126x.
    While it seems to be working, it should be used with caution!

    \returns Frequency error in Hz.
  */
  float getFrequencyError();

  /*!
    \brief Query modem for the packet length of received payload.
    \param update Update received packet length. Will return cached value when set to false.
    \returns Length of last received packet in bytes.
  */
  size_t getPacketLength(bool update = true) ;

  /*!
    \brief Set modem in fixed packet length mode. Available in FSK mode only.
    \param len Packet length.
    \returns \ref status_codes
  */
  int16_t fixedPacketLengthMode(uint8_t len = RADIOLIB_SX126X_MAX_PACKET_LENGTH);

  /*!
    \brief Set modem in variable packet length mode. Available in FSK mode only.
    \param len Maximum packet length.
    \returns \ref status_codes
  */
  int16_t variablePacketLengthMode(uint8_t maxLen = RADIOLIB_SX126X_MAX_PACKET_LENGTH);

  /*!
    \brief Get expected time-on-air for a given size of payload
    \param len Payload length in bytes.
    \returns Expected time-on-air in microseconds.
  */
  uint32_t getTimeOnAir(size_t len);

  /*!
    \brief Set implicit header mode for future reception/transmission.
    \param len Payload length in bytes.
    \returns \ref status_codes
  */
  int16_t implicitHeader(size_t len);

  /*!
    \brief Set explicit header mode for future reception/transmission.
    \returns \ref status_codes
  */
  int16_t explicitHeader();

  /*!
    \brief Set regulator mode to LDO.
    \returns \ref status_codes
  */
  int16_t setRegulatorLDO();

  /*!
    \brief Set regulator mode to DC-DC.
    \returns \ref status_codes
  */
  int16_t setRegulatorDCDC();

  /*!
    \brief Sets transmission encoding. Available in FSK mode only. Serves only as alias for PhysicalLayer compatibility.
    \param encoding Encoding to be used. Set to 0 for NRZ, and 2 for whitening.
    \returns \ref status_codes
  */
  int16_t setEncoding(uint8_t encoding) ;

//  /*! \copydoc Module::setRfSwitchPins */
//  void setRfSwitchPins(uint32_t rxEn, uint32_t txEn);
//
//  /*! \copydoc Module::setRfSwitchTable */
//  void setRfSwitchTable(const uint32_t (&pins)[RFSWITCH_MAX_PINS], const RfSwitchMode_t table[]);

  /*!
    \brief Forces LoRa low data rate optimization. Only available in LoRa mode. After calling this method,
    LDRO will always be set to the provided value, regardless of symbol length.
    To re-enable automatic LDRO configuration, call SX126x::autoLDRO()

    \param enable Force LDRO to be always enabled (true) or disabled (false).
    \returns \ref status_codes
  */
  int16_t forceLDRO(bool enable);

  /*!
    \brief Re-enables automatic LDRO configuration. Only available in LoRa mode.
    After calling this method, LDRO will be enabled automatically when symbol length exceeds 16 ms.

    \returns \ref status_codes
  */
  int16_t autoLDRO();

  /*!
    \brief Get one truly random byte from RSSI noise.
    \returns TRNG byte.
  */
  uint8_t randomByte();

  /*!
    \brief Enable/disable inversion of the I and Q signals
    \param enable QI inversion enabled (true) or disabled (false);
    \returns \ref status_codes
  */
  int16_t invertIQ(bool enable);

  // What's exclude direct receive?
#if !defined(RADIOLIB_EXCLUDE_DIRECT_RECEIVE)
//  /*!
//    \brief Set interrupt service routine function to call when data bit is receveid in direct mode.
//    \param func Pointer to interrupt service routine.
//  */
//  void setDirectAction(void (*func)(void));
//
//  /*!
//    \brief Function to read and process data bit in direct reception mode.
//    \param pin Pin on which to read.
//  */
//  void readBit(uint32_t pin);
#endif

  /*!
    \brief Upload binary patch into the SX126x device RAM.
    Patch is needed to e.g., enable spectral scan and must be uploaded again on every power cycle.
    \param patch Binary patch to upload.
    \param len Length of the binary patch in 4-byte words.
    \param nonvolatile Set to true when the patch is saved in non-volatile memory of the host processor,
    or to false when the patch is in its RAM.
    \returns \ref status_codes
  */
  int16_t uploadPatch(const uint32_t* patch, size_t len, bool nonvolatile = true);

  /*!
    \brief Start spectral scan. Requires binary path to be uploaded.
    \param numSamples Number of samples for each scan. Fewer samples = better temporal resolution.
    \param window RSSI averaging window size.
    \param interval Scan interval length, one of RADIOLIB_SX126X_SCAN_INTERVAL_* macros.
    \returns \ref status_codes
  */
  int16_t spectralScanStart(uint16_t numSamples, uint8_t window = RADIOLIB_SX126X_SPECTRAL_SCAN_WINDOW_DEFAULT, uint8_t interval = RADIOLIB_SX126X_SCAN_INTERVAL_8_20_US);

  /*!
    \brief Abort an ongoing spectral scan.
  */
  void spectralScanAbort();

  /*!
    \brief Read the status of spectral scan.
    \returns \ref status_codes
  */
  int16_t spectralScanGetStatus();

  /*!
    \brief Read the result of spectral scan.
    \param results Array to which the results will be saved, must be RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE long.
    \returns \ref status_codes
  */
  int16_t spectralScanGetResult(uint16_t* results);


  /*!
    \brief Sets output power. Allowed values are in range from -9 to 22 dBm.
    This method is virtual to allow override from the SX1261 class.
    \param power Output power to be set in dBm.
    \returns \ref status_codes
  */
  int16_t setOutputPower(int8_t power);

  /**
   * @brief The IrqMask masks or unmasks the IRQ which can be triggered by the device. By default, all IRQ are masked (all ‘0’) and the user can enable them one by one (or several at a time) by setting the corresponding mask to ‘1’.
   */
  int16_t setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask = RADIOLIB_SX126X_IRQ_NONE, uint16_t dio3Mask = RADIOLIB_SX126X_IRQ_NONE);

  /**
   * @brief `startReceive` but infinite timeout
   */
  void rx();

#if !defined(RADIOLIB_GODMODE)
protected:
#endif
  // SX126x SPI command implementations
  int16_t setFs();
  int16_t setTx(uint32_t timeout = 0);
  int16_t setRx(uint32_t timeout);
  int16_t setCad(uint8_t symbolNum, uint8_t detPeak, uint8_t detMin);
  int16_t setPaConfig(uint8_t paDutyCycle, uint8_t deviceSel, uint8_t hpMax = RADIOLIB_SX126X_PA_CONFIG_HP_MAX, uint8_t paLut = RADIOLIB_SX126X_PA_CONFIG_PA_LUT);
  int16_t writeRegister(uint16_t addr, uint8_t* data, uint8_t numBytes);
  int16_t readRegister(uint16_t addr, uint8_t* data, uint8_t numBytes);
  int16_t writeBuffer(uint8_t* data, uint8_t numBytes, uint8_t offset = 0x00);
  int16_t readBuffer(uint8_t* data, uint8_t numBytes, uint8_t offset = 0x00);
  int16_t clearIrqStatus(uint16_t clearIrqParams = RADIOLIB_SX126X_IRQ_ALL);
  int16_t setRfFrequency(uint32_t frf);
  int16_t calibrateImage(uint8_t* data);
  uint8_t getPacketType();
  int16_t setTxParams(uint8_t power, uint8_t rampTime = RADIOLIB_SX126X_PA_RAMP_200U);
  int16_t setModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro);
  int16_t setModulationParamsFSK(uint32_t br, uint8_t sh, uint8_t rxBw, uint32_t freqDev);
  int16_t setPacketParams(uint16_t preambleLen, uint8_t crcType, uint8_t payloadLen, uint8_t hdrType, uint8_t invertIQ);
  int16_t setPacketParamsFSK(uint16_t preambleLen, uint8_t crcType, uint8_t syncWordLen, uint8_t addrCmp, uint8_t whiten, uint8_t packType = RADIOLIB_SX126X_GFSK_PACKET_VARIABLE, uint8_t payloadLen = 0xFF, uint8_t preambleDetectorLen = RADIOLIB_SX126X_GFSK_PREAMBLE_DETECT_16);
  int16_t setBufferBaseAddress(uint8_t txBaseAddress = 0x00, uint8_t rxBaseAddress = 0x00);
  int16_t setRegulatorMode(uint8_t mode);
  uint8_t getStatus();
  uint32_t getPacketStatus();
  uint16_t getDeviceErrors();
  int16_t clearDeviceErrors();

  int16_t beforeStartReceive(uint32_t timeout);
  int16_t setFrequencyRaw(float freq);
  int16_t setPacketMode(uint8_t mode, uint8_t len);
  int16_t setHeaderType(uint8_t hdrType, size_t len = 0xFF);
  int16_t directMode();
  int16_t packetMode();

  // fixes to errata
  int16_t fixSensitivity();
  int16_t fixPaClamping(bool enable = true);
  int16_t fixImplicitTimeout();
  int16_t fixInvertedIQ(uint8_t iqConfig);

#if !defined(RADIOLIB_GODMODE) && !defined(RADIOLIB_LOW_LEVEL)
protected:
#endif
  // Module* mod;

  // common low-level SPI interface
  static int16_t SPIparseStatus(uint8_t in);

#if !defined(RADIOLIB_GODMODE)
protected:
#endif

  uint8_t bandwidth = 0, spreadingFactor = 0, codingRate = 0, ldrOptimize = 0, crcTypeLoRa = 0, headerType = 0;
  uint16_t preambleLengthLoRa = 0;
  fixed_16_16 bandwidthKhz = fixed_16_16 {0};
  bool ldroAuto = true;

  uint32_t bitRate = 0, frequencyDev = 0;
  uint8_t rxBandwidth = 0, pulseShape = 0, crcTypeFSK = 0, syncWordLength = 0, addrComp = 0, whitening = 0, packetType = 0;
  uint16_t preambleLengthFSK = 0;
  float rxBandwidthKhz = 0;

  float dataRateMeasured = 0;

  uint32_t tcxoDelay = 0;

  size_t implicitLen = 0;
  uint8_t invertIQEnabled = RADIOLIB_SX126X_LORA_IQ_STANDARD;
  const char* chipType;

  // Allow subclasses to define different TX modes
  uint8_t txMode = MODE_TX;

  int16_t config(uint8_t modem);
  bool findChip(const char* verStr);

};

#endif // SIMPLE_LLCC68_H
