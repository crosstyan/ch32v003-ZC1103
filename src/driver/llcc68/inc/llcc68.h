//
// Created by Kurosu Chan on 2023/6/20.
//

#ifndef SIMPLE_LLCC68_H
#define SIMPLE_LLCC68_H

#include "spi.h"
#include "clock.h"
#include "ch32v003fun.h"
#include "ch32v003_SPI.h"
#include "printf.h"
#include "gpio.h"
#include "unit.h"
#include <etl/optional.h>
#include <etl/vector.h>
#include <etl/delegate.h>
#include "radio_sx126x.h"
#include "TypeDef.h"

// basically LLCC68 is a SX126x
// LLCC68芯片引脚兼容SX1262,且在设计、驱动代码及应用上与SX1262完全相同


class LLCC68 {
  bool _is_initialized = false;

  /// 芯片复位脚，低电平有效，复位后寄存器数值丢失，全部变为默认值。
  pin_size_t RST_PIN = GPIO::C0;
  /// CS/NSS 低电平选中
  pin_size_t CS_PIN = GPIO::C4;
  /// 高电平表示忙
  pin_size_t BUSY_PIN = GPIO::C3;
  pin_size_t TXEN_PIN = GPIO::C1;
  pin_size_t RXEN_PIN = GPIO::C2;

  void gpioConfigure();

  void spiConfigure();

  /**
 * \brief  设置PA增益
 * \param [IN]  gain 增益
 * \retval  None
   */
//  void setPA(PowerAmpGain gain);

  void RST_LOW();

  void RST_HIGH();

  void CS_HIGH();

  void CS_LOW();


  /**
 * \brief  通过spi传输一个字节
 * \param  [IN] byte 发送的字节
 * \retval  接收的字节
   */
  uint8_t sendByte(uint8_t byte);

  /// when you want to send a bunch of bytes and don't care about the return value
  void sendBytes(const uint8_t *bytes, size_t len);

  void sendBytes(const char *bytes, size_t len);

  /**
 * \brief  写 RF 寄存器
 * \param[IN] addr 寄存器地址 取值0x00 - 0x7F
 * \param[IN] val  写入的值
   */
  void write(uint8_t addr, uint8_t val);

  /**
 * \brief  读 RF 寄存器
 * \param[IN] addr 寄存器地址 取值0x00 - 0x7F
   */
  uint8_t read(uint8_t addr);

protected:
  /// ch32v003 only has one SPI so there's no need to specify the SPI bus
  /// Have to set to default constructor or the linker will complain
  LLCC68() = default;
  ~LLCC68() = default;

public:
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
    \brief Initialization method for FSK modem.
    \param br FSK bit rate in kbps. Allowed values range from 0.6 to 300.0 kbps.
    \param freqDev Frequency deviation from carrier frequency in kHz. Allowed values range from 0.0 to 200.0 kHz.
    \param rxBw Receiver bandwidth in kHz. Allowed values are 4.8, 5.8, 7.3, 9.7, 11.7, 14.6, 19.5, 23.4, 29.3, 39.0,
    46.9, 58.6, 78.2, 93.8, 117.3, 156.2, 187.2, 234.3, 312.0, 373.6 and 467.0 kHz.
    \param preambleLength FSK preamble length in bits. Allowed values range from 0 to 65535.
    \param tcxoVoltage TCXO reference voltage to be set on DIO3. Defaults to 1.6 V, set to 0 to skip.
    \param useRegulatorLDO Whether to use only LDO regulator (true) or DC-DC regulator (false). Defaults to false.
    \returns \ref status_codes
  */
  int16_t beginFSK(float br, float freqDev, float rxBw, uint16_t preambleLength, float tcxoVoltage, bool useRegulatorLDO = false);

  /*!
    \brief Reset method. Will reset the chip to the default state using RST pin.
    \param verify Whether correct module startup should be verified. When set to true, RadioLib will attempt to verify the module has started correctly
    by repeatedly issuing setStandby command. Enabled by default.
    \returns \ref status_codes
  */
  int16_t reset(bool verify = true);

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
    \brief Blocking binary receive method.
    Overloads for string-based transmissions are implemented in PhysicalLayer.
    \param data Binary data to be sent.
    \param len Number of bytes to send.
    \returns \ref status_codes
  */
  int16_t receive(uint8_t* data, size_t len);

  /*!
    \brief Starts direct mode transmission.
    \param frf Raw RF frequency value. Defaults to 0, required for quick frequency shifts in RTTY.
    \returns \ref status_codes
  */
  int16_t transmitDirect(uint32_t frf = 0);

  /*!
    \brief Starts direct mode reception. Only implemented for PhysicalLayer compatibility, as %SX126x series does not support direct mode reception.
    Will always return RADIOLIB_ERR_UNKNOWN.
    \returns \ref status_codes
  */
  int16_t receiveDirect();

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
  int16_t standby();

  /*!
    \brief Sets the module to standby mode.
    \param mode Oscillator to be used in standby mode. Can be set to RADIOLIB_SX126X_STANDBY_RC (13 MHz RC oscillator)
    or RADIOLIB_SX126X_STANDBY_XOSC (32 MHz external crystal oscillator).
    \param wakeup Whether to force the module to wake up. Setting to true will immediately attempt to wake up the module.
    \returns \ref status_codes
  */
  int16_t standby(uint8_t mode, bool wakeup = true);

  // interrupt methods

  /*!
    \brief Sets interrupt service routine to call when DIO1 activates.
    \param func ISR to call.
  */
  void setDio1Action(void (*func)(void));

  /*!
    \brief Clears interrupt service routine to call when DIO1 activates.
  */
  void clearDio1Action();

  /*!
    \brief Interrupt-driven binary transmit method.
    Overloads for string-based transmissions are implemented in PhysicalLayer.
    \param data Binary data to be sent.
    \param len Number of bytes to send.
    \param addr Address to send the data to. Will only be added if address filtering was enabled.
    \returns \ref status_codes
  */
  int16_t startTransmit(uint8_t* data, size_t len, uint8_t addr = 0);

  /*!
    \brief Clean up after transmission is done.
    \returns \ref status_codes
  */
  int16_t finishTransmit();

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
  int16_t startReceive(uint32_t timeout, uint16_t irqFlags = RADIOLIB_SX126X_IRQ_RX_DEFAULT, uint16_t irqMask = RADIOLIB_SX126X_IRQ_RX_DONE, size_t len = 0);

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
  int16_t readData(uint8_t* data, size_t len);

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
  int16_t setFrequencyDeviation(float freqDev);

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
  int16_t setDataShaping(uint8_t sh);

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
  size_t getPacketLength(bool update = true);

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

  /*! \copydoc Module::setRfSwitchPins */
  void setRfSwitchPins(uint32_t rxEn, uint32_t txEn);

  /*! \copydoc Module::setRfSwitchTable */
  // void setRfSwitchTable(const uint32_t (&pins)[Module::RFSWITCH_MAX_PINS], const Module::RfSwitchMode_t table[]);

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

#if !defined(RADIOLIB_EXCLUDE_DIRECT_RECEIVE)
  /*!
    \brief Set interrupt service routine function to call when data bit is receveid in direct mode.
    \param func Pointer to interrupt service routine.
  */
  void setDirectAction(void (*func)(void));

  /*!
    \brief Function to read and process data bit in direct reception mode.
    \param pin Pin on which to read.
  */
  void readBit(uint32_t pin);
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

  void reset();

};

#endif // SIMPLE_LLCC68_H
