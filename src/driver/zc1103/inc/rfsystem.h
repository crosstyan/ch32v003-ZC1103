
#ifndef _BSP_RFSYSTEM_H
#define _BSP_RFSYSTEM_H

#include "spi.h"
#include <cmath>
#include <cstring>
#include <cstdint>
#include "printf.h"
#include "clock.h"
#include "ch32v003fun.h"
#include "ch32v003_SPI.h"
#include "gpio.h"
#include "unit.h"
#include <etl/optional.h>
#include <etl/vector.h>
#include <etl/delegate.h>

/// 0x46 (8'h46)
struct RfStatus {
  bool idle = false;
  bool tx = false;
  bool rx = false;
  bool fs = false;
  bool scan = false;
  bool rc_cal = false;
  bool vco_cal = false;
  bool wor = false;
};

/// 0x40 (8'h40)
struct RfState {
  bool sync_word_rev = false;
  bool preamble_rev = false;
  bool crc_error = false;
  bool pkt_flag = false;
  bool fifo_flag = false;
  uint8_t rx_pkt_state = 0;
};

/*
 * +------------+-------------+
 * |            |             |
 * |  Preamble  |  Sync Word  |
 * |            |             |
 * +------------+-------------+
 *   10101...10    16/32bit
 */

namespace RF {
  const uint8_t NO_PACKET_RECEIVED = 0x03;
  // set the global rx flag
  void setRxFlag(bool flag);
  // get the global rx flag
  bool rxFlag();
  enum class DataRate{
    K2_4,
    K5,
    K9_6,
    K10,
    K19_2,
    K100,
    K200,
    K250,
  };
  void printStatus(const RfStatus &status);
  void printState(const RfState &state);
}

enum class PowerAmpGain {
  DBM20 = 0,
  DBM19,
  DBM18,
  DBM17,
  DBM16,
  DBM15,
  DBM14,
  DBM13,
  DBM12,
  DBM11,
  DBM10,
  DBM9,
  DBM8,
  DBM7,
  DBM6,
  DBM5,
  DBM4,
  DBM3,
  DBM2,
  DBM1,
  DBM_0,
  DBM_1,
  DBM_2,
  DBM_3,
  DBM_4,
  DBM_5,
  DBM_6,
};

class RfSystem {
  bool _is_initialized = false;

  /// 芯片复位脚，低电平有效，复位后寄存器数值丢失，全部变为默认值。
  pin_size_t RST_PIN = GPIO::C0;
  /// 使能信号，低有效，拉低可使芯片退出 sleep mode
  pin_size_t CS_PIN = GPIO::C4;
  /// IRQ
  pin_size_t PKT_FLAG_PIN = GPIO::C3;
  /// 芯片关断使能，高有效
  pin_size_t SDN_PIN = GPIO::C2;

  void gpioConfigure();

  void spiConfigure();

  void setDR(RF::DataRate data_rate);

/**
  * \brief  读数据
  * \param [OUT] dst 保存数据地址
  * \param [IN] len 读取长度
  */
  void readFifo(uint8_t *dst, uint8_t len);

/**
* \brief  发送单音载波
  */
  void txCW();

  void idle();

/**
 * \brief  设置PA增益
 * \param [IN]  gain 增益
 * \retval  None
 */
  void setPA(PowerAmpGain gain);

  void RST_LOW();

  void RST_HIGH();

  void SDN_LOW();

  void SDN_HIGH();

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
  * \brief  使能接收到同步字后锁定rssi
  */
  void setSyncLockRssi();

  void setVcoFreq(double freq);

  void setFreq(uint8_t N);

  void setFreqStep(double step);


/**
  * \brief  发送数据 (without size written in the first byte)
  * \param [IN] src
  * \param [IN] len
  */
  void writeFifo(const char *src, uint8_t len);

/**
  * \brief  发送数据
  * \param [IN] src 待发送数据 (单字节)
  */
  void writeFifo(char src);

  /**
   * @brief write src to FIFO with size written in the first byte
   * @param [IN] src
   * @param len
   */
  void writeFifoWithSize(const char *src, uint8_t len);

/**
 * \brief  初始化 rf 寄存器
 */
  void registerConfigure();

  void setFreq(double f0, uint8_t N, double step);

  /**
   * @param s1 r(0x11)
   * @param s2 r(0x12)
   * @param s3 r(0x13)
   * @param s4 r(0x14)
   * @see r(0x06)
   */
  void setSync(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4);
  /**
   * @param s1 r(0x11)
   * @param s2 r(0x12)
   * @see r(0x06)
   */
  void setSync(uint8_t s1, uint8_t s2);

  // 芯片包含一个集成唤醒定时器，可用来定期从 standby 状态唤醒芯片。
  // 计数器可配置一个最大值 wortimer_set 和一个中间值 wor_rxtimer_set，
  // 这样可以方便的产生一个类似 PWM 信号输出。
  // 当接收到一个有效的数据包后，芯片会退出自动唤醒状态，同时给出 PKT_FLAG 标志通知 MCU 处理数据。
  // Wortimer_set 设置的整个的一个周期时间，
  void setWorTimer(uint16_t t);
  // wor_rxtimer_set 设置唤醒后接收的时间。
  void setWorRxTimer(uint16_t t);

protected:
  /// ch32v003 only has one SPI so there's no need to specify the SPI bus
  /// Have to set to default constructor or the linker will complain
  RfSystem() = default;
  ~RfSystem() = default;

public:
  /**
   * @effect reset chip. set GPIO Pins register. set SPI register. Transfer register init data to chip and set the `_is_initialized` flag.
   * @note allowed to call multiple times as long as the GPIO/SPI pins are not changed.
   */
  void begin();

  void reset();

/**
  * \brief  发送数据包
  * \param [IN] buffer 发送数据
  * \param [IN] size   发送数数据长度
  * \retval optional<Unit> success or failure (timeout)
  */
  etl::optional<Unit>
  send(const char *buffer, uint8_t size, bool check_tx = false);

/**
  * @brief  retrieve data from FIFO. Works with C style pointer.
  * @param [OUT]buf buffer pointer
  * @return the length of the data received, if any
  */
  etl::optional<size_t> recv(char *buf);

/**
  * @brief  retrieve data from FIFO. Works with ETL vector.
  * @param [OUT]buf ETL vector
  * @return the length of the data received, if any
  */
  etl::optional<size_t> RfSystem::recv(etl::ivector<char>& buf);

/**
  * @brief  retrieve data from FIFO. The resize function is useful for C++ STL like container.
  * @param [OUT]buf C style pointer. could be `buf.data()` or `&buf[0]` or `buf.begin()`.
  * @param resize  a lambda to resize the buffer. Capture the container by reference.
  * @return the length of the data received, if any
  */
  etl::optional<size_t> recv(char *buf, etl::delegate<void(size_t)> resize);

/**
 * \brief  设置频率
 * \param [IN]  freq 频率值
 */
  void setRefFreq(double freq);

  RfState pollState();

  uint8_t version();

  /// frequency synthesizer 频率合成器/频综
  ///
  /// 让频综打开后保持在这个状态，在频综保持状态当收到 TX/RX 会马上进入 TX/RX 状态。
  void fs();

/**
* \brief  切换到发送状态
  */
  void tx();

  void wor();

/**
  * \brief  切换到睡眠状态
  */
  void sleep();

/**
  * \brief  切换到待机状态
  */
  void standBy();

/**
  * @brief  使能接收模式
  * @note
  * 收到接收数据命令后，芯片先打开 PLL 及 VCO，进行校准，等待至 PLL 达到要求接收的频率，
  * 启用接收器电路(LNA，混频器、及 ADC)，再启用数字解调器的接收模式。
  * 直到收到接收到一包数据完成的指示信号或者是 SWOR 功能超时信号，
  * 如果是 SWOR 功能超时信号状态，则直接进入 STANDBY 模式;
  */
  void rx();


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

  /**
   * @brief print all registers
   */
  void printRegisters();

  RfSystem(const RfSystem &) = delete;

  RfSystem &operator=(const RfSystem &) = delete;

  // refresh status by 0x46 register
  RfStatus pollStatus();

  [[nodiscard]]
  bool isInitialized() const {
    return _is_initialized;
  };

  /**
   * @brief set pins. BEFORE calling `begin()`!
   * @return true if success. false if `begin()` has been called (initialized)
   * @see begin()
   */
  bool setPins(pin_size_t rst_pin, pin_size_t cs_pin, pin_size_t flag_pin, pin_size_t sdn_pin);

  /**
   * @brief get the singleton instance
   * @return the singleton instance reference. User should use `auto &` to get the reference.
   */
  static RfSystem &get() {
    static RfSystem instance = RfSystem();
    return instance;
  }

  /**
   * @brief enter SSCANR mode
   */
  void scanR();

  /// read RSSI
  uint8_t rssi();

  uint8_t pollTxPktSt();

/**
  * @brief  clear tx fifo write pointer
  */
  void clrTxFifoWrPtr();

  void clrRxFifoWrPtr();
  void clrRxFifoRdPtr();
  void clrRxFifo();

  PinStatus pollPktFlagPin();
};


#endif

