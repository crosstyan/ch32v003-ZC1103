
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

#define RF_RSSI_THRESHOLD                   65

enum PA_LEVEL {
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

class RfSystem {
  volatile uint32_t preamble_timeout = 0;
  struct RfStatus systemStatus = {
      .idle = true,
      .tx = false,
      .rx = false,
      .fs = false,
      .scan = false,
      .rc_cal = false,
      .vco_cal = false,
      .wor = false
  };
  volatile bool _rx_flag = false;

  /// 芯片复位脚，低电平有效，复位后寄存器数值丢失，全部变为默认值。
  pin_size_t RST_PIN;
  /// 使能信号，低有效，拉低可使芯片退出 sleep mode
  pin_size_t CS_PIN;
  /// IRQ
  pin_size_t IRQ_PIN;
  /// 芯片关断使能，高有效
  pin_size_t SDN_PIN;

  void gpioConfigure();

  void spiConfigure();

/**
  * \brief  读数据
  * \param [OUT] dst 保存数据地址
  * \param [IN] len 读取长度
  * \retval None
  */
  void readFifo(unsigned char *dst, unsigned char Len);

/**
  * \brief  读取Rssi值
  * \param  None
  * \retval
  */
  unsigned char readRssi(void);

/**
  * \brief  使能接收模式
  * \param  None
  * \retval None
  */
  void rx();

/**
* \brief  发送单音载波
  * \param  None
  * \retval None
  */
  void txCW();

  void idle();

/**
* \brief  切换到发送状态
  * \param  None
  * \retval None
  */
  void tx();

/**
 * \brief  设置PA增益
 * \param [IN]  x_dBm 增益
 * \retval  None
 */
  void setPA(PA_LEVEL x_dBm);

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
  uint8_t sendByte(unsigned char byte);

/**
  * \brief  使能接收到同步字后锁定rssi
  * \param  None
  * \retval  None
  */
  void setSyncLockRssi(void);

  void setVcoFreq(const double freq);

  void setFreq(const unsigned char N);

  void setFreqStep(double step);

/**
  * \brief  清空发送区域
  */
  void clrTxFifoWrPtr();

/**
  * \brief  发送数据 (without size written in the first byte)
  * \param [IN] src
  * \param [IN] len
  * \retval None
  */
  void writeFifo(const char *src, uint8_t len);

/**
  * \brief  发送数据
  * \param [IN] src 待发送数据 (单字节)
  * \retval None
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
  void registerInit();

  int RF_IRQ_INPUT();

  void freqSet(const double f0, const unsigned char N, const double step);

public:
  void begin();

  void reset();

/**
  * \brief  发送数据包
  * \param [IN] buffer 发送数据
  * \param [IN] size   发送数数据长度
  */
  void dataPackageSend(const char *buffer, const unsigned char size);

/**
  * @brief  接收数据包
  * @param [OUT] buf 接收数据
  * @return 接收数据长度
  */
  int packageRecv(char *buf);


/**
 * \brief  设置频率
 * \param [IN]  freq 频率值
 */
  void setRefFreq(const double freq);

/**
  * \brief  外部检查是否有中断发生
  * \retval  0 没有 RF 中断; 1 有 RF 中断;
  */
  bool rxFlag() const;

  void resetRxFlag();

  /**
   * @brief polling IRQ pin
   * @effect `rxFlag` will be set to true if IRQ pin is high
   */
  bool pollIrqPin();

  uint8_t version();

/**
  * \brief  切换到睡眠状态
  */
  void sleep();

/**
  * \brief  切换到待机状态
  */
  void standBy();

  void isr();

/**
 * \brief  写RF寄存器
 * \param[IN] addr 寄存器地址 取值0x00 - 0x7F
 * \param[IN] val  写入的值
 * \retval  None
 */
  void registerWrite(const unsigned char addr, const unsigned char val);

/**
 * \brief  读RF寄存器
 * \param[IN] addr 寄存器地址 取值0x00 - 0x7F
 * \retval  读取寄存器的值
 */
  unsigned char registerRead(const unsigned char addr);

  /// ch32v003 only has one SPI so there's no need to specify the SPI bus
  RfSystem(pin_size_t rst_pin, pin_size_t cs_pin, pin_size_t irq_pin, pin_size_t sdn_pin);

  /**
   * @brief print all registers
   */
  void printRegisters();

  RfSystem(const RfSystem &) = delete;

  RfSystem &operator=(const RfSystem &) = delete;

  RfSystem(RfSystem &&) = delete;

  RfSystem &operator=(RfSystem &&) = delete;

  // refresh status by 0x46 register
  void refreshStatus();

  // get status but not refresh
  [[nodiscard]]
  const RfStatus &getStatus() const;
};

namespace RF {
  void printStatus(const RfStatus &status);
}

#endif

