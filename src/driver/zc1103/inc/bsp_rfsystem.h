
#ifndef _BSP_RFSYSTEM_H
#define _BSP_RFSYSTEM_H

#include <cmath>
#include <cstring>
#include <cstdint>
#include "printf.h"

#define RF_RSSI_THRESHOLD                   65


typedef enum {
  DBM20,
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
} PA_LEVEL;

#define RFSYSSTAIDLE      (0)
#define RFSYSSTAREC        (2)
#define RFSYSSTATRAN      (1)
#define RFSYSSTASLEEP      (3)
#define RFSYSSTATSTANDBY    (4)

class RfSystem {
  volatile uint32_t preamble_timeout = 0;
  volatile uint8_t rece_falg = 0;
  int systemStatus = 0;
  unsigned char g_paValue = 10;
  volatile unsigned char rf_interrupt_pending = 0;
  double g_freq = 476.3;

  void gpioConfigure();

  void spiConfigure();

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

/**
  * \brief  读数据
  * \param [OUT] StoreBuf 保存数据地址
  * \param [IN] len 读取长度
  * \retval None
  */
  void readFifo(unsigned char *StoreBuf, unsigned char Len);

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
  unsigned char sendByte(unsigned char byte);

  void setSyncLockRssi(void);

  void setVcoFreq(const double freq);

  void setFreq(const unsigned char N);

  void setFreqStep(double step);

  void clrTxFifoWrPtr();

/**
  * \brief  发送数据
  * \param [IN] SrcBuf 待发送数据
  * \param [IN] len 待发送数据长度
  * \retval None
  */
  void writeFifo(const unsigned char *SrcBuf, unsigned char len);

/**
 * \brief  初始化rf寄存器
 * \param  None
 * \retval  None
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
  void dataPackageSend(const unsigned char *buffer, const unsigned char size);

/**
  * @brief  接收数据包
  * @param [OUT] buf 接收数据
  * @return 接收数据长度
  */
  int packageRecv(char *buf);

  int getSystemStatus();

  unsigned char getPktStatus();

/**
 * \brief  设置频率
 * \param [IN]  freq 频率值
 * \retval  None
 */
  void setRefFreq(const double freq);

/**
  * \brief  外部检查是否有中断发生
  * \param   None
  * \retval  0 没有 RF 中断; 1 有 RF 中断
  */
  unsigned char is_interrupt_pending();

  void clear_interrupt_flags();

/**
  * \brief  切换到睡眠状态
  * \param  None
  * \retval None
  */
  void sleep();

/**
  * \brief  切换到待机状态
  * \param  None
  * \retval None
  */
  void standBy();

  void isr();
};


#endif

