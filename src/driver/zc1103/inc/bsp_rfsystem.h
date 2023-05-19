
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

  void configure();

  void testPackageSend(const unsigned char *buffer, const unsigned char size);

  void dataPackageSend(const unsigned char *buffer, const unsigned char size);

  int packageRecv(char *buf);

  void freqSet(const double f0, const unsigned char N, const double step);

  void registerWrite(const unsigned char addr, const unsigned char val);

  unsigned char registerRead(const unsigned char addr);

  void readFifo(unsigned char *StoreBuf, unsigned char Len);

  unsigned char readRssi(void);

  void recEn();

  void txCW();

  void idleEn();

  int getSystemStatus();

  void sleepEn();

  void standByEn();

  void tranEn();

  void setPA(PA_LEVEL x_dBm);

  void setRefFreq(const double freq);

  void isr();

  unsigned char is_interrupt_pending();

  void clear_interrupt_flags();

  void RF_RST_LOW();

  void RF_RST_HIGH();

  void RF_SDN_LOW();

  void RF_SDN_HIGH();

  void RfCsHigh();

  void RfCsLow();

  void reset();

  unsigned char sendByte(unsigned char byte);

  void setSyncLockRssi(void);

  void setVcoFreq(const double freq);

  void setFreq_N(const unsigned char N);

  void setFreqStep(double step);

  void clrTxFifoWrPtr(void);

  void writeFifo(const unsigned char *SrcBuf, unsigned char len);

  void registerInit();

  unsigned char getPktStatus(void);

  int RF_IRQ_INPUT();
};


#endif

