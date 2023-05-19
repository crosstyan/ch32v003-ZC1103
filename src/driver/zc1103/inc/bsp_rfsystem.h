
#ifndef _BSP_RFSYSTEM_H
#define _BSP_RFSYSTEM_H

#define RF_RSSI_THRESHOLD                   65


typedef enum
{
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
}PA_LEVEL;

#define RFSYSSTAIDLE  		(0)
#define RFSYSSTAREC	  		(2)
#define RFSYSSTATRAN  		(1)
#define RFSYSSTASLEEP  		(3)
#define RFSYSSTATSTANDBY  	(4)

extern void RfConfigure(void);

extern void RfTestPackageSend(const unsigned char *buffer,const unsigned char size);

extern void RfDataPackageSend(const unsigned char *buffer, const unsigned char size);

extern int  RfPackageRecv(char *buf);

extern void RfFreqSet(const double f0,const unsigned char N,const double step);

extern void RfRegisterWrite(const unsigned char addr,const unsigned char val);

extern unsigned char RfRegisterRead(const unsigned char addr);

extern void RfReadFIFO(unsigned char *StoreBuf,unsigned char Len);

extern unsigned char RfReadRssi(void);

extern void RfRecEn(void);

extern void RF_TxCW(void);

extern void RfIdleEn(void);

extern int  RfSystemStatus(void);

extern void RfSleepEn(void);

extern void RfStandByEn(void);

extern void RfTranEn(void);

extern void RfSetPA(PA_LEVEL x_dBm);

extern void  RFSetRefFreq(const double freq);

extern void rf_isr(void);

unsigned char is_there_a_rf_interrupt_pending(void);

void clear_rf_interrupt_flags(void);



#endif

