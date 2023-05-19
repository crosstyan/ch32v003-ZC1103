#include "bsp_rfsystem.h"

//#define RF_RST_HIGH()       GPIO_WriteBit(RF_RST_Port, RF_RST_Pin, Bit_SET)
//#define RF_RST_LOW()        GPIO_WriteBit(RF_RST_Port, RF_RST_Pin, Bit_RESET)
//
//#define RF_SDN_HIGH()       GPIO_WriteBit(RF_SDN_Port, RF_SDN_Pin, Bit_SET)
//#define RF_SDN_LOW()        GPIO_WriteBit(RF_SDN_Port, RF_SDN_Pin, Bit_RESET)
//
//#define  RF_IRQ_INPUT()     GPIO_ReadInputDataBit(RF_IRQ_Port, RF_IRQ_Pin)
//
//#define  RfCsHigh()         SPI_NSS_OUT_High
//#define  RfCsLow()           SPI_NSS_OUT_Low

inline void RfSystem::RST_LOW() {
  // unimplemented
}

inline void RfSystem::RST_HIGH() {
  // unimplemented
}

inline void RfSystem::SDN_LOW() {
  // unimplemented
}

inline void RfSystem::SDN_HIGH() {
  // unimplemented
}

inline void RfSystem::CS_HIGH() {
  // unimplemented
}

inline void RfSystem::CS_LOW() {
  // unimplemented
}

void RfSystem::gpioConfigure() {
  // unimplemented
}

void RfSystem::spiConfigure() {
  // unimplemented
}

inline int RfSystem::RF_IRQ_INPUT() {
  // unimplemented
}

/**
 * \brief  复位射频芯片
 * \param   None
 * \retval  None
 */
void RfSystem::reset() {

  unsigned int loopCount;
  unsigned int i;
  /*复位寄存器*/
  RST_LOW();
  for (i = 10; i > 0; i--) {
    for (loopCount = 0xfff; loopCount != 0; loopCount--);
  }
  RST_HIGH();
  SDN_LOW();
  for (i = 10; i > 0; i--) {
    for (loopCount = 0xfff; loopCount != 0; loopCount--);
  }
}

unsigned char RfSystem::sendByte(unsigned char byte) {
  //  unsigned char tmp = 0;
  //  tmp = Spi_Read_Write_Byte(byte);
  //  return tmp;
  // unimplemented
}

void RfSystem::registerWrite(const unsigned char addr, const unsigned char val) {
  CS_LOW();
  sendByte(addr & 0x7f);
  sendByte(val);
  CS_HIGH();
}

unsigned char RfSystem::registerRead(const unsigned char addr) {
  CS_LOW();
  sendByte(addr | 0x80);
  auto readData = sendByte(0xff);
  CS_HIGH();
  return readData;
}

void RfSystem::registerInit() {
  unsigned int i = 0;
  unsigned int loopCount = 0;

  registerWrite(0x09, 0x08);/*Debug*/
  registerWrite(0x0c, 0x03);
  registerWrite(0x0e, 0xA1);
  registerWrite(0x0F, 0x0A);
  registerWrite(0x10, 0x54);
  registerWrite(0x1b, 0x25);


  registerWrite(0x20, 0xa4);
  registerWrite(0x21, 0x37);
  registerWrite(0x22, 0x3a);         /*VCO Config  3a*/  //3a -> 0azhangjun 20200612
  registerWrite(0x23, 0x36);         /*SYN Config   bit[7]enable wideband */
  /*registerWrite(0x23, ((RfRegisterRead(0x23)&0x8F)|0x50));  //bit[6-4] Vco ldo output voltage */
  registerWrite(0x2F, 0xe0);         /*rx rssi threshold*/
  registerWrite(0x2E, 0x00);

  registerWrite(0x30, 0x00);         /*ber optimize 0x40->0x00 by 20211126 juner*/
  registerWrite(0x31, 0x00);
  registerWrite(0x32, 0x00);
  registerWrite(0x33, 0x00);
  registerWrite(0x34, 0x00);
  registerWrite(0x35, 0x00);
  registerWrite(0x36, 0x00);


  registerWrite(0x39, 0x74); /*enable demode reset */
  registerWrite(0x3A, 0x61);
  registerWrite(0x4a, 0x60);
  registerWrite(0x4d, 0x0b);
  registerWrite(0x4e, 0x7c);/*ber optimize 0x6c->0x7c by 20211126 juner*/
  registerWrite(0x4f, 0xc5);
  ////10kps
  registerWrite(0x74, 0x9d);/*bit[7-6] ADC clock select*/
  registerWrite(0x08, 0x01);/*方法1设置频偏25k   */
  registerWrite(0x24, 0x19);/*中频设置[7-0]      */
  registerWrite(0x3D, 0x53);/*中频设置[7-0]      */

  registerWrite(0x38, 0x56);
  registerWrite(0x3C, 0xD1);
  registerWrite(0x3E, 0x83);
  registerWrite(0x3F, 0x08);

  registerWrite(0x58, 0x00);
  registerWrite(0x59, 0x07);
  registerWrite(0x5A, 0x08);
  registerWrite(0x5B, 0x09);
  registerWrite(0x5C, 0x03);
  registerWrite(0x5D, 0x71);
  registerWrite(0x5e, 0x00);
  registerWrite(0x5f, 0xDF);


  //25k->20k   0x40->0xC0
  //25k->20k   0x66->0x51
  //25k->20k   0x66->0xEC

  registerWrite(0x78, 0xC0);  //方法2设置频偏10k --未使用 25->20K
  registerWrite(0x79, 0x51);  //25->20K
  registerWrite(0x7a, 0xEC);  //25->20K

  registerWrite(0x7b, 0x5A);
  registerWrite(0x7c, 0x7C);
  registerWrite(0x7d, 0x01);
  registerWrite(0x7e, 0x00);
  registerWrite(0x7f, 0x70);

  registerWrite(0x15, 0x21);
  registerWrite(0x07, 0x5d);
  registerWrite(0x18, 0x20);
  registerWrite(0x2a, 0x14);
  registerWrite(0x37, 0x99);

  registerWrite(0x06, 0x3a);//0x3a /*syncwordlen = 2bytes,length = 1byte,CRC,SCramble_on*/   bit[3] share fifo
  registerWrite(0x04, 0x50);/*preamble length 80 bytes*/

  for (i = 10; i > 0; i--) {
    for (loopCount = 0xffff; loopCount != 0; loopCount--);
  }
}

void RfSystem::setRefFreq(const double freq) {
  unsigned int Fre = 0;
  unsigned char reg73 = 0, reg72 = 0, reg71 = 0, reg70 = 0;
  Fre = (unsigned int) (freq * pow(2.0, 24.0));

  reg73 = (unsigned char) (Fre & 0xFF);
  reg72 = (unsigned char) ((Fre >> 8) & 0xFF);
  reg71 = (unsigned char) ((Fre >> 16) & 0xFF);
  reg70 = (unsigned char) ((Fre >> 24) & 0xFF);

  registerWrite(0x73, reg73);
  registerWrite(0x72, reg72);
  registerWrite(0x71, reg71);
  registerWrite(0x70, reg70);
}

void RfSystem::setPA(PA_LEVEL x_dBm) {
  unsigned char r_reg;
#if 0
  code   unsigned char vReg25Tbl[] =      {0x3a,0x2b,0x25,0x1a,0x0d,0x0b,0x09,0x07,0x06,0x04,0x03,0x01,0x4c,
             0x4a,0x46,0x04,0x04,0x05,0x04,0x01,0X01,0X01,0X00,0X00,0X02,0X01,0X00};
  code   unsigned char vReg26Tbl[] =      {0xdd,0xdd,0xbf,0xdd,0xdd,0xdd,0xdd,0xbf,0xbf,0xbf,0xbf,0xbf,0x3a,
             0x3a,0x30,0x35,0x25,0x11,0X0e,0X17,0X11,0X0c,0X11,0X0a,0X02,0x03,0x03};
  code   unsigned char vReg25Tbl_h3[] ={ 0x3f,0xff,0xa6,0x95,0x94,0xce,0x91,0x09,0x08,0x03,\
                                     0x7f,0x3f,0x2f,0x1f,0x1c,0x15,0x13,0x11,0x10,0x10,\
                                     0x07,0x05,0x05,0x03,0x03,0x03,0x03};

  code   unsigned char vReg26Tbl_h3[] ={ 0x81,0x81,0x83,0x9a,0x82,0x82,0x80,0x83,0x9f,0x98,\
                                     0x7f,0x64,0x67,0x7d,0x7d,0x7c,0x77,0x74,0x75,0x65,\
                                     0x6c,0x77,0x67,0x7d,0x6b,0x5e,0x56};
#else
  const unsigned char vReg25Tbl_h4[] = {0x3f, 0x38, 0x25, 0x1a, 0x0f, 0x0d, 0x0b, 0x0a, 0x09, 0x08, 0x04, 0x03, 0x86,
                                        0x82, 0x01, 0x01, 0x02, 0x02, 0x00, 0x00, 0x24, 0x20, 0x16, 0x14, 0x11, 0x0d,
                                        0x0d};

  const unsigned char vReg26Tbl_h4[] = {0xb0, 0xb0, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0x9f, 0x9f, 0xbf, 0x8f, 0x80,
                                        0xbf, 0x9f, 0x84, 0x81, 0x80, 0xad, 0x88, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x72,
                                        0x62};


  const unsigned char vReg25Tbl_h31[] = {0xbf, 0xff, 0x1d, 0x1c, 0x0f, 0x0e, 0x07, 0x06, 0x05, 0x04, \
                                    0x03, 0x3f, 0x2f, 0x2f, 0x1d, 0x17, 0x13, 0x11, 0x10, 0x10, \
                                    0x07, 0x05, 0x05, 0x03, 0x03, 0x03, 0x03};

  const unsigned char vReg26Tbl_h31[] = {0x83, 0x81, 0xaf, 0x82, 0x8f, 0x85, 0x95, 0xbf, 0x81, 0xab, \
                                    0x81, 0x68, 0x6b, 0x5c, 0x75, 0x72, 0x7b, 0x79, 0x7a, 0x68, \
                                    0x6f, 0x7c, 0x6a, 0x7f, 0x6e, 0x62, 0x58};
#endif
  g_paValue = x_dBm;
  r_reg = registerRead(0x47);
  if ((r_reg & 0x07) == 0x04) {
    registerWrite(0x25, vReg25Tbl_h4[x_dBm]);
    registerWrite(0x26, vReg26Tbl_h4[x_dBm]);
  } else {
    registerWrite(0x25, vReg25Tbl_h31[x_dBm]);
    registerWrite(0x26, vReg26Tbl_h31[x_dBm]);
  }
}

/**
  * \brief  使能接收到同步字后锁定rssi
  * \param  None
  * \retval  None
  */
void RfSystem::setSyncLockRssi(void) {
  registerWrite(0x3e, registerRead(0x3e) | 0x40);
}

void RfSystem::setVcoFreq(const double freq) {
  unsigned int Fre = 0;
  unsigned char reg77 = 0, reg76 = 0, reg75 = 0, reg74 = 0, temp = 0;
  Fre = (unsigned int) (freq * pow(2.0, 20.0));

  reg77 = (unsigned char) (Fre & 0xFF);
  reg76 = (unsigned char) ((Fre >> 8) & 0xFF);
  reg75 = (unsigned char) ((Fre >> 16) & 0xFF);
  reg74 = (unsigned char) (((Fre >> 24) & 0xFF) | (registerRead(0x74) & 0xc0));

  temp = registerRead(0x00);
  registerWrite(0x00, (0x80 | temp));

  registerWrite(0x77, reg77);
  registerWrite(0x76, reg76);
  registerWrite(0x75, reg75);
  registerWrite(0x74, reg74);
}

void RfSystem::setFreq(const unsigned char N) {
  if (N > 0x7F)
    return;
  registerWrite(0x00, (0x80 | N));
}


void RfSystem::setFreqStep(double step) {
  unsigned int fre = 0;
  unsigned char reg1 = 0, reg2 = 0, reg3 = 0;
  fre = (unsigned int) (step * pow(2.0, 20.0));
  reg3 = (unsigned char) (fre & 0xFF);
  reg2 = (unsigned char) ((fre >> 8) & 0xFF);
  reg1 = (unsigned char) ((fre >> 16) & 0x7F);
  registerWrite(0x03, reg3);
  registerWrite(0x02, reg2);
  registerWrite(0x01, reg1);
}

void RfSystem::freqSet(const double f0, const unsigned char N, const double step) {

  g_freq = f0;
  setVcoFreq(f0);
  setFreq(N);
  setFreqStep(step);
}

/**
  * \brief  清空发送区域
  * \param  None
  * \retval  None
  */
void RfSystem::clrTxFifoWrPtr(void) {
  registerWrite(0x53, 0x80);      /*Reset FIFO write Pointer*/
}

/**
  * \brief  获取包状态
  * \param  None
  * \retval
  */
unsigned char RfSystem::getPktStatus(void) {

  if (RF_IRQ_INPUT()) {
    return 1;
  } else {
    return 0;
  }

}

unsigned char RfSystem::readRssi(void) {
  unsigned char r_reg;

  r_reg = registerRead(0x43);
  return r_reg / 2;
}

void RfSystem::writeFifo(const unsigned char *SrcBuf, unsigned char len) {
  unsigned char i = 0;

  CS_LOW();
  sendByte(0x55 & 0x7F);
  for (i = 0; i < len; i++) {
    sendByte(*(SrcBuf++));
  }
  CS_HIGH();
}

void RfSystem::readFifo(unsigned char *StoreBuf, unsigned char Len) {
  unsigned char i = 0;
  CS_LOW();
  sendByte(0x52 | 0x80);
  for (i = 0; i < Len; i++) {
    *(StoreBuf + i) = sendByte(0xFF);
  }
  CS_HIGH();
}

/**
  * \brief  RF 当前状态
  * \param  None
  * \retval rf芯片状态
  */

int RfSystem::getSystemStatus() {
  return systemStatus;
  /*
    switch(registerRead(0x46))
    {
      case 0x80:
         return systemStatus;
      break;
      case 0x20:
        return 2;
      break;
      case 0x40:
        return 1;
      break;
      case 0x10:
        return 6;
      break;
      case 0x08:
        return 7;
      case 0x04:
        return 8;
      case 0x02:
        return 9;
      case 0x01:
        return 10;
      break;
      default:
        break;
    }*/
}

/**
  * \brief  使能IDLE 模式
  * \param  None
  * \retval None
  */
void RfSystem::idle(void) {
  int i = 0;
  registerWrite(0x60, 0xff);
  while (registerRead(0x46) != 0x80) {
    if (i++ > 256) {
      systemStatus = 0xff;
      return;
    }
  }
  systemStatus = 0;
}

void RfSystem::rx(void) {
  int i = 0;
  registerWrite(0x51, 0x80);
  idle();
  registerWrite(0x66, 0xff);
  while (registerRead(0x46) != 0x20) {
    if (i++ > 256) {
      systemStatus = 0xff;
      return;
    }
  }
  systemStatus = 2;
}

void RfSystem::tx(void) {
  idle();
  registerWrite(0x65, 0xff);
  while (registerRead(0x46) != 0x40);
  systemStatus = 1;
}

void RfSystem::sleep() {
  idle();
  registerWrite(0x67, 0xff);
  systemStatus = 3;
}

void RfSystem::standBy(void) {
  idle();
  registerWrite(0x68, 0xff);
  systemStatus = 4;
}

void RfSystem::txCW(void) {
  idle();
  registerWrite(0x24, (registerRead(0x24) | 0x80));
  registerWrite(0x06, (registerRead(0x06) & 0xFC));
  tx();
}

void RfSystem::dataPackageSend(const unsigned char *buffer, const unsigned char size) {

  /*Fix SPI concurrency conflicts, disable irq */
  if (size > 0) {
    unsigned char buf[264] = {0};


    buf[0] = size;
    memcpy(buf + 1, buffer, size);
    idle();
    clrTxFifoWrPtr();
    writeFifo(&buf[0], size + 1);
    tx();

  }

}

int RfSystem::packageRecv(char *buf) {
  int len;

  unsigned char rx_rssi = 0;

  registerWrite(0x51, 0x80);
  len = registerRead(0x52 | 0x80);
  if (len == 0) {
    rx();
    return -3;
  } else {
    rx_rssi = readRssi();
    readFifo((uint8_t *) buf, len);
    rx();
    #if 0
    printf("rece data len = %d  rssi = -%bd dB\r\n", len,rx_rssi);
    for(i=0; i<len; i++){
        printf("0x%bx\t",buf[i] );
    }
    #endif
    return len;
  }
}


/**
  * \brief RF芯片配置
  * \param None
  * \retval None
  */
void RfSystem::begin(void) {
  unsigned int i = 0;


  gpioConfigure();

  CS_HIGH();

  spiConfigure();
  //重启芯片
//  delay_ms(200);
  reset();
  //初始化rf 参数

  registerInit();
  //设置参考频率
  registerWrite(0x70, 0x12);
  registerWrite(0x71, 0x14);
  registerWrite(0x72, 0x7A); // 6D 48    A1  7A 18.08
  registerWrite(0x73, 0xE1); // 32 00    84  E1
  //设置锁频
  setSyncLockRssi();
  //设置中心频点
  freqSet(476.0, 0, 0);
  //设置发射功率
  setPA(DBM20);
  //设置为接收态
  rx();
  //打印初始化参数

  for (i = 0; i <= 0x7f; i++) {
    printf("read  reg0x%02x = %2.2x \n", i, registerRead(i));
  }
}


/**
 * \brief   rf 中断底半段
 * \param   None
 * \retval  None
 */
void RfSystem::isr() {
  unsigned char tmp = 0;

  tmp = registerRead(0x40);

  printf("%2.2x\r\n", tmp);

  if (tmp & (1 << 6)) {          /*接收到正确的preamble*/
    if (!(tmp & (1 << 7))) {   /*接收到正确的syncword*/
      preamble_timeout = 200;

    } else if (!(tmp & (1 << 5))) { /*Crc 错误指示 */
      preamble_timeout = 0;
      rece_falg = 0x01;
    } else {
      rece_falg = 0x02;
      //RfRecEn();
      preamble_timeout = 0;
    }
  } else { /*发送完成*/
    preamble_timeout = 0;
    // RfRecEn();
    idle();
  }

}

unsigned char RfSystem::is_interrupt_pending() {
  return rf_interrupt_pending;
}

void RfSystem::clear_interrupt_flags(void) {
  rf_interrupt_pending = 0;
}
