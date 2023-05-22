#include "rfsystem.h"

inline void RfSystem::RST_LOW() {
  digitalWrite(this->RST_PIN, LOW);
}

inline void RfSystem::RST_HIGH() {
  digitalWrite(this->RST_PIN, HIGH);
}

inline void RfSystem::SDN_LOW() {
  digitalWrite(this->SDN_PIN, LOW);
}

inline void RfSystem::SDN_HIGH() {
  digitalWrite(this->SDN_PIN, HIGH);
}

inline void RfSystem::CS_HIGH() {
  digitalWrite(this->CS_PIN, HIGH);
}

inline void RfSystem::CS_LOW() {
  digitalWrite(this->CS_PIN, LOW);
}

inline void RfSystem::gpioConfigure() {
  pinMode(this->RST_PIN, OUTPUT);
  pinMode(this->SDN_PIN, OUTPUT);
  pinMode(this->CS_PIN, OUTPUT);
  pinMode(this->IRQ_PIN, INPUT);
}

inline void RfSystem::spiConfigure() {
  SPI_init();
  SPI_begin_8();
}

inline int RfSystem::RF_IRQ_INPUT() {
  return digitalRead(this->IRQ_PIN);
}

void RfSystem::reset() {
  unsigned int loopCount;
  unsigned int i;
  RST_LOW();
  Delay_Ms(10);
  RST_HIGH();
  SDN_LOW();
}

inline uint8_t RfSystem::sendByte(unsigned char byte) {
  auto data = SPI_transfer_8(byte);
  return data;
}

void RfSystem::registerWrite(const uint8_t addr, const uint8_t val) {
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
  r_reg = registerRead(0x47);
  if ((r_reg & 0x07) == 0x04) {
    registerWrite(0x25, vReg25Tbl_h4[x_dBm]);
    registerWrite(0x26, vReg26Tbl_h4[x_dBm]);
  } else {
    registerWrite(0x25, vReg25Tbl_h31[x_dBm]);
    registerWrite(0x26, vReg26Tbl_h31[x_dBm]);
  }
}

void RfSystem::setSyncLockRssi() {
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
  setVcoFreq(f0);
  setFreq(N);
  setFreqStep(step);
}

void RfSystem::clrTxFifoWrPtr() {
  registerWrite(0x53, 0x80);      /*Reset FIFO write Pointer*/
}

unsigned char RfSystem::readRssi() {
  unsigned char r_reg;

  r_reg = registerRead(0x43);
  return r_reg / 2;
}

void RfSystem::writeFifo(const char *src, uint8_t len) {
  CS_LOW();
  sendByte(0x55 & 0x7F);
  for (auto i = 0; i < len; i++) {
    sendByte(static_cast<uint8_t>(*(src++)));
  }
  CS_HIGH();
}

void RfSystem::writeFifo(char src) {
  CS_LOW();
  sendByte(0x55 & 0x7F);
  sendByte(static_cast<uint8_t>(src));
  CS_HIGH();
}

void RfSystem::writeFifoWithSize(const char *src, uint8_t len) {
  CS_LOW();
  sendByte(0x55 & 0x7F);
  sendByte(len);
  for (auto i = 0; i < len; i++) {
    sendByte(static_cast<uint8_t>(*(src++)));
  }
  CS_HIGH();
}

void RfSystem::readFifo(unsigned char *dst, unsigned char Len) {
  unsigned char i = 0;
  CS_LOW();
  sendByte(0x52 | 0x80);
  for (i = 0; i < Len; i++) {
    *(dst + i) = sendByte(0xFF);
  }
  CS_HIGH();
}

void RfSystem::refreshStatus() {
  auto s = registerRead(0x46);
  auto status = RfStatus{
      .idle = (s & 0x01) == 0x01,
      .tx = (s & 0x02) == 0x02,
      .rx = (s & 0x04) == 0x04,
      .fs = (s & 0x08) == 0x08,
      .scan = (s & 0x10) == 0x10,
      .rc_cal = (s & 0x20) == 0x20,
      .vco_cal = (s & 0x40) == 0x40,
      .wor = (s & 0x80) == 0x80,
  };
  this->systemStatus = status;
}

const RfStatus & RfSystem::getStatus() const {
  return this->systemStatus;
}

/**
  * \brief  使能IDLE 模式
  */
void RfSystem::idle() {
  registerWrite(0x60, 0xff);
}

void RfSystem::rx() {
  registerWrite(0x51, 0x80);
  idle();
  registerWrite(0x66, 0xff);
}

void RfSystem::tx() {
  idle();
  registerWrite(0x65, 0xff);
}

void RfSystem::sleep() {
  idle();
  registerWrite(0x67, 0xff);
}

void RfSystem::standBy() {
  idle();
  registerWrite(0x68, 0xff);
}

void RfSystem::txCW() {
  idle();
  registerWrite(0x24, (registerRead(0x24) | 0x80));
  registerWrite(0x06, (registerRead(0x06) & 0xFC));
  tx();
}

void RfSystem::dataPackageSend(const char *buffer, const unsigned char size) {
  if (size > 0) {
    idle();
    clrTxFifoWrPtr();
    writeFifoWithSize(buffer, size);
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
    return len;
  }
}


/**
  * \brief RF芯片配置
  * \param None
  * \retval None
  */
void RfSystem::begin() {
  gpioConfigure();

  CS_HIGH();

  spiConfigure();
  Delay_Ms(200);
  reset();

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

  rx();
}

void RfSystem::printRegisters() {
  for (auto i = 0; i <= 0x7f; i++) {
    printf("reg(0x%02x) = 0x%02x \n", i, registerRead(i));
  }
}

/**
 * \brief   rf 中断底半段
 * \param   None
 * \retval  None
 */
void RfSystem::isr() {
  auto tmp = registerRead(0x40);

  // 接收到正确的 preamble
  if (tmp & (1 << 6)) {
    // 接收到正确的 sync word
    if (!(tmp & (1 << 7))) {
      preamble_timeout = 200;
      // Crc 错误指示
    } else if (!(tmp & (1 << 5))) {
      preamble_timeout = 0;
    } else {
      preamble_timeout = 0;
    }
    // 发送完成
  } else {
    preamble_timeout = 0;
    idle();
  }

}

/// should be 0
uint8_t RfSystem::version() {
  auto tmp = registerRead(0x04);
  // only need first two bit
  return tmp & 0b11;
};

bool RfSystem::rx_flag() const {
  return _rx_flag;
}

void RfSystem::reset_rx_flag() {
  _rx_flag = false;
}

RfSystem::RfSystem(pin_size_t rst_pin, pin_size_t cs_pin, pin_size_t irq_pin, pin_size_t sdn_pin) :
    RST_PIN(rst_pin), CS_PIN(cs_pin), IRQ_PIN(irq_pin), SDN_PIN(sdn_pin) {}


void RF::printStatus(const RfStatus &status) {
  printf("idle=%d, tx=%d, rx=%d, fs=%d, scan=%d, rc_cal=%d, vco_cal=%d, wor=%d\n",
         status.idle,
         status.tx,
         status.rx,
         status.fs,
         status.scan,
         status.rc_cal,
         status.vco_cal,
         status.wor);
};
