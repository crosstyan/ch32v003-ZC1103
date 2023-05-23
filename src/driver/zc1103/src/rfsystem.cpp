#include "rfsystem.h"

/// a utility function to check if a bit is set at shift.
/// note that shift is 0-indexed
inline static bool shift_equal(uint8_t byte, uint8_t shift) {
  return (byte & (1 << shift)) == (1 << shift);
}

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

/// 芯片的所有控制都是通 SPI 接口操作，支持的模式是时钟极性为正，相位极性可选，
/// 当 ckpha=1 时，为下降沿采样，ckpha=0 时，上升沿采样。
///
/// See also `src/inc/spi.h`
inline void RfSystem::spiConfigure() {
  SPI_init();
  SPI_begin_8();
}

inline int RfSystem::RF_IRQ_INPUT() {
  return digitalRead(this->IRQ_PIN);
}

void RfSystem::reset() {
  RST_LOW();
  Delay_Ms(10);
  RST_HIGH();
  SDN_LOW();
}

inline uint8_t RfSystem::sendByte(unsigned char byte) {
  return SPI_transfer_8(byte);
}

inline void RfSystem::sendBytes(uint8_t const *bytes, size_t len) {
  for (size_t i = 0; i < len; i++) {
    sendByte(bytes[i]);
  }
};

inline void RfSystem::sendBytes(char const *bytes, size_t len) {
  for (size_t i = 0; i < len; i++) {
    sendByte(bytes[i]);
  }
};

/// On this particular device, you transmit a 7-bit register address
/// (with the high-bit set to 1=read, 0=write),
/// then either read or write the value of the register from there.
void RfSystem::write(const uint8_t addr, const uint8_t val) {
  // 0x7f = 0b0111_1111 i.e. set the high bit to 0 to indicate write
  CS_LOW();
  sendByte(addr & 0x7f);
  sendByte(val);
  CS_HIGH();
}

unsigned char RfSystem::read(const unsigned char addr) {
  // 0x80 = 0b1000_0000 i.e. set the high bit to 1 to indicate read
  CS_LOW();
  sendByte(addr | 0x80);
  auto data = sendByte(0xff);
  CS_HIGH();
  return data;
}

void RfSystem::registerInit() {
  // 0x08 = 0b00001000
  write(0x09, 0x08);/*Debug*/
  // 0x03 = 0b00000011
  write(0x0c, 0x03);
  // raw RSSI 最低 1bit 表示小数
  write(0x0d, 0x70);
  // 0xa1 = 0b10100001
  //             00001 allowed error bit
  //            1 该位选择 pkt_flag 有效 后，是否自动拉低。如果自动拉低, 脉冲宽度大约 1us
  //           0  syncword 中断使能，当接收端收到有效的 同步字 syncword 时产生中断信号
  //          1   preamble 中断使能，当接收端收到有效 的前导码 preamble 时产生中断信号
//  write(0x0e, 0b00100001);
  write(0x0e, 0b00000001);
  // 0x0a = 0b00001010
  //             00000 发送 FIFO 空门限，发送 FIFO 还剩余字节数低于门限值时会产生 fifo_flag 标志
  write(0x0f, 0x0f);
  write(0x10, 0x54);
  write(0x1b, 0x25);


  write(0x20, 0xa4);
  write(0x21, 0x37);
  write(0x22, 0x3a);         /*VCO Config  3a*/  //3a -> 0azhangjun 20200612
  write(0x23, 0x36);         /*SYN Config   bit[7]enable wideband */
  /*registerWrite(0x23, ((RfRegisterRead(0x23)&0x8F)|0x50));  //bit[6-4] Vco ldo output voltage */
  write(0x2F, 0xe0);         /*rx rssi threshold*/
  write(0x2E, 0x00);

  write(0x30, 0x00);         /*ber optimize 0x40->0x00 by 20211126 juner*/
  write(0x31, 0x00);
  write(0x32, 0x00);
  write(0x33, 0x00);
  write(0x34, 0x00);
  write(0x35, 0x00);
  write(0x36, 0x00);


  write(0x39, 0x74); /*enable demode reset */
  write(0x3A, 0x61);
  write(0x4a, 0x60);
  // should be 0x0b and no need to write it
  // registerWrite(0x4d, 0x0b);
  write(0x4e, 0x7c);/*ber optimize 0x6c->0x7c by 20211126 juner*/
  write(0x4f, 0xc5);
  ////10kps
  write(0x74, 0x9d);/*bit[7-6] ADC clock select*/
  write(0x08, 0x01);/*方法1设置频偏25k   */
  write(0x24, 0x19);/*中频设置[7-0]      */
  write(0x3D, 0x53);/*中频设置[7-0]      */

  write(0x38, 0x56);
  write(0x3C, 0xD1);
  write(0x3E, 0x83);
  write(0x3F, 0x08);

  write(0x58, 0x00);
  write(0x59, 0x07);
  write(0x5A, 0x08);
  write(0x5B, 0x09);
  write(0x5C, 0x03);
  write(0x5D, 0x71);
  write(0x5e, 0x00);
  write(0x5f, 0xDF);


  //25k->20k   0x40->0xC0
  //25k->20k   0x66->0x51
  //25k->20k   0x66->0xEC

  write(0x78, 0xC0);  //方法2设置频偏10k --未使用 25->20K
  write(0x79, 0x51);  //25->20K
  write(0x7a, 0xEC);  //25->20K

  write(0x7b, 0x5A);
  write(0x7c, 0x7C);
  write(0x7d, 0x01);
  write(0x7e, 0x00);
  write(0x7f, 0x70);

  write(0x15, 0x21);
  write(0x07, 0x5d);
  write(0x18, 0x20);
  write(0x2a, 0x14);
  write(0x37, 0x99);
  // 0x3a = 0b00111010
  //                 0 HW_TERM_EN
  //                1  PKT_LENGTH_EN
  //               0   DIRECT_MODE
  //              1    FIFO_SHARE_EN
  //             1     SCRAMBLE_EN
  //            1      CRC_EN
  //           0       LENGTH_SEL: 默认为数据包的第一个字节为包长度 (0: 1 byte, 1: 2 bytes)
  //          0        SYNCWORD_LEN: 同步字长度设置
  write(0x06, 0b00110010);
  // preamble length 80 bytes
  write(0x04, 0x50);
}

void RfSystem::setRefFreq(const double freq) {
  auto f = static_cast<uint8_t>(freq * pow(2.0, 24.0));

  uint8_t reg73 = f & 0xFF;
  uint8_t reg72 = (f >> 8) & 0xFF;
  uint8_t reg71 = (f >> 16) & 0xFF;
  uint8_t reg70 = (f >> 24) & 0xFF;

  write(0x73, reg73);
  write(0x72, reg72);
  write(0x71, reg71);
  write(0x70, reg70);
}

void RfSystem::setPA(PA_LEVEL x_dBm) {
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
  auto r_reg = read(0x47);
  if ((r_reg & 0x07) == 0x04) {
    write(0x25, vReg25Tbl_h4[x_dBm]);
    write(0x26, vReg26Tbl_h4[x_dBm]);
  } else {
    write(0x25, vReg25Tbl_h31[x_dBm]);
    write(0x26, vReg26Tbl_h31[x_dBm]);
  }
}

void RfSystem::setSyncLockRssi() {
  write(0x3e, read(0x3e) | 0x40);
}

void RfSystem::setVcoFreq(const double freq) {
  unsigned int Fre = 0;
  unsigned char reg77 = 0, reg76 = 0, reg75 = 0, reg74 = 0, temp = 0;
  Fre = (unsigned int) (freq * pow(2.0, 20.0));

  reg77 = (unsigned char) (Fre & 0xFF);
  reg76 = (unsigned char) ((Fre >> 8) & 0xFF);
  reg75 = (unsigned char) ((Fre >> 16) & 0xFF);
  reg74 = (unsigned char) (((Fre >> 24) & 0xFF) | (read(0x74) & 0xc0));

  temp = read(0x00);
  write(0x00, (0x80 | temp));

  write(0x77, reg77);
  write(0x76, reg76);
  write(0x75, reg75);
  write(0x74, reg74);
}

// TODO: find documentation for this
void RfSystem::setFreq(const unsigned char N) {
  if (N > 0x7F) {
    return;
  }
  write(0x00, (0x80 | N));
}


void RfSystem::setFreqStep(double step) {
  unsigned int fre = 0;
  unsigned char reg1 = 0, reg2 = 0, reg3 = 0;
  fre = (unsigned int) (step * pow(2.0, 20.0));
  reg3 = (unsigned char) (fre & 0xFF);
  reg2 = (unsigned char) ((fre >> 8) & 0xFF);
  reg1 = (unsigned char) ((fre >> 16) & 0x7F);
  write(0x03, reg3);
  write(0x02, reg2);
  write(0x01, reg1);
}

void RfSystem::freqSet(const double f0, const unsigned char N, const double step) {
  setVcoFreq(f0);
  setFreq(N);
  setFreqStep(step);
}

void RfSystem::clrTxFifoWrPtr() {
  // 0x80 = 0b1000_0000
  write(0x53, 0x80);      /*Reset FIFO write Pointer*/
}

/// the bigger the number the more power
unsigned char RfSystem::rssi() {
  unsigned char r_reg;

  r_reg = read(0x43);
  return r_reg / 2;
}

void RfSystem::writeFifo(const char *src, uint8_t len) {
  CS_LOW();
  sendByte(0x55 & 0x7F);
  sendBytes(src, len);
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
  sendBytes(src, len);
  CS_HIGH();
}

void RfSystem::readFifo(unsigned char *dst, unsigned char len) {
  CS_LOW();
  sendByte(0x52 | 0x80);
  for (auto i = 0; i < len; i++) {
    *(dst + i) = sendByte(0xFF);
  }
  CS_HIGH();
}

RfStatus RfSystem::pollStatus() {
  auto s = read(0x46);
  // idle is bit 7
  auto status = RfStatus{
      .idle = shift_equal(s, 7),
      .tx = shift_equal(s, 6),
      .rx = shift_equal(s, 5),
      .fs = shift_equal(s, 4),
      .scan = shift_equal(s, 3),
      .rc_cal = shift_equal(s, 2),
      .vco_cal = shift_equal(s, 1),
      .wor = shift_equal(s, 0),
  };
  return status;
}

inline void RfSystem::idle() {
  write(0x60, 0xff);
}

inline void RfSystem::fs() {
  write(0x64, 0xff);
}

inline void RfSystem::rx() {
  write(0x51, 0x80);
  write(0x66, 0xff);
}

inline void RfSystem::tx() {
  write(0x65, 0xff);
}

inline void RfSystem::sleep() {
  idle();
  write(0x67, 0xff);
}

inline void RfSystem::standBy() {
  idle();
  write(0x68, 0xff);
}

void RfSystem::txCW() {
  write(0x24, (read(0x24) | 0x80));
  write(0x06, (read(0x06) & 0xFC));
  tx();
}

etl::optional<Unit>
RfSystem::send(const char *buffer, const unsigned char size) {
  if (size > 0) {
    fs();
    // clrTxFifoWrPtr();
    writeFifoWithSize(buffer, size);
    tx();
    // check if tx mode entered
    auto counter = 0;
    while (!this->pollStatus().tx) {
      if (counter > 31) {
        return etl::nullopt;
      }
      counter += 1;
    }
    // don't check pkg_flag in register
    // only works for GPIO pin output
  }
  auto u = Unit{};
  return etl::make_optional(u);
}

etl::optional<size_t>
RfSystem::recv(char *buf) {
  write(0x51, 0x80);
  size_t len = read(0x52 | 0x80);
  if (len == 0) {
    rx();
    return etl::nullopt;
  } else {
    readFifo((uint8_t *) buf, len);
    rx();
    return etl::make_optional(len);
  }
}

void RfSystem::begin() {
  gpioConfigure();

  CS_HIGH();

  spiConfigure();
  Delay_Ms(200);
  reset();

  registerInit();

  //设置参考频率
  write(0x70, 0x12);
  write(0x71, 0x14);
  write(0x72, 0x7A); // 6D 48    A1  7A 18.08
  write(0x73, 0xE1); // 32 00    84  E1

  //设置锁频
  setSyncLockRssi();

  //设置中心频点
  freqSet(476.0, 0, 0);

  //设置发射功率
  setPA(DBM20);
  fs();
  _is_initialized = true;
}

void RfSystem::printRegisters() {
  for (auto i = 0; i <= 0x7f; i++) {
    printf("r(0x%02x)=0x%02x \n", i, read(i));
  }
}

/// should be 0
uint8_t RfSystem::version() {
  auto tmp = read(0x04);
  // only need first two bit
  return tmp & 0b11;
};

bool RfSystem::rxFlag() const {
  return _rx_flag;
}

void RfSystem::resetRxFlag() {
  _rx_flag = false;
}

bool RfSystem::setPins(pin_size_t rst_pin, pin_size_t cs_pin, pin_size_t irq_pin, pin_size_t sdn_pin) {
  if (_is_initialized) {
    return false;
  }
  RST_PIN = rst_pin;
  CS_PIN = cs_pin;
  IRQ_PIN = irq_pin;
  SDN_PIN = sdn_pin;
  return true;
}

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

void RF::printState(const RfState &state) {
  printf("sync_word_rev=%d, preamble_rev=%d, crc_error=%d, pkt_flag=%d, fifo_flag=%d, rx_pkt_state=0x%02x\n",
         state.sync_word_rev,
         state.preamble_rev,
         state.crc_error,
         state.pkt_flag,
         state.fifo_flag,
         state.rx_pkt_state);
}

/// Pkt_flag 分为 4 个功能:前导匹配、同步字匹配、接收或发送包完成。
/// 在 pkt_length_en=1(payload 第 1 个字节为包长度)的情况下，pkt_flag 可设为
/// 同步字匹配或包完成状态，默认为包完成。在 pkt_length_en=0 时，
/// pkt_flag 表示前导匹配或同步字匹配。在发送状态下表示包完成。
///
/// Fifo_flag 表示 FIFO full 或 empty，在发送模块时表示 fifo empty，
/// 在接收模式时表示 fifo full。
struct RfState RfSystem::pollState() {
  auto s = read(0x40);
  uint8_t rx_pkg_st = s & 0b111;
  auto state = RfState{
      .sync_word_rev = shift_equal(s, 7),
      .preamble_rev = shift_equal(s, 6),
      .crc_error = shift_equal(s, 5),
      .pkt_flag = shift_equal(s, 4),
      .fifo_flag = shift_equal(s, 3),
      .rx_pkt_state = rx_pkg_st,
  };
  return state;
}

void RfSystem::scanR(){
  write(0x63, 0xff);
}

void RfSystem::wor(){
  write(0x6a, 0xff);
}
