#include "rfsystem.h"

/// a utility function to check if a bit is set at shift.
/// note that shift is 0-indexed.
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

inline PinStatus RfSystem::pollIrqPin() {
  return digitalRead(this->IRQ_PIN);
}

void RfSystem::reset() {
  RST_LOW();
  Delay_Ms(10);
  RST_HIGH();
  SDN_LOW();
}

inline uint8_t RfSystem::sendByte(uint8_t byte) {
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

uint8_t RfSystem::read(const uint8_t addr) {
  // 0x80 = 0b1000_0000 i.e. set the high bit to 1 to indicate read
  CS_LOW();
  sendByte(addr | 0x80);
  auto data = sendByte(0xff);
  CS_HIGH();
  return data;
}

void RfSystem::registerConfigure() {
  write(0x09, 0x08);
  write(0x0c, 0x03);
  // r(0x0e)
  // 0xa1 = 0b10100001
  //             00001 allowed error bit
  //            1 该位选择 pkt_flag 有效 后，是否自动拉低。如果自动拉低, 脉冲宽度大约 1us
  //           0  syncword 中断使能，当接收端收到有效的 同步字 syncword 时产生中断信号
  //          1   preamble 中断使能，当接收端收到有效 的前导码 preamble 时产生中断信号
  write(0x0e, 0xa1);
  // r(0x0f)
  // 0x0a = 0b00001010
  //             00000 发送 FIFO 空门限，发送 FIFO 还剩余字节数低于门限值时会产生 fifo_flag 标志
  write(0x0F, 0x0A);
  write(0x10, 0x54);
  // r(0x1b) WOR
  // 0x25 = 0b00100101
  //          0          Reserved
  //           0         自动唤醒后执行的命令 (0: RX, 1: TX)
  //            1        内部低频 RC 振荡时钟校准使能
  //             0       自动唤醒功能使能
  //              0101   WOR 功能计数器时钟选择 (32KHz/2^n)
  //                     0b0101 = 5 = 32KHz/2^5 = 1KHz i.e. 1ms per tick
  //  write(0x1b, 0b00110101);
  write(0x1b, 0b00100101);

  write(0x20, 0xa4);
  write(0x21, 0x37);
  write(0x22, 0x3a);         /*VCO Config  3a*/  //3a -> 0azhangjun 20200612
  write(0x23, 0x36);         /*SYN Config   bit[7]enable wideband */
  write(0x2F, 0xe0);         // rx rssi threshold
  write(0x2E, 0x00);

  write(0x30, 0x00);         // ber optimize 0x40->0x00 by 20211126 juner
  write(0x31, 0x00);
  write(0x32, 0x00);
  write(0x33, 0x00);
  write(0x34, 0x00);
  write(0x35, 0x00);
  write(0x36, 0x00);

  // r(0x39)
  // 0x74 = 0b01110100
  //          0          Preamble Threshold
  //           1         该功能使能时，接收端找到谱线后一定周期内没有没有收到同步字，则进行复位
  //            1        使能在 100K 以上数据率时自动识别信号 到达时先进行软复位
  //             1       找到谱线后一定周期内没有收到有效的 preamble 则进行接收机复位
  //              -      rest trivial
  write(0x39, 0x74); //enable demode reset
  write(0x3A, 0x61);
  write(0x4a, 0x60);
  write(0x4d, 0x0b);
  write(0x4e, 0x7c); //ber optimize 0x6c->0x7c by 20211126 juner
  write(0x4f, 0xc5);


  write(0x15, 0x21);
  write(0x07, 0x5d);
  write(0x18, 0x20);
  write(0x2a, 0x14);
  write(0x37, 0x99);

  // r(0x06)
  // 0x3a = 0b00111010
  //                 0 HW_TERM_EN
  //                1  PKT_LENGTH_EN
  //               0   DIRECT_MODE
  //              1    FIFO_SHARE_EN
  //             1     SCRAMBLE_EN
  //            1      CRC_EN
  //           0       LENGTH_SEL: 默认为数据包的第一个字节为包长度 (0: 1 byte, 1: 2 bytes)
  //          0        SYNCWORD_LEN: 同步字长度设置
  write(0x06, 0x3a); //数据包设置
  write(0x04, 0x0A); //前导码长度
  write(0x3B, 0x04); //前导码门限
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

void RfSystem::setPA(PowerAmpGain x_dBm) {
  const uint8_t vReg25Tbl_h4[] = {0x3f, 0x38, 0x25, 0x1a, 0x0f, 0x0d, 0x0b, 0x0a, 0x09, 0x08, 0x04, 0x03, 0x86,
                                  0x82, 0x01, 0x01, 0x02, 0x02, 0x00, 0x00, 0x24, 0x20, 0x16, 0x14, 0x11, 0x0d,
                                  0x0d};

  const uint8_t vReg26Tbl_h4[] = {0xb0, 0xb0, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0x9f, 0x9f, 0xbf, 0x8f, 0x80,
                                  0xbf, 0x9f, 0x84, 0x81, 0x80, 0xad, 0x88, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x72,
                                  0x62};


  const uint8_t vReg25Tbl_h31[] = {0xbf, 0xff, 0x1d, 0x1c, 0x0f, 0x0e, 0x07, 0x06, 0x05, 0x04, \
                                    0x03, 0x3f, 0x2f, 0x2f, 0x1d, 0x17, 0x13, 0x11, 0x10, 0x10, \
                                    0x07, 0x05, 0x05, 0x03, 0x03, 0x03, 0x03};

  const uint8_t vReg26Tbl_h31[] = {0x83, 0x81, 0xaf, 0x82, 0x8f, 0x85, 0x95, 0xbf, 0x81, 0xab, \
                                    0x81, 0x68, 0x6b, 0x5c, 0x75, 0x72, 0x7b, 0x79, 0x7a, 0x68, \
                                    0x6f, 0x7c, 0x6a, 0x7f, 0x6e, 0x62, 0x58};
  auto v = version();
  auto idx = static_cast<size_t>(x_dBm);
  if (v == 0x04) {
    write(0x25, vReg25Tbl_h4[idx]);
    write(0x26, vReg26Tbl_h4[idx]);
  } else {
    write(0x25, vReg25Tbl_h31[idx]);
    write(0x26, vReg26Tbl_h31[idx]);
  }
}

void RfSystem::setSyncLockRssi() {
  write(0x3e, read(0x3e) | 0x40);
}

void RfSystem::setVcoFreq(const double freq) {
  auto f = static_cast<uint8_t>(freq * pow(2.0, 20.0));

  uint8_t reg77 = f & 0xFF;
  uint8_t reg76 = (f >> 8) & 0xFF;
  uint8_t reg75 = (f >> 16) & 0xFF;
  uint8_t reg74 = ((f >> 24) & 0xFF) | (read(0x74) & 0xc0);

  auto temp = read(0x00);
  write(0x00, (0x80 | temp));

  write(0x77, reg77);
  write(0x76, reg76);
  write(0x75, reg75);
  write(0x74, reg74);
}

// TODO: find documentation for this
void RfSystem::setFreq(uint8_t N) {
  if (N > 0x7F) {
    return;
  }
  write(0x00, (0x80 | N));
}


void RfSystem::setFreqStep(double step) {
  auto fre = static_cast<uint8_t>(step * pow(2.0, 20.0));
  auto reg3 = fre & 0xFF;
  auto reg2 = (fre >> 8) & 0xFF;
  auto reg1 = (fre >> 16) & 0x7F;
  write(0x03, reg3);
  write(0x02, reg2);
  write(0x01, reg1);
}

void RfSystem::setFreq(const double f0, const uint8_t N, const double step) {
  setVcoFreq(f0);
  setFreq(N);
  setFreqStep(step);
}

inline void RfSystem::clrTxFifoWrPtr() {
  // 0x80 = 0b1000_0000
  write(0x53, 0x80);      /*Reset FIFO write Pointer*/
}

/// the bigger the number the more power
uint8_t RfSystem::rssi() {
  uint8_t r_reg;

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

void RfSystem::readFifo(uint8_t *dst, uint8_t len) {
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

// 收到接收数据命令后，芯片先打开 PLL 及 VCO，进行校准，等待至 PLL 达到要求接收的频率，
// 启用接收器电路(LNA，混频器、及 ADC)，再启用数字解调器的接收模式。
// 直到收到接收到一包数据完成的指示信号或者是 SWOR 功能超时信号，
// 如果是 SWOR 功能超时信号状态，则直接进入 STANDBY 模式;
inline void RfSystem::rx() {
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
RfSystem::send(const char *buffer, const uint8_t size, bool check_tx) {
  if (size > 0) {
    // must clear tx fifo write pointer before writing to fifo
    clrTxFifoWrPtr();
    writeFifoWithSize(buffer, size);
    tx();
    // check if tx mode entered
    auto counter = 0;
    if (check_tx) {
      while (!this->pollStatus().tx) {
        if (counter > 31) {
          return etl::nullopt;
        }
        counter += 1;
      }
    }
    // don't check pkg_flag in register
    // since that flag only works for GPIO pin output
  }
  auto u = Unit{};
  return etl::make_optional(u);
}

etl::optional<size_t>
RfSystem::recv(char *buf) {
  //清接收FIFO读指针
  clrRxFifoRdPtr();
  //读取接收FIFO的寄存器映射地址，此为接收到的数据长度
  size_t len = read(0x52 | 0x80);
  if (len == 0) {
    rx();
    return etl::nullopt;
  } else {
    readFifo(reinterpret_cast<uint8_t *>(buf), len);
    rx();
    return etl::make_optional(len);
  }
}

void RfSystem::begin() {
  gpioConfigure();

  CS_HIGH();

  spiConfigure();
  Delay_Ms(100);
  reset();
  // a delay is needed after reset
  Delay_Ms(30);

  registerConfigure();

  setDR(RF::DataRate::K9_6);
  setSync(0x41, 0x53, 0x41, 0x53);

  //设置参考频率
  write(0x70, 0x12);
  write(0x71, 0x14);
  write(0x72, 0x7A); // 6D 48    A1  7A 18.08
  write(0x73, 0xE1); // 32 00    84  E1

  //设置锁频
  setSyncLockRssi();

  //设置中心频点
  setFreq(476.0, 0, 0);

  //设置发射功率
  setPA(PowerAmpGain::DBM20);
  fs();
  _is_initialized = true;
}

void RfSystem::printRegisters() {
  for (auto i = 0; i <= 0x7f; i++) {
    printf("r(0x%02x)=0x%02x \n", i, read(i));
  }
}

uint8_t RfSystem::version() {
  auto tmp = read(0x47);
  return tmp & 0x07;
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

uint8_t RfSystem::pollTxPktSt() {
  auto s = read(0x47);
  // 0b0011_1000
  return (s & 0b00111000) >> 3;
}

void RfSystem::scanR() {
  write(0x63, 0xff);
}

void RfSystem::wor() {
  write(0x6a, 0xff);
}

inline void RfSystem::setSync(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4) {
  write(0x11, s1);
  write(0x12, s2);
  write(0x12, s3);
  write(0x12, s4);
}

// hope the compiler can optimize this
void RfSystem::setDR(RF::DataRate data_rate) {
  switch (data_rate) {
    case RF::DataRate::K2_4:
      write(0x08, 0x81);
      write(0x24, 0x19);
      write(0x3D, 0x53);
      write(0x38, 0xb6);
      write(0x3C, 0x91);
      write(0x3E, 0x13);
      write(0x3F, 0x20);
      write(0x58, 0x00);
      write(0x59, 0x11);
      write(0x5A, 0x00);
      write(0x5B, 0x0B);
      write(0x5C, 0x01);
      write(0x5D, 0x60);
      write(0x5e, 0x00);
      write(0x5f, 0x9F);
      write(0x78, 0xC0);
      write(0x79, 0x51);
      write(0x7a, 0xC3);
      write(0x7b, 0x5D);
      write(0x7c, 0xF8);
      write(0x7d, 0x03);
      write(0x7e, 0x04);
      write(0x7f, 0x67);
      write(0x74, 0x9D);
      break;
    case RF::DataRate::K5:
      write(0x08, 0x71);
      write(0x24, 0x19);
      write(0x3D, 0x53);
      write(0x38, 0xA8);
      write(0x3C, 0x91);
      write(0x3E, 0x03);
      write(0x3F, 0x2C);
      write(0x58, 0x00);
      write(0x59, 0x3F);
      write(0x5A, 0x00);
      write(0x5B, 0x14);
      write(0x5C, 0x01);
      write(0x5D, 0x32);
      write(0x5E, 0x00);
      write(0x5F, 0x4F);
      write(0x78, 0xC0);
      write(0x79, 0x3D);
      write(0x7A, 0x71);
      write(0x7B, 0x5D);
      write(0x7C, 0x68);
      write(0x7D, 0x01);
      write(0x7E, 0x00);
      write(0x7F, 0xE1);
      write(0x74, 0x9D);
      break;
    case RF::DataRate::K9_6:
      write(0x08, 0x01);
      write(0x24, 0x19);
      write(0x3D, 0x53);
      write(0x38, 0xA8);
      write(0x3C, 0x91);
      write(0x3E, 0x03);
      write(0x3F, 0x2C);
      write(0x58, 0x00);
      write(0x59, 0x11);
      write(0x5A, 0x00);
      write(0x5B, 0x0B);
      write(0x5C, 0x01);
      write(0x5D, 0x60);
      write(0x5E, 0x00);
      write(0x5F, 0x4F);
      write(0x78, 0xC0);
      write(0x79, 0x99);
      write(0x7A, 0x4D);
      write(0x7B, 0x5A);
      write(0x7C, 0xF8);
      write(0x7D, 0x06);
      write(0x7E, 0x02);
      write(0x7F, 0x2F);
      write(0x74, 0x9D);
      break;
    case RF::DataRate::K10:
      write(0x08, 0x01);
      write(0x24, 0x19);
      write(0x3D, 0x53);
      write(0x38, 0xC6);
      write(0x3C, 0x91);
      write(0x3E, 0x03);
      write(0x3F, 0x08);
      write(0x58, 0x00);
      write(0x59, 0x07);
      write(0x5A, 0x08);
      write(0x5B, 0x09);
      write(0x5C, 0x03);
      write(0x5D, 0x71);
      write(0x5e, 0x00);
      write(0x5f, 0xDF);
      write(0x78, 0x40);
      write(0x79, 0x66);
      write(0x7a, 0x66);
      write(0x7b, 0x5A);
      write(0x7c, 0x78);
      write(0x7d, 0x01);
      write(0x7e, 0x00);
      write(0x7f, 0x70);
      write(0x74, 0x9D);
      break;
    case RF::DataRate::K19_2:
      write(0x08, 0x51);
      write(0x24, 0x19);
      write(0x3D, 0x53);
      write(0x38, 0xA8);
      write(0x3C, 0xC1);
      write(0x3E, 0x03);
      write(0x3F, 0x2C);
      write(0x58, 0x00);
      write(0x59, 0x4F);
      write(0x5A, 0x08);
      write(0x5B, 0x05);
      write(0x5C, 0x0C);
      write(0x5D, 0x71);
      write(0x5E, 0x02);
      write(0x5F, 0x29);
      write(0x78, 0xC0);
      write(0x79, 0x99);
      write(0x7A, 0x4D);
      write(0x7B, 0x5A);
      write(0x7C, 0xF8);
      write(0x7D, 0x0C);
      write(0x7E, 0x02);
      write(0x7F, 0x29);
      write(0x74, 0x9D);
      break;
    case RF::DataRate::K100:
      write(0x08, 0x23);
      write(0x24, 0x19);
      write(0x3D, 0x53);
      write(0x38, 0xA8);
      write(0x3C, 0x81);
      write(0x3E, 0x03);
      write(0x3F, 0x2C);
      write(0x58, 0x02);
      write(0x59, 0x95);
      write(0x5A, 0x08);
      write(0x5B, 0x01);
      write(0x5C, 0x59);
      write(0x5D, 0x53);
      write(0x5E, 0x00);
      write(0x5F, 0xC9);
      write(0x78, 0x40);
      write(0x79, 0x66);
      write(0x7A, 0x66);
      write(0x7B, 0x5B);
      write(0x7C, 0x48);
      write(0x7D, 0x0A);
      write(0x7E, 0x00);
      write(0x7F, 0x67);
      write(0x74, 0x9D);
      break;
    case RF::DataRate::K200:
      write(0x08, 0x36);
      write(0x24, 0x19);
      write(0x3D, 0x53);
      write(0x38, 0xA8);
      write(0x3C, 0xC1);
      write(0x3E, 0x03);
      write(0x3F, 0x2C);
      write(0x58, 0x02);
      write(0x59, 0x95);
      write(0x5A, 0x08);
      write(0x5B, 0x01);
      write(0x5C, 0x59);
      write(0x5D, 0x53);
      write(0x5E, 0x00);
      write(0x5F, 0xC9);
      write(0x78, 0x40);
      write(0x79, 0x66);
      write(0x7A, 0x66);
      write(0x7B, 0x5B);
      write(0x7C, 0x48);
      write(0x7D, 0x0A);
      write(0x7E, 0x00);
      write(0x7F, 0x67);
      write(0x74, 0x9D);
      break;
    case RF::DataRate::K250:
      write(0x08, 0x47);
      write(0x24, 0x19);
      write(0x3D, 0x53);
      write(0x38, 0xA8);
      write(0x3C, 0xC1);
      write(0x3E, 0x03);
      write(0x3F, 0x2C);
      write(0x58, 0x02);
      write(0x59, 0x95);
      write(0x5A, 0x08);
      write(0x5B, 0x01);
      write(0x5C, 0x59);
      write(0x5D, 0x53);
      write(0x5E, 0x00);
      write(0x5F, 0xC9);
      write(0x78, 0x40);
      write(0x79, 0x66);
      write(0x7A, 0x66);
      write(0x7B, 0x5B);
      write(0x7C, 0x48);
      write(0x7D, 0x0A);
      write(0x7E, 0x00);
      write(0x7F, 0x67);
      write(0x74, 0x9D);
    default:
      // unreachable
      break;
  }
}

void RfSystem::setWorTimer(uint16_t t) {
  auto h = static_cast<uint8_t>(t >> 8);
  auto l = static_cast<uint8_t>(t & 0xff);
  write(0x1c, h);
  write(0x1d, l);
}

void RfSystem::setWorRxTimer(uint16_t t) {
  auto h = static_cast<uint8_t>(t >> 8);
  auto l = static_cast<uint8_t>(t & 0xff);
  write(0x1e, h);
  write(0x1f, l);
}

inline void RfSystem::clrRxFifoRdPtr() {
  write(0x51, 0x80);
}
