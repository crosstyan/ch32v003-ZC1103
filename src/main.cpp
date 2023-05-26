#include "spi.h"
#include "clock.h"
#include "ch32v003fun.h"
#include "system_tick.h"
#include "gpio.h"
#include "instant.h"
#include <etl/vector.h>
#include "rfsystem.h"
#include <printf.h>

/*
 * @brief Configure the EXTI for GPIO pin C3
 * @see https://github.com/cnlohr/ch32v003fun/blob/master/examples/exti_pin_change_isr/exti_pin_change_isr.c
 */
void static configureEXTI(){
  asm volatile(
#if __GNUC__ > 10
      ".option arch, +zicsr\n"
      #endif
      "addi t1, x0, 3\n"
      "csrrw x0, 0x804, t1\n"
      : : :  "t1" );
  // Enable GPIOs
  // AFIOEN: I/O auxiliary function module clock enable bit.
  RCC->APB2PCENR = RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO;
  // GPIO C3 for input pin change.
  constexpr uint8_t CNF_AND_MODE_WIDTH = 4; // 4 bytes for each pin.
  GPIOC->CFGLR |= (GPIO_SPEED_IN | GPIO_CNF_IN_PUPD)<<(CNF_AND_MODE_WIDTH * 3);
  GPIOC->CFGLR |= (GPIO_CNF_IN_PUPD)<<(CNF_AND_MODE_WIDTH * 1);  // Keep SWIO enabled.
  // GPIO and Alternate function (AFIO)
  // Configure the IO as an interrupt.
  // (x=0-7), external interrupt input pin configuration bit.
  // Used to determine to which port pins the external interrupt pins are mapped.
  // 00: xth pin of the PA pin.
  // 10: xth pin of the PC pin.
  // 11: xth pin of the PD pin.
  constexpr uint8_t EXTICR_EXTIx_WIDTH = 2;
  AFIO->EXTICR = 0b10<<(EXTICR_EXTIx_WIDTH * 3);
  EXTI->INTENR = 1<<3; // Enable EXT3
  EXTI->RTENR = 1<<3;  // Rising edge trigger

  // enable interrupt
  NVIC_EnableIRQ( EXTI7_0_IRQn );
}

/// won't add trailing `LF` or `CRLF` and caller should decide whether to add one.
void static printWithSize(const char *str, size_t size, bool hex = false) {
  for (size_t i = 0; i < size; i++) {
    if (hex) {
      // https://stackoverflow.com/questions/61518810/print-the-value-of-a-pointer-in-hex-format-without-printf
      uint8_t hi = (str[i] >> 4) & 0xf;
      uint8_t lo = str[i] & 0xf;
      uint8_t tmp[2] = {hi, lo};

      tmp[0] += hi < 10 ? '0' : 'a' - 10;
      tmp[1] += lo < 10 ? '0' : 'a' - 10;
      putchar(tmp[0]);
      putchar(tmp[1]);
    } else {
      putchar(str[i]);
    }
  }
};

static const pin_size_t PKT_FLAG_PIN = GPIO::C3;
static const pin_size_t SDN_PIN = GPIO::C2;
static const pin_size_t CS_PIN = GPIO::C4;
static const pin_size_t RST_PIN = GPIO::C1;

int main() {
  SystemInit48HSI();
  SysTick_init();
  SetupDebugPrintf();
  printf("[INFO] booting\n");

  pin_size_t LED_pin = GPIO::D6;
  pinMode(LED_pin, OUTPUT);
  configureEXTI();

  auto &rf = RfSystem::get();
  auto success = rf.setPins(RST_PIN, CS_PIN, PKT_FLAG_PIN, SDN_PIN);
  if (!success) {
    printf("[ERROR] failed to set pins\n");
  }
  rf.begin();

  // expect to be 0x03
  auto version = rf.version();
  printf("[INFO] version=%d\n", version);
  rf.printRegisters();
//  #define TX
  #ifdef TX
  printf("[INFO] TX mode\n");
  #else
  printf("RX mode\n");
  rf.wor();
  rf.rx();
  #endif

  while (true) {
    #ifdef TX
    auto d = std::chrono::duration<uint16_t , std::milli>(1000);
    if (instant.elapsed() >= d) {
      // construct a payload
      etl::string<32> payload = "hello world:";
      auto r = utils::rand_range(0, 65535);
      etl::to_string(r, payload, true);
      payload.append("\r\n");
      auto status = rf.pollStatus();
      if (!status.tx) {
        rf.tx();
      }
      auto res = rf.send(payload.c_str(), payload.length());
      if (!res.has_value()){
        printf("[ERROR] TX timeout\n");
      }
      printWithSize(payload.c_str(), payload.length());
      digitalWrite(GPIO::D6, HIGH);
      Delay_Ms(10);
      digitalWrite(GPIO::D6, LOW);
      auto state = rf.pollState();
      RF::printState(state);
      instant.reset();
    }
    #else // RX
    // See also `exti.cpp`
    if (RF::rxFlag()) {
      digitalWrite(GPIO::D6, HIGH);
      auto state = rf.pollState();
      if (state.crc_error) {
        printf("[ERROR] CRC error\n");
      }
      etl::vector<char, 256> buf;
      // when a valid packet is received the state should be 0xc0
      // (at least the rx_pkt_state would be 0x00)
      // (sync_word_rev = 1, preamble_rev = 1) but the pkg_flag is useless
      // one should only use interrupt to detect the packet
      if (state.rx_pkt_state != RF::NO_PACKET_RECEIVED) {
        if (auto maybe = rf.recv(buf.data(), [&buf](size_t s) { buf.resize(s); })) {
          printf("len=%d; ", buf.size());
          printf("buf=");
          printWithSize(buf.cbegin(), buf.size());
          if (*(buf.end() - 1) != '\n') {
            printf("\n");
          }
          rf.clrRxFifo();
          RF::setRxFlag(false);
          rf.wor();
        }
      }
      digitalWrite(GPIO::D6, LOW);
    }
    #endif
  }
}
