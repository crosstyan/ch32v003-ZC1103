// #define TX
//  #define DISABLE_LED
// # define DISABLE_STANDBY

#include "funconfig.h"
#include "clock.h"
#include "ch32v003fun.h"
#include "system_tick.h"
#include "gpio.h"
#include "instant.h"
#include "exti.h"
#include "llcc68.h"
#include "message_wrapper.h"
#include "utils.h"
#include <printf.h>
#include "led.h"
#include <etl/random.h>
#include "flags.h"
#include "flash.h"
#include "spot.h"
#include "adc.h"
#include "boring.h"

#ifdef TX
#include <pb_encode.h>
#endif

using namespace RfMessage;

// TODO: what's the IRQ pin?
// DIO2 is connected to IRQ (at PD1)
static const pin_size_t IRQ_PIN   = GPIO::D1;
static const pin_size_t BUSY_PIN  = GPIO::C3;
static const pin_size_t CS_PIN    = GPIO::C4;
static const pin_size_t RST_PIN   = GPIO::C0;
static const pin_size_t RX_EN_PIN = GPIO::C1;
static const pin_size_t TX_EN_PIN = GPIO::C2;

static const auto ADDR_BYTES                     = 3;
static const uint8_t MY_ADDR[ADDR_BYTES]         = {0x01, 0x02, 0x03};
static const uint8_t BROAD_CAST_ADDR[ADDR_BYTES] = {0xFF, 0xFF, 0xFF};

bool isValidAddr(const uint8_t *addr) {
  auto is_bc_addr = memcmp(addr, BROAD_CAST_ADDR, ADDR_BYTES) == 0;
  if (is_bc_addr) {
    return true;
  }
  auto is_my_addr = memcmp(addr, MY_ADDR, ADDR_BYTES) == 0;
  if (is_my_addr) {
    return true;
  }
  return false;
}

int main() {
  // SystemInit48HSI();
  SystemInit();
  SysTick_init();
  // SetupDebugPrintf();
  auto p = GPIO::D6;
  GPIO::pinMode(p, GPIO::OUTPUT);
  auto flip = [](GPIO::PinStatus status){
    if(status == GPIO::HIGH){
      return GPIO::LOW;
    } else {
      return GPIO::HIGH;
    }
  };
  auto s = GPIO::LOW;
  GPIO::digitalWrite(p, s);
  auto counter = 0;
#ifndef DISABLE_STANDBY
  // enable power interface module clock
  RCC->APB1PCENR |= RCC_APB1Periph_PWR;

  // enable low speed oscillator (LSI)
  RCC->RSTSCKR |= RCC_LSION;
  while ((RCC->RSTSCKR & RCC_LSIRDY) == 0) {}

  // enable AutoWakeUp event
  EXTI->EVENR |= EXTI_Line9;
  EXTI->FTENR |= EXTI_Line9;

  // configure AWU prescaler
  // A precision internal low frequency 128KHz crystal LSI can be selected as the auto-wakeup count time base.
  // 128K divided by 4096 = 31.25Hz
  // 128K divided by 1024 = 125Hz
  PWR->AWUPSC |= PWR_AWU_Prescaler_4096;

  // configure AWU window comparison value
  PWR->AWUWR &= ~0x3f;
  // AWUUWR only has 6 bits
  PWR->AWUWR |= 16;

  // enable AWU
  PWR->AWUCSR |= (1 << 1);

  // select standby on power-down
  PWR->CTLR |= PWR_CTLR_PDDS;

  // peripheral interrupt controller send to deep sleep
  PFIC->SCTLR |= (1 << 2);
#endif
  Delay_Ms(250);
  while (true) {
#ifndef DISABLE_STANDBY
    __WFE();
    // restore clock to full speed
    SystemInit();
    printf("[INFO] wake up with %d\n", counter++);
    s = flip(s);
    GPIO::digitalWrite(p, s);
#else
    printf("[INFO] Normal \n");
    s = flip(s);
    GPIO::digitalWrite(p, s);
    Delay_Ms(1000);
#endif
  }
}
