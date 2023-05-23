#include "spi.h"
#include "clock.h"
#include "ch32v003fun.h"
#include "system_tick.h"
#include "gpio.h"
#include "utils.h"
#include "instant.h"
#include <etl/string.h>
#include <etl/to_string.h>
#include "rfsystem.h"
#include <printf.h>

static const pin_size_t IRQ_PIN = GPIO::C3;
static const pin_size_t SDN_PIN = GPIO::C2;
static const pin_size_t CS_PIN = GPIO::C4;
static const pin_size_t RST_PIN = GPIO::C0;

int main() {
  SystemInit48HSI();
  SysTick_init();
  SetupDebugPrintf();
  printf("restart\n");

  pin_size_t LED_pin = GPIO::D6;
  pinMode(LED_pin, OUTPUT);

  auto rf = RfSystem(RST_PIN, CS_PIN, IRQ_PIN, SDN_PIN);
  rf.begin();

  // expect 0x00
  auto version = rf.version();
  printf("version=%d\n", version);

  auto instant = Instant();
  while (true) {
    #ifdef TX
    auto d = std::chrono::duration<uint64_t, std::milli>(500);
    if (instant.elapsed() >= d) {
      // construct a payload
      etl::string<32> payload = "hello world:";
      auto r = utils::rand_range(0, 100);
      etl::to_string(r, payload, true);
      payload.append("\n");

      rf.dataPackageSend(payload.c_str(), payload.length());
      rf.refreshStatus();
      auto status = rf.getStatus();
      RF::printStatus(status);
      instant.reset();
    }
    #else // RX
    auto d = std::chrono::duration<uint64_t, std::milli>(1000);
    if (instant.elapsed() > d) {
      rf.refreshStatus();
      auto status = rf.getStatus();
      RF::printStatus(status);
      instant.reset();
    }
    etl::string<256> buf;
    auto maybe = rf.packageRecv(buf.data());
    if (maybe) {
      buf.resize(maybe.value());
      printf("recv: %s\n", buf.c_str());
      rf.resetRxFlag();
    }
    #endif
  }
}
