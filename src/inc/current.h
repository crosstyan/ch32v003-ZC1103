//
// Created by Kurosu Chan on 2023/6/29.
//

#ifndef SIMPLE_CURRENT_H
#define SIMPLE_CURRENT_H

#include "flash.h"
#include "error.h"
#include "etl/expected.h"

/// do not frequently update the EEPROM or you may prematurely wear out the flash.
namespace Current {
const auto SET_CURRENT_MAGIC = 0x86;
const auto OFFSET            = 1;
const auto CURRENT_ID_ADDR   = EEPROM_START + OFFSET;
/**
 * @brief get current from flash
 * @return int16_t
 */
int16_t get() {
  return FlashReadHalfWord(CURRENT_ID_ADDR);
}

/**
 * @brief set current to flash
 * @param current id
 */
FLASH_Status set(int16_t current) {
  return FlashWrite(CURRENT_ID_ADDR, (u16 *)&current, 1);
}

/**
 * @brief decode
 * @param bytes
 * @return etl::expected<uint16_t, CurrentParseStatus>
 * @see https://github.com/crosstyan/ch32v003-ZC1103/blob/cnl/docs/protocol/set_current.ksy
 */
etl::expected<uint16_t, ParseResult> fromBytes(u8 *bytes) {
  auto magic = bytes[0];
  if (magic != SET_CURRENT_MAGIC) {
    auto ue = etl::unexpected<ParseResult>(ParseResult::MAGIC_ERROR);
    return etl::expected<uint16_t, ParseResult>(std::move(ue));
  }
  auto current = __ntohs(*(u16 *)(bytes + 1));
  // check for absurd value
  if (current > 400) {
    auto ue = etl::unexpected<ParseResult>(ParseResult::VALUE_ERROR);
    return etl::expected<uint16_t, ParseResult>(std::move(ue));
  }
  return etl::expected<uint16_t, ParseResult>(current);
}

}

#endif // SIMPLE_CURRENT_H
