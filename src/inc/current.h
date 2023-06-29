//
// Created by Kurosu Chan on 2023/6/29.
//

#ifndef SIMPLE_CURRENT_H
#define SIMPLE_CURRENT_H

#include "flash.h"

/// do not frequently update the EEPROM or you may prematurely wear out the flash.
namespace Current {
  const auto OFFSET = 1;
  const auto FLASH_ADDR = FLASH_BASE + OFFSET;
  /**
   * @brief get current from flash
   * @return int16_t
   */
  int16_t get() {
    return FlashReadHalfWord(FLASH_ADDR);
  }

  /**
   * @brief set current to flash
   * @param current
   */
  void set(int16_t current) {
    FlashWrite(FLASH_ADDR, (u16 *) &current, 1);
  }
}

#endif // SIMPLE_CURRENT_H
