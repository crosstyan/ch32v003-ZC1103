//
// Created by Kurosu Chan on 2023/6/29.
//

#ifndef SIMPLE_CURRENT_H
#define SIMPLE_CURRENT_H

#include "flash.h"

/// do not frequently update the EEPROM or you may prematurely wear out the flash.
namespace Current {
  const auto OFFSET = 1;
  const auto CURRENT_ID_ADDR = EEPROM_START + OFFSET;
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
    return FlashWrite(CURRENT_ID_ADDR, (u16 *) &current, 1);
  }
}

#endif // SIMPLE_CURRENT_H
