//
// Created by Kurosu Chan on 2023/6/29.
//
#include "flash.h"

u16 FlashReadHalfWord(u32 addr) {
  return *(vu16 *)addr;
}

void FlashRead(u32 ReadAddr, u16 *pBuffer, u16 size) {
  for (auto i = 0; i < size; i++) {
    pBuffer[i] = FlashReadHalfWord(ReadAddr);
    ReadAddr += 2;
  }
}

FLASH_Status FlashWriteNoCheck(u32 WriteAddr, u16 *pBuffer, u16 size) {
  for (auto i = 0; i < size; i++) {
    auto res = FLASH_ProgramHalfWord(WriteAddr, pBuffer[i]);
    if (res != FLASH_COMPLETE) {
      return res;
    }
    WriteAddr += 2;
  }
  return FLASH_COMPLETE;
}

FLASH_Status FlashWrite(u32 WriteAddr, u16 *pBuffer, u16 size) {
  FLASH_Unlock();
  if (WriteAddr < EEPROM_START || WriteAddr > EEPROM_END) {
    FLASH_Lock();
    return FLASH_ERROR_PG;
  } else {
    auto res = FlashWriteNoCheck(WriteAddr, pBuffer, size);
    FLASH_Lock();
    return res;
  }
}
