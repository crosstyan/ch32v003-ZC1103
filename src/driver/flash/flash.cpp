//
// Created by Kurosu Chan on 2023/6/29.
//
#include "flash.h"

u16 FlashReadHalfWord(u32 addr) {
  return *(vu16 *)addr;
}

void FlashRead(u32 ReadAddr, u16 *pBuffer, u16 NumToRead) {
  for (auto i = 0; i < NumToRead; i++) {
    pBuffer[i] = FlashReadHalfWord(ReadAddr); // 读取2个字节.
    ReadAddr += 2;                            // 偏移2个字节.
  }
}

FLASH_Status FlashWriteNoCheck(u32 WriteAddr, u16 *pBuffer, u16 size) {
  u16 i;
  for (i = 0; i < size; i++) {
    auto res = FLASH_ProgramHalfWord(WriteAddr, pBuffer[i]);
    if (res != FLASH_COMPLETE) {
      return res;
    }
    WriteAddr += 2; // 地址增加2.
  }
  return FLASH_COMPLETE;
}

FLASH_Status FlashWrite(u32 WriteAddr, u16 *pBuffer, u16 size) {
  FLASH_Unlock();
  auto start_addr = reinterpret_cast<uint32_t>(&_EEPROM_start);
  auto end_addr   = reinterpret_cast<uint32_t>(&_EEPROM_end);
  if (WriteAddr < start_addr || WriteAddr > end_addr) {
    FLASH_Lock();
    return FLASH_ERROR_PG;
  } else {
    auto res = FlashWriteNoCheck(WriteAddr, pBuffer, size);
    FLASH_Lock();
    return res;
  }
}
