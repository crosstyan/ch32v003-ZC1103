//
// Created by Kurosu Chan on 2023/6/29.
// copy and paste from
// https://www.cnblogs.com/Li-Share/p/16793638.html
//

#include "ch32v00x_flash.h"
#ifndef SIMPLE_FLASH_H
#define SIMPLE_FLASH_H

// Magic variable provided by the linker script
extern "C" uint8_t _EEPROM_start;
extern "C" uint8_t _EEPROM_end;

u16 FlashReadHalfWord(u32 addr);

/**
 * @brief read flash
 * @param ReadAddr
 * @param pBuffer
 * @param NumToRead
 */
void FlashRead(u32 ReadAddr, u16 *pBuffer, u16 NumToRead);

/**
 * @brief write flash without check
 * @param WriteAddr
 * @param pBuffer
 * @param size
 */
FLASH_Status FlashWriteNoCheck(u32 WriteAddr, u16 *pBuffer, u16 size);

FLASH_Status FlashWrite(u32 WriteAddr, u16 *pBuffer, u16 size);

#endif // SIMPLE_FLASH_H
