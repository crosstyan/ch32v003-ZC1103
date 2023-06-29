//
// Created by Kurosu Chan on 2023/6/29.
// copy and paste from
// https://www.cnblogs.com/Li-Share/p/16793638.html
//

#include "ch32v00x_flash.h"
#ifndef SIMPLE_FLASH_H
#define SIMPLE_FLASH_H
#define FLASH_SIZE 16
#define FLASH_SECTOR_SIZE  64 // in bytes

u16 FlashReadHalfWord(u32 faddr);

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
 * @param NumToWrite
 */
void FlashWriteNoCheck(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite);

void FlashWrite(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite);

#endif // SIMPLE_FLASH_H
