//
// Created by Kurosu Chan on 2023/6/29.
//
#include "flash.h"

u16 FLASH_BUF[FLASH_SECTOR_SIZE / 2]; // 最多是 32*2 字节

u16 FlashReadHalfWord(u32 faddr) {
  return *(vu16 *)faddr;
}

void FlashRead(u32 ReadAddr, u16 *pBuffer, u16 NumToRead) {
  for (auto i = 0; i < NumToRead; i++) {
    pBuffer[i] = FlashReadHalfWord(ReadAddr); // 读取2个字节.
    ReadAddr += 2;                            // 偏移2个字节.
  }
}

void FlashWriteNoCheck(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite) {
  u16 i;
  for (i = 0; i < NumToWrite; i++) {
    FLASH_ProgramHalfWord(WriteAddr, pBuffer[i]);
    WriteAddr += 2; // 地址增加2.
  }
}

void FlashWrite(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite) {
  u32 sec_pos;    // 扇区地址
  u16 sec_off;    // 扇区内偏移地址(16位字计算)
  u16 sec_remain; // 扇区内剩余地址(16位字计算)
  u32 off_addr;   // 去掉 0x08000000 后的地址
  // I'm not sure what this global variable is for.
  u32 i;
  if (WriteAddr < FLASH_BASE || (WriteAddr >= (FLASH_BASE + 1024 * FLASH_SIZE))) {
    return;
  }
  FLASH_Unlock();                                                                        // 解锁
  off_addr   = WriteAddr - FLASH_BASE;                                                   // 实际偏移地址.
  sec_pos    = off_addr / FLASH_SECTOR_SIZE;                                             // 扇区地址  0~127 for STM32F103RBT6
  sec_off    = (off_addr % FLASH_SECTOR_SIZE) / 2;                                       // 在扇区内的偏移(2个字节为基本单位.)
  sec_remain = FLASH_SECTOR_SIZE / 2 - sec_off;                                          // 扇区剩余空间大小
  if (NumToWrite <= sec_remain) sec_remain = NumToWrite;                                 // 不大于该扇区范围
  while (true) {
    // 读出整个扇区的内容
    FlashRead(sec_pos * FLASH_SECTOR_SIZE + FLASH_BASE, FLASH_BUF, FLASH_SECTOR_SIZE / 2);
    // 校验数据
    for (i = 0; i < sec_remain; i++) {
      if (FLASH_BUF[sec_off + i] != 0XFFFF) {
        break; // 需要擦除
      }
    }
    // 需要擦除
    if (i < sec_remain) {
      FLASH_ErasePage(sec_pos * FLASH_SECTOR_SIZE + FLASH_BASE); // 擦除这个扇区
      // 复制
      for (i = 0; i < sec_remain; i++) {
        FLASH_BUF[i + sec_off] = pBuffer[i];
      }
      FlashWriteNoCheck(sec_pos * FLASH_SECTOR_SIZE + FLASH_BASE, FLASH_BUF, FLASH_SECTOR_SIZE / 2); // 写入整个扇区
    } else {
      FlashWriteNoCheck(WriteAddr, pBuffer, sec_remain); // 写已经擦除了的,直接写入扇区剩余区间.
    }
    if (NumToWrite == sec_remain) {
      break;
    } else {
      sec_pos++;                     // 扇区地址增1
      sec_off = 0;                   // 偏移位置为0
      pBuffer += sec_remain;         // 指针偏移
      WriteAddr += (sec_remain * 2); // 写地址偏移
      NumToWrite -= sec_remain;      // 字节(16位)数递减
      if (NumToWrite > (FLASH_SECTOR_SIZE / 2))
        sec_remain = FLASH_SECTOR_SIZE / 2; // 下一个扇区还是写不完
      else
        sec_remain = NumToWrite; // 下一个扇区可以写完了
    }
  };
  FLASH_Lock(); // 上锁
}
