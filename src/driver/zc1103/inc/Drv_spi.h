#ifndef _DRV_SPI_H
#define _DRV_SPI_H

#include "sys.h"

#define SPI_SCLK_RCCLK              RCC_APB2Periph_GPIOA
#define SPI_SCLK_Port               GPIOA
#define SPI_SCLK_Pin                GPIO_Pin_5
            
#define SPI_NSS_RCCLK               RCC_APB2Periph_GPIOC
#define SPI_NSS_Port                GPIOC
#define SPI_NSS_Pin                 GPIO_Pin_4
            
#define SPI_MISO_RCCLK              RCC_APB2Periph_GPIOA
#define SPI_MISO_Port               GPIOA
#define SPI_MISO_Pin                GPIO_Pin_6

#define SPI_MOSI_RCCLK              RCC_APB2Periph_GPIOA
#define SPI_MOSI_Port               GPIOA
#define SPI_MOSI_Pin                GPIO_Pin_7

#define SPI_NSS_OUT_High            do{GPIO_SetBits( SPI_NSS_Port , SPI_NSS_Pin );delay_us(100);}while(0) 
#define SPI_NSS_OUT_Low             do{GPIO_ResetBits( SPI_NSS_Port , SPI_NSS_Pin);delay_us(100);}while(0) 

#define SPI_MOSI_OUT_High           GPIO_SetBits( SPI_MOSI_Port , SPI_MOSI_Pin)  
#define SPI_MOSI_OUT_Low            GPIO_ResetBits( SPI_MOSI_Port , SPI_MOSI_Pin)  

#define SPI_SCK_OUT_High            GPIO_SetBits( SPI_SCLK_Port , SPI_SCLK_Pin)  
#define SPI_SCK_OUT_Low             GPIO_ResetBits( SPI_SCLK_Port , SPI_SCLK_Pin)  

#define SPI_READ_MISO_Value         GPIO_ReadInputDataBit( SPI_MISO_Port , SPI_MISO_Pin)

#define MISO_READ					PAin(6)

void SWSPI_GPIO_Init( void );
uint8_t drv_spi_read_write_byte( uint8_t dat );
uint8_t Spi_Read_Write_Byte( uint8_t txDat );

#endif
