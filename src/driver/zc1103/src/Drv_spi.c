#include "Drv_spi.h"
#include "delay.h"

/*--射频模块与单片机接线
    SX1268   STM32
    SCK         PA5     --输出
    MISO        PA6     --输入
    MOSI        PA7     --输出
    NSS         PA4     --片选
    
  */
/*----SPI GPIO 初始化----*/
void SWSPI_GPIO_Init( void )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    //MISO;
    RCC_APB2PeriphClockCmd( SPI_MISO_RCCLK  ,ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = SPI_MISO_Pin;
    GPIO_Init( SPI_MISO_Port ,&GPIO_InitStructure);  
    
    //MOSI 
    RCC_APB2PeriphClockCmd( SPI_MOSI_RCCLK  ,ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = SPI_MOSI_Pin;
    GPIO_Init( SPI_MOSI_Port ,&GPIO_InitStructure);
    //SCLK 
    RCC_APB2PeriphClockCmd( SPI_SCLK_RCCLK ,ENABLE);
    GPIO_InitStructure.GPIO_Pin = SPI_SCLK_Pin;
    GPIO_Init( SPI_SCLK_Port ,&GPIO_InitStructure);
    //NSS
    RCC_APB2PeriphClockCmd( SPI_NSS_RCCLK ,ENABLE);
    GPIO_InitStructure.GPIO_Pin = SPI_NSS_Pin;
    GPIO_Init( SPI_NSS_Port ,&GPIO_InitStructure);
}

///*----软件 SPI ----*/

uint8_t Spi_Read_Write_Byte( uint8_t txDat )
{
	uint8_t i ;
        
	for( i = 0 ; i < 8 ; i++ )
	{
		if( txDat & 0X80 )
		{	SPI_MOSI_OUT_High;}
		else
		{	SPI_MOSI_OUT_Low;}
		txDat <<= 1;
		
		SPI_SCK_OUT_High;
		delay_us(100);
        
		if( SPI_READ_MISO_Value )
			txDat |= 0X01;
        
		SPI_SCK_OUT_Low;
        delay_us(100);
	}
	return txDat;
}
