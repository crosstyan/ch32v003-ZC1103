#include "debug.h"
#include "ch32v00x_spi.h"
#include <printf.h>


void GPIO_Toggle_init() {
  GPIO_InitTypeDef GPIO_InitStructure = {0};
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void init(){
  auto spi_config = SPI_InitTypeDef{
    .SPI_Direction = SPI_Direction_2Lines_FullDuplex,
    .SPI_Mode = SPI_Mode_Master,
    .SPI_DataSize = SPI_DataSize_16b,
    .SPI_CPOL = SPI_CPOL_Low,
    .SPI_CPHA = SPI_CPHA_1Edge,
    .SPI_NSS = SPI_NSS_Soft,
    .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2,
    .SPI_FirstBit = SPI_FirstBit_MSB,
    .SPI_CRCPolynomial = 7,
  };
  SPI_Init(SPI1, &spi_config);
}

int main() {
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  Delay_Init();
  Delay_Ms(100);
  USART_Printf_Init(115200);

  SystemCoreClockUpdate();
  printf("SystemClk:%lu\r\n", SystemCoreClock);

  GPIO_Toggle_init();

  uint8_t i = 0;
  while (true) {
    Delay_Ms(100);
    GPIO_WriteBit(GPIOD, GPIO_Pin_6, static_cast<BitAction>((i == 0) ? (i = Bit_SET) : (i = Bit_RESET)));
  }
}