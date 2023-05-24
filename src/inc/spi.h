//
// Created by Kurosu Chan on 2023/5/22.
// Basically copy and paste of
// https://github.com/crosstyan/ch32v003fun/blob/2aa519b999e90d2de7cc1589ee867941b4284522/examples/spi_24L01_tx/nrf24l01_low_level.c#L6-L10
//

#ifndef SIMPLE_SPI_H
#define SIMPLE_SPI_H

// need to include this before `ch32v003_SPI.h` is included
#define CH32V003_SPI_SPEED_HZ 1000000 // 1MHz
#define CH32V003_SPI_DIRECTION_2LINE_TXRX
// 支持的模式是时钟极性为正，相位极性可选
// 当 ckpha=1 时，为下降沿采样，ckpha=0 时，上升沿采样。
#define CH32V003_SPI_CLK_MODE_POL0_PHA1
#define CH32V003_SPI_NSS_SOFTWARE_ANY_MANUAL
#define CH32V003_SPI_IMPLEMENTATION

#endif //SIMPLE_SPI_H
