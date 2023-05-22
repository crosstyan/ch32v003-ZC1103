//
// Created by Kurosu Chan on 2023/5/22.
// See also
// https://github.com/crosstyan/ch32v003fun/blob/2aa519b999e90d2de7cc1589ee867941b4284522/examples/systick_irq/systick_irq.c#L8C11-L10
//

#ifndef SIMPLE_CLOCK_H
#define SIMPLE_CLOCK_H

// need to include this before `ch32v003.h` is included
#define SYSTEM_CORE_CLOCK 48000000
#define SYSTICK_USE_HCLK
#define APB_CLOCK SYSTEM_CORE_CLOCK

#endif //SIMPLE_CLOCK_H
