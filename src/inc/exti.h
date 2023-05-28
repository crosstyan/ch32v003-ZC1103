//
// Created by Kurosu Chan on 2023/5/26.
//

#ifndef SIMPLE_EXTI_H
#define SIMPLE_EXTI_H

#include "ch32v003fun.h"

void configureEXTI();
extern "C" void EXTI7_0_IRQHandler( void ) __attribute__((interrupt));

#endif //SIMPLE_EXTI_H
