//
// Created by Kurosu Chan on 2023/5/23.
//
#include "ext.h"
#include "rfsystem.h"

extern "C" void EXTI7_0_IRQHandler(){
  RF::setRxFlag(true);
  // Acknowledge the interrupt
  EXTI->INTFR = 1<<3;
}
