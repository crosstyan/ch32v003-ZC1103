//
// Created by Kurosu Chan on 2023/5/22.
//

#include "system_tick.h"

/* some bit definitions for systick regs */
#define SYSTICK_SR_CNTIF (1<<0)
#define SYSTICK_CTLR_STE (1<<0)
#define SYSTICK_CTLR_STIE (1<<1)
#define SYSTICK_CTLR_STCLK (1<<2)
#define SYSTICK_CTLR_STRE (1<<3)
#define SYSTICK_CTLR_SWIE (1<<31)

volatile uint32_t systick_cnt;

/*
 * Start up the SysTick IRQ
 */
void SysTick_init() {
  /* disable default SysTick behavior */
  SysTick->CTLR = 0;

  /* enable the SysTick IRQ */
  NVIC_EnableIRQ(SysTicK_IRQn);

  /* Set the tick interval to 1ms for normal op */
  SysTick->CMP = DELAY_MS_TIME - 1;

  /* Start at zero */
  SysTick->CNT = 0;
  systick_cnt = 0;

  /* Enable SysTick counter, IRQ, HCLK/1 */
  SysTick->CTLR = SYSTICK_CTLR_STE | SYSTICK_CTLR_STIE |
                  SYSTICK_CTLR_STCLK;
}

void SysTick_Handler() {
  // move the compare further ahead in time.
  // as a warning, if more than this length of time
  // passes before triggering, you may miss your
  // interrupt.
  SysTick->CMP += DELAY_MS_TIME;

  /* clear IRQ */
  SysTick->SR = 0;

  /* update counter */
  systick_cnt++;
}

uint32_t millis() {
  return systick_cnt;
}
