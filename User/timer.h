#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f4xx.h"

#define BASIC_TIM           		TIM6
#define BASIC_TIM_CLK       		RCC_APB1Periph_TIM6

#define BASIC_TIM_IRQn					TIM6_DAC_IRQn
#define BASIC_TIM_IRQHandler    TIM6_DAC_IRQHandler


void TIM6_Init(void);
int Timer_ms_Get(void);
void Timer_ms_Reset(void);

#endif /*__TIMER_H*/
