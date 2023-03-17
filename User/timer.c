#include "timer.h"

static int timer_ms = 0;

static void TIM6_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    // 设置中断组为0
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    // 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannel = BASIC_TIM_IRQn;
    // 设置抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    // 设置子优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
 * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
 * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
 * 另外三个成员是通用定时器和高级定时器才有.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         都有
 * TIM_CounterMode			 TIMx,x[6,7]没有，其他都有（基本定时器）
 * TIM_Period            都有
 * TIM_ClockDivision     TIMx,x[6,7]没有，其他都有(基本定时器)
 * TIM_RepetitionCounter TIMx,x[1,8]才有(高级定时器)
 *-----------------------------------------------------------------------------
 */
static void TIM6_Mode_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // 开启TIMx_CLK,x[6,7]
    RCC_APB1PeriphClockCmd(BASIC_TIM_CLK, ENABLE);

    /* 累计 TIM_Period个后产生一个更新或者中断*/
    // 当定时器从0计数到4999，即为5000次，为一个定时周期
    TIM_TimeBaseStructure.TIM_Period = 10 - 1;

    // 定时器时钟源TIMxCLK = 2 * PCLK1
    //				PCLK1 = HCLK / 4
    //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=84MHz
    // 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=10000Hz
    TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;

    // 初始化定时器TIMx, x[2,3,4,5]
    TIM_TimeBaseInit(BASIC_TIM, &TIM_TimeBaseStructure);

    // 清除定时器更新中断标志位
    TIM_ClearFlag(BASIC_TIM, TIM_FLAG_Update);

    // 开启定时器更新中断
    TIM_ITConfig(BASIC_TIM, TIM_IT_Update, ENABLE);

    // 使能定时器
    TIM_Cmd(BASIC_TIM, ENABLE);
}

/**
 * @brief  初始化基本定时器定时，1ms产生一次中断
 * @param  无
 * @retval 无
 */
void TIM6_Init(void)
{
    TIM6_NVIC_Configuration();

    TIM6_Mode_Config();
}

void BASIC_TIM_IRQHandler(void)
{
    if (TIM_GetITStatus(BASIC_TIM, TIM_IT_Update) != RESET) {
        timer_ms++;
        TIM_ClearITPendingBit(BASIC_TIM, TIM_IT_Update);
    }
}


int Timer_ms_Get(void)
{
    return timer_ms;
}
void Timer_ms_Reset(void)
{
    timer_ms = 0;
}
