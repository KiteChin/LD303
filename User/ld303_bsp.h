#ifndef LD303_BSP_H__
#define LD303_BSP_H__

#include "stm32f4xx.h"

// 引脚定义
/*******************************************************/
#define LD303_USART USART2
#define LD303_USART_CLK RCC_APB1Periph_USART2

#define LD303_USART_RX_GPIO_PORT GPIOA
#define LD303_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define LD303_USART_RX_PIN GPIO_Pin_3
#define LD303_USART_RX_AF GPIO_AF_USART2
#define LD303_USART_RX_SOURCE GPIO_PinSource3

#define LD303_USART_TX_GPIO_PORT GPIOA
#define LD303_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define LD303_USART_TX_PIN GPIO_Pin_2
#define LD303_USART_TX_AF GPIO_AF_USART2
#define LD303_USART_TX_SOURCE GPIO_PinSource2

#define LD303_USART_IRQHandler USART2_IRQHandler
#define LD303_USART_IRQ USART2_IRQn
/************************************************************/

// 串口波特率
#define LD303_USART_BAUDRATE 115200

void LD303_Init(void);
void LD303_SendStr_length(USART_TypeDef* pUSARTx, uint8_t* str, uint32_t strlen);
void LD303_SendString(USART_TypeDef* pUSARTx, uint8_t* str);

int LD303_Dis_Get(void);
int LD303_Intensity_Get(void);
int LD303_Complete_Check(void);
void LD303_Complete_Reset(void);
#endif /*LD303_BSP_H__*/
