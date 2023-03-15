#include "ld303_bsp.h"
#include "stm32f4xx.h"

#define RX_BUFFER_SIZE 13

static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint8_t rx_index = 0;
static uint8_t rx_complete = 0;

int LD303_Dis_Get(void)
{
    return rx_buffer[4] * 256 + rx_buffer[5];
}
int LD303_Intensity_Get(void)
{
    return rx_buffer[8] * 256 + rx_buffer[9];
}
int LD303_Complete_Check(void)
{
    return rx_complete; 
}
void LD303_Complete_Reset(void)
{
    rx_complete = 0;
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        // Check for frame header
        if (rx_index == 0 && USART_ReceiveData(USART2) != 0x55) {
            USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // Enable receive interrupt
            return;
        }
        if (rx_index == 1 && USART_ReceiveData(USART2) != 0xA5) {
            USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // Enable receive interrupt
            rx_index = 0;
            return;
        }

        // Read received byte into buffer
        rx_buffer[rx_index++] = USART_ReceiveData(USART2);

        // Check for complete frame
        if (rx_index == RX_BUFFER_SIZE) {
            uint8_t checksum = 0;
            for (int i = 0; i < RX_BUFFER_SIZE - 1; i++) {
                checksum ^= rx_buffer[i];
            }
            if (checksum == rx_buffer[RX_BUFFER_SIZE - 1]) {
                // Frame is valid
                // Process the received data here
            }
            rx_index = 0;
        }

        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // Enable receive interrupt
    }
}

static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    /* 配置中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = LD303_USART_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  LD303_USART GPIO 配置,工作模式配置。115200 8-N-1 ，中断接收模式
 * @param  无
 * @retval 无
 */
void LD303_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(LD303_USART_RX_GPIO_CLK | LD303_USART_TX_GPIO_CLK, ENABLE);

    /* 使能 UART 时钟 */
    RCC_APB1PeriphClockCmd(LD303_USART_CLK, ENABLE);

    /* 连接 PXx 到 USARTx_Tx*/
    GPIO_PinAFConfig(LD303_USART_RX_GPIO_PORT, LD303_USART_RX_SOURCE, LD303_USART_RX_AF);

    /*  连接 PXx 到 USARTx__Rx*/
    GPIO_PinAFConfig(LD303_USART_TX_GPIO_PORT, LD303_USART_TX_SOURCE, LD303_USART_TX_AF);

    /* 配置Tx引脚为复用功能  */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

    GPIO_InitStructure.GPIO_Pin = LD303_USART_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LD303_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* 配置Rx引脚为复用功能 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = LD303_USART_RX_PIN;
    GPIO_Init(LD303_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    /* 配置串口LD303_USART 模式 */
    USART_InitStructure.USART_BaudRate = LD303_USART_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(LD303_USART, &USART_InitStructure);

    NVIC_Configuration();
    /*配置串口接收中断*/
    USART_ITConfig(LD303_USART, USART_IT_RXNE, ENABLE);

    USART_Cmd(LD303_USART, ENABLE);
}

/*****************  发送一个字符 **********************/
static void LD303_SendByte(USART_TypeDef* pUSARTx, uint8_t ch)
{
    /* 发送一个字节数据到USART2 */
    USART_SendData(pUSARTx, ch);

    /* 等待发送完毕 */
    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET)
        ;
}
/*****************  指定长度的发送字符串 **********************/
void LD303_SendStr_length(USART_TypeDef* pUSARTx, uint8_t* str, uint32_t strlen)
{
    unsigned int k = 0;
    do {
        LD303_SendByte(pUSARTx, *(str + k));
        k++;
    } while (k < strlen);
}

/*****************  发送字符串 **********************/
void LD303_SendString(USART_TypeDef* pUSARTx, uint8_t* str)
{
    unsigned int k = 0;
    do {
        LD303_SendByte(pUSARTx, *(str + k));
        k++;
    } while (*(str + k) != '\0');
}
