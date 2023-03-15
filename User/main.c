#include "delay.h"
#include "lcd.h"
#include "ld303_bsp.h"
#include "stm32f4xx.h"
#include "sys.h"
#include <stdio.h>

int main(void)
{
    char distance[10] = { 0 };
    char intensity[10] = { 0 };

    delay_init(168); // ��ʼ����ʱ����
    LCD_Init(); // LCD��ʼ��
    LD303_Init();
    Draw_Font24B(70, 20, BLACK, (u8*)"LD303 ������");
    Draw_Font16B(90, 70, BLACK, (u8*)"��������:");
    Draw_Font16B(90, 100, BLACK, (u8*)"�ź�ǿ��:");

    while (1) {
		Draw_Font16B(170, 70, BLUE, (u8*)"              ");
		Draw_Font16B(170, 100, RED, (u8*)"              ");
		sprintf(distance, "%d", LD303_Dis_Get());
		sprintf(intensity, "%d", LD303_Intensity_Get());
		Draw_Font16B(170, 70, BLUE, (u8*)distance);
		Draw_Font16B(170, 100, RED, (u8*)intensity);
		delay_ms(50);	
    }
}
