#include "delay.h"
#include "lcd.h"
#include "ld303_bsp.h"
#include "stm32f4xx.h"
#include "sys.h"
#include "timer.h"
#include <stdio.h>
#include <string.h>

#define X_BASE 130
#define Y_BASE 220
#define X_LENGTH 150
#define Y_LENGTH 150
#define INTENS_MAX 15000
#define DISTANCE_MAX 150

void GUI_Init(void);
void GUI_Axes_Display(void);
void GUI_Digital_Display(int distance, int intensity);
void GUI_Axes_Point(int distance, int intensity);

int main(void)
{
    delay_init(168);
    LCD_Init();
    LD303_Init();
    TIM6_Init();
    GUI_Init();

    while (1) {
        if (Timer_ms_Get() >= 100) {
            int distance = LD303_Dis_Get();
            int intensity = LD303_Intensity_Get();

            GUI_Digital_Display(distance,intensity);
            GUI_Axes_Point(distance,intensity);
            Timer_ms_Reset();
        }
    }
}

void GUI_Axes_Point(int distance, int intensity)
{
    if (intensity == 0 || \
        distance == 0 || \
        intensity > INTENS_MAX || \
        distance > DISTANCE_MAX) {
        return;
    }

    int x = (int)(distance + 0.5);
    int y = (int)(intensity / 100 + 0.5);

    LCD_Fast_DrawPoint(X_BASE + x, Y_BASE - y, RED);
}

void GUI_Init(void)
{
    GUI_Axes_Display();
    Draw_Font24B(80, 20, BLACK, (u8*)"LD303 ≤‚æ‡∑÷Œˆ");
    Draw_Font16B(20, 100, BLACK, (u8*)"æ‡¿Î:");
    Draw_Font16B(20, 150, BLACK, (u8*)"«ø∂»:");
}
void GUI_Digital_Display(int distance, int intensity)
{
    char c_distance[10] = { 0 };
    char c_intensity[10] = { 0 };
    char dis_unit[] = { 'c', 'm' };
    char intens_unit[] = { 'k' };

    sprintf(c_distance, "%d", distance);
    sprintf(c_intensity, "%d", intensity);
    strcat(c_distance, dis_unit);
    strcat(c_intensity, intens_unit);

    Draw_Font16B(60, 100, BLUE, (u8*)"        "); 
    Draw_Font16B(60, 150, RED, (u8*)"        ");
    Draw_Font16B(60, 100, BLUE, (u8*)c_distance);
    Draw_Font16B(60, 150, RED, (u8*)c_intensity);
}

void GUI_Axes_Display(void)
{
    LCD_DrawLine(X_BASE, Y_BASE, X_BASE, Y_BASE - Y_LENGTH, BLACK);
    LCD_DrawLine(X_BASE, Y_BASE, X_BASE + X_LENGTH, Y_BASE, BLACK);
    Draw_Font16B(X_BASE + X_LENGTH, Y_BASE, BLUE, (u8*)"S");
    Draw_Font16B(X_BASE - 10, Y_BASE - Y_LENGTH, BLUE, (u8*)"I");
}
