#ifndef __OLED_H
#define __OLED_H


#include "stm32f407xx.h"
#include "stdlib.h"
#include "sys.h" 

#define OLED_SCLK PDout(3)
#define OLED_SDIN PDout(5)
#define OLED_RS		PDout(1)
#define OLED_RST 	PCout(12)
#define OLED_CS 	PCout(10)


#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define X_WIDTH 	128
#define Y_WIDTH 	64	

#define OLED_CMD 	0
#define OLED_DATA 1

void oled_GPIO_Init(void);
void oled_Init(void);
void OLED_Clear(void);
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
u32 oled_pow(u8 m,u8 n);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Display_Off(void);
void OLED_Display_On(void);
void OLED_WR_Byte(u8 dat,u8 cmd);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowString(u8 x,u8 y,const char *chr, u8 size);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowCHinese(u8 x,u8 y,char (*text)[32], u8 no, u8 mode);
void OLED_ShowFloat(u8 x,u8 y,float num,u8 len,u8 size,u8 mode);
void OLED_ShowC_NMKC(u8 x,u8 y,u8 package,u8 mode);
void OLED_Refresh_Gram(void);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
#endif /*__OLED_H*/

