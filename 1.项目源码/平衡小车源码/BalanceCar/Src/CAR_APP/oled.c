#include "oled.h"
#include "oledfont.h" 

u8 OLED_GRAM[128][8];//用于显示CCD宽度数据的缓存
/***************************************************************************************************************
*函数名:OLED_WR_Byte()
*功能:oled读写一个字节
*形参:(u8 dat):数据/(u8 cmd):命令
*返回值:无
***************************************************************************************************************/
void OLED_WR_Byte(u8 dat,u8 cmd)
{
	u8 i;
	OLED_CS=0;
	if(cmd)
		OLED_RS=1;
	else
		OLED_RS=0;
	for(i=0;i<8;i++){
		OLED_SCLK=0;
		if(dat&0x80)
			OLED_SDIN=1;
		else
			OLED_SDIN=0;
		
		OLED_SCLK=1;
		dat<<=1;
	}
	OLED_RS=1;
	OLED_CS=1;
}

/*************************************************************************************************************
*函数名:OLED_Display_On()
*功能:开启显示
*形参:无
*返回值:无
**************************************************************************************************************/
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  
	OLED_WR_Byte(0X14,OLED_CMD);  
	OLED_WR_Byte(0XAF,OLED_CMD); 
}

/**************************************************************************************************************
*函数名:OLED_Display_Off()
*功能:关闭显示
*形参:无
*返回值:无
***************************************************************************************************************/
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD); 
	OLED_WR_Byte(0X10,OLED_CMD);  
	OLED_WR_Byte(0XAE,OLED_CMD);  
}	

/**************************************************************************************************************
*函数名:OLED_ShowChar()
*功能:显示一个字符
*形参:(u8 x,u8 y):的位置[x:0~128,y:0~7]/(u8 chr):要显示的字符/(u8 size):字符大小
*返回值：无
**************************************************************************************************************/
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size)
{      	
	unsigned char c=0,i=0;	
		c=chr-' ';	
		if(x>Max_Column-1){x=0;y=y+2;}
		if(size ==8)
		{
			OLED_Set_Pos(x,y);	
			for(i=0;i<8;i++)
			OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
			OLED_Set_Pos(x,y+1);
			for(i=0;i<8;i++)
			OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
		}
		else 
		{	
				OLED_Set_Pos(x,y);
				for(i=0;i<6;i++)
				OLED_WR_Byte(F6x8[c][i],OLED_DATA);
		}
}
/**************************************************************************************************************
*函数名:OLED_Refresh_Gram()
*功能:显示OLED_GRAM[][]中缓存的数据(用于显示CCD宽度和位置)
*形参:无 
*返回值:无
**************************************************************************************************************/
void OLED_Refresh_Gram(void)
{
	u8 i,n;				
		for(i=0;i<8;i++){
		OLED_WR_Byte (0xb0+7,OLED_CMD);    
		OLED_WR_Byte (0x00,OLED_CMD);     
		OLED_WR_Byte (0x10,OLED_CMD);      
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA); 
	}
}

/**************************************************************************************************************
*函数名:OLED_DrawPoint()
*功能:向OLED_GRAM[][]写入数据
*形参:(u8 x,u8 y):要显示的位置,(u8 t):数据(1:填充、2:取消)
*返回值:无
**************************************************************************************************************/
void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	if(x>127||y>63)return;
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}

/**************************************************************************************************************
*函数名:oled_pow()
*功能:用于计算m^n
*形参:m,n
*返回值:计算之后的结果
**************************************************************************************************************/
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}	

/*************************************************************************************************************
*函数名:OLED_ShowC_NMKC()
*功能:显示汉字
*形参:(u8 x,u8 y):要显示的位置/(u8 package):汉字包/(u8 mode):显示模式
*返回值:无
*************************************************************************************************************/
void OLED_ShowC_NMKC(u8 x,u8 y,u8 package,u8 mode)
{
	if(package==1){
	OLED_ShowCHinese(x,y,&CSXS[0],0,mode);
	OLED_ShowCHinese(x+16,y,&CSXS[0],1,mode);
	OLED_ShowCHinese(x+32,y,&CSXS[0],2,mode);
	OLED_ShowCHinese(x+48,y,&CSXS[0],3,mode);
	}
	if(package==2){
	OLED_ShowCHinese(x,y,&MSSZ[0],0,mode);
	OLED_ShowCHinese(x+16,y,&MSSZ[0],1,mode);
	OLED_ShowCHinese(x+32,y,&MSSZ[0],2,mode);
	OLED_ShowCHinese(x+48,y,&MSSZ[0],3,mode);
	}
	if(package==3){
	OLED_ShowCHinese(x,y,&SDSD[0],0,mode);
	OLED_ShowCHinese(x+16,y,&SDSD[0],1,mode);
	OLED_ShowCHinese(x+32,y,&SDSD[0],2,mode);
	OLED_ShowCHinese(x+48,y,&SDSD[0],3,mode);
	}
	if(package==4){
	OLED_ShowCHinese(x,y,&TXSZ[0],0,mode);
	OLED_ShowCHinese(x+16,y,&TXSZ[0],1,mode);
	OLED_ShowCHinese(x+32,y,&TXSZ[0],2,mode);
	OLED_ShowCHinese(x+48,y,&TXSZ[0],3,mode);
	}
	if(package==5){
	OLED_ShowCHinese(x,y,&HQYJ[0],1,mode);
	OLED_ShowCHinese(x+16,y,&HQYJ[0],2,mode);
	OLED_ShowCHinese(x+32,y,&HQYJ[0],3,mode);
	OLED_ShowCHinese(x+48,y,&HQYJ[0],4,mode);
	}
	if(package==6){
	OLED_ShowCHinese(x,y,&ZXSZ[0],0,mode);
	OLED_ShowCHinese(x+16,y,&ZXSZ[0],1,mode);
	OLED_ShowCHinese(x+32,y,&ZXSZ[0],2,mode);
	OLED_ShowCHinese(x+48,y,&ZXSZ[0],3,mode);
	}
	if(package==7){
	OLED_ShowCHinese(x,y,&SDZ[0],0,mode);
	OLED_ShowCHinese(x+16,y,&SDZ[0],1,mode);
	OLED_ShowCHinese(x+32,y,&SDZ[0],2,mode);
	OLED_ShowCHinese(x+48,y,&SDZ[0],3,mode);
	}
	if(package==8){
	OLED_ShowCHinese(x,y,&SDZ[0],0,mode);
	OLED_ShowCHinese(x+16,y,&SDZ[0],1,mode);
	OLED_ShowCHinese(x+32,y,&SDZ[0],2,mode);
	OLED_ShowCHinese(x+48,y,&SDZ[0],3,mode);
	}
	if(package==9){
	OLED_ShowCHinese(x,y,&PHMS[0],0,mode);
	OLED_ShowCHinese(x+16,y,&PHMS[0],1,mode);
	OLED_ShowCHinese(x+32,y,&PHMS[0],2,mode);
	OLED_ShowCHinese(x+48,y,&PHMS[0],3,mode);
	}
	if(package==10){
	OLED_ShowCHinese(x,y,&XXMS[0],0,mode);
	OLED_ShowCHinese(x+16,y,&XXMS[0],1,mode);
	OLED_ShowCHinese(x+32,y,&XXMS[0],2,mode);
	OLED_ShowCHinese(x+48,y,&XXMS[0],3,mode);
	}
	if(package==11){
	OLED_ShowCHinese(x,y,&BZMS[0],0,mode);
	OLED_ShowCHinese(x+16,y,&BZMS[0],1,mode);
	OLED_ShowCHinese(x+32,y,&BZMS[0],2,mode);
	OLED_ShowCHinese(x+48,y,&BZMS[0],3,mode);
	}
}

/*************************************************************************************************************
*函数名:OLED_ShowCHinese()
*功能:显示单一汉字
*形参:(u8 x,u8 y):显示位置/(char (*text)[32])汉字包/(u8 no):包中的第几个汉字/(u8 mode):显示模式
*返回值:无
*************************************************************************************************************/
void OLED_ShowCHinese(u8 x,u8 y,char (*text)[32], u8 no, u8 mode)
{      			    
	u8 t=0;
	y = y * 2;
	OLED_Set_Pos(x,y);	
	for(t=0;t<16;t++)
	{
		if(mode)
			OLED_WR_Byte(~ *(*(text+2*no)+t),OLED_DATA);
		else
			OLED_WR_Byte(*(*(text+2*no)+t),OLED_DATA);
	 }	
	OLED_Set_Pos(x,y+1);	
	for(t=0;t<16;t++)
	{	
		if(mode)
			OLED_WR_Byte(~ *(*(text+2*no+1)+t),OLED_DATA);
		else
			OLED_WR_Byte(*(*(text+2*no+1)+t),OLED_DATA);
	}					
}

void OLED_ShowFloat(u8 x,u8 y,float num,u8 len,u8 size,u8 mode)
{
	float temp;
	if(num>=0){
		temp=num;
		OLED_ShowChar(x,y,'+',size);
	}
	else{
		temp=-num;
		OLED_ShowChar(x,y,'-',size);
	}
	OLED_ShowNum(x+size,y,(u32)temp,len,size);
	if(mode==0) return;
	temp*=10;
	OLED_ShowChar(x+(size*3),y,'.',size);
	temp=(u32)temp%10;
	OLED_ShowNum(x+(size*3)+6,y,(u32)temp,1,size);
}

/***************************************************************************************************************
*函数名:OLED_ShowNum()
*功能:显示数字
*形参:(u8 x,u8 y):要显示的位置/(u32 num):要显示的数子/(u8 len):位数/(u8 size):字体大小
*返回值:无
***************************************************************************************************************/
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+size*t,y,' ',size);
				continue;
			}
			else 
				enshow=1; 
		}
	 	OLED_ShowChar(x+size*t,y,temp+'0',size); 
	}
} 
/****************************************************************************************************************
*函数名:OLED_DrawBMP()
*功能:显示图片
*形参:
*返回值:无
****************************************************************************************************************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{ 	
 unsigned int j=0;
 unsigned char x,y;
  
  if(y1%8==0) y=y1/8;      
  else y=y1/8+1;
	for(y=y0;y<y1;y++)
	{
		OLED_Set_Pos(x0,y);
    for(x=x0;x<x1;x++)
	    {      
	    	OLED_WR_Byte(BMP[j++],OLED_DATA);	    	
	    }
	}
} 


/**************************************************************************************************************
*函数名:OLED_Clear()
*功能:清屏函数
*形参:无
*返回值:无
**************************************************************************************************************/
void OLED_Clear(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    
		OLED_WR_Byte (0x00,OLED_CMD);     
		OLED_WR_Byte (0x10,OLED_CMD);      
		for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
	} 
}

/**************************************************************************************************************
*函数名:OLED_ShowString()
*功能:显示一个字符串
*形参:(u8 x,u8 y):要显示的位置/(const char *chr):要显示的字符串/(u8 size):字体大小
*返回值:无
**************************************************************************************************************/
void OLED_ShowString(u8 x,u8 y,const char *chr, u8 size)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		
		OLED_ShowChar(x,y,chr[j],size);
		x+=size;
		if(x>128) return;
			//{x=0;y+=2;}
			j++;
	}
}

/***************************************************************************************************************
*函数名:oled_GPIO_Init()
*功能:初始化OLED所需的GPIO 
*形参:无
*返回值:无
***************************************************************************************************************/
void oled_GPIO_Init(void)
{
	
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC10 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD1 PD3 PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	
	GPIOD->BSRR = GPIO_PIN_1;
	GPIOD->BSRR = GPIO_PIN_3;
	GPIOD->BSRR = GPIO_PIN_5;
	GPIOC->BSRR = GPIO_PIN_12;
	GPIOC->BSRR = GPIO_PIN_10;
	OLED_RST=0;
	HAL_Delay(100);
	OLED_RST=1;	
}

/***************************************************************************************************************
*函数名:oled_Mode_Init()
*功能:初始化SSD1306，详细设置请查阅相关手册
*形参:无
*返回值:无
***************************************************************************************************************/
void oled_Mode_Init(void)
{
	OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
	OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
	OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
	OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
	OLED_WR_Byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
	OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0???? 0xa1??
	OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0???? 0xc8??
	OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
	OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
	OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_WR_Byte(0x00,OLED_CMD);//-not offset
	OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
	OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
	OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
	OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
	OLED_WR_Byte(0x12,OLED_CMD);
	OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
	OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
	OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
	OLED_WR_Byte(0x02,OLED_CMD);//
	OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
	OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
	OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
	OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7) 
	OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel
	
	OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/ 
	OLED_Clear();
	OLED_Set_Pos(0,0); 	
}

/**************************************************************************************************************
*函数名:OLED_Set_Pos()
*功能:设置显示坐标
*形参:(unsigned char x, unsigned char y):坐标值
*返回值:无
**************************************************************************************************************/
void OLED_Set_Pos(unsigned char x, unsigned char y) 
{ 
	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f)|0x01,OLED_CMD); 
}  

/*************************************************************************************************************
*函数名:oled_Init()
*功能:初始化OLED对应的GPIO和相应的SSD1306驱动芯片
*形参:无
*返回值:无
*************************************************************************************************************/
void oled_Init(void)
{
	oled_GPIO_Init();
	oled_Mode_Init();
}
/***********************************************END*********************************************************/

