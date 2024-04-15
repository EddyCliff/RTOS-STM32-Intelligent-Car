#ifndef _MYIIC_H
#define _MYIIC_H
#include "sys.h"

//IO方向设置
#define SDA_IN()  {GPIOC->MODER&=~(3<<(11*2));GPIOC->MODER|=0<<11*2;}	//PC11输入模式
#define SDA_OUT() {GPIOC->MODER&=~(3<<(11*2));GPIOC->MODER|=1<<11*2;}   //PC11输出模式
//IO操作

#define IIC_SCL    PDout(0) 
//IIC SDA为PC11
#define IIC_SDA    PCout(11)  
//读取SDA对应引脚的电平
#define READ_SDA   PCin(11) 


//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 
#endif

