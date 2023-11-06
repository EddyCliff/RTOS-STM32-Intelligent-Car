#include "i2c.h"
#include "delay.h"

void IIC_Init(void)
{	
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
 
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	//����SCL �� SDA ��ֹ��������״��
	IIC_SCL=1;
	IIC_SDA=1;
}
	

/**************************************************************************************************************
*������:I2C_Start()
*����:����IIC��ʼ�ź�
*�β�:��
*����ֵ:��
**************************************************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//��ʼ�ź�(ʱ���߸ߵ�ƽ���������ɸ߱��)
	delay_us(4);
	IIC_SCL=0;//���ͻ�������� 
}	  
/**************************************************************************************************************
*������:I2C_Stop()
*����:����IICֹͣ�ź�
*�β�:��
*����ֵ:��
**************************************************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//ֹͣ�źţ�ʱ���߸ߵ�ƽ���������ɵͱ�ߣ�
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}
/**************************************************************************************************************
*������:I2C_Wait_Ack()
*����:�ȴ�IIC���豸������Ӧ�ź�
*�β�:��
*����ֵ:0:�õ���Ӧ/1:û�еõ���Ӧ
**************************************************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
/**************************************************************************************************************
*������:I2C_Ack()
*����:����һ��IICӦ���ź�
*�β�:��
*����ֵ:��
**************************************************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
/**************************************************************************************************************
*������:I2C_NAck()
*����:����һ��IIC��Ӧ���ź�
*�β�:��
*����ֵ:��
***************************************************************************************************************/	   		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
/**************************************************************************************************************
*������:I2C_Send_Byte()
*����:IIC�ֽ�д
*�β�:(u8 txd):�Ĵ�����ַ
*����ֵ:��
**************************************************************************************************************/					  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
/**************************************************************************************************************
*������:I2C_Read_Byte()
*����:��ȡһ���ֽ�
*�β�:�Ĵ�����ַ
*����ֵ:��ȡ����IIC����
**************************************************************************************************************/   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}


