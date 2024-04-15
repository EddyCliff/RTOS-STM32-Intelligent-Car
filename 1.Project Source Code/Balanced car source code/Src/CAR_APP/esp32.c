
#include "esp32.h"
#include "usart.h"
#include <stdarg.h>
#include <string.h>
#include "connect.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


uint8_t NET_MODE = 0;             //0������ģʽ     1��wifiģʽ   Ĭ������
uint8_t WIFI_CONNECT_FLAG = 0;    //WIFI���ӱ�־λ
uint8_t BLE_CONNECT_FLAG = 0;     //BLE���ӱ�־λ

tEsp32_RcvBuf Esp32Rcv;           //ESP���ݽ��ջ�����


tTimeDelay    ESP32_TimeDelay;


extern 	QueueHandle_t Message_Queue;

#define AT_CWSAP_STRING   "AT+CWSAP=\"FarsightESP32\",\"123456789\",5,3\r\n"
#define AT_BLEADVDATA_STRING    "AT+BLEADVDATA=\"0201060B09466172736967687420030302A0\"\r\n"

volatile tATCmd  ATCmds[20]=
{
  //*CmdSend,              *CmdRcv,    TimeOut,   CmdStatus��  
  {NULL,NULL,0,NO_RCV},
  {"AT\r\n",             "OK",         5000,      NO_RCV, },   //���ATָ��       
  {"AT+CIPAPMAC?\r\n",   "CIPAPMAC",   2000,      NO_RCV, },	 //��ȡMAC��ַ    
  {AT_CWSAP_STRING,       "CWSAP" ,    2000,      NO_RCV, },   //����MAC��ص�AP����  
  {"AT+CWMODE=3\r\n",    "OK" ,        2000,      NO_RCV, },   //����WIFIģʽAP+Station
  {"AT+CIPMUX=1\r\n",    "OK" ,        2000,      NO_RCV, },   //���ö�����
  {"AT+CIPSERVER=1\r\n", "OK" ,        2000,      NO_RCV, },   //��ʼ��TCP������ Ĭ��IP��192.168.4.1��Ĭ�϶˿ںţ�333��
  {"AT+CIPSTO=0\r\n",    "OK" ,        2000,      NO_RCV, },   //����TCP����ʱ��
  {"AT+CIPSEND=0\r\n",   "OK" ,        500,       NO_RCV, },   //TCP��������

	{"AT+RST\r\n",          "ready" ,    1000,      NO_RCV, },   //����ATָ�
  {"AT+BLEINIT=2\r\n",   "OK" ,        1000,      NO_RCV, },   //��ʼ��Ϊ BLE server��
  {"AT+BLEADDR?\r\n",    "BLEADDR" ,   2000,      NO_RCV, },   //��ѯ����� BLE ��ַ
  {AT_BLEADVDATA_STRING,  "OK" ,       2000,      NO_RCV, },   //���ù㲥���ݰ�
  {"AT+BLEGATTSSRVCRE\r\n",  "OK",     1000,      NO_RCV, },   //��������
  {"AT+BLEGATTSSRVSTART\r\n", "OK" ,   3000,      NO_RCV, },   //�������� 
  {"AT+BLEADVSTART\r\n",   "OK" ,      1000,      NO_RCV, },   //��ʼ�㲥
  {"AT+BLEGATTSNTFY\r\n" , ">" ,       500,       NO_RCV, },   //��������������
	
  {"CMDSTR_NOUSE",       "OK" ,        2000,      NO_RCV, }, 
};


void uart_data_send(uint8_t *fmt, uint16_t len)
{
	taskENTER_CRITICAL();  
	HAL_UART_Transmit(&huart6, (uint8_t *)fmt, len,100);
	taskEXIT_CRITICAL(); 
}


tCmdStatus ESPSend_Cmd(tATCmdNum ATCmdNum)
{		
		uint8_t len;
	
		//��ս��ջ����Լ�����״̬

		ATCmds[ATCmdNum].CmdStatus = NO_RCV;
		
		//��������
		len = strlen(ATCmds[ATCmdNum].CmdSend);
		uart_data_send((uint8_t *)ATCmds[ATCmdNum].CmdSend, len);
		HAL_UART_Transmit(&huart1,(uint8_t *)ATCmds[ATCmdNum].CmdSend, len,100);

		
	 //�򿪳�ʱ��ʱ��
	 SetTime(&ESP32_TimeDelay, ATCmds[ATCmdNum].TimeOut);
		 
	 while(ATCmds[ATCmdNum].CmdStatus != RCV_SUCCESS)
	 {
		 
			ESP32_Cmd_Rcv(ATCmdNum);
		  if(ATCmds[ATCmdNum].CmdStatus == RCV_TIMEOUT)
				return RCV_TIMEOUT;
	 }
	 
	 return RCV_SUCCESS;
}

/*�������ݺ���*/
tCmdStatus ESP32_Send_Data(uint8_t *SendBuf,uint8_t len)
{
	  uint8_t buf[30] =  {0};
		tATCmdNum ATCmdNum;
	
		if(! (BLE_CONNECT_FLAG || WIFI_CONNECT_FLAG))  //δ����״̬���ܷ�������
		{
			DBG("δ�����豸\n");
			return NO_CONNECT;
		}		
		
		if(NET_MODE == BLE_MODE)    //����ģʽ
		{
			sprintf((char *)buf,"AT+BLEGATTSNTFY=%d,%d,%d,%d\r\n",0,1,2,len);
			ATCmdNum = AT_BLEGATTSNTFY;		
		}
		else 			//WIFIģʽ
		{
			sprintf((char *)buf,"AT+CIPSEND=%d,%d\r\n",0,len);
			ATCmdNum = AT_CIPSEND;
		}
			
		uart_data_send(buf,strlen((char *)buf));     //��������
		
		//�򿪳�ʱ��ʱ��
	 ATCmds[ATCmdNum].CmdStatus = NO_RCV;        //�����״̬
	 SetTime(&ESP32_TimeDelay, ATCmds[ATCmdNum].TimeOut);
		 
	 while(ATCmds[ATCmdNum].CmdStatus != RCV_SUCCESS)
	 {		 
			ESP32_Cmd_Rcv(ATCmdNum);
		  if(ATCmds[ATCmdNum].CmdStatus == RCV_TIMEOUT)
				return RCV_TIMEOUT;
	 }
		
		uart_data_send( SendBuf,len);                //��������

		DBG("send data ok\n");
	 
	 return RCV_SUCCESS;

}



void ESP32_Cmd_Rcv(tATCmdNum ATCmdNum)
{
	memset(&Esp32Rcv,0,sizeof(Esp32Rcv));
	
	if(xQueueReceive(Message_Queue, &Esp32Rcv,0 ))
	{
				DBG("%s", Esp32Rcv.RcvBuf);
		
				//���մ�������
				if(strstr((const char*)Esp32Rcv.RcvBuf,ATCmds[ATCmdNum].CmdRcv) != NULL)
				{
					ATCmds[ATCmdNum].CmdStatus = RCV_SUCCESS;						
				}			
	 
				//�򿪽���ָʾ��
				//SetLedRun(LED_RX);
				
				
	}
	else
	{
			if(CompareTime(&ESP32_TimeDelay))
			{
				ATCmds[ATCmdNum].CmdStatus = RCV_TIMEOUT;
			}
	}	

}

void ESP32_Data_Rcv(void)
{
	memset(&Esp32Rcv,0,sizeof(Esp32Rcv));
	if(xQueueReceive(Message_Queue, &Esp32Rcv,0 ))
	{
			 //���մ������ݣ������ͻ��˷��������ݣ����������������ݣ�			

				DBG("%s", Esp32Rcv.RcvBuf);
		
			 if(NET_MODE == BLE_MODE)    //����ģʽ
			 {
						
						if(strstr((char *)(Esp32Rcv.RcvBuf),"WRITE") != NULL ) //�յ��ͻ�������
						{
								DBG("�յ���λ������\n");
				
								BLE_CONNECT_FLAG = 1;   //�Է��򿪶�д����ֵʱ�������ӱ�־
							
							
								//��ȡ��������;
								EP32_RcvData_Extract(Esp32Rcv.RcvBuf,Esp32Rcv.DataLen);	
												
								return ;			          				
						}
						
						if(strstr((char *)(Esp32Rcv.RcvBuf),"BLEDISCONN") != NULL) //�ͻ��˶Ͽ�����
					 {
							 DBG("�����Ͽ����ӣ����¹㲥\n");	
						 
							 BLE_CONNECT_FLAG = 0;    //������ӱ�־λ
								//���¹㲥
							 ESPSend_Cmd(AT_BLEADVDATA);
							 ESPSend_Cmd(AT_BLEADVSTART);
					 }

		  }
			else     //WIFIģʽ
			{
				
				   if((!WIFI_CONNECT_FLAG) && (strstr((char *)(Esp32Rcv.RcvBuf),"CONNECT")!=NULL )) //�յ��ͻ�������
					 {
							DBG("WIFI������\n");	
						 
							WIFI_CONNECT_FLAG = 1;   //�����ӱ�־λ
					 }

			
				
					if(strstr((char *)(Esp32Rcv.RcvBuf),"+IPD") != NULL  ) //�յ��ͻ�������
				  {
				 		DBG("WIFI�յ���λ������\n");	
						
						//��ȡ����������;
						EP32_RcvData_Extract(Esp32Rcv.RcvBuf,Esp32Rcv.DataLen);	
											
						return ;						
					
				  }	

					if(strstr((char *)(Esp32Rcv.RcvBuf),"CLOSED") != NULL) //�ͻ��˶Ͽ�����
					 {
							 DBG("WIFI�Ͽ�����\n");	
						 
							 WIFI_CONNECT_FLAG = 0;    //������ӱ�־λ

					 }	
				
			}
			
			
	}
}
	



void ESP32_Init(void)
{
		tATCmdNum i = AT_IDIE;
	
		if(NET_MODE == BLE_MODE)                //����ģʽ��ʼ��
		{
			for(i = AT_BLEINIT; i<=AT_BLEADVSTART ; i++)
			{
				if( ESPSend_Cmd(i) != RCV_SUCCESS)
				{				
					DBG("PES32 Init failed\n");
					return ;
				}
				
			}
		}
		else                                  //WIFIģʽ��ʼ��
		{
					for(i = AT; i<=AT_CIPSTO ; i++)
					{						
						if( ESPSend_Cmd(i) != RCV_SUCCESS)
						{
							DBG("PES32 Init failed\n");
							return ;
						}
					}
					
						DBG("PES32 Init Success\n");
		}
}
