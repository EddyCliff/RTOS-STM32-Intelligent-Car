
#include "esp32.h"
#include "usart.h"
#include <stdarg.h>
#include <string.h>
#include "connect.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


uint8_t NET_MODE = 0;             //0、蓝牙模式     1、wifi模式   默认蓝牙
uint8_t WIFI_CONNECT_FLAG = 0;    //WIFI连接标志位
uint8_t BLE_CONNECT_FLAG = 0;     //BLE连接标志位

tEsp32_RcvBuf Esp32Rcv;           //ESP数据接收缓冲区


tTimeDelay    ESP32_TimeDelay;


extern 	QueueHandle_t Message_Queue;

#define AT_CWSAP_STRING   "AT+CWSAP=\"FarsightESP32\",\"123456789\",5,3\r\n"
#define AT_BLEADVDATA_STRING    "AT+BLEADVDATA=\"0201060B09466172736967687420030302A0\"\r\n"

volatile tATCmd  ATCmds[20]=
{
  //*CmdSend,              *CmdRcv,    TimeOut,   CmdStatus，  
  {NULL,NULL,0,NO_RCV},
  {"AT\r\n",             "OK",         5000,      NO_RCV, },   //检测AT指令       
  {"AT+CIPAPMAC?\r\n",   "CIPAPMAC",   2000,      NO_RCV, },	 //获取MAC地址    
  {AT_CWSAP_STRING,       "CWSAP" ,    2000,      NO_RCV, },   //建立MAC相关的AP名称  
  {"AT+CWMODE=3\r\n",    "OK" ,        2000,      NO_RCV, },   //设置WIFI模式AP+Station
  {"AT+CIPMUX=1\r\n",    "OK" ,        2000,      NO_RCV, },   //设置多连接
  {"AT+CIPSERVER=1\r\n", "OK" ,        2000,      NO_RCV, },   //初始化TCP服务器 默认IP（192.168.4.1）默认端口号（333）
  {"AT+CIPSTO=0\r\n",    "OK" ,        2000,      NO_RCV, },   //设置TCP连接时间
  {"AT+CIPSEND=0\r\n",   "OK" ,        500,       NO_RCV, },   //TCP发送数据

	{"AT+RST\r\n",          "ready" ,    1000,      NO_RCV, },   //重启AT指令：
  {"AT+BLEINIT=2\r\n",   "OK" ,        1000,      NO_RCV, },   //初始化为 BLE server：
  {"AT+BLEADDR?\r\n",    "BLEADDR" ,   2000,      NO_RCV, },   //查询自身的 BLE 地址
  {AT_BLEADVDATA_STRING,  "OK" ,       2000,      NO_RCV, },   //配置广播数据包
  {"AT+BLEGATTSSRVCRE\r\n",  "OK",     1000,      NO_RCV, },   //创建服务：
  {"AT+BLEGATTSSRVSTART\r\n", "OK" ,   3000,      NO_RCV, },   //开启服务 
  {"AT+BLEADVSTART\r\n",   "OK" ,      1000,      NO_RCV, },   //开始广播
  {"AT+BLEGATTSNTFY\r\n" , ">" ,       500,       NO_RCV, },   //服务器发送数据
	
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
	
		//清空接收缓存以及接收状态

		ATCmds[ATCmdNum].CmdStatus = NO_RCV;
		
		//发送命令
		len = strlen(ATCmds[ATCmdNum].CmdSend);
		uart_data_send((uint8_t *)ATCmds[ATCmdNum].CmdSend, len);
		HAL_UART_Transmit(&huart1,(uint8_t *)ATCmds[ATCmdNum].CmdSend, len,100);

		
	 //打开超时定时器
	 SetTime(&ESP32_TimeDelay, ATCmds[ATCmdNum].TimeOut);
		 
	 while(ATCmds[ATCmdNum].CmdStatus != RCV_SUCCESS)
	 {
		 
			ESP32_Cmd_Rcv(ATCmdNum);
		  if(ATCmds[ATCmdNum].CmdStatus == RCV_TIMEOUT)
				return RCV_TIMEOUT;
	 }
	 
	 return RCV_SUCCESS;
}

/*发送数据函数*/
tCmdStatus ESP32_Send_Data(uint8_t *SendBuf,uint8_t len)
{
	  uint8_t buf[30] =  {0};
		tATCmdNum ATCmdNum;
	
		if(! (BLE_CONNECT_FLAG || WIFI_CONNECT_FLAG))  //未连接状态不能发送数据
		{
			DBG("未连接设备\n");
			return NO_CONNECT;
		}		
		
		if(NET_MODE == BLE_MODE)    //蓝牙模式
		{
			sprintf((char *)buf,"AT+BLEGATTSNTFY=%d,%d,%d,%d\r\n",0,1,2,len);
			ATCmdNum = AT_BLEGATTSNTFY;		
		}
		else 			//WIFI模式
		{
			sprintf((char *)buf,"AT+CIPSEND=%d,%d\r\n",0,len);
			ATCmdNum = AT_CIPSEND;
		}
			
		uart_data_send(buf,strlen((char *)buf));     //发送命令
		
		//打开超时定时器
	 ATCmds[ATCmdNum].CmdStatus = NO_RCV;        //清接收状态
	 SetTime(&ESP32_TimeDelay, ATCmds[ATCmdNum].TimeOut);
		 
	 while(ATCmds[ATCmdNum].CmdStatus != RCV_SUCCESS)
	 {		 
			ESP32_Cmd_Rcv(ATCmdNum);
		  if(ATCmds[ATCmdNum].CmdStatus == RCV_TIMEOUT)
				return RCV_TIMEOUT;
	 }
		
		uart_data_send( SendBuf,len);                //发送数据

		DBG("send data ok\n");
	 
	 return RCV_SUCCESS;

}



void ESP32_Cmd_Rcv(tATCmdNum ATCmdNum)
{
	memset(&Esp32Rcv,0,sizeof(Esp32Rcv));
	
	if(xQueueReceive(Message_Queue, &Esp32Rcv,0 ))
	{
				DBG("%s", Esp32Rcv.RcvBuf);
		
				//接收处理命令
				if(strstr((const char*)Esp32Rcv.RcvBuf,ATCmds[ATCmdNum].CmdRcv) != NULL)
				{
					ATCmds[ATCmdNum].CmdStatus = RCV_SUCCESS;						
				}			
	 
				//打开接收指示灯
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
			 //接收处理数据（保护客户端发来的数据，还有其他调试数据）			

				DBG("%s", Esp32Rcv.RcvBuf);
		
			 if(NET_MODE == BLE_MODE)    //蓝牙模式
			 {
						
						if(strstr((char *)(Esp32Rcv.RcvBuf),"WRITE") != NULL ) //收到客户端数据
						{
								DBG("收到上位机数据\n");
				
								BLE_CONNECT_FLAG = 1;   //对方打开读写特征值时，置连接标志
							
							
								//提取处理数据;
								EP32_RcvData_Extract(Esp32Rcv.RcvBuf,Esp32Rcv.DataLen);	
												
								return ;			          				
						}
						
						if(strstr((char *)(Esp32Rcv.RcvBuf),"BLEDISCONN") != NULL) //客户端断开连接
					 {
							 DBG("蓝牙断开连接，重新广播\n");	
						 
							 BLE_CONNECT_FLAG = 0;    //清除连接标志位
								//重新广播
							 ESPSend_Cmd(AT_BLEADVDATA);
							 ESPSend_Cmd(AT_BLEADVSTART);
					 }

		  }
			else     //WIFI模式
			{
				
				   if((!WIFI_CONNECT_FLAG) && (strstr((char *)(Esp32Rcv.RcvBuf),"CONNECT")!=NULL )) //收到客户端数据
					 {
							DBG("WIFI已连接\n");	
						 
							WIFI_CONNECT_FLAG = 1;   //置连接标志位
					 }

			
				
					if(strstr((char *)(Esp32Rcv.RcvBuf),"+IPD") != NULL  ) //收到客户端数据
				  {
				 		DBG("WIFI收到上位机数据\n");	
						
						//提取并处理数据;
						EP32_RcvData_Extract(Esp32Rcv.RcvBuf,Esp32Rcv.DataLen);	
											
						return ;						
					
				  }	

					if(strstr((char *)(Esp32Rcv.RcvBuf),"CLOSED") != NULL) //客户端断开连接
					 {
							 DBG("WIFI断开连接\n");	
						 
							 WIFI_CONNECT_FLAG = 0;    //清除连接标志位

					 }	
				
			}
			
			
	}
}
	



void ESP32_Init(void)
{
		tATCmdNum i = AT_IDIE;
	
		if(NET_MODE == BLE_MODE)                //蓝牙模式初始化
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
		else                                  //WIFI模式初始化
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
