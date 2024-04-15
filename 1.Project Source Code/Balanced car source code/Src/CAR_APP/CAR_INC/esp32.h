#ifndef __ESP32_H
#define __ESP32_H

#include <stdint.h>
#include <stdio.h>
#include "delay.h"

extern uint8_t Esp32_RcvBuf [255];
extern uint8_t Esp32_RBuffLen ; 

//#define DEBUG 

#ifdef DEBUG 
#define DBG(x...)   printf(x) 
#else 
#define DBG(x...) 
#endif  



#define  BLE_MODE   0
#define  WIFI_MODE  1



//AT命令序列号
typedef enum
{
	AT_IDIE  = 0,
	AT,
	AT_CIPAPMAC,
	AT_CWSAP,
	AT_CWMODE,
	AT_CIPMUX,
	AT_CIPSERVER,
	AT_CIPSTO,
	AT_CIPSEND,
	
	AT_RST,
	AT_BLEINIT,
	AT_BLEADDR,
	AT_BLEADVDATA,
	AT_BLEGATTSSRVCRE,
	AT_BLEGATTSSRVSTART,
	AT_BLEADVSTART,
	AT_BLEGATTSNTFY,
	CMDSTR_NOUSE,

}tATCmdNum;


//命令返回结果的状态
typedef enum{
	NO_RCV  = 0,
	RCV_SUCCESS,
	RCV_TIMEOUT,
	NO_CONNECT,
}tCmdStatus;


typedef struct{
	char *CmdSend;         //发送的命令
	char *CmdRcv; 	       //正确返回包含的字符串
	uint16_t TimeOut;      //超时的时间
	tCmdStatus CmdStatus;  //命令返回的状态
}tATCmd; 

typedef struct {
	uint8_t DataLen;
	uint8_t RcvBuf  [255];
}tEsp32_RcvBuf;

extern tTimeDelay    ESP32_TimeDelay; 

extern uint8_t Esp32_RcvBuf  [255];
extern uint8_t ESP32_RCV_FLAG;
extern uint8_t NET_MODE ; 
extern uint8_t WIFI_CONNECT_FLAG ;    //WIFI连接标志位
extern uint8_t BLE_CONNECT_FLAG ;     //BLE连接标志位


void ESP32_Cmd_Rcv(tATCmdNum ATCmdNum);
tCmdStatus ESP32_Send_Data(uint8_t *SendBuf,uint8_t len);
void ESP32_Init(void);
void ESP32_Data_Rcv(void);

#endif /*__ESP32_H*/

