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



//AT�������к�
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


//����ؽ����״̬
typedef enum{
	NO_RCV  = 0,
	RCV_SUCCESS,
	RCV_TIMEOUT,
	NO_CONNECT,
}tCmdStatus;


typedef struct{
	char *CmdSend;         //���͵�����
	char *CmdRcv; 	       //��ȷ���ذ������ַ���
	uint16_t TimeOut;      //��ʱ��ʱ��
	tCmdStatus CmdStatus;  //����ص�״̬
}tATCmd; 

typedef struct {
	uint8_t DataLen;
	uint8_t RcvBuf  [255];
}tEsp32_RcvBuf;

extern tTimeDelay    ESP32_TimeDelay; 

extern uint8_t Esp32_RcvBuf  [255];
extern uint8_t ESP32_RCV_FLAG;
extern uint8_t NET_MODE ; 
extern uint8_t WIFI_CONNECT_FLAG ;    //WIFI���ӱ�־λ
extern uint8_t BLE_CONNECT_FLAG ;     //BLE���ӱ�־λ


void ESP32_Cmd_Rcv(tATCmdNum ATCmdNum);
tCmdStatus ESP32_Send_Data(uint8_t *SendBuf,uint8_t len);
void ESP32_Init(void);
void ESP32_Data_Rcv(void);

#endif /*__ESP32_H*/

