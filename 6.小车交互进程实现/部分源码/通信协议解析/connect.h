#ifndef __CONNECT_H
#define __CONNECT_H

#include <stdint.h>


//功能宏定义
#define MOVEMENT              0X01         //小车动作
#define READINFO              0X02         //读取PID、模式、版本，恢复默认参数
#define SETMODE               0X03         //设置小车工作模式
#define WRITEPID1             0X10         //设置PID1
#define WRITEPID2             0X11         //设置PID2

//小车动作宏定义
#define CAR_STOP              0X00         //停止
#define CAR_FORWARD           0X01         //前进
#define CAR_BACK              0X02         //后退
#define CAR_TURN_LEFT         0X03         //左转
#define CAR_TURN_RIGHT        0X04         //右转


//小车工作模式定义
#define REMOTE_MODE           0X01        //遥控模式
#define	LINE_TRACK_MODE       0X02        //巡线模式
#define AVOID_MODE            0X03        //蔽障模式


//读取小车信息定义
#define READ_ALL_ARG          0X00        //读取所有的数据
#define READ_PID              0X01        //读取PID数据
#define	READ_WORK_MODE        0X02        //读取当前工作模式
#define	READ_VERINFO          0XA0        //读取版本信息
#define	RESTORE_DEFAULT_ARG   0XA1        //恢复默认参数


//定义版本相关信息
#define Hardware_Type           10
#define Hardware_VER            10
#define Software_VER            10
#define Protocol_VER            10

//定义小车上报数据的功能类型
#define CAR_VER										0x00					//版本信息
#define CAR_POSE								  0x01					//姿态
#define CAR_SENSER								0x02					//传感器原始数据
#define CAR_RCDATA								0x03					//小车接收到的遥控数据
#define CAR_POWER									0x04					//小车电量
#define CAR_MOTO									0x05					//电机转速
#define CAR_SENSER2								0x06					//超声波距离
#define CAR_MOD										0X07					//小车模式
#define CAR_PID_1									0x10					//PID1数据
#define CAR_PID_2									0x11					//PID2的数据
#define CAR_PID_3									0X12					//PID3的数据
#define CAR_CCD_SEN								0XF1					//CCD的数据 
#define CAR_User_Waveform					0xA1	




typedef struct 
{
		u8 send_check;
		u8 send_version;
		u8 send_status;
		u8 send_senser;
		u8 send_senser2;
		u8 send_pid1;
		u8 send_pid2;
		u8 send_pid3;
		u8 send_pid4;
		u8 send_pid5;
		u8 send_pid6;
		u8 send_rcdata;
		u8 send_offset;
		u8 send_motopwm;
		u8 send_power;
		u8 send_user;
		u8 send_speed;
		u8 send_location;

}dt_flag_t;

u8 EP32_RcvData_Extract(const uint8_t *Buff,int len);
void Connect_Send_data(u8 CMD_Data);

#endif

