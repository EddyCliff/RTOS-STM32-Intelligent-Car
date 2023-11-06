#ifndef __CONNECT_H
#define __CONNECT_H

#include <stdint.h>


//���ܺ궨��
#define MOVEMENT              0X01         //С������
#define READINFO              0X02         //��ȡPID��ģʽ���汾���ָ�Ĭ�ϲ���
#define SETMODE               0X03         //����С������ģʽ
#define WRITEPID1             0X10         //����PID1
#define WRITEPID2             0X11         //����PID2

//С�������궨��
#define CAR_STOP              0X00         //ֹͣ
#define CAR_FORWARD           0X01         //ǰ��
#define CAR_BACK              0X02         //����
#define CAR_TURN_LEFT         0X03         //��ת
#define CAR_TURN_RIGHT        0X04         //��ת


//С������ģʽ����
#define REMOTE_MODE           0X01        //ң��ģʽ
#define	LINE_TRACK_MODE       0X02        //Ѳ��ģʽ
#define AVOID_MODE            0X03        //����ģʽ


//��ȡС����Ϣ����
#define READ_ALL_ARG          0X00        //��ȡ���е�����
#define READ_PID              0X01        //��ȡPID����
#define	READ_WORK_MODE        0X02        //��ȡ��ǰ����ģʽ
#define	READ_VERINFO          0XA0        //��ȡ�汾��Ϣ
#define	RESTORE_DEFAULT_ARG   0XA1        //�ָ�Ĭ�ϲ���


//����汾�����Ϣ
#define Hardware_Type           10
#define Hardware_VER            10
#define Software_VER            10
#define Protocol_VER            10

//����С���ϱ����ݵĹ�������
#define CAR_VER										0x00					//�汾��Ϣ
#define CAR_POSE								  0x01					//��̬
#define CAR_SENSER								0x02					//������ԭʼ����
#define CAR_RCDATA								0x03					//С�����յ���ң������
#define CAR_POWER									0x04					//С������
#define CAR_MOTO									0x05					//���ת��
#define CAR_SENSER2								0x06					//����������
#define CAR_MOD										0X07					//С��ģʽ
#define CAR_PID_1									0x10					//PID1����
#define CAR_PID_2									0x11					//PID2������
#define CAR_PID_3									0X12					//PID3������
#define CAR_CCD_SEN								0XF1					//CCD������ 
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

