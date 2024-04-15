#include "inv_mpu_user.h"


#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

//q30��ʽ,longתfloatʱ�ĳ���.
#define q30  1073741824.0f


//�����Ƿ�������
static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};

																					 
//����ת��
unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

//�����Ƿ������
unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar; 
    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    return scalar;
}

u8 mpu_dmp_init(void)
{
	u8 res=0;
	IIC_Init(); 		//��ʼ��IIC����
	if(mpu_init()==0)	//��ʼ��MPU6050
	{	 
		res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);//��������Ҫ�Ĵ�����
		if(res)return 1; 
		res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);//����FIFO
		if(res)return 2; 
		res=mpu_set_sample_rate(DEFAULT_MPU_HZ);	//���ò�����
		if(res)return 3; 
		res=dmp_load_motion_driver_firmware();		//����dmp�̼�
		if(res)return 4; 
		res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//���������Ƿ���
		if(res)return 5; 
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	//����dmp����
		    DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
		    DMP_FEATURE_GYRO_CAL);
		if(res)return 6; 
		res=dmp_set_fifo_rate(DEFAULT_MPU_HZ);	//����DMP�������(��󲻳���200Hz)
		if(res)return 7;   
//		res=run_self_test();		//�Լ�
//		if(res)return 8;    
		res=mpu_set_dmp_state(1);	//ʹ��DMP
		if(res)return 9;     
	}
	printf("init ok");
	return 0;
}
/***************************************************************************************************************
*������:mpu_dmp_get_data()
*����:�õ�dmp����������(ע��,��������Ҫ�Ƚ϶��ջ,�ֲ������е��)
*�β�:(struct _out_angle *angle):DMP����õ�����̬
*����ֵ��0���ɹ�/1��DMP_FIFO��ȡʧ��/2:���ݶ�ȡʧ��
***************************************************************************************************************/

//������Щȫ�ֱ������ⲿ��ʹ��

float pitch,roll,yaw;
u8 mpu_dmp_get_data(void)
{
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

	short gyro_dmp[3], accel_dmp[3], sensors_dmp;
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4]; 

	if(dmp_read_fifo(gyro_dmp, accel_dmp, quat, &sensor_timestamp, &sensors_dmp,&more)) return 1;	

//			printf("%d\n",dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more));
	/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
	**/
	/*if (sensors & INV_XYZ_GYRO )
	send_packet(PACKET_TYPE_GYRO, gyro);
	if (sensors & INV_XYZ_ACCEL)
	send_packet(PACKET_TYPE_ACCEL, accel); */
	/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
	**/
	if(sensors_dmp&INV_WXYZ_QUAT) 
	{
		q0 = quat[0] / q30;	//q30��ʽת��Ϊ������
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30; 
		//����õ�������/�����/�����
		roll = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;																	// pitch
		pitch  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
		yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
	
	}else return 2;
	
	
	return 0;
}

