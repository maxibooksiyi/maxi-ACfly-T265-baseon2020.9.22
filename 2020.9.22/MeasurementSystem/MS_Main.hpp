#pragma once

#include "FreeRTOS.h"

/*����*/
	//ˮƽУ׼��Ԫ��
	struct AirframeCalibQuat
	{
		float q0,q1,q2,q3;
	}__attribute__((__packed__));
/*����*/

/*IMU����ʱ�䶨��*/
	extern double IMU_Accelerometer_UpdateT[];
	extern double IMU_Accelerometer_UpdateFreq[];
	void set_IMU_Accelerometer_UpdateFreq( uint8_t ind , double Freq );
	
	extern double IMU_Gyroscope_UpdateT[];
	extern double IMU_Gyroscope_UpdateFreq[];
	void set_IMU_Gyroscope_UpdateFreq( uint8_t ind , double Freq );
/*IMU����ʱ�䶨��*/

/*IMU�����ݼ��ٶȣ�����֪ͨ*/
	//���ݸ���֪ͨ
	bool MS_Notify_IMUGyroUpdate( uint8_t ind );
	bool MS_Notify_IMUGyroUpdateFromISR( uint8_t ind , BaseType_t *pxHigherPriorityTaskWoken );
	
	//���ٶȸ���֪ͨ
	bool MS_Notify_IMUAceelUpdate( uint8_t ind );
	bool MS_Notify_IMUAccelUpdateFromISR( uint8_t ind , BaseType_t *pxHigherPriorityTaskWoken );
/*IMU�����ݼ��ٶȣ�����֪ͨ*/

void init_MS_Main();