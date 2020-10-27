#include "MS_Main.hpp"

#include "MS_Attitude.hpp"
#include "MS_Position.hpp"
#include "Sensors.hpp"
#include "Parameters.hpp"
#include "StorageSystem.hpp"
#include "ControlSystem.hpp"

/*IMU����ʱ�䶨��*/
	double IMU_Accelerometer_UpdateT[ IMU_Sensors_Count ];
	double IMU_Accelerometer_UpdateFreq[ IMU_Sensors_Count ];
	void set_IMU_Accelerometer_UpdateFreq( uint8_t ind , double Freq )
	{
		IMU_Accelerometer_UpdateT[ind] = 1.0 / Freq;
		IMU_Accelerometer_UpdateFreq[ind] = Freq;
	}
	
	double IMU_Gyroscope_UpdateT[ IMU_Sensors_Count ];
	double IMU_Gyroscope_UpdateFreq[ IMU_Sensors_Count ];
	void set_IMU_Gyroscope_UpdateFreq( uint8_t ind , double Freq )
	{
		IMU_Gyroscope_UpdateT[ind] = 1.0 / Freq;
		IMU_Gyroscope_UpdateFreq[ind] = Freq;
	}
/*IMU����ʱ�䶨��*/
	
/*IMU�����ݼ��ٶȣ�����֪ͨ*/
	static TaskHandle_t MS_Main_GyroTaskHandle = 0;
	static TaskHandle_t MS_Main_TaskHandle = 0;

	//���ݸ���֪ͨ
	bool MS_Notify_IMUGyroUpdate( uint8_t ind )
	{
		if( ind >= IMU_Sensors_Count )
			return false;
		if( MS_Main_GyroTaskHandle == 0 )
			return false;
		xTaskNotify( MS_Main_GyroTaskHandle , 1 << ind , eSetBits );
		return true;
	}
	bool MS_Notify_IMUGyroUpdateFromISR( uint8_t ind , BaseType_t *pxHigherPriorityTaskWoken )
	{
		if( ind >= IMU_Sensors_Count )
			return false;
		if( MS_Main_GyroTaskHandle == 0 )
			return false;
		xTaskNotifyFromISR( MS_Main_GyroTaskHandle , 1 << ind , eSetBits , pxHigherPriorityTaskWoken );
		return true;
	}
	
	//���ٶȸ���֪ͨ
	bool MS_Notify_IMUAceelUpdate( uint8_t ind )
	{
		if( ind >= IMU_Sensors_Count )
			return false;
		if( MS_Main_TaskHandle == 0 )
			return false;
		xTaskNotify( MS_Main_TaskHandle , 1 << ind , eSetBits );
		return true;
	}
	bool MS_Notify_IMUAccelUpdateFromISR( uint8_t ind , BaseType_t *pxHigherPriorityTaskWoken )
	{
		if( ind >= IMU_Sensors_Count )
			return false;
		if( MS_Main_TaskHandle == 0 )
			return false;
		xTaskNotifyFromISR( MS_Main_TaskHandle , 1 << ind , eSetBits , pxHigherPriorityTaskWoken );
		return true;
	}
/*IMU�����ݼ��ٶȣ�����֪ͨ*/

void MS_Main_Gyro(void* pvParameters)
{
	//�ȴ���ʼ�����
	while( getInitializationCompleted() == false )
		os_delay(0.1);
	
	while(1)
	{
		//�ȴ���������
		uint32_t notify_value;
		xTaskNotifyWait( pdFALSE , pdTRUE , &notify_value , portMAX_DELAY );

		for( uint8_t i = 0 ; i < IMU_Sensors_Count ; ++i )
		{
			if( notify_value & (1<<i) )
			{
				//�������ݸ���
				//������������
				IMU_Sensor sensor;
				if( GetGyroscope( i , &sensor, 0.01 ) )
				{
					MS_Attitude_GyroIntegral( i , sensor );
				}
			}
		}
	}
}

void MS_Main(void* pvParameters)
{
	//�ȴ���ʼ�����
	while( getInitializationCompleted() == false )
		os_delay(0.1);
	
	uint8_t SDLog_Att_counter = 0;
	uint8_t SDLog_LocalNed_counter = 0;
	while(1)
	{	
		//�ȴ����ٶ�����
		uint32_t notify_value;
		xTaskNotifyWait( pdFALSE , pdTRUE , &notify_value , portMAX_DELAY );
		for( uint8_t i = 0 ; i < IMU_Sensors_Count ; ++i )
		{
			if( notify_value & (1<<i) )
			{
				//���ٶ����ݸ���
				//������ٶ�����
				IMU_Sensor sensor;
				if( GetAccelerometer( i, &sensor, 0.01 ) )
				{
					MS_Attitude( i , sensor );
					MS_Position( i , sensor );
				}
				
				/*���¼�¼����Log*/
					bool inFlight;
					get_is_inFlight(&inFlight);
					if( inFlight && i==0 )
					{
						uint64_t log;
						
						if( ReadParam( "SDLog_Att", 0, 0, (uint64_t*)&log, 0 ) == PR_OK )
						{	//��¼��̬
							if( log )
							{
								if( ++SDLog_Att_counter >= log )
								{
									SDLog_Att_counter = 0;
									SDLog_Msg_AttitudeQuaternion();
								}
							}
						}
						
						if( ReadParam( "SDLog_LocalNed", 0, 0, (uint64_t*)&log, 0 ) == PR_OK )
						{	//��¼λ���ٶ�
							if( log )
							{
								if( ++SDLog_LocalNed_counter >= log )
								{
									SDLog_LocalNed_counter = 0;
									SDLog_Msg_LocalPositionNed();
								}
							}
						}
					}
				/*���¼�¼����Log*/
			}
		}
	}
}

void init_MS_Main()
{
	init_MS_Attitude();
	init_MS_Position();
	
	/*����ע��*/
		//ˮƽУ׼��Ԫ������
		AirframeCalibQuat initial_AirframeCalibQuat = {1,0,0,0};
		MAV_PARAM_TYPE AirframeCalib_param_types[] = { MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64 };
		ParamGroupRegister( "AirframeCalib", 1, 2, AirframeCalib_param_types, 0, (uint64_t*)&initial_AirframeCalibQuat );
		
		//ע�������
		MAV_PARAM_TYPE WGA_param_types[] = { MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64 };
		ParamGroupRegister( "WGA_Code", 2, 2, WGA_param_types, 0, 0 );
	/*����ע��*/
	
	xTaskCreate( MS_Main_Gyro , "MS_Main_Gyro" ,1024, NULL, SysPriority_MeasurementSystem_Integral, &MS_Main_GyroTaskHandle);
	xTaskCreate( MS_Main , "MS_Main" ,4000, NULL, SysPriority_MeasurementSystem, &MS_Main_TaskHandle);
}