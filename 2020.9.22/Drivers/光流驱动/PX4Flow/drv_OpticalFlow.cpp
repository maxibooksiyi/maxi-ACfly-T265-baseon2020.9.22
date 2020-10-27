#include "drv_Uart5.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"

typedef struct
{
	int16_t frame_count;
	int16_t pixel_flow_x_integral;
	int16_t pixel_flow_y_integral;
	int16_t gyro_x_rate_integral;
	int16_t gyro_y_rate_integral;
	int16_t gyro_z_rate_integral;
	int32_t integration_timespan_integral;
	int32_t ultra_timestamp;
	int16_t ground_distance_integral;
	int16_t gyro_temperature;
	uint8_t qual;
}__PACKED _Flow;
static const unsigned char packet_ID[3] = { 0xaa , 0xaf , 0x01 };

static void OpticalFlow_Server(void* pvParameters)
{
	/*״̬��*/
		_Flow  Flow;
		unsigned char rc_counter = 0;
		signed char sum = 0;
	/*״̬��*/
	
	while(1)
	{
		uint8_t rdata;
		if( Read_Uart5( &rdata, 1, 2, 0.5 ) )
		{
			/*״̬��*/
				static _Flow  Flow;
				static unsigned char rc_counter = 0;
				static unsigned char sum = 0;
			/*״̬��*/
			
			if( rc_counter < 3 )
			{
				//���հ�ͷ
				if( rc_counter == 0 )
					sum = 0;
				
				if( rdata != packet_ID[ rc_counter ] )
					rc_counter = 0;
				else
				{
					++rc_counter;
					sum += rdata;
				}
			}
			else if( rc_counter == 3 )
			{
				//�������ݳ���
				if( rdata != 25 )
					rc_counter = 0;
				else
				{
					++rc_counter;
					sum += rdata;
				}
			}
			else if( rc_counter < 4+25 )
			{	//��������
				( (unsigned char*)&Flow )[ rc_counter - 4 ] = rdata;
				sum += rdata;
				++rc_counter;
			}
			else if( rc_counter >= 29 )
			{	//У��
				if( sum == rdata )
				{
					Flow.frame_count = __REV16( Flow.frame_count );
					Flow.pixel_flow_x_integral = __REV16( Flow.pixel_flow_x_integral );
					Flow.pixel_flow_y_integral = __REV16( Flow.pixel_flow_y_integral );
					Flow.gyro_x_rate_integral = __REV16( Flow.gyro_x_rate_integral );
					Flow.gyro_y_rate_integral = __REV16( Flow.gyro_y_rate_integral );
					Flow.gyro_z_rate_integral = __REV16( Flow.gyro_z_rate_integral );
					Flow.integration_timespan_integral = __REV( Flow.integration_timespan_integral );
					Flow.ultra_timestamp = __REV( Flow.ultra_timestamp );
					Flow.ground_distance_integral = __REV16( Flow.ground_distance_integral );
					Flow.gyro_temperature = __REV16( Flow.gyro_temperature );
							
					//���³���������
					vector3<double> position;
					position.z = Flow.ground_distance_integral * 0.1f;
					if( position.z > 1 && position.z < 600 )
					{
						//��ȡ���
						Quaternion quat;
						get_Airframe_quat( &quat );
						double lean_cosin = quat.get_lean_angle_cosin();
						//����
						position.z *= lean_cosin;
						PositionSensorUpdatePosition( default_ultrasonic_sensor_index, position, true );
					}
					else
						PositionSensorSetInavailable( default_ultrasonic_sensor_index );
					
					vector3<double> vel;
					PosSensorHealthInf1 ZRange_inf;
					if( get_PosSensorHealth_Z( &ZRange_inf, default_ultrasonic_sensor_index ) )
					{	//��ഫ��������
						if( ZRange_inf.last_healthy_TIME.is_valid() && ZRange_inf.last_healthy_TIME.get_pass_time() < 50 )
						{	//���50���ڽ���
							//��ȡ�߶�
							double height = ZRange_inf.HOffset + ZRange_inf.PositionENU.z;
							//��ȡ���ٶ�
							vector3<double> AngularRate;
							get_AngularRate_Ctrl( &AngularRate );
							if( Flow.integration_timespan_integral > 0 )
							{
								double freq = 1.0 / ( Flow.integration_timespan_integral * 1e-6 );
								vel.x = -(Flow.pixel_flow_x_integral - Flow.gyro_x_rate_integral)*1e-4*freq * height;
								vel.y = (Flow.pixel_flow_y_integral - Flow.gyro_y_rate_integral)*1e-4*freq * height;
							}
							else
								vel.x = vel.y = 0;
							PositionSensorUpdateVel( default_optical_flow_index , vel , true );
						}
					}
					else
						PositionSensorSetInavailable( default_optical_flow_index );
				}
				rc_counter = 0;
			}
		}
	}
}

void init_drv_OpticalFlow()
{
	//������19200
	SetBaudRate_Uart5( 19200, 2, 2 );
	//ע�ᳬ����������
	PositionSensorRegister( default_ultrasonic_sensor_index , \
													Position_Sensor_Type_RangePositioning , \
													Position_Sensor_DataType_s_z , \
													Position_Sensor_frame_ENU , \
													0.05 , //��ʱ
													0 ,	//xy���ζ�
													0 //z���ζ�
													);
	//ע�����������
	PositionSensorRegister( default_optical_flow_index , \
													Position_Sensor_Type_RelativePositioning , \
													Position_Sensor_DataType_v_xy , \
													Position_Sensor_frame_BodyHeading , \
													0.1, 100 );
	xTaskCreate( OpticalFlow_Server, "OpticalFlow", 1024, NULL, SysPriority_ExtSensor, NULL);
}