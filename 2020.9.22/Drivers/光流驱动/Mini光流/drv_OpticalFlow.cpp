#include "drv_Uart5.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"

typedef struct
{   //迷你光流模块数据40ms更新一次，光流数据有更新时，数据从UART-TX发出
	int16_t flow_x_integral;	// X轴从最后一次数据更新到当前更新的位移
	int16_t flow_y_integral;	// Y轴从最后一次数据更新到当前更新的位移
	uint8_t flow_sum;	// 光流数据累加和取低8位
	uint8_t quality;	//图像质量0到100，30以下数据较差建议舍弃
}__PACKED _Flow;

static const unsigned char packet_ID[2] = { 0xfe , 0x04 };
static void OpticalFlow_Server(void* pvParameters)
{
	/*状态机*/
		_Flow  Flow;
		unsigned char rc_counter = 0;
		signed char sum = 0;
	/*状态机*/
	
	while(1)
	{
		uint8_t rdata;
		if( Read_Uart5( &rdata, 1, 2, 0.5 ) )
		{
			if( rc_counter < 2 )
			{
				//接收包头
				if( rdata != packet_ID[ rc_counter ] )
					rc_counter = 0;
				else
				{
					++rc_counter;
					sum = 0;
				}
			}
			else if( rc_counter < 8 )
			{	//接收数据
				( (unsigned char*)&Flow )[ rc_counter - 2 ] = rdata;
				if(rc_counter < 6)
				sum += (signed char)rdata;
				++rc_counter;
			}
			else
			{	//接收包尾
				if( rdata == 0xAA && Flow.quality > 30 )
				{
					sum = 0 ;
					PosSensorHealthInf1 ZRange_inf;
					if( get_OptimalRange_Z( &ZRange_inf ) )
					{	//测距传感器可用
						if( ZRange_inf.last_healthy_TIME.is_valid() && ZRange_inf.last_healthy_TIME.get_pass_time() < 50 )
						{	//测距50秒内健康
							//获取高度
							double height = ZRange_inf.HOffset + ZRange_inf.PositionENU.z;
							//获取角速度
							vector3<double> AngularRate;
							get_AngularRate_Ctrl( &AngularRate );
							//获取倾角
							Quaternion quat_flow;
							get_Airframe_quat( &quat_flow );
							double lean_cosin = quat_flow.get_lean_angle_cosin();
							//补偿光流
							vector3<double> vel;
							#define OKp 400
							double rotation_compensation_x = -constrain( AngularRate.y * OKp , 4500000000.0 );
							double rotation_compensation_y = constrain(  AngularRate.x * OKp , 4500000000.0 );						
							//求积分时间
							Position_Sensor sensor;
							GetPositionSensor( default_optical_flow_index, &sensor );
							double integral_time = sensor.last_update_time.get_pass_time();
							if( integral_time > 1e-3 )
							{
								double freq = 1.0 / integral_time;
								vel.x = ( -(double)Flow.flow_x_integral*freq - rotation_compensation_x ) * (1.0/OKp) * ( 1 + height );
								vel.y = ( (double)Flow.flow_y_integral*freq - rotation_compensation_y ) * (1.0/OKp) * ( 1 + height );
								PositionSensorUpdateVel( default_optical_flow_index , vel , true );
							}
							else
								PositionSensorSetInavailable( default_optical_flow_index );
						}
						else
							PositionSensorSetInavailable( default_optical_flow_index );
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
	//波特率19200
	SetBaudRate_Uart5( 19200, 2, 2 );
	//注册传感器
	PositionSensorRegister( default_optical_flow_index , \
													Position_Sensor_Type_RelativePositioning , \
													Position_Sensor_DataType_v_xy , \
													Position_Sensor_frame_BodyHeading , \
													0.08, 100 );
	xTaskCreate( OpticalFlow_Server, "OpticalFlow", 1024, NULL, SysPriority_ExtSensor, NULL);
}