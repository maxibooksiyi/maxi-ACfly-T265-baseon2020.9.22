#include "drv_Uart5.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"

typedef struct
{
	unsigned char  Reserved;
	unsigned char  Quality;
	signed char Delta_X;
	signed char Delta_Y;
	unsigned char  Reference;
	unsigned char  Lightness;
	signed char Delta_Z;
	signed char Sub_X;
	signed char Sub_Y;
	unsigned char  Timestamp[4];
	signed char  Checksum;
	unsigned char  Packet_footer[2];
}__PACKED _Flow;
static const unsigned char packet_ID[4] = { '#' , 'J' , 'B' , '#' };

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
			if( rc_counter < 4 )
			{
				//���հ�ͷ
				if( rdata != packet_ID[ rc_counter ] )
					rc_counter = 0;
				else
				{
					++rc_counter;
					sum = 0;
				}
			}
			else if( rc_counter <= 17 )
			{	//��������
				( (unsigned char*)&Flow )[ rc_counter - 4 ] = rdata;
				sum += (signed char)rdata;
				++rc_counter;
			}
			else if( rc_counter == 18 )
			{	//У��
				if( sum != 0 || rdata != '\r' )
					rc_counter = 0;
				else
					++rc_counter;
			}
			else
			{	//���հ�β
				if( rdata == '\n' )
				{
					PosSensorHealthInf1 ZRange_inf;
					if( get_OptimalRange_Z( &ZRange_inf ) )
					{	//��ഫ��������
						if( ZRange_inf.last_healthy_TIME.is_valid() && ZRange_inf.last_healthy_TIME.get_pass_time() < 50 )
						{	//���50���ڽ���
							//��ȡ�߶�
							double height = ZRange_inf.HOffset + ZRange_inf.PositionENU.z;
							//��ȡ���ٶ�
							vector3<double> AngularRate;
							get_AngularRate_Ctrl( &AngularRate );
							//��������
							vector3<double> vel;
							float ultra_deadband = height - 1;
              if( ultra_deadband < 0 )
						    ultra_deadband = 0;							
					    double rotation_compensation_x = -constrain( rad2degree( AngularRate.y ) * 0.3 , 45.0 );
					    double rotation_compensation_y = constrain( rad2degree( AngularRate.x ) * 0.3 , 45.0 );
							vel.x = ( (float)Flow.Delta_X - rotation_compensation_x ) * 2.0f * (1 + ultra_deadband/40) ;
							vel.y = ( (float)Flow.Delta_Y - rotation_compensation_y ) * 2.0f * (1 + ultra_deadband/40) ;
							PositionSensorUpdateVel( default_optical_flow_index , vel , true );
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
	//������115200
	SetBaudRate_Uart5( 115200, 2, 2 );
	//ע�ᴫ����
	PositionSensorRegister( default_optical_flow_index , \
													Position_Sensor_Type_RelativePositioning , \
													Position_Sensor_DataType_v_xy , \
													Position_Sensor_frame_BodyHeading , \
													0.1, 100 );
	xTaskCreate( OpticalFlow_Server, "OpticalFlow", 1024, NULL, SysPriority_ExtSensor, NULL);
}