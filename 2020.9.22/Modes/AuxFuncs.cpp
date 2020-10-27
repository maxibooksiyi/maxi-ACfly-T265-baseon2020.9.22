#include "AuxFuncs.hpp"
#include "Basic.hpp"
#include "Parameters.hpp"
#include "Receiver.hpp"
#include "drv_PWMOut.hpp"
#include "StorageSystem.hpp"
#include "MeasurementSystem.hpp"

//����֮ǰͨ��ֵ���ڴ���
static double last_Channel_values[16];
//��̨�Ƿ��Զ�����
static bool GimbalCtrl_atLocked[16];

//��Ƭ���
static uint16_t PhotoIndex = 0;

void init_process_AuxFuncs()
{
	Receiver rc;
	if( getReceiver(&rc) )
	{
		//��λ����֮ǰͨ��
		for( uint8_t i = 0; i < rc.raw_available_channels; ++i )
			last_Channel_values[i] = rc.raw_data[i];
		for( uint8_t i = rc.raw_available_channels; i < 16; ++i )
			last_Channel_values[i] = -200;
	}
	else
	{
		//��λ����֮ǰͨ��
		for( uint8_t i = 0; i < 16; ++i )
			last_Channel_values[i] = -200;
	}
	//��λ�˶��Զ����Ʊ�־
	for( uint8_t i = 0; i < 16; ++i )
		GimbalCtrl_atLocked[i] = false;
}

void process_AuxFuncs(const Receiver* rc)
{
	extern uint8_t gps0_sat;
	AuxFuncsConfig aux_configs;
	ReadParamGroup( "AuxCfg", (uint64_t*)&aux_configs, 0 );

	uint8_t AuxChannelsCount = get_AuxChannelCount();
	for( uint8_t i = 0; i < AuxChannelsCount; ++i )
	{
		uint8_t aux_cfg = ((uint8_t*)&aux_configs)[i*8];
		if( aux_cfg>=1 && aux_cfg<=16 )
		{	//ӳ��ң����ͨ��
			if( rc->available )
			{
				uint8_t ref_chan = aux_cfg - 1;
				if( rc->raw_available_channels > ref_chan )
					Aux_PWM_Out( rc->raw_data[ref_chan], i );
			}
		}
		else if( aux_cfg>=25 && aux_cfg<=40 )
		{	//��ң������Ӧͨ������������Ŵ�����raw_data��
			if( rc->available )
			{
				uint8_t ref_chan = aux_cfg - 25;
				if( rc->raw_available_channels > ref_chan )
				{
					if( fabs(rc->raw_data[ref_chan] - last_Channel_values[ref_chan]) > 15 )
					{	//�������
						if( last_Channel_values[ref_chan] > -100 )							
						{
							AuxCamTakePhoto();
							
							Aux_PWM_Out( 100, i );
							os_delay(0.1);
							Aux_PWM_Out( 0, i );
							
							PosSensorHealthInf2 posInf;
							if( get_OptimalGlobal_XY(&posInf) )
							{
								Position_Sensor gps_sensor;
								if( GetPositionSensor( posInf.sensor_ind, &gps_sensor ) )
								{	
									//��ȡʱ��
									RTC_TimeStruct RTC_Time;
									RTC_Time = Get_RTC_Time();
									//��ȡ��̬
									Quaternion airframe_quat;
									get_Attitude_quat(&airframe_quat);
									airframe_quat.Enu2Ned();
									//��ȡ��γ��
									double lat, lon;
									map_projection_reproject( &posInf.mp, 
										posInf.PositionENU.x+posInf.HOffset.x, 
										posInf.PositionENU.y+posInf.HOffset.y,
										&lat, &lon );
									//��ȡ�ٶ�
									vector3<double> vel;
									get_VelocityENU_Ctrl(&vel);
									//д��pos
									char pos_txt_buf[200];
									int n = sprintf(
										pos_txt_buf,
										"%4d,%04d-%02d-%02d,%02d:%02d:%06.3f,γ:%10.7f,��:%11.7f,��:%7.2f,����:%4d,����:%4.1fm/s,����:%7.2f,����:%7.2f,���:%7.2f\r\n",
										PhotoIndex++,
										RTC_Time.Year, RTC_Time.Month, RTC_Time.Date,
										RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds_f,
										lat,
										lon,
										gps_sensor.position_Global.z*0.01,
										gps0_sat,	//����
										safe_sqrt(vel.x*vel.x+vel.y*vel.y)*0.01,	//����
										rad2degree(airframe_quat.getYaw()),	//����
										rad2degree(airframe_quat.getPitch()),	//����
										rad2degree(airframe_quat.getRoll())	//���
									);
									SDLog_Txt1( pos_txt_buf, n );
								}
								else
								{	//�޶�λ������
									//��ȡʱ��
									RTC_TimeStruct RTC_Time;
									RTC_Time = Get_RTC_Time();
									//д��pos
									char pos_txt_buf[200];
									int n = sprintf(
										pos_txt_buf,
										"%4d,%04d-%02d-%02d,%02d:%02d:%06.3f,γ:%10.7f,��:%11.7f,��:%7.2f,����:%4d,����:%4.1fm/s,����:%7.2f,����:%7.2f,���:%7.2f\r\n",
										PhotoIndex++,
										RTC_Time.Year, RTC_Time.Month, RTC_Time.Date,
										RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds_f,
										360.0,
										360.0,
										0.0,
										gps0_sat,	//����
										0.0,	//����
										0.0,	//����
										0.0,	//����
										0.0	//����
									);
									SDLog_Txt1( pos_txt_buf, n );
								}
							}
							else
							{	//�޶�λ������
								//��ȡʱ��
								RTC_TimeStruct RTC_Time;
								RTC_Time = Get_RTC_Time();
								//д��pos
								char pos_txt_buf[200];
								int n = sprintf(
									pos_txt_buf,
									"%4d,%04d-%02d-%02d,%02d:%02d:%06.3f,γ:%10.7f,��:%11.7f,��:%7.2f,����:%4d,����:%4.1fm/s,����:%7.2f,����:%7.2f,���:%7.2f\r\n",
									PhotoIndex++,
									RTC_Time.Year, RTC_Time.Month, RTC_Time.Date,
									RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds_f,
									360.0,
									360.0,
									0.0,
									gps0_sat,	//����
									0.0,	//����
									0.0,	//����
									0.0,	//����
									0.0	//����
								);
								SDLog_Txt1( pos_txt_buf, n );
							}
						}
						last_Channel_values[ref_chan] = rc->raw_data[ref_chan];
					}
					else
						Aux_PWM_Out( 0, i );
				}
				else
					Aux_PWM_Out( 0, i );
			}
		}
		else if( aux_cfg>=49 && aux_cfg<=64 )
		{	//��ң������Ӧͨ��������̨���ƣ�raw_data��
			if( rc->available )
			{
				uint8_t ref_chan = aux_cfg - 49;
				if( rc->raw_available_channels > ref_chan )
				{
					if( GimbalCtrl_atLocked[i] )
					{	//��̨�Զ�����
						//ͨ���仯������ֵ�ŵ�����̨
						if( fabs(rc->raw_data[ref_chan] - last_Channel_values[ref_chan]) > 10 )
						{
							Aux_PWM_Out( rc->raw_data[ref_chan], i );
							last_Channel_values[ref_chan] = rc->raw_data[ref_chan];
							GimbalCtrl_atLocked[i] = false;
						}
					}
					else
					{	//��̨�ֶ�����
						Aux_PWM_Out( rc->raw_data[ref_chan], i );
						last_Channel_values[ref_chan] = rc->raw_data[ref_chan];
					}
				}
			}
		}
	}
}

bool AuxCamTakePhoto()
{
	extern uint8_t gps0_sat;
	PosSensorHealthInf2 posInf;
	if( get_OptimalGlobal_XY(&posInf) )
	{
		Position_Sensor gps_sensor;
		if( GetPositionSensor( posInf.sensor_ind, &gps_sensor ) )
		{
			/*��ָ��Auxͨ������PWM�����ź�*/
				AuxFuncsConfig aux_configs;
				ReadParamGroup( "AuxCfg", (uint64_t*)&aux_configs, 0 );
				uint8_t AuxChannelsCount = get_AuxChannelCount();
				for( uint8_t i = 0; i < AuxChannelsCount; ++i )
				{
					uint8_t aux_cfg = ((uint8_t*)&aux_configs)[i*8];
					if( aux_cfg>=25 && aux_cfg<=40 )
						Aux_PWM_Out( 100, i );
				}			
				os_delay(0.1);
				for( uint8_t i = 0; i < AuxChannelsCount; ++i )
				{
					uint8_t aux_cfg = ((uint8_t*)&aux_configs)[i*8];
					if( aux_cfg>=25 && aux_cfg<=40 )
						Aux_PWM_Out( 0, i );
				}
			/*��ָ��Auxͨ������PWM�����ź�*/
			
			/*��¼���յ�����*/
				//��ȡʱ��
				RTC_TimeStruct RTC_Time;
				RTC_Time = Get_RTC_Time();
				//��ȡ��̬
				Quaternion airframe_quat;
				get_Attitude_quat(&airframe_quat);
				airframe_quat.Enu2Ned();
				//��ȡ��γ��
				double lat, lon;
				map_projection_reproject( &posInf.mp, 
					posInf.PositionENU.x+posInf.HOffset.x, 
					posInf.PositionENU.y+posInf.HOffset.y,
					&lat, &lon );
				//��ȡ�ٶ�
				vector3<double> vel;
				get_VelocityENU_Ctrl(&vel);
				//д��pos
				char pos_txt_buf[200];
				int n = sprintf(
					pos_txt_buf,
					"%4d,%04d-%02d-%02d,%02d:%02d:%06.3f,γ:%10.7f,��:%11.7f,��:%7.2f,����:%4d,����:%4.1fm/s,����:%7.2f,����:%7.2f,���:%7.2f\r\n",
					PhotoIndex++,
					RTC_Time.Year, RTC_Time.Month, RTC_Time.Date,
					RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds_f,
					lat,
					lon,
					gps_sensor.position_Global.z*0.01,
					gps0_sat,	//����
					safe_sqrt(vel.x*vel.x+vel.y*vel.y)*0.01,	//����
					rad2degree(airframe_quat.getYaw()),	//����
					rad2degree(airframe_quat.getPitch()),	//����
					rad2degree(airframe_quat.getRoll())	//���
				);
				SDLog_Txt1( pos_txt_buf, n );
			/*��¼���յ�����*/
				
			return true;
		}
	}
	return false;
}

bool AuxGimbalSetAngle( double angle )
{
	AuxFuncsConfig aux_configs;
	ReadParamGroup( "AuxCfg", (uint64_t*)&aux_configs, 0 );
	
	uint8_t AuxChannelsCount = get_AuxChannelCount();
	for( uint8_t i = 0; i < AuxChannelsCount; ++i )
	{
		uint8_t aux_cfg = ((uint8_t*)&aux_configs)[i*8];
		if( aux_cfg>=49 && aux_cfg<=64 )
		{
			Aux_PWM_Out( angle*100/90, i );
			GimbalCtrl_atLocked[i] = true;
		}
	}
	return true;
}

void init_AuxFuncs()
{
	//ע��ͨ�Ų���	
	AuxFuncsConfig initial_cfg;
	initial_cfg.Aux1Func[0] = 0;
	initial_cfg.Aux2Func[0] = 0;
	initial_cfg.Aux3Func[0] = 0;
	initial_cfg.Aux4Func[0] = 0;
	initial_cfg.Aux5Func[0] = 0;
	initial_cfg.Aux6Func[0] = 0;
	initial_cfg.Aux7Func[0] = 0;
	initial_cfg.Aux8Func[0] = 0;
	MAV_PARAM_TYPE param_types[] = {
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
	};
	SName param_names[] = {
		"Aux_1Func" ,
		"Aux_2Func" ,
		"Aux_3Func" ,
		"Aux_4Func" ,
		"Aux_5Func" ,
		"Aux_6Func" ,
		"Aux_7Func" ,
		"Aux_8Func" ,
	};
	ParamGroupRegister( "AuxCfg", 1,8, param_types, param_names, (uint64_t*)&initial_cfg );
}