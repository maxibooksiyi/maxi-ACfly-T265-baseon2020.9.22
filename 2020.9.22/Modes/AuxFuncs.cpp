#include "AuxFuncs.hpp"
#include "Basic.hpp"
#include "Parameters.hpp"
#include "Receiver.hpp"
#include "drv_PWMOut.hpp"
#include "StorageSystem.hpp"
#include "MeasurementSystem.hpp"

//保存之前通道值用于触发
static double last_Channel_values[16];
//云台是否自动控制
static bool GimbalCtrl_atLocked[16];

//相片序号
static uint16_t PhotoIndex = 0;

void init_process_AuxFuncs()
{
	Receiver rc;
	if( getReceiver(&rc) )
	{
		//复位保存之前通道
		for( uint8_t i = 0; i < rc.raw_available_channels; ++i )
			last_Channel_values[i] = rc.raw_data[i];
		for( uint8_t i = rc.raw_available_channels; i < 16; ++i )
			last_Channel_values[i] = -200;
	}
	else
	{
		//复位保存之前通道
		for( uint8_t i = 0; i < 16; ++i )
			last_Channel_values[i] = -200;
	}
	//复位运动自动控制标志
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
		{	//映射遥控器通道
			if( rc->available )
			{
				uint8_t ref_chan = aux_cfg - 1;
				if( rc->raw_available_channels > ref_chan )
					Aux_PWM_Out( rc->raw_data[ref_chan], i );
			}
		}
		else if( aux_cfg>=25 && aux_cfg<=40 )
		{	//用遥控器对应通道进行相机快门触发（raw_data）
			if( rc->available )
			{
				uint8_t ref_chan = aux_cfg - 25;
				if( rc->raw_available_channels > ref_chan )
				{
					if( fabs(rc->raw_data[ref_chan] - last_Channel_values[ref_chan]) > 15 )
					{	//触发相机
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
									//获取时间
									RTC_TimeStruct RTC_Time;
									RTC_Time = Get_RTC_Time();
									//获取姿态
									Quaternion airframe_quat;
									get_Attitude_quat(&airframe_quat);
									airframe_quat.Enu2Ned();
									//获取经纬度
									double lat, lon;
									map_projection_reproject( &posInf.mp, 
										posInf.PositionENU.x+posInf.HOffset.x, 
										posInf.PositionENU.y+posInf.HOffset.y,
										&lat, &lon );
									//获取速度
									vector3<double> vel;
									get_VelocityENU_Ctrl(&vel);
									//写入pos
									char pos_txt_buf[200];
									int n = sprintf(
										pos_txt_buf,
										"%4d,%04d-%02d-%02d,%02d:%02d:%06.3f,纬:%10.7f,经:%11.7f,高:%7.2f,卫星:%4d,航速:%4.1fm/s,航向:%7.2f,俯仰:%7.2f,横滚:%7.2f\r\n",
										PhotoIndex++,
										RTC_Time.Year, RTC_Time.Month, RTC_Time.Date,
										RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds_f,
										lat,
										lon,
										gps_sensor.position_Global.z*0.01,
										gps0_sat,	//卫星
										safe_sqrt(vel.x*vel.x+vel.y*vel.y)*0.01,	//航速
										rad2degree(airframe_quat.getYaw()),	//航向
										rad2degree(airframe_quat.getPitch()),	//俯仰
										rad2degree(airframe_quat.getRoll())	//横滚
									);
									SDLog_Txt1( pos_txt_buf, n );
								}
								else
								{	//无定位传感器
									//获取时间
									RTC_TimeStruct RTC_Time;
									RTC_Time = Get_RTC_Time();
									//写入pos
									char pos_txt_buf[200];
									int n = sprintf(
										pos_txt_buf,
										"%4d,%04d-%02d-%02d,%02d:%02d:%06.3f,纬:%10.7f,经:%11.7f,高:%7.2f,卫星:%4d,航速:%4.1fm/s,航向:%7.2f,俯仰:%7.2f,横滚:%7.2f\r\n",
										PhotoIndex++,
										RTC_Time.Year, RTC_Time.Month, RTC_Time.Date,
										RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds_f,
										360.0,
										360.0,
										0.0,
										gps0_sat,	//卫星
										0.0,	//航速
										0.0,	//航向
										0.0,	//俯仰
										0.0	//俯仰
									);
									SDLog_Txt1( pos_txt_buf, n );
								}
							}
							else
							{	//无定位传感器
								//获取时间
								RTC_TimeStruct RTC_Time;
								RTC_Time = Get_RTC_Time();
								//写入pos
								char pos_txt_buf[200];
								int n = sprintf(
									pos_txt_buf,
									"%4d,%04d-%02d-%02d,%02d:%02d:%06.3f,纬:%10.7f,经:%11.7f,高:%7.2f,卫星:%4d,航速:%4.1fm/s,航向:%7.2f,俯仰:%7.2f,横滚:%7.2f\r\n",
									PhotoIndex++,
									RTC_Time.Year, RTC_Time.Month, RTC_Time.Date,
									RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds_f,
									360.0,
									360.0,
									0.0,
									gps0_sat,	//卫星
									0.0,	//航速
									0.0,	//航向
									0.0,	//俯仰
									0.0	//俯仰
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
		{	//用遥控器对应通道进行云台控制（raw_data）
			if( rc->available )
			{
				uint8_t ref_chan = aux_cfg - 49;
				if( rc->raw_available_channels > ref_chan )
				{
					if( GimbalCtrl_atLocked[i] )
					{	//云台自动控制
						//通道变化大于阈值才调整云台
						if( fabs(rc->raw_data[ref_chan] - last_Channel_values[ref_chan]) > 10 )
						{
							Aux_PWM_Out( rc->raw_data[ref_chan], i );
							last_Channel_values[ref_chan] = rc->raw_data[ref_chan];
							GimbalCtrl_atLocked[i] = false;
						}
					}
					else
					{	//云台手动控制
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
			/*在指定Aux通道发出PWM触发信号*/
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
			/*在指定Aux通道发出PWM触发信号*/
			
			/*记录拍照点数据*/
				//获取时间
				RTC_TimeStruct RTC_Time;
				RTC_Time = Get_RTC_Time();
				//获取姿态
				Quaternion airframe_quat;
				get_Attitude_quat(&airframe_quat);
				airframe_quat.Enu2Ned();
				//获取经纬度
				double lat, lon;
				map_projection_reproject( &posInf.mp, 
					posInf.PositionENU.x+posInf.HOffset.x, 
					posInf.PositionENU.y+posInf.HOffset.y,
					&lat, &lon );
				//获取速度
				vector3<double> vel;
				get_VelocityENU_Ctrl(&vel);
				//写入pos
				char pos_txt_buf[200];
				int n = sprintf(
					pos_txt_buf,
					"%4d,%04d-%02d-%02d,%02d:%02d:%06.3f,纬:%10.7f,经:%11.7f,高:%7.2f,卫星:%4d,航速:%4.1fm/s,航向:%7.2f,俯仰:%7.2f,横滚:%7.2f\r\n",
					PhotoIndex++,
					RTC_Time.Year, RTC_Time.Month, RTC_Time.Date,
					RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds_f,
					lat,
					lon,
					gps_sensor.position_Global.z*0.01,
					gps0_sat,	//卫星
					safe_sqrt(vel.x*vel.x+vel.y*vel.y)*0.01,	//航速
					rad2degree(airframe_quat.getYaw()),	//航向
					rad2degree(airframe_quat.getPitch()),	//俯仰
					rad2degree(airframe_quat.getRoll())	//横滚
				);
				SDLog_Txt1( pos_txt_buf, n );
			/*记录拍照点数据*/
				
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
	//注册通信参数	
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