
#include "Basic.hpp"
#include "ctrl_Main.hpp"
#include "ctrl_Attitude.hpp"
#include "ctrl_Position.hpp"
#include "MeasurementSystem.hpp"
#include "ControlSystem.hpp"
#include "drv_ADC.hpp"
#include "Parameters.hpp"
#include "TD4.hpp"
#include "Filters_LP.hpp"
#include "Sensors.hpp"
#include "drv_PWMOut.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

/*控制系统互斥锁*/
	static SemaphoreHandle_t CtrlMutex = xSemaphoreCreateRecursiveMutex();
	bool LockCtrl( double TIMEOUT )
	{
		TickType_t TIMTOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMTOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMTOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTakeRecursive( CtrlMutex, TIMTOUT_Ticks ) == pdTRUE )
			return true;
		else
			return false;
	}
	void UnlockCtrl()
	{
		xSemaphoreGiveRecursive(CtrlMutex);
	}
/*控制系统互斥锁*/
	
/*控制系统安全时间*/
	TIME last_XYCtrlTime;
	TIME last_ZCtrlTime;
	//获取控制器上次控制时间
	//太久不进行控制将进入MSafe模式
	bool get_lastXYCtrlTime( TIME* t, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*t = last_XYCtrlTime;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool get_lastZCtrlTime( TIME* t, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*t = last_ZCtrlTime;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	
	extern bool ForceRTL;
	//将控制器上次控制时间设为不可用
	//强制进入MSafe模式
	//MSafe模式下无法关闭位置控制器
	//同时作出XYZ位置控制可退出MSafe
	//（水平控制不可用时控制角度）
	//forceRTL：是否在有遥控器器时也执行返航（遥控器需要回中）
	//TIMEOUT：超时时间
	bool enter_MSafe( bool forceRTL, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			ForceRTL = forceRTL;
			last_XYCtrlTime.set_invalid();
			last_ZCtrlTime.set_invalid();
			UnlockCtrl();
			return true;
		}
		return false;
	}
	
	//强制Safe控制
	bool ForceMSafeCtrl = false;
	
	//获取是否进入了MSafe模式
	bool is_MSafeCtrl()
	{
		return ForceMSafeCtrl;
	}
/*控制系统安全时间*/

/*电压电流功率数据*/
	//滤波电压（V）
	static float MainBatteryVoltage_filted = 0;
	//总使用功耗（W*h）
	static float MainBatteryPowerUsage = 0;
	//滤波功率（W）
	static float MainBatteryPower_filted = 0;
	//剩余电量（%）
	static float MainBatteryRMPercent = 0;
	//使用率滤波器
	static TD4_Lite RMPercentFilter;
	
	float get_MainBatteryVoltage() { return Get_MainBaterry_Voltage(); }
	float get_MainBatteryVoltage_filted() { return MainBatteryVoltage_filted; }
	float get_MainBatteryPowerUsage() { return MainBatteryPowerUsage; }
	float get_MainBatteryPower_filted() { return MainBatteryPower_filted; }
	float get_MainBatteryCurrent() { return Get_MainBaterry_Current(); }
	float get_CPUTemerature() { return Get_Temperature(); }
	static inline  float get_MainBatteryRMPercent( const BatteryCfg* cfg )
	{
		float volt = Get_MainBaterry_Voltage();
		if( volt > cfg->STVoltage[0] + cfg->VoltP10[0] )
			return 100;
		for( int8_t i = 10; i >= 1 ; --i )
		{
			float P1 = cfg->STVoltage[0] + (&(cfg->VoltP0[0]))[(i-1)*2];					
			if( volt > P1 )
			{
				float P2 = cfg->STVoltage[0] + (&(cfg->VoltP0[0]))[(i-0)*2];
				return (i-1)*10 + 10*( volt - P1 ) / ( P2 - P1 );
			}
		}
		return 0;
	}
	bool  get_MainBatteryRMPercent( float* RMPercent )
	{ 
		float volt = Get_MainBaterry_Voltage();
		BatteryCfg cfg;
		if( ReadParamGroup( "Battery", (uint64_t*)&cfg, 0 ) == PR_OK )
		{
			*RMPercent = get_MainBatteryRMPercent(&cfg);
			return true;
		}
		return false;
	}
	void get_MainBatteryInf( float* Volt, float* Current, float* PowerUsage, float* Power_filted, float* RMPercent )
	{		
		float volt = Get_MainBaterry_Voltage();
		if( Volt != 0 )
			*Volt = volt;
		if( *Current != 0 )
			*Current = Get_MainBaterry_Current();
		if( *PowerUsage != 0 )
			*PowerUsage = MainBatteryPowerUsage;
		if( *Power_filted != 0 )
			*Power_filted = MainBatteryPower_filted;
		if( *RMPercent != 0 )
		{
			BatteryCfg cfg;
			if( ReadParamGroup( "Battery", (uint64_t*)&cfg, 0 ) == PR_OK )
			{
				*RMPercent = get_MainBatteryRMPercent(&cfg);
			}
			else
				*RMPercent = 150;
		}
	}
/*电压电流功率数据*/
	
/*滤波器*/
	static Filter_Butter4_LP acc_filters[3];
	static Filter_Butter4_LP gyro_filters[3];
	bool get_AccelerationNC_filted( vector3<double>* vec, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			vec->set_vector(
				acc_filters[0].get_result(),
				acc_filters[1].get_result(),
				acc_filters[2].get_result());
			UnlockCtrl();
			return true;
		}
		return false;
	} 
	bool get_AngularRateNC_filted( vector3<double>* vec, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			vec->set_vector(
				gyro_filters[0].get_result(),
				gyro_filters[1].get_result(),
				gyro_filters[2].get_result());
			UnlockCtrl();
			return true;
		}
		return false;
	} 
/*滤波器*/	
	
void update_ESO_1();
void update_ESO_2();
static void ControlSystem_Task(void* pvParameters)
{
	//等待驱动初始化完成
	while( getInitializationCompleted() == false )
		os_delay(0.1);
	
	//等待姿态解算系统准备完成
	while( get_Attitude_MSStatus() != MS_Ready )
		os_delay(0.1);
	
	//等待位置解算系统准备完成
	while( get_Altitude_MSStatus() != MS_Ready )
		os_delay(0.1);
	
	//根据机型设置初始主通道数目
	uint8_t uav_type[8];
	ReadParam("AC_UAVType", 0, 0, (uint64_t*)uav_type, 0 );
	set_MainMotorCount( UAV_MainMotorCount(uav_type[0]) );
	
	//设置初始电压
	float Volt; float Current; float PowerUsage; float Power_filted; float RMPercent;
	uint16_t VoltMeas_counter = 0;
	get_MainBatteryInf( &Volt, &Current, &PowerUsage, &Power_filted, &RMPercent );
	MainBatteryVoltage_filted = Volt;
	RMPercentFilter.reset();
	RMPercentFilter.x1 = RMPercent;
	
	//滤波器
	acc_filters[0].set_cutoff_frequency( CtrlRateHz, 10 );
	acc_filters[1].set_cutoff_frequency( CtrlRateHz, 10 );
	acc_filters[2].set_cutoff_frequency( CtrlRateHz, 10 );
	gyro_filters[0].set_cutoff_frequency( CtrlRateHz, 30 );
	gyro_filters[1].set_cutoff_frequency( CtrlRateHz, 30 );
	gyro_filters[2].set_cutoff_frequency( CtrlRateHz, 30 );
	
	//分频
	uint16_t div_counter = 0;
	//准确周期延时
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		//800hz
		vTaskDelayUntil( &xLastWakeTime, 1 );
		
		//更新ESO估计1
		update_ESO_1();
		
		if( ++div_counter >= CtrlRateDiv )
		{	//400hz
			div_counter = 0;
			
			if( LockCtrl(0.02) )
			{
				ctrl_Position();
				ctrl_Attitude();
				
				/*陀螺加速度滤波*/
					IMU_Sensor sensor;
					GetAccelerometer( get_current_use_IMUAccelerometer(), &sensor );
					acc_filters[0].run(sensor.data_raw.x*sensor.sensitivity);
					acc_filters[1].run(sensor.data_raw.y*sensor.sensitivity);
					acc_filters[2].run(sensor.data_raw.z*sensor.sensitivity);
					GetGyroscope( get_current_use_IMUGyroscope(), &sensor );
					gyro_filters[0].run(sensor.data_raw.x*sensor.sensitivity);
					gyro_filters[1].run(sensor.data_raw.y*sensor.sensitivity);
					gyro_filters[2].run(sensor.data_raw.z*sensor.sensitivity);
				/*陀螺加速度滤波*/
				
				UnlockCtrl();
			}
						
			/*电压测量滤波*/
				#define BatMonT 0.2
				#define RMPercentFilterP 0.5
				if( ++VoltMeas_counter >= BatMonT*CtrlRateHz )
				{
					VoltMeas_counter = 0;
					
					//电压滤波
					get_MainBatteryInf( &Volt, &Current, &PowerUsage, &Power_filted, &RMPercent );
					MainBatteryVoltage_filted += 0.1f*BatMonT * ( Volt - MainBatteryVoltage_filted );
					//功率滤波
					MainBatteryPower_filted += 0.1f*BatMonT * ( Volt*Current - MainBatteryPower_filted );
					
					//求使用功耗（W*h）
					MainBatteryPowerUsage += (BatMonT / 60.0f) * Volt*Current;
				}
				//电池剩余百分百滤波
				RMPercentFilter.track4( RMPercent, 1.0/CtrlRateHz, RMPercentFilterP,RMPercentFilterP,RMPercentFilterP,RMPercentFilterP );
			/*电压测量滤波*/
		}
		
		//更新ESO估计2
		update_ESO_2();
	}
}

void init_ControlSystem()
{
	init_Ctrl_Attitude();
	init_Ctrl_Position();
	xTaskCreate( ControlSystem_Task, "ControlSystem", 4096, NULL, SysPriority_ControlSystem, NULL);
	
	/*注册电池参数*/
		BatteryCfg initial_cfg;
		initial_cfg.STVoltage[0] = 11.6;
		initial_cfg.VoltMKp[0] = 21;
		initial_cfg.CurrentMKp[0] = 0.02;
		initial_cfg.Capacity[0] = 300;
		initial_cfg.VoltP0[0] = -1.0;
		initial_cfg.VoltP1[0] = -0.6;
		initial_cfg.VoltP2[0] = -0.4;
		initial_cfg.VoltP3[0] = -0.2;
		initial_cfg.VoltP4[0] = -0.0;
		initial_cfg.VoltP5[0] = +0.1;
		initial_cfg.VoltP6[0] = +0.2;
		initial_cfg.VoltP7[0] = +0.3;
		initial_cfg.VoltP8[0] = +0.4;
		initial_cfg.VoltP9[0] = +0.5;
		initial_cfg.VoltP10[0] = +0.6;
	
		MAV_PARAM_TYPE param_types[] = {
			MAV_PARAM_TYPE_REAL32 ,	//STVoltage
			MAV_PARAM_TYPE_REAL32 ,	//VoltMKp
			MAV_PARAM_TYPE_REAL32 ,	//CurrentMKp
			MAV_PARAM_TYPE_REAL32 ,	//Capacity
			MAV_PARAM_TYPE_REAL32 ,	//VoltP0
			MAV_PARAM_TYPE_REAL32 ,	//VoltP1
			MAV_PARAM_TYPE_REAL32 ,	//VoltP2
			MAV_PARAM_TYPE_REAL32 ,	//VoltP3
			MAV_PARAM_TYPE_REAL32 ,	//VoltP4
			MAV_PARAM_TYPE_REAL32 ,	//VoltP5
			MAV_PARAM_TYPE_REAL32 ,	//VoltP6
			MAV_PARAM_TYPE_REAL32 ,	//VoltP7
			MAV_PARAM_TYPE_REAL32 ,	//VoltP8
			MAV_PARAM_TYPE_REAL32 ,	//VoltP9
			MAV_PARAM_TYPE_REAL32 ,	//VoltP10
		};
		SName param_names[] = {
			"Bat_STVoltage" ,	//UAV Type
			"Bat_VoltMKp" ,	//VoltMKp
			"Bat_CurrentMKp" ,	//CurrentMKp
			"Bat_Capacity" ,	//Capacity
			"Bat_VoltP0" ,	//VoltP0
			"Bat_VoltP1" ,	//VoltP1
			"Bat_VoltP2" ,	//VoltP2
			"Bat_VoltP3" ,	//VoltP3
			"Bat_VoltP4" ,	//VoltP4
			"Bat_VoltP5" ,	//VoltP5
			"Bat_VoltP6" ,	//VoltP6
			"Bat_VoltP7" ,	//VoltP7
			"Bat_VoltP8" ,	//VoltP8
			"Bat_VoltP9" ,	//VoltP9
			"Bat_VoltP10" ,	//VoltP10
		};
		ParamGroupRegister( "Battery", 1, 15, param_types, param_names, (uint64_t*)&initial_cfg );
	/*注册电池参数*/
}