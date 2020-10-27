#include "M11_TempCalib.hpp"
#include "vector3.hpp"
#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "drv_Sensors.hpp"
#include "ControlSystem.hpp"

//陀螺校准

//朱文杰 20181226
//请勿用于商业用途
//！！抄袭必究！！

M11_TempCalib::M11_TempCalib():Mode_Base( "TempCalib", 11 )
{
	
}

ModeResult M11_TempCalib::main_func( void* param1, uint32_t param2 )
{
	setLedMode(LEDMode_Processing2);
	
	//判断板子是否静止
	vector3<double> Calibration_Acc_Max , Calibration_Acc_Min;
	vector3<double> Calibration_Gyro_Max , Calibration_Gyro_Min;
	
	os_delay(2.0);
	
	//初始化静止检测
	vector3<double> acc_filted;
	vector3<double> gyro_filted;
	get_AccelerationNC_filted(&acc_filted);
	get_AngularRateNC_filted(&gyro_filted);
	Calibration_Acc_Max = Calibration_Acc_Min = acc_filted;
	Calibration_Gyro_Max = Calibration_Gyro_Min = gyro_filted;

	//校准状态机
	uint8_t calibration_step = 0;
	TIME calibration_start_time = TIME::now();
	//校准值
	uint32_t calib_n = 0;
	bool calib_gyroscope[IMU_Sensors_Count];
	vector3<double> sum_gyro[IMU_Sensors_Count];
	
	//初始化校准
	for( uint8_t i = 0; i < IMU_Sensors_Count; ++i )
	{
		IMU_Sensor sensor;
		//陀螺
		if( GetGyroscope( i, &sensor ) )
		{
			sum_gyro[i].zero();
			calib_gyroscope[i] = true;
		}
		else
			calib_gyroscope[i] = false;
	}
	
	while(1)
	{
		os_delay(0.01);
		
		/*静止检测*/
			vector3<double> acc_filted;
			vector3<double> gyro_filted;
			get_AccelerationNC_filted(&acc_filted);
			get_AngularRateNC_filted(&gyro_filted);
			
			if( acc_filted.x > Calibration_Acc_Max.x ) Calibration_Acc_Max.x = acc_filted.x;
			else if( acc_filted.x < Calibration_Acc_Min.x ) Calibration_Acc_Min.x = acc_filted.x;
			if( acc_filted.y > Calibration_Acc_Max.y ) Calibration_Acc_Max.y = acc_filted.y;
			else if( acc_filted.y < Calibration_Acc_Min.y ) Calibration_Acc_Min.y = acc_filted.y;
			if( acc_filted.z > Calibration_Acc_Max.z ) Calibration_Acc_Max.z = acc_filted.z;
			else if( acc_filted.z < Calibration_Acc_Min.z ) Calibration_Acc_Min.z = acc_filted.z;
			
			if( gyro_filted.x > Calibration_Gyro_Max.x ) Calibration_Gyro_Max.x = gyro_filted.x;
			else if( gyro_filted.x < Calibration_Gyro_Min.x ) Calibration_Gyro_Min.x = gyro_filted.x;
			if( gyro_filted.y > Calibration_Gyro_Max.y )Calibration_Gyro_Max.y = gyro_filted.y;
			else if( gyro_filted.y < Calibration_Gyro_Min.y ) Calibration_Gyro_Min.y = gyro_filted.y;
			if( gyro_filted.z > Calibration_Gyro_Max.z ) Calibration_Gyro_Max.z = gyro_filted.z;
			else if( gyro_filted.z < Calibration_Gyro_Min.z ) Calibration_Gyro_Min.z = gyro_filted.z;
			
			double acc_fluctuation_range;	double gyro_fluctuation_range;
			vector3<double> v2=Calibration_Acc_Max-Calibration_Acc_Min;
			vector3<double> v1=Calibration_Gyro_Max-Calibration_Gyro_Min;
		 
			gyro_fluctuation_range=safe_sqrt(v1.get_square());
			acc_fluctuation_range=safe_sqrt(v2.get_square());
		/*静止检测*/
		
		if( ( acc_fluctuation_range > 50 ) || ( gyro_fluctuation_range > 0.1 ) )
		{	//判断非静止
			sendLedSignal(LEDSignal_Err1);
			return MR_Err;
		}

		for( uint8_t i = 0; i < IMU_Sensors_Count; ++i )
		{
			if( calib_gyroscope[i] )
			{
				IMU_Sensor sensor;
				if( GetGyroscope( i, &sensor ) )
				{
					sum_gyro[i].x += sensor.data_raw.x;
					sum_gyro[i].y += sensor.data_raw.y;
					sum_gyro[i].z += sensor.data_raw.z;
				}
				else
					calib_gyroscope[i] = false;
			}
		}
		++calib_n;
		if( calib_n >= 200 )
			break;
	}
	
CalibFinish:
	PR_RESULT res = PR_OK;
	double invN = 1.0/calib_n;	
	for( uint8_t i = 0; i < IMU_Sensors_Count; ++i )
	{
		if( calib_gyroscope[i] )
		{
			IMU_Sensor sensor;
			if( GetGyroscope( i, &sensor ) )
			{

				vector3<double> avg_sum_gyro_y = sum_gyro[i]*invN;
				vector3<double> GyroOffset = avg_sum_gyro_y;
				
				IMUConfig cfg;
				cfg.scale[0] = cfg.scale[1] = cfg.scale[2] = 1;
				cfg.offset[0] = GyroOffset.x;	cfg.offset[1] = GyroOffset.y;	cfg.offset[2] = GyroOffset.z;
				cfg.STTemperature = 0;
				cfg.TemperatureCoefficient[0] = 0;	cfg.TemperatureCoefficient[1] = 0;	cfg.TemperatureCoefficient[2] = 0;
				if( res == PR_OK )
					res = UpdateParamGroup( sensor.name+"_Gyro", (uint64_t*)&cfg, 0, IMUConfigLength );
				else
					UpdateParamGroup( sensor.name+"_Gyro", (uint64_t*)&cfg, 0, IMUConfigLength );
			}
		}	
	}
	
	if( res == PR_OK )
	{
		sendLedSignal(LEDSignal_Success1);
		return MR_OK;
	}
	else
	{
		sendLedSignal(LEDSignal_Err1);
		return MR_Err;
	}
}