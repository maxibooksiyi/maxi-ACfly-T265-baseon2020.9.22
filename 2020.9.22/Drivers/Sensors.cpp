#include "Sensors.hpp"
#include "SensorsBackend.hpp"
#include "Basic.hpp"
#include "Parameters.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "StorageSystem.hpp"
#include "ControlSystem.hpp"

/*IMU*/

	const uint8_t External_Magnetometer_Index = 0;
	const uint8_t Internal_Magnetometer_Index = 2;

	static SemaphoreHandle_t AccelerometersMutex = xSemaphoreCreateMutex();
	static SemaphoreHandle_t GyroscopesMutex = xSemaphoreCreateMutex();
	static SemaphoreHandle_t MagnetometersMutex = xSemaphoreCreateMutex();
	static IMU_Sensor* Accelerometers[IMU_Sensors_Count] = {0};
	static IMU_Sensor* Gyroscopes[IMU_Sensors_Count] = {0};
	static IMU_Sensor* Magnetometers[IMU_Sensors_Count] = {0};
	
	/*IMU������ע�ắ��*/
		bool IMUAccelerometerRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index] != 0 )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				Accelerometers[index] = new IMU_Sensor;
				IMU_Sensor* sensor = Accelerometers[index];
				sensor->name = name;
				sensor->last_update_time = TIME::now();
				sensor->sensitivity = sensitivity;
				sensor->data.zero();
				sensor->data_raw.zero();
				
				//ע�����
				MAV_PARAM_TYPE types[] = { MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64 };
				IMUConfig initial_cfg;
				initial_cfg.offset[0] = initial_cfg.offset[1] = initial_cfg.offset[2] = 0;
				initial_cfg.scale[0] = initial_cfg.scale[1] = initial_cfg.scale[2] = 1;
				initial_cfg.STTemperature = 0;
				initial_cfg.TemperatureCoefficient[0] = initial_cfg.TemperatureCoefficient[1] = initial_cfg.TemperatureCoefficient[2] = 0;
				ParamGroupRegister( name+"_Acc", 3, IMUConfigLength, types, 0, (uint64_t*)&initial_cfg );
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		bool IMUAccelerometerUnRegister( uint8_t index, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index] == 0 )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				delete Accelerometers[index];
				Accelerometers[index] = 0;
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		
		bool IMUGyroscopeRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index] != 0 )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				Gyroscopes[index] = new IMU_Sensor;
				IMU_Sensor* sensor = Gyroscopes[index];
				sensor->name = name;
				sensor->last_update_time = TIME::now();
				sensor->sensitivity = sensitivity;
				sensor->data.zero();
				sensor->data_raw.zero();
				
				//ע�����
				MAV_PARAM_TYPE types[] = { MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64 };
				IMUConfig initial_cfg;
				initial_cfg.offset[0] = initial_cfg.offset[1] = initial_cfg.offset[2] = 0;
				initial_cfg.scale[0] = initial_cfg.scale[1] = initial_cfg.scale[2] = 1;
				initial_cfg.STTemperature = 0;
				initial_cfg.TemperatureCoefficient[0] = initial_cfg.TemperatureCoefficient[1] = initial_cfg.TemperatureCoefficient[2] = 0;
				ParamGroupRegister( name+"_Gyro", 3, IMUConfigLength, types, 0, (uint64_t*)&initial_cfg );
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		bool IMUGyroscopeUnRegister( uint8_t index, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index] == 0 )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				delete Gyroscopes[index];
				Gyroscopes[index] = 0;
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		
		bool IMUMagnetometerRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index] != 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}
				Magnetometers[index] = new IMU_Sensor;
				IMU_Sensor* sensor = Magnetometers[index];
				sensor->name = name;
				sensor->last_update_time = TIME::now();
				sensor->sensitivity = sensitivity;
				sensor->data.zero();
				sensor->data_raw.zero();
				
				//ע�����
				MAV_PARAM_TYPE types[] = { MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64 };
				IMUConfig initial_cfg;
				initial_cfg.offset[0] = initial_cfg.offset[1] = initial_cfg.offset[2] = 0;
				initial_cfg.scale[0] = initial_cfg.scale[1] = initial_cfg.scale[2] = 1;
				initial_cfg.STTemperature = 0;
				initial_cfg.TemperatureCoefficient[0] = initial_cfg.TemperatureCoefficient[1] = initial_cfg.TemperatureCoefficient[2] = 0;
				ParamGroupRegister( name+"_Mag", 3, IMUConfigLength, types, 0, (uint64_t*)&initial_cfg );
				
				xSemaphoreGive(MagnetometersMutex);
				return true;
			}
			return false;
		}
		bool IMUMagnetometerUnRegister( uint8_t index, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index] == 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}
				delete Magnetometers[index];
				Magnetometers[index] = 0;
				
				xSemaphoreGive(MagnetometersMutex);
				return true;
			}
			return false;
		}
	/*IMU������ע�ắ��*/
		
	/*IMU���������º���*/
		bool IMUAccelerometerUpdate( uint8_t index, vector3<int32_t> data, bool data_error, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
					
			vector3<int32_t> offset(0,0,0);			
			vector3<double> scale(1,1,1);			
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index] == 0 )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Accelerometers[index];
				//��ȡ����				
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Acc", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new )
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				
				//д�����ʱ��
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//д�봫��������
				sensor->have_temperature = false;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				sensor->data_rawTC.set_vector( data.x, data.y, data.z );
				sensor->data.x = (sensor->data_raw.x - offset.x) * sensor->sensitivity * scale.x;
				sensor->data.y = (sensor->data_raw.y - offset.y) * sensor->sensitivity * scale.y;
				sensor->data.z = (sensor->data_raw.z - offset.z) * sensor->sensitivity * scale.z;
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		bool IMUAccelerometerUpdateTC( uint8_t index, vector3<int32_t> data, bool data_error, double temperature, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
					
			vector3<int32_t> offset(0,0,0);			
			vector3<double> scale(1,1,1);			
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index] == 0 )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Accelerometers[index];
				//��ȡ����				
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Acc", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new )
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				
				//д�����ʱ��
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//д�봫��������
				sensor->have_temperature = true;
				sensor->data_error = data_error;
			 	sensor->temperature = temperature;
				sensor->data_raw = data;
				double temperature_err = temperature - cfg.STTemperature;
				sensor->data_rawTC.set_vector(
					sensor->data_raw.x - temperature_err*cfg.TemperatureCoefficient[0],
					sensor->data_raw.y - temperature_err*cfg.TemperatureCoefficient[1],
					sensor->data_raw.z - temperature_err*cfg.TemperatureCoefficient[2] );
				sensor->data.x = (sensor->data_rawTC.x - offset.x) * sensor->sensitivity * scale.x;
				sensor->data.y = (sensor->data_rawTC.y - offset.y) * sensor->sensitivity * scale.y;
				sensor->data.z = (sensor->data_rawTC.z - offset.z) * sensor->sensitivity * scale.z;
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		
		bool IMUGyroscopeUpdate( uint8_t index, vector3<int32_t> data, bool data_error, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			vector3<double> offset(0,0,0);
			vector3<double> scale(1,1,1);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index] == 0 )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Gyroscopes[index];
				//��ȡ����
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Gyro", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new )
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				
				//д�����ʱ��
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//д�봫��������
				sensor->have_temperature = false;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				sensor->data_rawTC.set_vector( data.x, data.y, data.z );
				sensor->data.x = (sensor->data_raw.x - offset.x) * sensor->sensitivity * scale.x;
				sensor->data.y = (sensor->data_raw.y - offset.y) * sensor->sensitivity * scale.y;
				sensor->data.z = (sensor->data_raw.z - offset.z) * sensor->sensitivity * scale.z;
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		bool IMUGyroscopeUpdateTC( uint8_t index, vector3<int32_t> data, bool data_error, double temperature, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			vector3<double> offset(0,0,0);
			vector3<double> scale(1,1,1);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index] == 0 )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Gyroscopes[index];
				//��ȡ����
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Gyro", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new )
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				
				//д�����ʱ��
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//д�봫��������
				sensor->have_temperature = true;
				sensor->temperature = temperature;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				double temperature_err = temperature - cfg.STTemperature;
				sensor->data_rawTC.set_vector(
					sensor->data_raw.x - temperature_err*cfg.TemperatureCoefficient[0],
					sensor->data_raw.y - temperature_err*cfg.TemperatureCoefficient[1],
					sensor->data_raw.z - temperature_err*cfg.TemperatureCoefficient[2] );
				sensor->data.x = (sensor->data_rawTC.x - offset.x) * sensor->sensitivity * scale.x;
				sensor->data.y = (sensor->data_rawTC.y - offset.y) * sensor->sensitivity * scale.y;
				sensor->data.z = (sensor->data_rawTC.z - offset.z) * sensor->sensitivity * scale.z;
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		
		bool IMUMagnetometerUpdate( uint8_t index, vector3<int32_t> data, bool data_error, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			vector3<double> offset(0,0,0);
			vector3<double> scale(1,1,1);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index] == 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Magnetometers[index];
				//��ȡ����
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Mag", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new )
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				
				//д�����ʱ��
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//д�봫��������
				sensor->have_temperature = false;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				sensor->data_rawTC.set_vector( data.x, data.y, data.z );
				sensor->data.x = (sensor->data_raw.x - offset.x) * sensor->sensitivity * scale.x;
				sensor->data.y = (sensor->data_raw.y - offset.y) * sensor->sensitivity * scale.y;
				sensor->data.z = (sensor->data_raw.z - offset.z) * sensor->sensitivity * scale.z;
				
				xSemaphoreGive(MagnetometersMutex);
				return true;
			}
			return false;
		}
		bool IMUMagnetometerUpdateTC( uint8_t index, vector3<int32_t> data, bool data_error, double temperature, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			vector3<double> offset(0,0,0);
			vector3<double> scale(1,1,1);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index] == 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Magnetometers[index];
				//��ȡ����
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Mag", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new )
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				
				//д�����ʱ��
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//д�봫��������
				sensor->have_temperature = true;
				sensor->temperature = temperature;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				double temperature_err = temperature - cfg.STTemperature;
				sensor->data_rawTC.set_vector(
					sensor->data_raw.x - temperature_err*cfg.TemperatureCoefficient[0],
					sensor->data_raw.y - temperature_err*cfg.TemperatureCoefficient[1],
					sensor->data_raw.z - temperature_err*cfg.TemperatureCoefficient[2] );
				sensor->data.x = (sensor->data_rawTC.x - offset.x) * sensor->sensitivity * scale.x;
				sensor->data.y = (sensor->data_rawTC.y - offset.y) * sensor->sensitivity * scale.y;
				sensor->data.z = (sensor->data_rawTC.z - offset.z) * sensor->sensitivity * scale.z;
				
				xSemaphoreGive(MagnetometersMutex);				
				return true;
			}
			return false;
		}
	/*IMU���������º���*/
		
	/*IMU��������ȡ����*/
		bool GetAccelerometer( uint8_t index, IMU_Sensor* sensor, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index] == 0 )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}				
				*sensor = *Accelerometers[index];
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		bool GetGyroscope( uint8_t index, IMU_Sensor* sensor, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index] == 0 )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}				
				*sensor = *Gyroscopes[index];
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		bool GetMagnetometer( uint8_t index, IMU_Sensor* sensor, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index] == 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}				
				*sensor = *Magnetometers[index];
				
				xSemaphoreGive(MagnetometersMutex);
				return true;
			}
			return false;
		}
	/*IMU��������ȡ����*/
		
/*IMU*/
		
/*λ�ô�����*/
		
	const uint8_t default_ultrasonic_sensor_index = 1;
	const uint8_t default_optical_flow_index = 8;
	const uint8_t external_baro_sensor_index = 3;
	const uint8_t internal_baro_sensor_index = 4;
	const uint8_t default_gps_sensor_index = 6;
		
	static SemaphoreHandle_t Position_Sensors_Mutex[Position_Sensors_Count] = {0};
	static Position_Sensor* Position_Sensors[Position_Sensors_Count];
		
	/*λ�ô�����ע�ắ��*/
		bool PositionSensorRegister( 
			uint8_t index ,\
			Position_Sensor_Type sensor_type ,\
			Position_Sensor_DataType sensor_data_type ,\
			Position_Sensor_frame sensor_vel_frame ,\
			double delay ,\
			double xy_trustD , \
			double z_trustD , \
			double TIMEOUT \
		)
		{
			if( index >= Position_Sensors_Count )
				return false;			
							
			if( delay < 0 )
				return false;
			if( xy_trustD < 0 )
				return false;
			if( z_trustD < 0 )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{
				if( Position_Sensors[index] != 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				Position_Sensors[index] = new Position_Sensor;
				Position_Sensor* sensor = Position_Sensors[index];
				sensor->available = false;
				sensor->available_status_update_time = TIME::now();
				sensor->last_update_time = TIME::now();
				sensor->sensor_type = sensor_type;
				sensor->sensor_DataType = sensor_data_type;
				sensor->velocity_data_frame = sensor_vel_frame;
				sensor->delay = delay;
				sensor->xy_trustD = xy_trustD;
				sensor->z_trustD = z_trustD;
				sensor->sample_time = -1;
				sensor->mp.initialized = false;
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//����������
			return false;
		}
		bool PositionSensorUnRegister( uint8_t index, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//����������
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				delete sensor;
				Position_Sensors[index] = 0;
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//����������
			return false;
		}
	/*λ�ô�����ע�ắ��*/
	
	//����λ�ô�����DataType
	bool PositionSensorChangeDataType( uint8_t index, Position_Sensor_DataType datatype, double TIMEOUT )
	{
		if( index >= Position_Sensors_Count )
			return false;
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
		{	//����������
			if( Position_Sensors[index] == 0 )
			{
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return false;
			}
			
			Position_Sensor* sensor = Position_Sensors[index];
			sensor->sensor_DataType = datatype;
			
			xSemaphoreGive(Position_Sensors_Mutex[index]);
			return true;
		}	//����������
		return false;
	}
		
	/*λ�ô��������º���*/
		
		//ʧ��λ�ô�����
		bool PositionSensorSetInavailable( uint8_t index, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//����������
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				if( sensor->available != false )
					sensor->available_status_update_time = TIME::now();
				//���¿���״̬
				if( sensor->available != false )
					sensor->available_status_update_time = TIME::now();
				sensor->available = false;
				//���²���ʱ��
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();			
				
				//��¼λ������
				if(inFlight && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);				
				return true;
			}	//����������
			return false;
		}
	
		bool PositionSensorUpdatePositionGlobal( uint8_t index, vector3<double> position_Global, bool available, double delay, double xy_trustD, double z_trustD, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//����������
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				
				//�жϴ��������͡������Ƿ���ȷ
				bool data_effective;
				switch( sensor->sensor_DataType )
				{
					case Position_Sensor_DataType_s_xy:
						if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || \
								__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_s_z:
						if( __ARM_isnan( position_Global.z ) || __ARM_isinf( position_Global.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_s_xyz:
						if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || __ARM_isnan( position_Global.z ) || \
								__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) || __ARM_isinf( position_Global.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//���ݳ����˳�
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}								
				
				//���¿���״̬
				if( sensor->available != available )
					sensor->available_status_update_time = TIME::now();
				sensor->available = available;
				
				//��������
				sensor->position_Global = position_Global;
				//��γ����ƽ��ͶӰ
				if( available )
				{					
					if( sensor->mp.initialized == false )
					{
						map_projection_init( &sensor->mp , position_Global.x , position_Global.y );
					}
					double pos_x , pos_y;
					map_projection_project( &sensor->mp , position_Global.x , position_Global.y , &pos_x , &pos_y );
					sensor->position.x = pos_x;
					sensor->position.y = pos_y;
					sensor->position.z = position_Global.z;
				}
					
				//���²���ʱ��
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();				
				//������ʱʱ��
				if( delay > 0 )
					sensor->delay = delay;
				//�������ζ�
				if( xy_trustD >= 0 )
					sensor->xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->z_trustD = z_trustD;
				
				//��¼λ������
				if(inFlight && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//����������
			return false;
		}
		bool PositionSensorUpdatePosition( uint8_t index, vector3<double> position, bool available, double delay, double xy_trustD, double z_trustD, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//����������
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				//�жϴ��������͡������Ƿ���ȷ
				bool data_effective;
				switch( sensor->sensor_DataType )
				{
					case Position_Sensor_DataType_s_xy:
						if( __ARM_isnan( position.x ) || __ARM_isnan( position.y ) || \
								__ARM_isinf( position.x ) || __ARM_isinf( position.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_s_z:
						if( __ARM_isnan( position.z ) || __ARM_isinf( position.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_s_xyz:
						if( __ARM_isnan( position.x ) || __ARM_isnan( position.y ) || __ARM_isnan( position.z ) || \
								__ARM_isinf( position.x ) || __ARM_isinf( position.y ) || __ARM_isinf( position.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//���ݳ����˳�
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				//���¿���״̬
				if( sensor->available != available )
					sensor->available_status_update_time = TIME::now();
				sensor->available = available;
				
				//��������
				sensor->position = position;
				
				//���²���ʱ��
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();			
				
				//��ʱ����0������ʱ
				if( delay > 0 )
					sensor->delay = delay;
				//�������ζ�
				if( xy_trustD >= 0 )
					sensor->xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->z_trustD = z_trustD;
				
				//��¼λ������
				if(inFlight && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//����������
			return false;
		}
		bool PositionSensorUpdatePositionGlobalVel( uint8_t index, vector3<double> position_Global, vector3<double> vel, bool available, double delay, double xy_trustD, double z_trustD, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//����������
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				//�жϴ��������͡������Ƿ���ȷ
				bool data_effective;
				switch( sensor->sensor_DataType )
				{
					case Position_Sensor_DataType_sv_xy:
						if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || \
								__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) || \
								__ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_sv_z:
						if( __ARM_isnan( position_Global.z ) || __ARM_isinf( position_Global.z ) || \
								__ARM_isnan( vel.z ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_sv_xyz:
						if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || __ARM_isnan( position_Global.z ) || \
								__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) || __ARM_isinf( position_Global.z ) || \
								__ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || __ARM_isnan( vel.z ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//���ݳ����˳�
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				//���¿���״̬
				if( sensor->available != available )
					sensor->available_status_update_time = TIME::now();
				sensor->available = available;
				
				//��������
				sensor->position_Global = position_Global;
				sensor->velocity = vel;
				//��γ����ƽ��ͶӰ				
				if( available )
				{
					if( sensor->mp.initialized == false )
					{
						map_projection_init( &sensor->mp , position_Global.x , position_Global.y );
					}
					double pos_x , pos_y;
					map_projection_project( &sensor->mp , position_Global.x , position_Global.y , &pos_x , &pos_y );
					sensor->position.x = pos_x;
					sensor->position.y = pos_y;
					sensor->position.z = position_Global.z;
				}
					
				//���²���ʱ��
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();				
				//������ʱʱ��
				if( delay > 0 )
					sensor->delay = delay;
				//�������ζ�
				if( xy_trustD >= 0 )
					sensor->xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->z_trustD = z_trustD;
				
				//��¼λ������
				if(inFlight && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//����������
			return false;
		}
		bool PositionSensorUpdatePositionVel( uint8_t index, vector3<double> position, vector3<double> vel, bool available, double delay, double xy_trustD, double z_trustD, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//����������
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				bool data_effective;
				switch( sensor->sensor_DataType )
				{
					case Position_Sensor_DataType_sv_xy:
						if( __ARM_isnan( position.x ) || __ARM_isnan( position.y ) || \
								__ARM_isinf( position.x ) || __ARM_isinf( position.y ) || \
								__ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_sv_z:
						if( __ARM_isnan( position.z ) || __ARM_isinf( position.z ) || \
								__ARM_isnan( vel.z ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_sv_xyz:
						if( __ARM_isnan( position.x ) || __ARM_isnan( position.y ) || __ARM_isnan( position.z ) || \
								__ARM_isinf( position.x ) || __ARM_isinf( position.y ) || __ARM_isinf( position.z ) || \
								__ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || __ARM_isnan( vel.z ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//���ݳ����˳�
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				//���¿���״̬
				if( sensor->available != available )
					sensor->available_status_update_time = TIME::now();
				sensor->available = available;
				
				//��������
				sensor->position = position;
				sensor->velocity = vel;
				
				//���²���ʱ��
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();			
				
				//��ʱ����0������ʱ
				if( delay > 0 )
					sensor->delay = delay;
				//�������ζ�
				if( xy_trustD >= 0 )
					sensor->xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->z_trustD = z_trustD;
				
				//��¼λ������
				if(inFlight && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//����������
			return false;
		}
		bool PositionSensorUpdateVel( uint8_t index, vector3<double> vel, bool available, double delay, double xy_trustD, double z_trustD, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//����������
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				
				//�жϴ��������͡������Ƿ���ȷ
				bool data_effective;
				switch( sensor->sensor_DataType )
				{
					case Position_Sensor_DataType_v_xy:
						if( __ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_v_z:
						if( __ARM_isnan( vel.z ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_v_xyz:
						if( __ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || __ARM_isnan( vel.z ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//���ݳ����˳�
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				//���¿���״̬
				if( sensor->available != available )
					sensor->available_status_update_time = TIME::now();
				sensor->available = available;
				
				//��������
				sensor->velocity = vel;
				
				//���²���ʱ��
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();			
				
				//��ʱ����0������ʱ
				if( delay > 0 )
					sensor->delay = delay;
				//�������ζ�
				if( xy_trustD >= 0 )
					sensor->xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->z_trustD = z_trustD;
				
				//��¼λ������
				if(inFlight && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//����������
			return false;
		}
	/*λ�ô��������º���*/
		
	/*λ�ô�������ȡ����*/
		bool GetPositionSensor( uint8_t index, Position_Sensor* result_sensor, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//����������
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				*result_sensor = *sensor;
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//����������
			return false;
		}
	/*λ�ô�������ȡ����*/
	
/*λ�ô�����*/
		
void init_Sensors()
{
	//��ʼ��������������
	for( int i = 0; i < Position_Sensors_Count; ++i )
		Position_Sensors_Mutex[i] = xSemaphoreCreateMutex();
	
	/*ע�ᴫ����λ��ƫ�Ʋ���*/
		SensorPosOffset initial_posoffset;
		//�ɿ�ƫ��
		initial_posoffset.Fc_x[0] = 0;
		initial_posoffset.Fc_y[0] = 0;
		initial_posoffset.Fc_z[0] = 0;
		//������0ƫ��
		initial_posoffset.S0_x[0] = 0;
		initial_posoffset.S0_y[0] = 0;
		initial_posoffset.S0_z[0] = 0;
		//������1ƫ��
		initial_posoffset.S1_x[0] = 0;
		initial_posoffset.S1_y[0] = 0;
		initial_posoffset.S1_z[0] = 0;
		//������2ƫ��
		initial_posoffset.S2_x[0] = 0;
		initial_posoffset.S2_y[0] = 0;
		initial_posoffset.S2_z[0] = 0;
		//������3ƫ��
		initial_posoffset.S3_x[0] = 0;
		initial_posoffset.S3_y[0] = 0;
		initial_posoffset.S3_z[0] = 0;
		//������4ƫ��
		initial_posoffset.S4_x[0] = 0;
		initial_posoffset.S4_y[0] = 0;
		initial_posoffset.S4_z[0] = 0;
		//������5ƫ��
		initial_posoffset.S5_x[0] = 0;
		initial_posoffset.S5_y[0] = 0;
		initial_posoffset.S5_z[0] = 0;
		//������6ƫ��
		initial_posoffset.S6_x[0] = 0;
		initial_posoffset.S6_y[0] = 0;
		initial_posoffset.S6_z[0] = 0;
		//������7ƫ��
		initial_posoffset.S7_x[0] = 0;
		initial_posoffset.S7_y[0] = 0;
		initial_posoffset.S7_z[0] = 0;
		//������8ƫ��
		initial_posoffset.S8_x[0] = 0;
		initial_posoffset.S8_y[0] = 0;
		initial_posoffset.S8_z[0] = 0;
		//������9ƫ��
		initial_posoffset.S9_x[0] = 0;
		initial_posoffset.S9_y[0] = 0;
		initial_posoffset.S9_z[0] = 0;
		//������10ƫ��
		initial_posoffset.S10_x[0] = 0;
		initial_posoffset.S10_y[0] = 0;
		initial_posoffset.S10_z[0] = 0;
		//������11ƫ��
		initial_posoffset.S11_x[0] = 0;
		initial_posoffset.S11_y[0] = 0;
		initial_posoffset.S11_z[0] = 0;
		//������12ƫ��
		initial_posoffset.S12_x[0] = 0;
		initial_posoffset.S12_y[0] = 0;
		initial_posoffset.S12_z[0] = 0;
		//������13ƫ��
		initial_posoffset.S13_x[0] = 0;
		initial_posoffset.S13_y[0] = 0;
		initial_posoffset.S13_z[0] = 0;
		//������14ƫ��
		initial_posoffset.S14_x[0] = 0;
		initial_posoffset.S14_y[0] = 0;
		initial_posoffset.S14_z[0] = 0;
		//������15ƫ��
		initial_posoffset.S15_x[0] = 0;
		initial_posoffset.S15_y[0] = 0;
		initial_posoffset.S15_z[0] = 0;
	
		MAV_PARAM_TYPE param_types[] = {
			//�ɿ�ƫ��
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//������0ƫ��
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//������1ƫ��
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//������2ƫ��
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//������3ƫ��
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//������4ƫ��
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//������5ƫ��
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//������6ƫ��
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//������7ƫ��
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//������8ƫ��
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//������9ƫ��
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//������10ƫ��
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//������11ƫ��
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//������12ƫ��
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//������13ƫ��
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//������14ƫ��
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//������15ƫ��
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
		};
		
		//�ɿ�ƫ��		
		SName param_names[] = {
			//�ɿ�ƫ��
			"POfs_Fc_x" ,	//x
			"POfs_Fc_y" ,	//y
			"POfs_Fc_z" ,	//z
			//������0ƫ��
			"POfs_S0_x" ,	//x
			"POfs_S0_y" ,	//y
			"POfs_S0_z" ,	//z
			//������1ƫ��
			"POfs_S1_x" ,	//x
			"POfs_S1_y" ,	//y
			"POfs_S1_z" ,	//z
			//������2ƫ��
			"POfs_S2_x" ,	//x
			"POfs_S2_y" ,	//y
			"POfs_S2_z" ,	//z
			//������3ƫ��
			"POfs_S3_x" ,	//x
			"POfs_S3_y" ,	//y
			"POfs_S3_z" ,	//z
			//������4ƫ��
			"POfs_S4_x" ,	//x
			"POfs_S4_y" ,	//y
			"POfs_S4_z" ,	//z
			//������5ƫ��
			"POfs_S5_x" ,	//x
			"POfs_S5_y" ,	//y
			"POfs_S5_z" ,	//z
			//������6ƫ��
			"POfs_S6_x" ,	//x
			"POfs_S6_y" ,	//y
			"POfs_S6_z" ,	//z
			//������7ƫ��
			"POfs_S7_x" ,	//x
			"POfs_S7_y" ,	//y
			"POfs_S7_z" ,	//z
			//������8ƫ��
			"POfs_S8_x" ,	//x
			"POfs_S8_y" ,	//y
			"POfs_S8_z" ,	//z
			//������9ƫ��
			"POfs_S9_x" ,	//x
			"POfs_S9_y" ,	//y
			"POfs_S9_z" ,	//z
			//������10ƫ��
			"POfs_S10_x" ,	//x
			"POfs_S10_y" ,	//y
			"POfs_S10_z" ,	//z
			//������11ƫ��
			"POfs_S11_x" ,	//x
			"POfs_S11_y" ,	//y
			"POfs_S11_z" ,	//z
			//������12ƫ��
			"POfs_S12_x" ,	//x
			"POfs_S12_y" ,	//y
			"POfs_S12_z" ,	//z
			//������13ƫ��
			"POfs_S13_x" ,	//x
			"POfs_S13_y" ,	//y
			"POfs_S13_z" ,	//z
			//������14ƫ��
			"POfs_S14_x" ,	//x
			"POfs_S14_y" ,	//y
			"POfs_S14_z" ,	//z
			//������15ƫ��
			"POfs_S15_x" ,	//x
			"POfs_S15_y" ,	//y
			"POfs_S15_z" ,	//z
		};
		ParamGroupRegister( "PosOffset", 1, 51, param_types, param_names, (uint64_t*)&initial_posoffset );
}