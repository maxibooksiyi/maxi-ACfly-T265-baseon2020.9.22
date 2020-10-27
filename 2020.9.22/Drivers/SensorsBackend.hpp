#pragma once

#include "Sensors.hpp"

void init_Sensors();

/*IMU*/
	/*IMU������ע�ắ��*/
		bool IMUAccelerometerRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT = -1 );
		bool IMUAccelerometerUnRegister( uint8_t index, double TIMEOUT = -1 );
		
		bool IMUGyroscopeRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT = -1 );
		bool IMUGyroscopeUnRegister( uint8_t index, double TIMEOUT = -1 );
		
		bool IMUMagnetometerRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT = -1 );
		bool IMUMagnetometerUnRegister( uint8_t index, double TIMEOUT = -1 );
	/*IMU������ע�ắ��*/
		
	/*IMU���������º���*/
		bool IMUAccelerometerUpdate( uint8_t index, vector3<int32_t> data, bool data_error, double TIMEOUT = -1 );
		bool IMUAccelerometerUpdateTC( uint8_t index, vector3<int32_t> data, bool data_error, double temperature, double TIMEOUT = -1 );
		
		bool IMUGyroscopeUpdate( uint8_t index, vector3<int32_t> data, bool data_error, double TIMEOUT = -1 );
		bool IMUGyroscopeUpdateTC( uint8_t index, vector3<int32_t> data, bool data_error, double temperature, double TIMEOUT = -1 );
			
		bool IMUMagnetometerUpdate( uint8_t index, vector3<int32_t> data, bool data_error, double TIMEOUT = -1 );
		bool IMUMagnetometerUpdateTC( uint8_t index, vector3<int32_t> data, bool data_error, double temperature, double TIMEOUT = -1 );
	/*IMU���������º���*/
/*IMU*/

/*λ�ô�����*/
	/*λ�ô�����ע�ắ��*/
		//������ϸ�����Sensors.h��λ�ô�����ע��
		bool PositionSensorRegister( 
			uint8_t index ,\
			Position_Sensor_Type sensor_type ,\
			Position_Sensor_DataType sensor_data_type ,\
			Position_Sensor_frame sensor_vel_frame ,\
			double delay ,\
			double xy_trustD = 0 ,\
			double z_trustD = 0 , \
			double TIMEOUT = -1 \
		);
		bool PositionSensorUnRegister( uint8_t index, double TIMEOUT = -1 );
	/*λ�ô�����ע�ắ��*/
	
	//����λ�ô�����DataType
	bool PositionSensorChangeDataType( uint8_t index, Position_Sensor_DataType datatype, double TIMEOUT = -1 );
		
	/*λ�ô��������º���*/		
		//ʧ��λ�ô�����
		bool PositionSensorSetInavailable( uint8_t index, double TIMEOUT = -1 );
	
		//delay����С��0�򲻻�ı�delay
		bool PositionSensorUpdatePositionGlobal( uint8_t index, vector3<double> position_Global, bool available, double delay = -1.0, double xy_trustD = -1, double z_trustD = -1, double TIMEOUT = -1 );
		bool PositionSensorUpdatePosition( uint8_t index, vector3<double> position, bool available, double delay = -1.0, double xy_trustD = -1, double z_trustD = -1, double TIMEOUT = -1 );
		bool PositionSensorUpdatePositionGlobalVel( uint8_t index, vector3<double> position_Global, vector3<double> vel, bool available, double delay = -1.0, double xy_trustD = -1, double z_trustD = -1, double TIMEOUT = -1 );
		bool PositionSensorUpdatePositionVel( uint8_t index, vector3<double> position, vector3<double> vel, bool available, double delay = -1.0, double xy_trustD = -1, double z_trustD = -1, double TIMEOUT = -1 );
		bool PositionSensorUpdateVel( uint8_t index, vector3<double> vel, bool available, double delay = -1.0, double xy_trustD = -1, double z_trustD = -1, double TIMEOUT = -1 );
	/*λ�ô��������º���*/
/*λ�ô�����*/