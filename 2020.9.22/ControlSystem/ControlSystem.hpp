#pragma once

#include "Basic.hpp"
#include "vector2.hpp"
#include "vector3.hpp"

//����������
enum UAVType
{
	UAVType_Rotor4_X = 10 ,	//������X��
	UAVType_Rotor6_X = 11 ,	//������X��
	UAVType_Rotor8_X = 12 ,	//������X��
	
	UAVType_Rotor4_C = 15 ,	//������ʮ����
	
	UAVType_Rotor42_C = 20 ,	//������Doubleʮ����
	
	UAVType_Rotor6_S1 = 32 ,	//�������칹
};
inline uint8_t UAV_MainMotorCount( uint8_t type )
{
	switch( type )
	{
		case UAVType_Rotor4_X:
			return 4;
		
		case UAVType_Rotor6_X:
			return 6;
			break;
		
		case UAVType_Rotor8_X:
			return 8;
			break;

		case UAVType_Rotor6_S1:
			return 6;
			break;
		
		default:
			return 0;
	}
}

/*�۲�ӿ�*/
	//��ȡ��ͣ����
	bool get_hover_throttle( double* result, double TIMEOUT = -1 );
	//��ȡ��ǰ������������Ӧ�ļ��ٶ�
	bool get_throttle_force( double* result, double TIMEOUT = -1 );
	//��ȡ�߶ȹ۲�������ʱ��T
	bool get_ESO_height_T( double* result, double TIMEOUT = -1 );
	//��ȡ����->���ٶ�����
	bool get_throttle_b( double* result, double TIMEOUT = -1 );
	//��ȡ�Ƿ��ڷ���
	bool get_is_inFlight( bool* result, double TIMEOUT = -1 );
	//��ȡ�����Ŷ���Ӧ�ļ��ٶ�
	bool get_WindDisturbance( vector3<double>* result, double TIMEOUT = -1 );
	
	//��ȡ�۲���ƽ��ٶ�
	bool get_EsAngularRate( vector3<double>* result, double TIMEOUT = -1 );
	//��ȡ�۲���ƽǼ��ٶ�
	bool get_EsAngularAcc( vector3<double>* result, double TIMEOUT = -1 );
/*�۲�ӿ�*/	

/*Homeλ��*/
	bool getHomeLocalZ( double* home, double TIMEOUT = -1 );
	bool getHomePoint( vector2<double>* home, double TIMEOUT = -1 );
	bool getHomeLatLon( vector2<double>* home, double TIMEOUT = -1 );
/*Homeλ��*/

/*�����ӿ�*/
	//��ȡ�����б��
	float get_maxLean();
	//��ȡ���ƫ���ٶ�
	float get_maxYawSpeed();

	//��ȡ��������ٶ�
	float get_maxVelUp();
	//��ȡ����½��ٶ�
	float get_maxVelDown();
	//��ȡ���ˮƽ�ٶ�
	float get_maxVelXY();
	//��ȡ���ˮƽ���ٶ�
	float get_maxAccXY();
/*�����ӿ�*/

/*��̬����*/
	enum Attitude_ControlMode
	{
		Attitude_ControlMode_Angle ,
		Attitude_ControlMode_AngularRate ,
		Attitude_ControlMode_Locking ,
	};
	
	//��ȡ��̬�������Ƿ��
	bool is_Attitude_Control_Enabled( bool* enabled, double TIMEOUT = -1 );
	//�򿪹ر���̬������
	bool Attitude_Control_Enable( double TIMEOUT = -1 );
	bool Attitude_Control_Disable( double TIMEOUT = -1 );

	//��ȡ��ǰ����
	bool get_Target_Throttle( double* result, double TIMEOUT = -1 );
	//�趨����
	bool Attitude_Control_set_Throttle( double thr, double TIMEOUT = -1 );
	//��ȡĿ��Roll Pitch
	bool Attitude_Control_get_Target_RollPitch( double* Roll, double* Pitch, double TIMEOUT = -1 );
	//�趨Ŀ��Roll Pitch
	bool Attitude_Control_set_Target_RollPitch( double Roll, double Pitch, double TIMEOUT = -1 );

	//�趨Ŀ��Yaw
	bool Attitude_Control_set_Target_Yaw( double Yaw, double TIMEOUT = -1 );
	bool Attitude_Control_set_Target_YawRelative( double Yaw, double TIMEOUT = -1 );
	//�趨Ŀ��Yaw�ٶ�
	bool Attitude_Control_set_Target_YawRate( double YawRate, double TIMEOUT = -1 );
	//����Yaw��ɲ�������Ƕȣ�
	bool Attitude_Control_set_YawLock( double TIMEOUT = -1 );
/*��̬����*/

/*λ�ÿ���*/
	enum Position_ControlMode
	{
		//������δ��
		Position_ControlMode_Null = 255 ,
		
		//��ͨģʽ	
		Position_ControlMode_Position = 12 ,	//λ������ģʽ
		Position_ControlMode_Velocity = 11 ,	//�ٶȿ���ģʽ
		Position_ControlMode_Locking = 10 ,	//ɲ������λ��
		
		//2D�Զ�ģʽ
		Position_ControlMode_Takeoff = 20 ,	//���ģʽ
		Position_ControlMode_RouteLine = 22 ,	//Ѳ��ģʽ
		
		//3D�Զ�ģʽ
		Position_ControlMode_RouteLine3D = 52 ,	//Ѳ��ģʽ
	};
	#define Is_2DAutoMode(x) (x>=20 && x<=49)
	#define Is_3DAutoMode(x) (x>=50 && x<=79)
	#define Is_AutoMode(x) (x>=20 && x<=79)
	
	/*�߶�*/
		//�򿪹رո߶ȿ�����
		bool is_Altitude_Control_Enabled( bool* ena, double TIMEOUT = -1 );
		bool Altitude_Control_Enable( double TIMEOUT = -1 );
		bool Altitude_Control_Disable( double TIMEOUT = -1 );
	
		//��ȡ��ǰ�߶ȿ���ģʽ
		bool get_Altitude_ControlMode( Position_ControlMode* mode, double TIMEOUT = -1 );
	
		//�趨Z�Զ������ٶ�
		bool Position_Control_get_ZAutoSpeed( double* SpUp, double* SpDown, double TIMEOUT = -1 );
		bool Position_Control_reset_ZAutoSpeed( double TIMEOUT = -1 );
		bool Position_Control_set_ZAutoSpeed( double SpUp, double SpDown, double TIMEOUT = -1 );
	
		//�ƶ�ZĿ��λ�ã�������λ�û����Զ�ģʽ��ʹ�ã�
		bool Position_Control_move_TargetPositionZRelative( double posz, double TIMEOUT = -1 );
	
		//�趨Ŀ��߶�
		bool Position_Control_set_TargetPositionZ( double posz, double vel = -1, double TIMEOUT = -1 );
		//�趨Ŀ��߶�(��Ե�ǰ�������½���
		bool Position_Control_set_TargetPositionZRelative( double posz, double vel = -1, double TIMEOUT = -1 );
		//�趨Ŀ��߶ȣ�ָ�����θ߶ȣ�
		bool Position_Control_set_TargetPositionZGlobal( double posz, double vel = -1, double TIMEOUT = -1 );
		//�趨Ŀ��߶ȣ������ɵ㣩
		bool Position_Control_set_TargetPositionZRA( double posz, double vel = -1, double TIMEOUT = -1 );
		
		//�趨Ŀ�괹ֱ�ٶ�
		bool Position_Control_set_TargetVelocityZ( double velz, double TIMEOUT = -1 );
		//ɲ�������߶�
		bool Position_Control_set_ZLock( double TIMEOUT = -1 );		
	
		//��ɵ���ǰ�߶��Ϸ���height�߶�
		bool Position_Control_Takeoff_HeightRelative( double height, double TIMEOUT = -1 );
		//��ɵ�ָ�����θ߶�
		bool Position_Control_Takeoff_HeightGlobal( double height, double TIMEOUT = -1 );
		//��ɵ�ָ���߶ȣ�Local���꣩
		bool Position_Control_Takeoff_Height( double height, double TIMEOUT = -1 );
	/*�߶�*/
	
	/*ˮƽλ��*/
		//�򿪹ر�ˮƽλ�ÿ���
		bool is_Position_Control_Enabled( bool* ena, double TIMEOUT = -1 );
		bool Position_Control_Enable( double TIMEOUT = -1 );
		bool Position_Control_Disable( double TIMEOUT = -1 );
		
		//��ȡ��ǰˮƽλ�ÿ���ģʽ
		bool get_Position_ControlMode( Position_ControlMode* mode, double TIMEOUT = -1 );
	
		//�趨�Զ������ٶ�
		bool Position_Control_get_XYAutoSpeed( double* AtVelXY, double TIMEOUT = -1 );
		bool Position_Control_reset_XYAutoSpeed( double TIMEOUT = -1 );
		bool Position_Control_set_XYAutoSpeed( double AtVelXY, double TIMEOUT = -1 );
		
		bool Position_Control_get_XYZAutoSpeed( double* AtVelXYZ, double TIMEOUT = -1 );
		bool Position_Control_reset_XYZAutoSpeed( double TIMEOUT = -1 );
		bool Position_Control_set_XYZAutoSpeed( double AtVelXYZ, double TIMEOUT = -1 );
	
		/*��ֱ��*/
			//�ɵ�Ŀ��ˮƽλ��
			bool Position_Control_set_TargetPositionXY( double posx, double posy, double vel = -1, double TIMEOUT = -1 );
			bool Position_Control_set_TargetPositionXYZ( double posx, double posy, double posz, double vel = -1, double TIMEOUT = -1 );
			//�ɵ�Ŀ��ˮƽλ�ã���Ե�ǰ���꣩
			bool Position_Control_set_TargetPositionXYRelative( double posx, double posy, double vel = -1, double TIMEOUT = -1 );
			bool Position_Control_set_TargetPositionXYZRelative( double posx, double posy, double posz, double vel = -1, double TIMEOUT = -1 );
			//�ɵ�Ŀ��ˮƽλ�ã���Ե�ǰ������ƫ����Bodyheadingϵ�£�
			bool Position_Control_set_TargetPositionXYRelativeBodyheading( double posx, double posy, double vel = -1, double TIMEOUT = -1 );
			bool Position_Control_set_TargetPositionXYZRelativeBodyheading( double posx, double posy, double posz, double vel = -1, double TIMEOUT = -1 );
			//���ݾ�γ�ȣ���������ȫ��λ���������趨Ŀ��ˮƽλ��
			bool Position_Control_set_TargetPositionXY_LatLon( double Lat, double Lon, double vel = -1, double TIMEOUT = -1 );
			//���ݾ�γ�Ⱥͺ��θ߶ȣ���������ȫ��λ���������趨Ŀ��λ�ã���άֱ�ߣ�
			bool Position_Control_set_TargetPositionXYZ_LatLon( double Lat, double Lon, double posz, double vel = -1, double TIMEOUT = -1 );
			//���ݾ�γ�ȣ���������ȫ��λ���������趨Ŀ��λ�ã���άֱ�ߣ�
			//poszΪ��Ե�ǰ�߶����ӵĸ߶�
			bool Position_Control_set_TargetPositionXYZRelative_LatLon( double Lat, double Lon, double posz, double vel = -1, double TIMEOUT = -1 );
			//���ݾ�γ�ȣ���������ȫ��λ���������趨Ŀ��λ�ã���άֱ�ߣ�
			//poszΪ������ɵ�Z����ĸ߶�
			bool Position_Control_set_TargetPositionXYZRA_LatLon( double Lat, double Lon, double posz, double vel = -1, double TIMEOUT = -1 );
			
			//��ȡֱ�߷����ѷ��о���
			bool Position_Control_get_LineFlightDistance( double* distance, double TIMEOUT = -1 );
		/*��ֱ��*/
		
		//�趨Ŀ��ˮƽ�ٶȣ�ENU���򣩲��������Ƕȣ�����������ĽǶȣ�
		//maxAngle<0�����ƽǶ�
		bool Position_Control_set_TargetVelocityXY_AngleLimit( double velx, double vely, double maxAngle = -1, double TIMEOUT = -1 );
		//�趨Ŀ��ˮƽ�ٶȣ�Bodyheading���򣩲��������Ƕȣ�����������ĽǶȣ�
		//maxRoll>0��maxPitch<0ʱ���ƺϽǶ�
		bool Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( double velx, double vely, double maxRoll = -1, double maxPitch = -1, double TIMEOUT = -1 );
		//ɲ��������ˮƽλ��
		bool Position_Control_set_XYLock( double TIMEOUT = -1 );
	/*ˮƽλ��*/
/*λ�ÿ���*/

/*��ȫ�ӿ�*/
	//��ȡ�������ϴο���ʱ��
	//̫�ò����п��ƽ�����MSafeģʽ
	//XYΪˮƽ���ƣ�������̬���ƣ�
	//ZΪ�߶ȿ��ƣ��������ſ��ƣ�
	bool get_lastXYCtrlTime( TIME* t, double TIMEOUT = -1 );
	bool get_lastZCtrlTime( TIME* t, double TIMEOUT = -1 );
	
	//���������ϴο���ʱ����Ϊ������
	//ǿ�ƽ���MSafeģʽ
	//MSafeģʽ���޷��ر�λ�ÿ�����
	//ͬʱ����XYZλ�ÿ��ƿ��˳�MSafe
	//��ˮƽ���Ʋ�����ʱ���ƽǶȣ�
	//forceRTL���Ƿ�����ң������ʱҲִ�з�����ң������Ҫ���У�
	//TIMEOUT����ʱʱ��
	bool enter_MSafe( bool forceRTL = false, double TIMEOUT = -1 );
	
	//��ȡ�Ƿ������MSafeģʽ
	bool is_MSafeCtrl();
/*��ȫ�ӿ�*/