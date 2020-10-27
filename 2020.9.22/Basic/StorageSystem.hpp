#pragma once

#include <stdint.h>
#include "Sensors.hpp"

//�洢�������
enum SS_RESULT
{
	SS_OK = 0 ,
	SS_ERR ,
	SS_TimeOut ,
};

/*�ڲ��洢*/
	/*���ô洢*/
		/*
			����洢�ļ�
			group_name���ļ��������
			name���ļ�����
			content������
			length�����ݳ���
			TIMEOUT���߳�ͬ����ʱʱ��
		*/
		SS_RESULT InternalStorage_SaveFile( const char* group_name, const char* name , const void* content , uint32_t length , double TIMEOUT = -1 );
		
		/*
			��ȡ�洢�ļ���С
			group_name���ļ��������
			name���ļ�����
			size����ȡ���ļ���С
			TIMEOUT���߳�ͬ����ʱʱ��
		*/
		SS_RESULT InternalStorage_GetFileSize( const char* group_name, const char* name , uint32_t* size , double TIMEOUT = -1 );
		
		/*
			��ȡ�洢�ļ�
			group_name���ļ��������
			name���ļ�����
			content����ȡ������
			length����ȡ���ļ�����
			TIMEOUT���߳�ͬ����ʱʱ��
		*/
		SS_RESULT InternalStorage_ReadFile( const char* group_name, const char* name , void* content , uint32_t* length , double TIMEOUT = -1 );
	/*���ô洢*/
	
	void init_InternalStorage();
/*�ڲ��洢*/

/*�ⲿ�洢*/
	/*Log����*/
		/*
			Log��ʽ��
				'A''C' + ���汾(0) + �ΰ汾(1)+���ݸ�ʽ(bit0=0:float bit0=1:double) + 24�ֽ�����
			����֡��
				'A''C' + ��Ϣ���� + ��Ϣ������ + (uint32_t)����ʱ��(100us) + (double)��������
		*/
		enum LogMsg
		{
			/*����������*/
			LogMsg_PosSensor = 28 ,
			/*ŷ������̬��roll pitch yaw rollspeed pitchspeed yawspeed*/
			LogMsg_Attitude = 30 ,
			/*��Ԫ����̬��q0 q1 q2 q3 rollspeed pitchspeed yawspeed*/
			LogMsg_AttitudeQuaternion = 31, 
			/*λ���ٶȣ�posx posy posz xspeed yspeed zspeed (accx accy accz) ��������ˮƽ������(int8_t) + ��ֱ������(int8_t)*/
			LogMsg_LocalPositionNed = 32, 
			
			/*Debug����*/
			LogMsg_DebugVect = 250, 
		};
	
		bool SDLog_Msg_PosSensor( uint8_t ind, Position_Sensor sensor, double SyncTIMEOUT = -1 );
		bool SDLog_Msg_Attitude( double SyncTIMEOUT = -1 );
		bool SDLog_Msg_AttitudeQuaternion( double SyncTIMEOUT = -1 );
		bool SDLog_Msg_LocalPositionNed( double SyncTIMEOUT = -1 );
		
		bool SDLog_Msg_DebugVect( const char* name, double vect[], uint8_t length, double SyncTIMEOUT = -1 );
	/*Log����*/
		
	/*Txt1 Log*/
		bool SDLog_Txt1( const char* txt, uint16_t length, double SyncTIMEOUT = -1 );
	/*Txt1 Log*/
		
	void init_SDStorage();
/*�ⲿ�洢*/
