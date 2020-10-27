#pragma once

#include "Basic.hpp"

struct MissionInf
{
	uint16_t cmd;
	uint8_t frame;
	uint8_t autocontinue;
	uint8_t rsv[4];
	double params[7];
}__attribute__((__packed__));


/*
	���õ�ǰ����
	wpInd����ǰ�������
*/
bool setCurrentMission( uint16_t wpInd );
/*
	��ȡ��ǰ�������
*/
uint16_t getCurrentMissionInd();

/*
	��ȡ�����������
*/
uint16_t getMissionsCount();
/*
	��ȡ�����ϴ��������
*/
uint16_t getUploadingMissionsCount();

/*
	������к�������
	TIMEOUT: �߳�ͬ����ʱʱ��

	����ֵ��
	true:��ӳɹ�
	false:���ʧ�ܣ��������򺽵���Ϣ�������������
*/
bool clearMissions( double TIMEOUT = -1 );

/*
	��Ӻ�������
	wp_inf��������Ϣ
	st���Ƿ�д��洢����ֻ�е�ǰʵ�ʺ�������Ϊ0�ſ��Ի��治д��洢����
	TIMEOUT: �߳�ͬ����ʱʱ��

	����ֵ��
	true:��ӳɹ�
	false:���ʧ�ܣ��������򺽵���Ϣ�������������
*/
bool addMission( MissionInf wp_inf, bool st = true, double TIMEOUT = -1 );

/*
	���溽������
	TIMEOUT: �߳�ͬ����ʱʱ��

	����ֵ��
	true:����ɹ�
	false:����ʧ�ܣ���ʱ��
*/
bool saveMissions( double TIMEOUT = -1 );

/*
	��ȡ��������
	wp_ind���������
	wp_inf����ȡ�ĺ�����Ϣ
	TIMEOUT: �߳�ͬ����ʱʱ��

	����ֵ��
	true:��ȡ�ɹ�
	false:��ȡʧ�ܣ��޺������������
*/
bool ReadMission( uint16_t wp_ind, MissionInf* wp_inf, double TIMEOUT = -1 );

/*
	��ȡ��ǰ��������
	wp_inf����ȡ�ĺ�����Ϣ
	ind����ǰ�������
	TIMEOUT: �߳�ͬ����ʱʱ��

	����ֵ��
	true:��ȡ�ɹ�
	false:��ȡʧ�ܣ��޺������������
*/
bool ReadCurrentMission( MissionInf* wp_inf, uint16_t* ind, double TIMEOUT = -1 );

void init_Missions();