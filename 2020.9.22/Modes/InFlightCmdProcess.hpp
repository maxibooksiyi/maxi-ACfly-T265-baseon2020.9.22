#pragma once

#include "Basic.hpp"
#include "mavlink.h"

/*
	Nav���п���ָ���
	����ָ�������ˮƽλ�ÿ������򿪵�ǰ����ִ��
	���в�����λ�Ƕ�Ϊ�ȣ������ٶ�Ϊ��
	������
		params��7����������
	���أ�
		true���ɹ�
		false��ʧ��
*/
bool Process_InflightCmd( uint16_t cmd, double params[] );


/*����*/
	//�����������
	void InflightCmd_CamTakePhoto();

	//�������վ���
	extern float CamTriggDist;
/*����*/