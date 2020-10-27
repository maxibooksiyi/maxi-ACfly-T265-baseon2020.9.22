#pragma once

#include "Basic.hpp"
#include <stdbool.h>

//���ջ�����
struct Receiver
{
	bool connected;	//�Ƿ�������
	bool available;	//�Ƿ����
	uint8_t available_channels;	//���õ�ͨ����Ŀ
	uint8_t raw_available_channels;	//ԭʼ���ݿ��õ�ͨ����Ŀ
	TIME last_update_time;	//�ϴθ���ʱ��
	double update_time;	//����ʱ����
	
	float raw_data[16];	//ԭʼ����
	float data[8];	//У׼�������
};
/*
	��ȡ�׸����ý��ջ�
	rc����ȡ�Ľ��ջ�
	name����ȡ���ջ����ƣ�Ϊ0����ȡ��
	TIMEOUT����ʱʱ��
*/
bool getReceiver( Receiver* rc, SName* name = 0, double TIMEOUT = -1 );
	
/*
	��ȡָ�����ջ�
	name�����ջ�����
	rc����ȡ�Ľ��ջ�
	TIMEOUT����ʱʱ��
*/
bool getReceiver( SName name, Receiver* rc, double TIMEOUT = -1 );