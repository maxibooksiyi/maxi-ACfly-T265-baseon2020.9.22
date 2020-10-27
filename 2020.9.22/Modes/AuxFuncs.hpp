#pragma once

#include "Basic.hpp"
#include "Receiver.hpp"

/*AuxFuncs
	0:��
	1-16:ӳ��ң������Ӧͨ����raw data��
	25-40:��ң������Ӧͨ������������Ŵ�����raw_data��
	49-64:��ң������Ӧͨ��������̨���ƣ�raw_data��
*/
struct AuxFuncsConfig
{
	uint8_t Aux1Func[8];
	uint8_t Aux2Func[8];
	uint8_t Aux3Func[8];
	uint8_t Aux4Func[8];
	uint8_t Aux5Func[8];
	uint8_t Aux6Func[8];
	uint8_t Aux7Func[8];
	uint8_t Aux8Func[8];
};

//��ʼ��Aux����
void init_process_AuxFuncs();
//����Aux����
void process_AuxFuncs(const Receiver* rc);

//����
bool AuxCamTakePhoto();
//�Զ�������̨�Ƕ�
bool AuxGimbalSetAngle( double angle );

void init_AuxFuncs();