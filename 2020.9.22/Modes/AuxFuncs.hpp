#pragma once

#include "Basic.hpp"
#include "Receiver.hpp"

/*AuxFuncs
	0:无
	1-16:映射遥控器对应通道（raw data）
	25-40:用遥控器对应通道进行相机快门触发（raw_data）
	49-64:用遥控器对应通道进行云台控制（raw_data）
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

//初始化Aux处理
void init_process_AuxFuncs();
//进行Aux处理
void process_AuxFuncs(const Receiver* rc);

//拍照
bool AuxCamTakePhoto();
//自动控制云台角度
bool AuxGimbalSetAngle( double angle );

void init_AuxFuncs();