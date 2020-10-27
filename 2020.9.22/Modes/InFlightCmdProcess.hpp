#pragma once

#include "Basic.hpp"
#include "mavlink.h"

/*
	Nav飞行控制指令处理
	所有指令必须在水平位置控制器打开的前提下执行
	所有参数单位角度为度，距离速度为米
	参数：
		params：7个参数数组
	返回：
		true：成功
		false：失败
*/
bool Process_InflightCmd( uint16_t cmd, double params[] );


/*拍照*/
	//控制相机拍照
	void InflightCmd_CamTakePhoto();

	//定距拍照距离
	extern float CamTriggDist;
/*拍照*/