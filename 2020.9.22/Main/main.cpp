#include "Basic.hpp"
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "drv_Main.hpp"
#include "MS_Main.hpp"
#include "Parameters.hpp"
#include "FlashIO.h"
#include "Modes.hpp"
#include "MSafe.hpp"
#include "ctrl_Main.hpp"
#include "drv_PWMOut.hpp"
#include "AC_Math.hpp"
#include "Missions.hpp"

#include "debug.hpp"

#if 1 //如果没有这段，则需要在target选项中选择使用USE microLIB

	__asm(".global __use_no_semihosting\n\t") ;//注释本行, 方法1
	extern "C"
	{
//		struct __FILE {
//		int handle;
//		};
//		std::FILE __stdout;

		void _sys_exit(int x)
		{
			x = x;
		}

		//__use_no_semihosting was requested, but _ttywrch was referenced, 增加如下函数, 方法2
		void _ttywrch(int ch)
		{
			ch = ch;
		}
		
		char *_sys_command_string(char *cmd, int len)
		{
				return 0;
		}
 
	}
#endif
	


void DriverInit_task(void* pvParameters)
{
	//初始化设备驱动
	init_drv_Main();
	//初始化解算系统
	init_MS_Main();
	init_Debug();
	init_Modes();
	init_MSafe();
	init_ControlSystem();	
	
	//注册初始化参数
	struct
	{
		uint32_t calib_ESC[2];	//校准电调
	}init_cfg;
	memset( &init_cfg, 0, sizeof(init_cfg) );
	MAV_PARAM_TYPE param_types[] = {
		MAV_PARAM_TYPE_UINT32 ,	//校准电调
	};
	SName param_names[] = {
		"Init_CalibESC",	//校准电调
	};
	ParamGroupRegister( "Init", 1, 1, param_types, param_names, (uint64_t*)&init_cfg );
	
	//完成初始化
	//完成后不能再进行初始化操作
	while( getInitializationCompleted() == false )
	{
		setInitializationCompleted();
		os_delay(0.1);
	}
	
	/*读取初始化参数进行初始化操作*/
		//读初始化参数
		ReadParamGroup( "Init", (uint64_t*)&init_cfg, 0 );
	
		//校准电调
		if( init_cfg.calib_ESC[0] == 21586 )
		{
			PWM_PullUpAll();
			os_delay(3.0);
			PWM_PullDownAll();
		}
	
		//重置初始化参数
		bool need_reset_init_param = false;
		for( uint8_t i = 0; i < sizeof(init_cfg); ++i )
		{
			if( ((uint8_t*)&init_cfg)[i] != 0 )
			{
				((uint8_t*)&init_cfg)[i] = 0;
				need_reset_init_param = true;
			}
		}
		if( need_reset_init_param )
			UpdateParamGroup( "Init", (uint64_t*)&init_cfg, 0, sizeof(init_cfg)/8 );
		
		//拉低电机输出
		PWM_PullDownAll();
	/*读取初始化参数进行初始化操作*/
	
	//飞行任务初始化
	init_Missions();
	
	//删除本任务
	vTaskDelete(0);
}
	
int main(void)
{	
	//初始化芯片时钟
	//时间基准等基础功能
  init_Basic();
			
	//创建初始化任务并进入任务调度器
	xTaskCreate( DriverInit_task , "Init" ,8192,NULL,3,NULL);
	vTaskStartScheduler();
	while(1);
}

extern "C" void HardFault_Handler()
{
	//错误中断拉低所有输出
	PWM_PullDownAll();
}

extern "C" void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
	static char StackOvTaskName[20];
	strcpy( StackOvTaskName, (char*)pcTaskName );
}



