#include "Basic.hpp"
#include "Modes.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "Receiver.hpp"
#include "Commulink.hpp"
#include "MeasurementSystem.hpp"
#include "StorageSystem.hpp"
#include "queue.h"
#include "mavlink.h"
#include "drv_PWMOut.hpp"
#include "AuxFuncs.hpp"

#include "M10_RCCalib.hpp"
#include "M11_TempCalib.hpp"
#include "M12_AccCalib.hpp"
#include "M13_MagCalib.hpp"

#include "M30_AttCtrl.hpp"
#include "M32_PosCtrl.hpp"
#include "M35_Auto1.hpp"

extern uint16_t mav_mode;
extern uint16_t mav_main_mode;
extern uint16_t mav_sub_mode;

Mode_Base* modes[80] = {0};
static QueueHandle_t message_queue = xQueueCreate( 20, sizeof(ModeMsg) );
bool SendMsgToMode( ModeMsg msg, double TIMEOUT )
{
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xQueueSend( message_queue, &msg, TIMEOUT_Ticks ) == pdTRUE )
		return true;
	else
		return false;
}
bool ModeReceiveMsg( ModeMsg* msg, double TIMEOUT )
{
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xQueueReceive( message_queue, msg, TIMEOUT_Ticks ) == pdTRUE )
		return true;
	else
		return false;
}
static bool changeMode( uint16_t mode_index, void* param1, uint32_t param2, ModeResult* result )
{
	if( modes[mode_index] != 0 )
	{					
		xQueueReset(message_queue);
		if( result != 0 )
			*result = modes[mode_index]->main_func( param1, param2 );
		else
			modes[mode_index]->main_func( param1, param2 );
		xQueueReset(message_queue);
		mav_mode = MAV_MODE_PREFLIGHT;
		mav_main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
		return true;
	}
	return false;
}

static void Modes_Server(void* pvParameters)
{
	//�ȴ�������ʼ�����
	while( getInitializationCompleted() == false )
		os_delay(0.1);
	setLedManualCtrl( 0, 0, 30, true, 800 );
	os_delay(0.2);
	setLedMode(LEDMode_Processing1);
	
	//�ȴ���̬����ϵͳ׼�����
	while( get_Attitude_MSStatus() != MS_Ready )
		os_delay(0.1);
	setLedManualCtrl( 0, 0, 30, true, 1000 );
	os_delay(0.2);
	setLedMode(LEDMode_Processing1);
	
	//�ȴ�λ�ý���ϵͳ׼�����
	while( get_Altitude_MSStatus() != MS_Ready )
		os_delay(0.1);
	setLedManualCtrl( 0, 0, 30, true, 1200 );
	os_delay(0.2);
	setLedMode(LEDMode_Processing1);
	sendLedSignal(LEDSignal_Start2);

	//��ʼ��Aux����
	init_process_AuxFuncs();
	
	//�������ģʽ
	xQueueReset(message_queue);
	mav_mode = MAV_MODE_PREFLIGHT;
	mav_main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
	mav_sub_mode = 0;
	setLedMode(LEDMode_Normal1);
	uint16_t pre_enter_mode_counter = 0;
	uint8_t last_pre_enter_mode = 0;
	while(1)
	{
		os_delay(0.02);
		
		//��ȡ���ջ�
		Receiver rc;
		getReceiver( &rc, 0, 0.02);

		//����Auxͨ��
		process_AuxFuncs(&rc);
		
		//��ȡ��Ϣ
		bool msg_available;
		ModeMsg msg;
		msg_available = ModeReceiveMsg( &msg, 0 );
		
		if( rc.available )
		{
			uint8_t pre_enter_mode = 0;
						
			if( (rc.data[0] < 10.0f) && (rc.data[1] < 10.0f) && (rc.data[2] < 10.0f) && (rc.data[3] < 10.0f) )
				pre_enter_mode = 12;	//���ٶ�У׼
			else if( (rc.data[0] < 10.0f) && (rc.data[1] < 10.0f) && (rc.data[2] > 90.0f) && (rc.data[3] < 10.0f) )
				pre_enter_mode = 13;	//������У׼
			else if( (rc.data[0] < 10.0f) && (rc.data[1] > 90.0f) && (rc.data[2] > 45.0f) && (rc.data[2] < 55.0f) && (rc.data[3] > 90.0f) )
				pre_enter_mode = 11;	//�¶�ϵ��У׼
			else if( (rc.data[0] < 10.0f) && (rc.data[1] > 90.0f) && (rc.data[2] < 10.0f) && (rc.data[3] < 10.0f) )
			{
				if( rc.data[4] < 40 )
					pre_enter_mode = 0;
				else if( rc.data[4] > 70 )
					pre_enter_mode = 32;
				else
					pre_enter_mode = 35;
			}
			
			//��������ģʽ
			if( pre_enter_mode==0 || pre_enter_mode!=last_pre_enter_mode )
				pre_enter_mode_counter = 0;
			else
			{
				if( ++pre_enter_mode_counter >= 50 )
				{
					if( modes[pre_enter_mode] != 0 )
					{						
						sendLedSignal(LEDSignal_Start1);
						changeMode( pre_enter_mode, 0, 0, 0 );
						//ģʽִ����Ϸ��ر�ģʽ
						setLedMode(LEDMode_Normal1);
						last_pre_enter_mode = 0;
						continue;
					}
				}				
			}
			last_pre_enter_mode = pre_enter_mode;
		}
		
		//������Ϣ
		if( msg_available )
		{
			switch( msg.cmd )
			{
				case 176:
				{	//do set mode
					sendLedSignal(LEDSignal_Start1);
					if( msg.params[0] == 0 )
						changeMode( msg.params[1], (void*)(uint32_t)msg.params[3], msg.params[2], 0 );
					else
					{	//mavlink����ģʽ
						px4_custom_mode t_mav_mode;
						t_mav_mode.data = msg.params[1];
						if( t_mav_mode.main_mode==PX4_CUSTOM_MAIN_MODE_AUTO && t_mav_mode.sub_mode==PX4_CUSTOM_SUB_MODE_AUTO_MISSION )
							//����ģʽ
							changeMode( 32, 0, t_mav_mode.data, 0 );
					}
					//ģʽִ����Ϸ��ر�ģʽ
					setLedMode(LEDMode_Normal1);
					last_pre_enter_mode = 0;
					break;
				}
				
				case 22:
				{	//takeoff���
					px4_custom_mode t_mav_mode;
					t_mav_mode.reserved = msg.frame;
					t_mav_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
					t_mav_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF;
					changeMode( 32, msg.params, t_mav_mode.data, 0 );
					break;
				}
				
				case MAV_CMD_COMPONENT_ARM_DISARM:
				{	//����
					if( msg.params[0] == 1 )
						changeMode( 32, 0, 0, 0 );
					break;
				}
				
			}
		}
	}
}

void ModeRegister( Mode_Base* mode, uint8_t id )
{
	if( modes[id] == 0 )
		modes[id] = mode;
}

void init_Modes()
{
	//ע��ģʽ
	new M10_RCCalib();
	new M11_TempCalib();
	new M12_AccCalib();
	new M13_MagCalib();
	
	new M30_AttCtrl();
	new M32_PosCtrl();
	new M35_Auto1();
	
	init_AuxFuncs();
	
	xTaskCreate( Modes_Server, "Modes", 4096, NULL, SysPriority_UserTask, NULL);
}