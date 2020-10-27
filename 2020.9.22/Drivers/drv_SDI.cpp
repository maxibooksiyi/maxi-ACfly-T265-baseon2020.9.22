#include "drv_Uart3.hpp"
#include "drv_Uart5.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "Modes.hpp"
#include "Commulink.hpp"
#include "mavlink.h"

struct header_pack
{
	uint8_t header;
	uint8_t len;
	uint8_t fun;
	uint8_t id;
	uint8_t order;
}__attribute__((packed));

struct takeoff_pack
{
	header_pack header;
	float height;
}__attribute__((packed));

struct goto_pack
{
	header_pack header;
	int32_t lat;
	int32_t lon;
	float z;
}__attribute__((packed));

static void SDI_Server(void* pvParameters)
{
	/*状态机变量*/
		static uint8_t rc_step1 = 0;	//0：接收包头0xBB或0xAA
																	//1：接收1字节长度
																	//2：接收数据包内容
																	//3：接收校验
		static uint8_t rc_step2 = 0;
	
		#define MAX_SDI_PACKET_SIZE 255
		static uint8_t msg_type;
		static uint8_t msg_length;
		ALIGN4 static uint8_t msg_pack[MAX_SDI_PACKET_SIZE];
		static uint8_t sum;
		
		#define reset_SDI_RC ( rc_step1 = rc_step2 = 0 )
	/*状态机变量*/
	
	while(1)
	{
		uint8_t r_data;
		if( Read_Uart3( &r_data, 1, 2, 0.5 ) )
		{
			switch( rc_step1 )
			{
				case 0 :					
					if( r_data==0xBB || r_data==0xAA )
					{
						msg_pack[ 0 ] = r_data;
						rc_step1 = 1;
						rc_step2 = 0;
						sum = r_data;
					}
					else
						rc_step2 = 0;
					break;
					
				case 1:
					//接收消息长度
					msg_pack[ 1 ] = r_data;
					msg_length = r_data;
					sum += r_data;
					rc_step1 = 2;
					rc_step2 = 0;
					break;
				
				case 2:
					//接收消息内容
					if( rc_step2==0 && ( ( msg_pack[0]==0xBB && r_data!=0xf4 ) || ( msg_pack[0]==0xAA && r_data!=0x31 ) ) )
					{	//校验功能字不正确
						reset_SDI_RC;	
						break;
					}
					msg_pack[ rc_step2+2 ] = r_data;				
					sum += r_data;
					++rc_step2;
					if( rc_step2 >= msg_length )
					{
						rc_step1 = 3;
						rc_step2 = 0;
					}
					break;
					
				case 3:					
				{	//校验
					msg_pack[ msg_length+2 ] = r_data;
					header_pack* pack = (header_pack*)msg_pack;
					if( sum == r_data )
					{	//校验成功
						if( pack->id!=get_CommulinkSysId() || pack->header==0xAA )
						{	//id不为本机从另一端口转发出去														
							Write_Uart5( msg_pack, msg_length+2+1, 0.1, 0.1 );
						}
						else
						{	//id为本机id
							switch(pack->order)
							{
								case 0x00:
								{	//起飞
									takeoff_pack* pack = (takeoff_pack*)msg_pack;
									
									ModeMsg msg;
									msg.cmd = 176;
									msg.params[0] = 0;
									msg.params[1] = 35;
									msg.params[2] = pack->height;
									SendMsgToMode(msg,0.1);
									break;
								}
								case 0x01:
								{	//直达
									goto_pack* pack = (goto_pack*)msg_pack;
									
									ModeMsg msg;
									msg.cmd = MAV_CMD_NAV_WAYPOINT;
									msg.params[4] = pack->lat*1e-7;
									msg.params[5] = pack->lon*1e-7;
									msg.params[6] = pack->z*100;
									SendMsgToMode(msg,0.1);
									break;
								}
								case 0x02:
								{	//降落
									ModeMsg msg;
									msg.cmd = MAV_CMD_NAV_LAND;
									SendMsgToMode(msg,0.1);
									break;
								}
							}
						}
					}
					reset_SDI_RC;	
					break;
				}
			}					
		}
	}
}

static void SDI2_Server(void* pvParameters)
{
	/*状态机变量*/
		static uint8_t rc_step1 = 0;	//0：接收包头0xBB或0xAA
																	//1：接收1字节长度
																	//2：接收数据包内容
																	//3：接收校验
		static uint8_t rc_step2 = 0;
	
		#define MAX_SDI_PACKET_SIZE 255
		static uint8_t msg_type;
		static uint8_t msg_length;
		ALIGN4 static uint8_t msg_pack[MAX_SDI_PACKET_SIZE];
		static uint8_t sum;
		
		#define reset_SDI_RC ( rc_step1 = rc_step2 = 0 )
	/*状态机变量*/
	
	while(1)
	{
		uint8_t r_data;
		if( Read_Uart5( &r_data, 1, 2, 0.5 ) )
		{
			switch( rc_step1 )
			{
				case 0 :					
					if( r_data==0xBB || r_data==0xAA )
					{
						msg_pack[ 0 ] = r_data;
						rc_step1 = 1;
						rc_step2 = 0;
						sum = r_data;
					}
					else
						rc_step2 = 0;
					break;
					
				case 1:
					//接收消息长度
					msg_pack[ 1 ] = r_data;
					msg_length = r_data;
					sum += r_data;
					rc_step1 = 2;
					rc_step2 = 0;
					break;
				
				case 2:
					//接收消息内容
					if( rc_step2==0 && ( ( msg_pack[0]==0xBB && r_data!=0xf4 ) || ( msg_pack[0]==0xAA && r_data!=0x31 ) ) )
					{	//校验功能字不正确
						reset_SDI_RC;	
						break;
					}
					msg_pack[ rc_step2+2 ] = r_data;				
					sum += r_data;
					++rc_step2;
					if( rc_step2 >= msg_length )
					{
						rc_step1 = 3;
						rc_step2 = 0;
					}
					break;
					
				case 3:					
				{	//校验
					msg_pack[ msg_length+2 ] = r_data;
					header_pack* pack = (header_pack*)msg_pack;
					if( sum == r_data )
					{	//校验成功
						if( pack->id!=get_CommulinkSysId() || pack->header==0xAA )
						{	//id不为本机从另一端口转发出去														
							Write_Uart3( msg_pack, msg_length+2+1, 0.1, 0.1 );
						}
						else
						{	//id为本机id
							switch(pack->order)
							{
								case 0x00:
								{	//起飞
									takeoff_pack* pack = (takeoff_pack*)msg_pack;
									
									ModeMsg msg;
									msg.cmd = 176;
									msg.params[0] = 0;
									msg.params[1] = 35;
									msg.params[2] = pack->height;
									SendMsgToMode(msg,0.1);
									break;
								}
								case 0x01:
								{	//直达
									goto_pack* pack = (goto_pack*)msg_pack;
									
									ModeMsg msg;
									msg.cmd = MAV_CMD_NAV_WAYPOINT;
									msg.params[4] = pack->lat*1e-7;
									msg.params[5] = pack->lon*1e-7;
									msg.params[6] = pack->z*100;
									SendMsgToMode(msg,0.1);
									break;
								}
								case 0x02:
								{	//降落
									ModeMsg msg;
									msg.cmd = MAV_CMD_NAV_LAND;
									SendMsgToMode(msg,0.1);
									break;
								}
							}
						}
					}
					reset_SDI_RC;	
					break;
				}
			}					
		}
	}
}

void init_drv_SDI()
{
	//波特率115200
	SetBaudRate_Uart3( 115200, 2, 2 );
	xTaskCreate( SDI_Server, "SDI", 1024, NULL, SysPriority_UserTask, NULL);
}