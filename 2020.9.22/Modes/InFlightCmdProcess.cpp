#include "InFlightCmdProcess.hpp"
#include "Basic.hpp"
#include "MeasurementSystem.hpp"
#include "ControlSystem.hpp"
#include "mavlink.h"
#include "Parameters.hpp"
#include "AuxFuncs.hpp"
#include "StorageSystem.hpp"

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

/*InflightCmd178_MAV_CMD_DO_CHANGE_SPEED
	MAV_CMD_DO_CHANGE_SPEED
	更改飞行速度
	参数:
		<description>Change speed and/or throttle set points.</description>
		<param index="1" label="Speed Type" minValue="0" maxValue="3" increment="1">Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)</param>
		<param index="2" label="Speed" units="m/s" minValue="-1">Speed (-1 indicates no change)</param>
		<param index="3" label="Throttle" units="%" minValue="-1">Throttle (-1 indicates no change)</param>
		<param index="4" label="Relative" minValue="0" maxValue="1" increment="1">0: absolute, 1: relative</param>
		<param index="5">Empty</param>
		<param index="6">Empty</param>
		<param index="7">Empty</param>
*/
static bool InflightCmd178_MAV_CMD_DO_CHANGE_SPEED( uint16_t cmd, double params[] )
{
	if( params[0]==0 || params[0]==1 )
	{	//XY速度
		if( params[1] != -1 )
		{
			if( params[3] == 0 )
			{	//绝对速度
				Position_Control_set_XYAutoSpeed( params[1]*100 );
				Position_Control_set_XYZAutoSpeed( params[1]*100 );
				return true;
			}
			else if( params[3] == 1 )
			{	//相对当前
				double sp;
				Position_Control_get_XYAutoSpeed(&sp);
				Position_Control_set_XYAutoSpeed( sp + params[1]*100 );
				Position_Control_get_XYZAutoSpeed(&sp);
				Position_Control_set_XYZAutoSpeed( sp + params[1]*100 );
				return true;
			}
		}
	}
	else if( params[0]==2 )
	{	//上升速度
		if( params[1] != -1 )
		{
			if( params[3] == 0 )
			{	//绝对速度
				Position_Control_set_ZAutoSpeed( params[1]*100, -1 );
				return true;
			}
			else if( params[3] == 1 )
			{	//相对当前
				double spUp, spDn;
				Position_Control_get_ZAutoSpeed( &spUp, &spDn );
				Position_Control_set_ZAutoSpeed( spUp + params[1]*100, -1 );
				return true;
			}
		}
	}
	else if( params[0]==3 )
	{	//下降速度
		if( params[1] != -1 )
		{
			if( params[3] == 0 )
			{	//绝对速度
				Position_Control_set_ZAutoSpeed( -1, params[1]*100 );
				return true;
			}
			else if( params[3] == 1 )
			{	//相对当前
				double spUp, spDn;
				Position_Control_get_ZAutoSpeed( &spDn, &spDn );
				Position_Control_set_ZAutoSpeed( -1, spDn + params[1]*100 );
				return true;
			}
		}
	}
	return false;
}

/*拍照*/

	void InflightCmd_CamTakePhoto()
	{			
		AuxCamTakePhoto();
	}
	
	void InflightCmd_SetGimbalPitch( double angle )
	{		
		AuxGimbalSetAngle(angle);
	}

	//定距拍照距离
	float CamTriggDist = 0;
	
	/*InflightCmd205_MAV_CMD_DO_MOUNT_CONTROL
		MAV_CMD_DO_MOUNT_CONTROL
		控制云台
		参数:
			<deprecated since="2020-01" replaced_by="MAV_CMD_DO_GIMBAL_MANAGER_ATTITUDE">This message is ambiguous and inconsistent. It has been superseded by MAV_CMD_DO_GIMBAL_MANAGER_ATTITUDE and MAV_CMD_DO_SET_ROI_*. The message can still be used to communicate with legacy gimbals implementing it.</deprecated>
			<description>Mission command to control a camera or antenna mount</description>
			<param index="1" label="Pitch">pitch depending on mount mode (degrees or degrees/second depending on pitch input).</param>
			<param index="2" label="Roll">roll depending on mount mode (degrees or degrees/second depending on roll input).</param>
			<param index="3" label="Yaw">yaw depending on mount mode (degrees or degrees/second depending on yaw input).</param>
			<param index="4" label="Altitude" units="m">altitude depending on mount mode.</param>
			<param index="5" label="Latitude">latitude, set if appropriate mount mode.</param>
			<param index="6" label="Longitude">longitude, set if appropriate mount mode.</param>
			<param index="7" label="Mode" enum="MAV_MOUNT_MODE">Mount mode.</param>
	*/
	static bool InflightCmd205_MAV_CMD_DO_MOUNT_CONTROL( uint16_t cmd, double params[] )
	{
		InflightCmd_SetGimbalPitch( params[0] );
		return true;
	}
	
	/*InflightCmd206_MAV_CMD_DO_SET_CAM_TRIGG_DIST
		MAV_CMD_DO_SET_CAM_TRIGG_DIST
		设置定距拍照参数
		参数:
			<description>Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera.</description>
			<param index="1" label="Distance" units="m" minValue="0">Camera trigger distance. 0 to stop triggering.</param>
			<param index="2" label="Shutter" units="ms" minValue="-1" increment="1">Camera shutter integration time. -1 or 0 to ignore</param>
			<param index="3" label="Trigger" minValue="0" maxValue="1" increment="1">Trigger camera once immediately. (0 = no trigger, 1 = trigger)</param>
			<param index="4">Empty</param>
			<param index="5">Empty</param>
			<param index="6">Empty</param>
			<param index="7">Empty</param>
	*/
	static bool InflightCmd206_MAV_CMD_DO_SET_CAM_TRIGG_DIST( uint16_t cmd, double params[] )
	{
		CamTriggDist = params[0]*100;
		return true;
	}
/*拍照*/

static bool (*const InflightCmdProcess[])( uint16_t cmd, double params[] ) = 
{
	/*000-*/	0	,
	/*001-*/	0	,
	/*002-*/	0	,
	/*003-*/	0	,
	/*004-*/	0	,
	/*005-*/	0	,
	/*006-*/	0	,
	/*007-*/	0	,
	/*008-*/	0	,
	/*009-*/	0	,
	/*010-*/	0	,
	/*011-*/	0	,
	/*012-*/	0	,
	/*013-*/	0	,
	/*014-*/	0	,
	/*015-*/	0	,
	/*016-*/	0	,
	/*017-*/	0	,
	/*018-*/	0	,
	/*019-*/	0	,
	/*020-*/	0	,
	/*021-*/	0	,
	/*022-*/	0	,
	/*023-*/	0	,
	/*024-*/	0	,
	/*025-*/	0	,
	/*026-*/	0	,
	/*027-*/	0	,
	/*028-*/	0	,
	/*029-*/	0	,
	/*030-*/	0	,
	/*031-*/	0	,
	/*032-*/	0	,
	/*033-*/	0	,
	/*034-*/	0	,
	/*035-*/	0	,
	/*036-*/	0	,
	/*037-*/	0	,
	/*038-*/	0	,
	/*039-*/	0	,
	/*040-*/	0	,
	/*041-*/	0	,
	/*042-*/	0	,
	/*043-*/	0	,
	/*044-*/	0	,
	/*045-*/	0	,
	/*046-*/	0	,
	/*047-*/	0	,
	/*048-*/	0	,
	/*049-*/	0	,
	/*050-*/	0	,
	/*051-*/	0	,
	/*052-*/	0	,
	/*053-*/	0	,
	/*054-*/	0	,
	/*055-*/	0	,
	/*056-*/	0	,
	/*057-*/	0	,
	/*058-*/	0	,
	/*059-*/	0	,
	/*060-*/	0	,
	/*061-*/	0	,
	/*062-*/	0	,
	/*063-*/	0	,
	/*064-*/	0	,
	/*065-*/	0	,
	/*066-*/	0	,
	/*067-*/	0	,
	/*068-*/	0	,
	/*069-*/	0	,
	/*070-*/	0	,
	/*071-*/	0	,
	/*072-*/	0	,
	/*073-*/	0	,
	/*074-*/	0	,
	/*075-*/	0	,
	/*076-*/	0	,
	/*077-*/	0	,
	/*078-*/	0	,
	/*079-*/	0	,
	/*080-*/	0	,
	/*081-*/	0	,
	/*082-*/	0	,
	/*083-*/	0	,
	/*084-*/	0	,
	/*085-*/	0	,
	/*086-*/	0	,
	/*087-*/	0	,
	/*088-*/	0	,
	/*089-*/	0	,
	/*090-*/	0	,
	/*091-*/	0	,
	/*092-*/	0	,
	/*093-*/	0	,
	/*094-*/	0	,
	/*095-*/	0	,
	/*096-*/	0	,
	/*097-*/	0	,
	/*098-*/	0	,
	/*099-*/	0	,
	/*100-*/	0	,
	/*101-*/	0	,
	/*102-*/	0	,
	/*103-*/	0	,
	/*104-*/	0	,
	/*105-*/	0	,
	/*106-*/	0	,
	/*107-*/	0	,
	/*108-*/	0	,
	/*109-*/	0	,
	/*110-*/	0	,
	/*111-*/	0	,
	/*112-*/	0	,
	/*113-*/	0	,
	/*114-*/	0	,
	/*115-*/	0	,
	/*116-*/	0	,
	/*117-*/	0	,
	/*118-*/	0	,
	/*119-*/	0	,
	/*120-*/	0	,
	/*121-*/	0	,
	/*122-*/	0	,
	/*123-*/	0	,
	/*124-*/	0	,
	/*125-*/	0	,
	/*126-*/	0	,
	/*127-*/	0	,
	/*128-*/	0	,
	/*129-*/	0	,
	/*130-*/	0	,
	/*131-*/	0	,
	/*132-*/	0	,
	/*133-*/	0	,
	/*134-*/	0	,
	/*135-*/	0	,
	/*136-*/	0	,
	/*137-*/	0	,
	/*138-*/	0	,
	/*139-*/	0	,
	/*140-*/	0	,
	/*141-*/	0	,
	/*142-*/	0	,
	/*143-*/	0	,
	/*144-*/	0	,
	/*145-*/	0	,
	/*146-*/	0	,
	/*147-*/	0	,
	/*148-*/	0	,
	/*149-*/	0	,
	/*150-*/	0	,
	/*151-*/	0	,
	/*152-*/	0	,
	/*153-*/	0	,
	/*154-*/	0	,
	/*155-*/	0	,
	/*156-*/	0	,
	/*157-*/	0	,
	/*158-*/	0	,
	/*159-*/	0	,
	/*160-*/	0	,
	/*161-*/	0	,
	/*162-*/	0	,
	/*163-*/	0	,
	/*164-*/	0	,
	/*165-*/	0	,
	/*166-*/	0	,
	/*167-*/	0	,
	/*168-*/	0	,
	/*169-*/	0	,
	/*170-*/	0	,
	/*171-*/	0	,
	/*172-*/	0	,
	/*173-*/	0	,
	/*174-*/	0	,
	/*175-*/	0	,
	/*176-*/	0	,
	/*177-*/	0	,
	/*178-*/	InflightCmd178_MAV_CMD_DO_CHANGE_SPEED	,
	/*179-*/	0	,
	/*180-*/	0	,
	/*181-*/	0	,
	/*182-*/	0	,
	/*183-*/	0	,
	/*184-*/	0	,
	/*185-*/	0	,
	/*186-*/	0	,
	/*187-*/	0	,
	/*188-*/	0	,
	/*189-*/	0	,
	/*190-*/	0	,
	/*191-*/	0	,
	/*192-*/	0	,
	/*193-*/	0	,
	/*194-*/	0	,
	/*195-*/	0	,
	/*196-*/	0	,
	/*197-*/	0	,
	/*198-*/	0	,
	/*199-*/	0	,
	/*200-*/	0	,
	/*201-*/	0	,
	/*202-*/	0	,
	/*203-*/	0	,
	/*204-*/	0	,
	/*205-*/	InflightCmd205_MAV_CMD_DO_MOUNT_CONTROL	,
	/*206-*/	InflightCmd206_MAV_CMD_DO_SET_CAM_TRIGG_DIST	,
	/*207-*/	0	,
	/*208-*/	0	,
	/*209-*/	0	,
	/*210-*/	0	,
	/*211-*/	0	,
	/*212-*/	0	,
	/*213-*/	0	,
	/*214-*/	0	,
	/*215-*/	0	,
	/*216-*/	0	,
	/*217-*/	0	,
	/*218-*/	0	,
	/*219-*/	0	,
	/*220-*/	0	,
	/*221-*/	0	,
	/*222-*/	0	,
	/*223-*/	0	,
	/*224-*/	0	,
	/*225-*/	0	,
	/*226-*/	0	,
	/*227-*/	0	,
	/*228-*/	0	,
	/*229-*/	0	,
	/*230-*/	0	,
	/*231-*/	0	,
	/*232-*/	0	,
	/*233-*/	0	,
	/*234-*/	0	,
	/*235-*/	0	,
	/*236-*/	0	,
	/*237-*/	0	,
	/*238-*/	0	,
	/*239-*/	0	,
	/*240-*/	0	,
	/*241-*/	0	,
	/*242-*/	0	,
	/*243-*/	0	,
	/*244-*/	0	,
	/*245-*/	0	,
	/*246-*/	0	,
	/*247-*/	0	,
	/*248-*/	0	,
	/*249-*/	0	,
	/*250-*/	0	,
	/*251-*/	0	,
	/*252-*/	0	,
	/*253-*/	0	,
	/*254-*/	0	,
	/*255-*/	0	,
	/*256-*/	0	,
	/*257-*/	0	,
	/*258-*/	0	,
	/*259-*/	0	,
	/*260-*/	0	,
	/*261-*/	0	,
	/*262-*/	0	,
	/*263-*/	0	,
	/*264-*/	0	,
	/*265-*/	0	,
	/*266-*/	0	,
	/*267-*/	0	,
	/*268-*/	0	,
	/*269-*/	0	,
	/*270-*/	0	,
	/*271-*/	0	,
	/*272-*/	0	,
	/*273-*/	0	,
	/*274-*/	0	,
	/*275-*/	0	,
	/*276-*/	0	,
	/*277-*/	0	,
	/*278-*/	0	,
	/*279-*/	0	,
	/*280-*/	0	,
	/*281-*/	0	,
	/*282-*/	0	,
	/*283-*/	0	,
	/*284-*/	0	,
	/*285-*/	0	,
	/*286-*/	0	,
	/*287-*/	0	,
	/*288-*/	0	,
	/*289-*/	0	,
	/*290-*/	0	,
	/*291-*/	0	,
	/*292-*/	0	,
	/*293-*/	0	,
	/*294-*/	0	,
	/*295-*/	0	,
	/*296-*/	0	,
	/*297-*/	0	,
	/*298-*/	0	,
	/*299-*/	0	,
	/*300-*/	0	,
	/*400-*/	0	,
	/*401-*/	0	,
	/*402-*/	0	,
	/*403-*/	0	,
	/*404-*/	0	,
	/*405-*/	0	,
	/*406-*/	0	,
	/*407-*/	0	,
	/*408-*/	0	,
	/*409-*/	0	,
	/*410-*/	0	,
	/*411-*/	0	,
	/*412-*/	0	,
	/*413-*/	0	,
	/*414-*/	0	,
	/*415-*/	0	,
	/*416-*/	0	,
	/*417-*/	0	,
	/*418-*/	0	,
	/*419-*/	0	,
	/*420-*/	0	,
	/*421-*/	0	,
	/*422-*/	0	,
	/*423-*/	0	,
	/*424-*/	0	,
	/*425-*/	0	,
	/*426-*/	0	,
	/*427-*/	0	,
	/*428-*/	0	,
	/*429-*/	0	,
	/*430-*/	0	,
	/*431-*/	0	,
	/*432-*/	0	,
	/*433-*/	0	,
	/*434-*/	0	,
	/*435-*/	0	,
	/*436-*/	0	,
	/*437-*/	0	,
	/*438-*/	0	,
	/*439-*/	0	,
	/*440-*/	0	,
	/*441-*/	0	,
	/*442-*/	0	,
	/*443-*/	0	,
	/*444-*/	0	,
	/*445-*/	0	,
	/*446-*/	0	,
	/*447-*/	0	,
	/*448-*/	0	,
	/*449-*/	0	,
	/*450-*/	0	,
	/*451-*/	0	,
	/*452-*/	0	,
	/*453-*/	0	,
	/*454-*/	0	,
	/*455-*/	0	,
	/*456-*/	0	,
	/*457-*/	0	,
	/*458-*/	0	,
	/*459-*/	0	,
	/*460-*/	0	,
	/*461-*/	0	,
	/*462-*/	0	,
	/*463-*/	0	,
	/*464-*/	0	,
	/*465-*/	0	,
	/*466-*/	0	,
	/*467-*/	0	,
	/*468-*/	0	,
	/*469-*/	0	,
	/*470-*/	0	,
	/*471-*/	0	,
	/*472-*/	0	,
	/*473-*/	0	,
	/*474-*/	0	,
	/*475-*/	0	,
	/*476-*/	0	,
	/*477-*/	0	,
	/*478-*/	0	,
	/*479-*/	0	,
	/*480-*/	0	,
	/*481-*/	0	,
	/*482-*/	0	,
	/*483-*/	0	,
	/*484-*/	0	,
	/*485-*/	0	,
	/*486-*/	0	,
	/*487-*/	0	,
	/*488-*/	0	,
	/*489-*/	0	,
	/*490-*/	0	,
	/*491-*/	0	,
	/*492-*/	0	,
	/*493-*/	0	,
	/*494-*/	0	,
	/*495-*/	0	,
	/*496-*/	0	,
	/*497-*/	0	,
	/*498-*/	0	,
	/*499-*/	0	,
};
const uint16_t NavCmdProcess_Count = sizeof( InflightCmdProcess ) / sizeof( void* );

bool Process_InflightCmd( uint16_t cmd, double params[] )
{
	//无此指令返回错误
	if( cmd>=NavCmdProcess_Count || InflightCmdProcess[cmd]==0 )
		return false;
	
	return InflightCmdProcess[cmd]( cmd, params );
}