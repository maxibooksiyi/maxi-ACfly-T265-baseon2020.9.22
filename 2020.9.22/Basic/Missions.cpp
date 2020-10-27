#include "Missions.hpp"

#include "Parameters.hpp"
#include "semphr.h"

#define MissionParamVersion 1
#define MaxMissions 512
#define MissionsInParamGroupBit 5
#define MissionsInParamGroup (1<<MissionsInParamGroupBit)

static SemaphoreHandle_t MissionsSemphr = xSemaphoreCreateMutex();

static inline bool Lock_Missions( double TIMEOUT = -1 )
{
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake( MissionsSemphr , TIMEOUT_Ticks ) )
		return true;
	return false;
}
static inline void UnLock_Missions()
{
	xSemaphoreGive(MissionsSemphr);
}

//�������
static uint16_t MissionsCount = 0;
static uint16_t UploadingMissionsCount = 0;

//��ǰ����
static uint16_t CurrentMission = 0;
/*
	���õ�ǰ����
	wpInd����ǰ�������
*/
bool setCurrentMission( uint16_t wpInd )
{
	if( MissionsCount==0 && wpInd==0 )
	{
		CurrentMission = 0;
		return false;
	}
	if( wpInd >= MissionsCount )
		return false;
	CurrentMission = wpInd;
	return true;
}
/*
	��ȡ��ǰ�������
*/
uint16_t getCurrentMissionInd() { return CurrentMission; }

/*
	��ȡ�������
*/
uint16_t getMissionsCount()
{
	return MissionsCount;
}
/*
	��ȡ�����ϴ��������
*/
uint16_t getUploadingMissionsCount()
{
	return UploadingMissionsCount;
}

/*
	������к�������
	TIMEOUT: �߳�ͬ����ʱʱ��

	����ֵ��
	true:��ӳɹ�
	false:���ʧ�ܣ��������򺽵���Ϣ�������������
*/
bool clearMissions( double TIMEOUT )
{
	if( Lock_Missions(TIMEOUT) )
	{
		MissionInf wp_inf;
		wp_inf.cmd = 0;
		UpdateParamGroup( "Mission_0", (uint64_t*)&wp_inf, 0*8, 8 );
		
		CurrentMission = MissionsCount = UploadingMissionsCount = 0;
		
		UnLock_Missions();
		return true;
	}
	return false;
}

/*
	��Ӻ�������
	wp_inf��������Ϣ
	st���Ƿ�д��洢����ֻ�е�ǰʵ�ʺ�������Ϊ0�ſ��Ի��治д��洢����
	TIMEOUT: �߳�ͬ����ʱʱ��

	����ֵ��
	true:��ӳɹ�
	false:���ʧ�ܣ��������򺽵���Ϣ�������������
*/
bool addMission( MissionInf wp_inf, bool st, double TIMEOUT )
{
	if( wp_inf.cmd == 0 )
		return false;
	if( MissionsCount > MaxMissions )
		return false;
	if( st==false && MissionsCount>0 )
		return false;
	if( st && MissionsCount!=UploadingMissionsCount )
		return false;
	
	if( Lock_Missions(TIMEOUT) )
	{
		//ע�������
		uint16_t ParamGroupInd = UploadingMissionsCount >> MissionsInParamGroupBit;
		char WPGroupName[17];
		sprintf( WPGroupName, "Mission_%d", ParamGroupInd );
		MissionInf wp_infs[MissionsInParamGroup];
		for( uint8_t i = 0; i < MissionsInParamGroup; ++i )
			wp_infs[i].cmd = 0;
		PR_RESULT res = ParamGroupRegister( SName(WPGroupName), MissionParamVersion, MissionsInParamGroup*8, 0, 0, (uint64_t*)wp_infs );
		if( res == PR_ERR )
			return false;
		
		MissionInf st_infs[2];
		st_infs[0] = wp_inf;
		st_infs[1].cmd = 0;
		
		//���ĵ�ǰ����
		uint16_t wpInd = UploadingMissionsCount & (MissionsInParamGroup-1);
		if( wpInd < MissionsInParamGroup - 1 )
			UpdateParamGroup( SName(WPGroupName), (uint64_t*)&st_infs[0], wpInd*8, 8*2, st );
		else
		{
			UpdateParamGroup( SName(WPGroupName), (uint64_t*)&st_infs[0], wpInd*8, 8, st );
			//����һ�������cmd��Ϊ0
			sprintf( WPGroupName, "Mission_%d", ParamGroupInd+1 );
			UpdateParamGroup( SName(WPGroupName), (uint64_t*)&st_infs[1], 0*8, 8, st );
		}

		if(st)
		{
			++MissionsCount;
			UploadingMissionsCount = MissionsCount;
		}
		else
			++UploadingMissionsCount;
		
		UnLock_Missions();
		return true;
	}
	return false;
}

/*
	���溽������
	TIMEOUT: �߳�ͬ����ʱʱ��

	����ֵ��
	true:����ɹ�
	false:����ʧ�ܣ���ʱ��
*/
bool saveMissions( double TIMEOUT )
{
	if( UploadingMissionsCount == 0 )
		return true;
	if( Lock_Missions(TIMEOUT) )
	{
		uint16_t ParamGroupInd = (UploadingMissionsCount-1) >> MissionsInParamGroupBit;
		for( uint16_t i = 0; i <= ParamGroupInd; ++i )
		{
			char WPGroupName[17];
			sprintf( WPGroupName, "Mission_%d", i );
			SaveParamGroup( WPGroupName );
		}
		MissionsCount = UploadingMissionsCount;
		
		UnLock_Missions();
		return true;
	}
	return false;
}

/*
	��ȡ��������
	wp_ind���������
	wp_inf����ȡ�ĺ�����Ϣ
	TIMEOUT: �߳�ͬ����ʱʱ��

	����ֵ��
	true:��ȡ�ɹ�
	false:��ȡʧ�ܣ��޺������������
*/
bool ReadMission( uint16_t wp_ind, MissionInf* wp_inf, double TIMEOUT )
{
	if( wp_ind >= MissionsCount )
		return false;
	
	uint16_t ParamGroupInd = wp_ind >> MissionsInParamGroupBit;
	uint16_t wpInd = wp_ind & (MissionsInParamGroup-1);
	char WPGroupName[17];
	sprintf( WPGroupName, "Mission_%d", ParamGroupInd );
	PR_RESULT res = ReadParamGroup( SName(WPGroupName), (uint64_t*)wp_inf, 0, wpInd*8, 8 );
	if( res == PR_ERR )
		return false;
	if( wp_inf->cmd == 0 )
		return false;
	return true;
}

/*
	��ȡ��ǰ��������
	wp_inf����ȡ�ĺ�����Ϣ
	ind����ǰ�������
	TIMEOUT: �߳�ͬ����ʱʱ��

	����ֵ��
	true:��ȡ�ɹ�
	false:��ȡʧ�ܣ��޺������������
*/
bool ReadCurrentMission( MissionInf* wp_inf, uint16_t* ind, double TIMEOUT )
{
	if( MissionsCount == 0 )
		return false;
	if( CurrentMission >= MissionsCount )
		return false;
	
	uint16_t ParamGroupInd = CurrentMission >> MissionsInParamGroupBit;
	uint16_t wpInd = CurrentMission & (MissionsInParamGroup-1);
	char WPGroupName[17];
	sprintf( WPGroupName, "Mission_%d", ParamGroupInd );
	PR_RESULT res = ReadParamGroup( SName(WPGroupName), (uint64_t*)wp_inf, 0, wpInd*8, 8 );
	if( res == PR_ERR )
		return false;
	if( wp_inf->cmd == 0 )
		return false;
	if(ind!=0)
		*ind = CurrentMission;
	return true;
}

void init_Missions()
{	
	char WPGroupName[17];	
	MissionsCount = UploadingMissionsCount = 0;
	
	while(1)
	{
		uint16_t ParamGroupInd = UploadingMissionsCount >> MissionsInParamGroupBit;
		sprintf( WPGroupName, "Mission_%d", ParamGroupInd );	
		
		PR_RESULT res = ParamGroupRegister( SName(WPGroupName), MissionParamVersion, MissionsInParamGroup*8, 0 );
		if( res == PR_ERR )
			break;  
		
		for( uint8_t i = 0; i < MissionsInParamGroup; ++i )
		{
			MissionInf wp_inf;
			res = ReadParamGroup( SName(WPGroupName), (uint64_t*)&wp_inf, 0, i*8, 8 );
			if( res == PR_ERR )
				return;
			if( wp_inf.cmd == 0 )
				return;
			
			++MissionsCount;
			UploadingMissionsCount = MissionsCount;
			if( MissionsCount > MaxMissions )
				return;
		}		
	}
}