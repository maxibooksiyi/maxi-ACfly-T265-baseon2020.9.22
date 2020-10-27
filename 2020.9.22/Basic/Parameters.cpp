#include "Parameters.hpp"
#include <map>
#include "StorageSystem.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "Basic.hpp"
#include "mavlink.h"

using namespace std;

/*
	�������ʽ��
		������汾version(0:1) 15λ�����汾 ���λΪ1��ʾ��������б�
		��������n(2:3)
		��������(4:7)
		n������(32�ֽ�һ������������б����
		{
			��������MAV_PARAM_TYPE(8+n*32:9+n*32)
			����(10+n*32:15+n*32)
			����������(16+n*32:31+n*32)
			����ֵ(32+n*32:39+n*32)
		}
		n������(16�ֽ�һ��������������б����
		{
			��������MAV_PARAM_TYPE(8+n*16:9+n*16)
			����(10+n*16:15+n*16)
			����ֵ(16+n*16:23+n*16)
		}
*/

struct ParamGroup
{
	//����������
	SName name;
	//�Ƿ�Ϊ�²���
	bool is_new;
	//����������ʻ����ź���
	SemaphoreHandle_t ReadSemphr;
	//����������ָ��
	//ParamGroupInf+n*ParamData
	void* data;
};
struct ParamGroupInf
{
	uint16_t version;
	uint16_t params_count;
	uint32_t update_date;
}__attribute__((__packed__));
struct ParamData1
{
	uint16_t type;
	uint8_t resv1[6];
	SName name;
	uint64_t data;
}__attribute__((__packed__));
struct ParamData2
{
	uint64_t data;
}__attribute__((__packed__));
struct Param
{
	ParamGroup* group;
	ParamData1* data;
	uint32_t index;
};

//�������飩��д������
static SemaphoreHandle_t ParamMap_Semphr;
//�������
static map<SName, ParamGroup*> ParamGroups;
//������
static map<SName, Param> GroupParameters;
//��������
static uint32_t ParametersCount = 0;

/*
	������ע�ᣨ��������С�ڵ���16�ֽڣ�
	ע�⣺�����ڲ������ʼ�����ǰ����
	name������������
	param_version�������汾
	params_count����������n

	param_types����������*n
	param_names����������*n������ΪNULL������Ҫ��������б�
	initial_params��������ʼֵ*n

	����ֵ��
	PR_ERR���������ʼ��δ���
	PR_Existed: ������������ͬ������
	PR_TimeOut�����ʲ����������ʱ
	PR_NewParam��ע��Ĳ������²������洢����û�У�
	PR_OK��ע��Ĳ����ڴ洢�����ҵ���������param
*/
PR_RESULT ParamGroupRegister( SName name, uint16_t version, uint32_t params_count,
															const MAV_PARAM_TYPE param_types[], const SName param_names[], const uint64_t initial_params[] )
{	
	//�Ƿ���Ҫ��������б�
	bool addParameters = false;
	if( param_names != 0 )
		addParameters = true;
	
	//��ʼ����ɺ�������Ӳ�����
	if( addParameters )
	{
		LockInitializationStatus();
		if( getInitializationCompleted() == true )
		{	//�������ʼ����ɺ�����ע�����
			UnLockInitializationStatus();
			return PR_ERR;
		}
	}
	
	if( xSemaphoreTake( ParamMap_Semphr, portMAX_DELAY ) == pdTRUE )
	{		
		//���������Ƿ��Ѵ���
		if( ParamGroups.find(name) != ParamGroups.end() )
		{
			xSemaphoreGive(ParamMap_Semphr);
			return PR_Existed;
		}
				
		//�����鳤��
		uint32_t ParamGroup_length;
		//���ò�����汾������
		if( addParameters )
		{
			//������汾
			version |= (1<<15);
			ParamGroup_length = 8 + sizeof(ParamData1)*params_count;
		}
		else
		{
			version &= ~(1<<15);
			ParamGroup_length = 8 + sizeof(ParamData2)*params_count;
		}
		//�����ڴ棨portBYTE_ALIGNMENT����Ϊ8�����������Ա�֤64λ������ʣ�
		uint8_t* newParamGroupData = new uint8_t[ParamGroup_length];
		
		//���Ҫ��������������̴Ӵ洢����ȡ����
		PR_RESULT rt_res = PR_ERR;
		if(addParameters)
		{
			//��ȡ�洢�Ĳ���������Ϣ			
			uint32_t stored_ParamGroup_length;
			char name_ch[17];
			name.get_CharStr(name_ch);
			SS_RESULT res = InternalStorage_GetFileSize( "Config", name_ch, &stored_ParamGroup_length );
			if( res==SS_OK && stored_ParamGroup_length==ParamGroup_length )
			{	//�����ѱ����ڴ洢��
				//��ȡ����Ĳ���
				InternalStorage_ReadFile( "Config", name_ch, newParamGroupData, &stored_ParamGroup_length );
				ParamGroupInf* inf = (ParamGroupInf*)&(newParamGroupData[0]);
				if( inf->version==version && params_count==inf->params_count )
					rt_res = PR_OK;
				if( rt_res == PR_OK )
				{
					for( uint16_t i = 0 ; i < params_count ; ++i )
					{
						ParamData1* data = (ParamData1*)&(newParamGroupData[8+i*sizeof(ParamData1)]);
						if( data->type!=param_types[i] || data->name!=param_names[i] )
						{
							rt_res = PR_ERR;
							break;
						}						
					}	
				}
			}
		}
		if( rt_res != PR_OK )
		{	//����δ�����ڴ洢��
			//��ʼ������
			RTC_TimeStruct RTCTime = Get_RTC_Time();
			uint32_t file_create_time =  ( (RTCTime.Year - 1980) << 25 ) 	|
																	 ( RTCTime.Month << 21 ) 	|
																	 ( RTCTime.Date << 16 ) 	|
																	 ( RTCTime.Hours << 11 ) 	|
																	 ( RTCTime.Minutes << 5 ) 	|
																	 ( RTCTime.Seconds << 0 );
			ParamGroupInf* inf = (ParamGroupInf*)&(newParamGroupData[0]);
			inf->version = version;
			inf->params_count = params_count;
			inf->update_date = file_create_time;
			if(addParameters)
			{	//���������Ĳ�����Ҫ����
				for( uint16_t i = 0 ; i < params_count ; ++i )
				{
					ParamData1* data = (ParamData1*)&(newParamGroupData[8+i*sizeof(ParamData1)]);
					data->type = param_types[i];
					data->name = param_names[i];
					if( initial_params != 0 )
						data->data = initial_params[i];
				}	
			}
			else
			{	//�����������Ĳ���û��������Ϣ
				for( uint16_t i = 0 ; i < params_count ; ++i )
				{
					ParamData2* data = (ParamData2*)&(newParamGroupData[8+i*sizeof(ParamData2)]);
					if( initial_params != 0 )
						data->data = initial_params[i];
				}	
			}
			rt_res = PR_NewParam;
		}
		
		//���������������
		ParamGroup* newParamGroup = new ParamGroup();
		newParamGroup->name = name;
		if( rt_res == PR_NewParam )
			newParamGroup->is_new = true;
		else
			newParamGroup->is_new = false;
		newParamGroup->ReadSemphr = xSemaphoreCreateMutex();
		newParamGroup->data = newParamGroupData;
		ParamGroups.insert( 
			pair<SName, ParamGroup*>( name, newParamGroup ) );
		
		if(addParameters)
		{
			//������������б�
			for( uint16_t i = 0 ; i < params_count ; ++i )
			{
				Param param = { newParamGroup, (ParamData1*)&(newParamGroupData[8+i*sizeof(ParamData1)]), ParametersCount };			
				GroupParameters.insert( pair<SName, Param>( param_names[i], param ) );
				++ParametersCount;
			}
		}
		
		//����
		xSemaphoreGive(ParamMap_Semphr);
		if(addParameters)
			UnLockInitializationStatus();
		
		//�������Ҫ��������������ٴӴ洢����ȡ����
		if(!addParameters)
		{
			//��ȡ�洢�Ĳ���������Ϣ			
			uint32_t stored_ParamGroup_length;
			char name_ch[17];
			name.get_CharStr(name_ch);
			SS_RESULT res = InternalStorage_GetFileSize( "Config", name_ch, &stored_ParamGroup_length );
			if( res==SS_OK && stored_ParamGroup_length==ParamGroup_length )
			{	//�����ѱ����ڴ洢��
				//�����ڴ棨portBYTE_ALIGNMENT����Ϊ8�����������Ա�֤64λ������ʣ�
				uint8_t* UpParamGroupData = new uint8_t[ParamGroup_length];
				//��ȡ����Ĳ���
				InternalStorage_ReadFile( "Config", name_ch, UpParamGroupData, &stored_ParamGroup_length );
				ParamGroupInf* inf = (ParamGroupInf*)&(UpParamGroupData[0]);
				if( inf->version == version )
				{	//�汾��Ϣ����
					//���²���
					if( xSemaphoreTake( ParamMap_Semphr, portMAX_DELAY ) == pdTRUE )
					{
						map<SName, ParamGroup*>::iterator it = ParamGroups.find(name);
						ParamGroup* group = it->second;
						if( xSemaphoreTake( group->ReadSemphr, portMAX_DELAY ) == pdTRUE )
						{
							//�����ڴ��еĲ���
							memcpy( newParamGroupData, UpParamGroupData, ParamGroup_length );
							group->is_new = false;
							//����
							xSemaphoreGive(group->ReadSemphr);
						}
						//����
						xSemaphoreGive(ParamMap_Semphr);
					}
				}
				//ɾ��������ڴ�
				delete[] UpParamGroupData;
			}
		}
		
		return PR_OK;
	}
	else
	{
		UnLockInitializationStatus();
		return PR_ERR;
	}
}

/*
	������ע�ᣨ��������С�ڵ���16�ֽڣ�
	ע�⣺�˺���ֻ��ע�����ڴ洢���еĲ���
	ע�⣺�����ڲ������ʼ�����ǰ����
	name������������
	param_version�������汾
	params_count����������n

	param_names����������*n������ΪNULL������Ҫ��������б�

	����ֵ��
	PR_ERR���������ʼ��δ��ɻ�����ڴ洢���в�����
	PR_Existed: ������������ͬ������
	PR_TimeOut�����ʲ����������ʱ
	PR_NewParam��ע��Ĳ������²������洢����û�У�
	PR_OK��ע��Ĳ����ڴ洢�����ҵ���������param
*/
PR_RESULT ParamGroupRegister( SName name, uint16_t version, uint32_t params_count, const SName param_names[] )
{	
	//�Ƿ���Ҫ��������б�
	bool addParameters = false;
	if( param_names != 0 )
		addParameters = true;
	
	//��ʼ����ɺ�������Ӳ�����
	if( addParameters )
	{
		LockInitializationStatus();
		if( getInitializationCompleted() == true )
		{	//�������ʼ����ɺ�����ע�����
			UnLockInitializationStatus();
			return PR_ERR;
		}
	}
	
	if( xSemaphoreTake( ParamMap_Semphr, portMAX_DELAY ) == pdTRUE )
	{		
		//���������Ƿ��Ѵ���
		if( ParamGroups.find(name) != ParamGroups.end() )
		{
			xSemaphoreGive(ParamMap_Semphr);
			return PR_Existed;
		}
				
		//�����鳤��
		uint32_t ParamGroup_length;
		//���ò�����汾������
		if( addParameters )
		{
			//������汾
			version |= (1<<15);
			ParamGroup_length = 8 + sizeof(ParamData1)*params_count;
		}
		else
		{
			version &= ~(1<<15);
			ParamGroup_length = 8 + sizeof(ParamData2)*params_count;
		}
		//�����ڴ棨portBYTE_ALIGNMENT����Ϊ8�����������Ա�֤64λ������ʣ�
		uint8_t* newParamGroupData = new uint8_t[ParamGroup_length];
		
		//���̴Ӵ洢����ȡ����
		PR_RESULT rt_res = PR_ERR;
		//��ȡ�洢�Ĳ���������Ϣ			
		uint32_t stored_ParamGroup_length;
		char name_ch[17];
		name.get_CharStr(name_ch);
		SS_RESULT res = InternalStorage_GetFileSize( "Config", name_ch, &stored_ParamGroup_length );
		if( res==SS_OK && stored_ParamGroup_length==ParamGroup_length )
		{	//�����ѱ����ڴ洢��
			//��ȡ����Ĳ���
			InternalStorage_ReadFile( "Config", name_ch, newParamGroupData, &stored_ParamGroup_length );
			ParamGroupInf* inf = (ParamGroupInf*)&(newParamGroupData[0]);
			if( inf->version == version )
				rt_res = PR_OK;
		}
		
		if( rt_res != PR_OK )
		{	//����δ�����ڴ洢��
			//�˳�
			delete[] newParamGroupData;
			xSemaphoreGive(ParamMap_Semphr);
			return PR_ERR;
		}
		
		//���������������
		ParamGroup* newParamGroup = new ParamGroup();
		newParamGroup->name = name;
		if( rt_res == PR_NewParam )
			newParamGroup->is_new = true;
		else
			newParamGroup->is_new = false;
		newParamGroup->ReadSemphr = xSemaphoreCreateMutex();
		newParamGroup->data = newParamGroupData;
		ParamGroups.insert( 
			pair<SName, ParamGroup*>( name, newParamGroup ) );
		
		if(addParameters)
		{
			//������������б�
			for( uint16_t i = 0 ; i < params_count ; ++i )
			{
				Param param = { newParamGroup, (ParamData1*)&(newParamGroupData[8+i*sizeof(ParamData1)]), ParametersCount };			
				GroupParameters.insert( pair<SName, Param>( param_names[i], param ) );
				++ParametersCount;
			}
		}
		
		//����
		xSemaphoreGive(ParamMap_Semphr);
		if(addParameters)
			UnLockInitializationStatus();
		
		return PR_OK;
	}
	else
	{
		UnLockInitializationStatus();
		return PR_ERR;
	}
}

/*
	�Ӵ洢���������������
	ע�⣺�����ڲ������ʼ����ɺ����
	name������������

	����ֵ��
	PR_ERR���������ʼ��δ��ɻ�����ڴ洢���в�����
	PR_OK��ע��Ĳ����ڴ洢�����ҵ���������param
*/
PR_RESULT ReloadParamGroup( SName name )
{	
	//������δ��ɳ�ʼ���������ȡ����
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	map<SName, ParamGroup*>::iterator it = ParamGroups.find(name);
	if( it == ParamGroups.end() )
		//�޴˲����˳�
		return PR_ERR;
	ParamGroup* group = it->second;
	
	if( xSemaphoreTake( ParamMap_Semphr, portMAX_DELAY ) == pdTRUE )
	{		
		ParamGroupInf* group_inf = (ParamGroupInf*)group->data;

		bool is_param1 = ( group_inf->version & (1<<15) ) != 0;				
		uint32_t ParamGroup_length;
		if( is_param1 )
			ParamGroup_length = 8 + sizeof(ParamData1)*group_inf->params_count;
		else
			ParamGroup_length = 8 + sizeof(ParamData2)*group_inf->params_count;
		
		//�����ڴ棨portBYTE_ALIGNMENT����Ϊ8�����������Ա�֤64λ������ʣ�
		uint8_t* newParamGroupData = new uint8_t[ParamGroup_length];
		
		//���̴Ӵ洢����ȡ����
		PR_RESULT rt_res = PR_ERR;
		//��ȡ�洢�Ĳ���������Ϣ			
		uint32_t stored_ParamGroup_length;
		char name_ch[17];
		name.get_CharStr(name_ch);
		SS_RESULT res = InternalStorage_GetFileSize( "Config", name_ch, &stored_ParamGroup_length );
		if( res==SS_OK && stored_ParamGroup_length==ParamGroup_length )
		{	//�����ѱ����ڴ洢��
			//��ȡ����Ĳ���
			InternalStorage_ReadFile( "Config", name_ch, newParamGroupData, &stored_ParamGroup_length );
			ParamGroupInf* inf = (ParamGroupInf*)&(newParamGroupData[0]);
			if( inf->version != group_inf->version )
			{
				xSemaphoreGive(ParamMap_Semphr);
				delete[] newParamGroupData;
				return PR_ERR;
			}
		}
		
		if( xSemaphoreTake(group->ReadSemphr, portMAX_DELAY) == pdTRUE )
		{
			memcpy( group->data, newParamGroupData, ParamGroup_length );
			xSemaphoreGive(group->ReadSemphr);
		}
		delete[] newParamGroupData;
		
		//����
		xSemaphoreGive(ParamMap_Semphr);		
		return PR_OK;
	}
	else
	{
		UnLockInitializationStatus();
		return PR_ERR;
	}
}


static SemaphoreHandle_t Parameters_it_Mutex = xSemaphoreCreateMutex();
static map<SName, Param>::iterator Parameters_it = GroupParameters.end();
/*
	��ȡ�������������
	ע�⣺�����ڲ������ʼ����ɺ����
	count�����ز�������
����ֵ��
	PR_ERR��δ��ɳ�ʼ��
	PR_OK����ȡ�ɹ�
*/
PR_RESULT GetParametersCount( uint32_t* count )
{
	//������δ��ɳ�ʼ���������ȡ����
	if( getInitializationCompleted() == false )
		return PR_ERR;
	*count = ParametersCount;
	return PR_OK;
}
/*
	���ò������ȡ������
	ע�⣺�����ڲ������ʼ����ɺ����
	count�����ز�������
����ֵ��
	PR_ERR��δ��ɳ�ʼ��
	PR_OK�������ɹ�
*/
PR_RESULT ResetParametersIterator()
{
	//������δ��ɳ�ʼ���������ȡ����
	if( getInitializationCompleted() == false )
		return PR_ERR;
	xSemaphoreTake(Parameters_it_Mutex,portMAX_DELAY);
		Parameters_it = GroupParameters.begin();
	xSemaphoreGive(Parameters_it_Mutex);
	return PR_OK;
}
/*
	����������
	ע�⣺�����ڲ������ʼ����ɺ����
����ֵ��
	PR_ERR��δ��ɳ�ʼ�����Ѿ���ĩβ
	PR_OK�������ɹ�
*/
PR_RESULT ParameterIteratorMoveNext()
{
	//������δ��ɳ�ʼ���������ȡ����
	if( getInitializationCompleted() == false )
		return PR_ERR;
	xSemaphoreTake(Parameters_it_Mutex,portMAX_DELAY);
		if( Parameters_it == GroupParameters.end() )
		{
			xSemaphoreGive(Parameters_it_Mutex);			
			return PR_ERR;
		}
		++Parameters_it;
	xSemaphoreGive(Parameters_it_Mutex);
	return PR_OK;
}
/*
	����ǰ����
	ע�⣺�����ڲ������ʼ����ɺ����
	index����ǰ�������
	name����ȡ�Ĳ�������
	type����ȡ�Ĳ�������
	value����ȡ�Ĳ�����ֵ
	is_new��������Ϊ�£����ڴ洢����
����ֵ��
	PR_ERR��δ��ɳ�ʼ�����Ѿ���ĩβ
	PR_TimeOut�����ʲ����������ʱ
	PR_OK�������ɹ�
*/
PR_RESULT ReadCurrentParameter( SName* name, uint32_t* index, MAV_PARAM_TYPE* type, uint64_t* value, bool* is_new, double TIMEOUT )
{
	//������δ��ɳ�ʼ���������ȡ����
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	Param param;
	xSemaphoreTake(Parameters_it_Mutex,portMAX_DELAY);
		if( Parameters_it == GroupParameters.end() )
		{
			xSemaphoreGive(Parameters_it_Mutex);
			return PR_ERR;
		}
		param = Parameters_it->second;		
	xSemaphoreGive(Parameters_it_Mutex);
	if( index != 0 )
		*index = param.index;
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake(param.group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE )
	{
		if( name != 0 )
			*name = param.data->name;
		if( type != 0 )
			*type = (MAV_PARAM_TYPE)param.data->type;
		if( value != 0 )
			*value = param.data->data;
		if( is_new != 0 )
			*is_new = param.group->is_new;
		xSemaphoreGive(param.group->ReadSemphr);
		return PR_OK;
	}
	return PR_TimeOut;
}

/*
	��ȡָ�����Ʋ���
	ע�⣺�����ڲ������ʼ����ɺ����
	name����������
	index����ȡ�Ĳ������
	type����ȡ�Ĳ�������
	value��Ҫ��ȡ�Ĳ�����ֵ
	is_new��������Ϊ�£����ڴ洢����

	����ֵ��
	PR_ERR�����������޴˲���
	PR_TimeOut�����ʲ����������ʱ
	PR_OK����ȡ�ɹ�
*/
PR_RESULT ReadParam( SName name, uint32_t* index, MAV_PARAM_TYPE* type, uint64_t* value, bool* is_new, double TIMEOUT )
{
	//������δ��ɳ�ʼ���������ȡ����
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	map<SName, Param>::iterator it = GroupParameters.find(name);
	if( it == GroupParameters.end() )
		//�޴˲����˳�
		return PR_ERR;
	Param param = it->second;
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake(param.group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE )
	{
		if( index != 0 )
			*index = param.index;
		if( type != 0 )
			*type = (MAV_PARAM_TYPE)param.data->type;
		if( value != 0 )
			*value = param.data->data;
		if( is_new != 0 )
			*is_new = param.group->is_new;
		xSemaphoreGive(param.group->ReadSemphr);
		return PR_OK;
	}
	return PR_TimeOut;
}
/*
	��ȡָ����Ų�������Ч�ʣ�
	ע�⣺�����ڲ������ʼ����ɺ����
	index���������
	name����������
	type����ȡ�Ĳ�������
	value��Ҫ��ȡ�Ĳ�����ֵ
	is_new��������Ϊ�£����ڴ洢����

	����ֵ��
	PR_ERR�����������޴˲���
	PR_TimeOut�����ʲ����������ʱ
	PR_OK����ȡ�ɹ�
*/
PR_RESULT ReadParam( uint32_t index, SName* name, MAV_PARAM_TYPE* type, uint64_t* value, bool* is_new, double TIMEOUT )
{
	//������δ��ɳ�ʼ���������ȡ����
	if( getInitializationCompleted() == false )
		return PR_ERR;
	if( index >= ParametersCount )
		return PR_ERR;
	
	map<SName, Param>::iterator it = GroupParameters.begin();
	for( ; it != GroupParameters.end(); ++it )
	{
		if( it->second.index == index )
			break;
	}
	if( it == GroupParameters.end() )
		//�޴˲����˳�
		return PR_ERR;
	Param param = it->second;
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake(param.group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE )
	{
		if( name != 0 )
			*name = param.data->name;
		if( type != 0 )
			*type = (MAV_PARAM_TYPE)param.data->type;
		if( value != 0 )
			*value = param.data->data;
		if( is_new != 0 )
			*is_new = param.group->is_new;
		xSemaphoreGive(param.group->ReadSemphr);
		return PR_OK;
	}
	return PR_TimeOut;
}

/*
	��ȡָ�����Ʋ�����
	ע�⣺�����ڲ������ʼ����ɺ����
	name������������
	data��Ҫ��ȡ�Ĳ����������ֵ

	����ֵ��
	PR_ERR�����������޴˲���
	PR_TimeOut�����ʲ����������ʱ
	PR_OK����ȡ�ɹ�
*/
PR_RESULT ReadParamGroup( SName name, uint64_t data[], bool* is_new, double TIMEOUT )
{
	//������δ��ɳ�ʼ���������ȡ����
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	map<SName, ParamGroup*>::iterator it = ParamGroups.find(name);
	if( it == ParamGroups.end() )
		//�޴˲����˳�
		return PR_ERR;
	ParamGroup* group = it->second;
	
	ParamGroupInf* group_inf = (ParamGroupInf*)group->data;
	bool is_param1 = ( group_inf->version & (1<<15) ) != 0;
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE )
	{
		if( is_new != 0 )
			*is_new = group->is_new;
		for( uint16_t i = 0 ; i < group_inf->params_count ; ++i )
		{
			if( is_param1 )
			{
				ParamData1* param = (ParamData1*)&((uint8_t*)group->data)[8+i*sizeof(ParamData1)];
				data[i] = param->data;
			}
			else
			{
				ParamData2* param = (ParamData2*)&((uint8_t*)group->data)[8+i*sizeof(ParamData2)];
				data[i] = param->data;
			}
		}
		xSemaphoreGive(group->ReadSemphr);
		return PR_OK;
	}
	return PR_TimeOut;
}
/*
	��ȡָ�����Ʋ�����
	ע�⣺�����ڲ������ʼ����ɺ����
	name������������
	data��Ҫ��ȡ�Ĳ����������ֵ
	start�������start��ʼ��ȡ��0��ʼ��
	read_count����ȡ��Ŀ

	����ֵ��
	PR_ERR�����������޴˲������ȡ�Ĳ�������������Χ
	PR_TimeOut�����ʲ����������ʱ
	PR_OK����ȡ�ɹ�
*/
PR_RESULT ReadParamGroup( SName name, uint64_t data[], bool* is_new, uint16_t start, uint16_t read_count, double TIMEOUT )
{
	//������δ��ɳ�ʼ���������ȡ����
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	map<SName, ParamGroup*>::iterator it = ParamGroups.find(name);
	if( it == ParamGroups.end() )
		//�޴˲����˳�
		return PR_ERR;
	ParamGroup* group = it->second;
	
	ParamGroupInf* group_inf = (ParamGroupInf*)group->data;
	bool is_param1 = ( group_inf->version & (1<<15) ) != 0;
	uint16_t read_to = start + read_count;
	if( start>=group_inf->params_count || read_to>group_inf->params_count )
		return PR_ERR;
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE )
	{
		if( is_new != 0 )
			*is_new = group->is_new;
		for( uint16_t i = start ; i < read_to ; ++i )
		{
			if( is_param1 )
			{
				ParamData1* param = (ParamData1*)&((uint8_t*)group->data)[8+i*sizeof(ParamData1)];
				data[i-start] = param->data;
			}
			else
			{
				ParamData2* param = (ParamData2*)&((uint8_t*)group->data)[8+i*sizeof(ParamData2)];
				data[i-start] = param->data;
			}
		}
		xSemaphoreGive(group->ReadSemphr);
		return PR_OK;
	}
	return PR_TimeOut;
}

/*
	����ָ�����Ʋ�����
	ע�⣺�����ڲ������ʼ����ɺ����
	name������������
	data��Ҫ���µĲ����������ֵ
	start�������start��ʼ���£�0��ʼ��
	write_count��������Ŀ
	st���Ƿ�д��洢��
	TIMEOUT����ʱʱ��

	����ֵ��
	PR_ERR�����������޴˲������ȡ�Ĳ�������������Χ
	PR_TimeOut�����ʲ����������ʱ
	PR_OK���ɹ�
*/
PR_RESULT UpdateParamGroup( SName name, const uint64_t data[], uint16_t start, uint16_t write_count, bool st, double TIMEOUT )
{
	//������δ��ɳ�ʼ����������²���
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	map<SName, ParamGroup*>::iterator it = ParamGroups.find(name);
	if( it == ParamGroups.end() )
		//�޴˲����˳�
		return PR_ERR;
	ParamGroup* group = it->second;
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake( ParamMap_Semphr, TIMEOUT_Ticks ) == pdTRUE )
	{
		ParamGroupInf* group_inf = (ParamGroupInf*)group->data;
		uint16_t write_to = start + write_count;
		if( start>=group_inf->params_count || write_to>group_inf->params_count )
		{	//����������Χ�˳�
			xSemaphoreGive(ParamMap_Semphr);
			return PR_ERR;
		}
		bool is_param1 = ( group_inf->version & (1<<15) ) != 0;
		//���ڴ��и��²���
		if( xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE )
		{
			for( uint16_t i = start ; i < write_to ; ++i )
			{
				if( is_param1 )
				{
					ParamData1* param = (ParamData1*)&((uint8_t*)group->data)[8+i*sizeof(ParamData1)];
					param->data = data[i-start];
				}
				else
				{
					ParamData2* param = (ParamData2*)&((uint8_t*)group->data)[8+i*sizeof(ParamData2)];
					param->data = data[i-start];
				}
			}
			xSemaphoreGive(group->ReadSemphr);
		}
		else
		{
			xSemaphoreGive(ParamMap_Semphr);
			return PR_TimeOut;
		}
		
		if(st)
		{	//����������洢��
			char name_ch[17];
			name.get_CharStr(name_ch);
			uint32_t ParamGroup_length;
			if( is_param1 )
				ParamGroup_length = 8 + sizeof(ParamData1)*group_inf->params_count;
			else
				ParamGroup_length = 8 + sizeof(ParamData2)*group_inf->params_count;
			SS_RESULT ss_res = InternalStorage_SaveFile( "Config", name_ch, group->data, ParamGroup_length );
			if( ss_res == SS_OK )
				group->is_new = false;
			xSemaphoreGive(ParamMap_Semphr);
			if( ss_res == SS_OK )
				return PR_OK;
			else if( ss_res == SS_TimeOut )
				return PR_TimeOut;
			else
				return PR_ERR;
		}
		xSemaphoreGive(ParamMap_Semphr);
		return PR_OK;
	}
	else
		return PR_TimeOut;
}

/*
	��ָ�����Ʋ��������ƵĲ������浽�洢��
	ע�⣺�����ڲ������ʼ����ɺ����
	name������������
	TIMEOUT����ʱʱ��

	����ֵ��
	PR_ERR�����������޴˲������ȡ�Ĳ�������������Χ
	PR_TimeOut�����ʲ����������ʱ
	PR_OK������ɹ�
*/
PR_RESULT SaveParamGroup( SName name, double TIMEOUT )
{
	//������δ��ɳ�ʼ����������²���
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	map<SName, ParamGroup*>::iterator it = ParamGroups.find(name);
	if( it == ParamGroups.end() )
		//�޴˲����˳�
		return PR_ERR;
	ParamGroup* group = it->second;
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake( ParamMap_Semphr, TIMEOUT_Ticks ) == pdTRUE )
	{
		ParamGroupInf* group_inf = (ParamGroupInf*)group->data;
		bool is_param1 = ( group_inf->version & (1<<15) ) != 0;
		
		//����������洢��
		char name_ch[17];
		name.get_CharStr(name_ch);
		uint32_t ParamGroup_length;
		if( is_param1 )
			ParamGroup_length = 8 + sizeof(ParamData1)*group_inf->params_count;
		else
			ParamGroup_length = 8 + sizeof(ParamData2)*group_inf->params_count;
		SS_RESULT ss_res = InternalStorage_SaveFile( "Config", name_ch, group->data, ParamGroup_length );
		if( ss_res == SS_OK )
			group->is_new = false;
		xSemaphoreGive(ParamMap_Semphr);
		if( ss_res == SS_OK )
			return PR_OK;
		else if( ss_res == SS_TimeOut )
			return PR_TimeOut;
		else
			return PR_ERR;
	}
	else
		return PR_TimeOut;
}

/*
	����ָ�����Ʋ���
	ע�⣺�����ڲ������ʼ����ɺ����
	name������������
	data��Ҫ���µĲ����������ֵ
	TIMEOUT����ʱʱ��

	����ֵ��
	PR_ERR�����������޴˲���
	PR_TimeOut�����ʲ����������ʱ
	PR_OK�����³ɹ�
*/
PR_RESULT UpdateParam( SName name, const uint64_t data, double TIMEOUT )
{
	//������δ��ɳ�ʼ����������²���
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	map<SName, Param>::iterator it = GroupParameters.find(name);
	if( it == GroupParameters.end() )
		//�޴˲����˳�
		return PR_ERR;
	Param param = it->second;
	ParamGroup* group = param.group;
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake( ParamMap_Semphr, TIMEOUT_Ticks ) == pdTRUE )
	{
		//���ڴ��и��²���
		if( xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE )
		{
			param.data->data = data;
			xSemaphoreGive(group->ReadSemphr);
		}
		else
		{
			xSemaphoreGive(ParamMap_Semphr);
			return PR_TimeOut;
		}
		//����������洢��
		char name_ch[17];
		group->name.get_CharStr(name_ch);
		ParamGroupInf* group_inf = (ParamGroupInf*)group->data;
		bool is_param1 = ( group_inf->version & (1<<15) ) != 0;
		uint32_t ParamGroup_length;
		if( is_param1 )
			ParamGroup_length = 8 + sizeof(ParamData1)*group_inf->params_count;
		else
			ParamGroup_length = 8 + sizeof(ParamData2)*group_inf->params_count;
		SS_RESULT ss_res = InternalStorage_SaveFile( "Config", name_ch, group->data, ParamGroup_length );
		if( ss_res == SS_OK )
			group->is_new = false;
		xSemaphoreGive(ParamMap_Semphr);
		if( ss_res == SS_OK )
			return PR_OK;
		else if( ss_res == SS_TimeOut )
			return PR_TimeOut;
		else
			return PR_ERR;
	}
	else
		return PR_TimeOut;
}


void init_Parameters()
{
	ParamMap_Semphr = xSemaphoreCreateMutex();
}