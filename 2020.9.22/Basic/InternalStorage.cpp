#include "StorageSystem.hpp"

#include "drv_ADC.hpp"
#include "fatfs.h"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

//�Ƿ��ѳ�ʼ��
static bool InternalStorage_Initialized = false;
//�ڲ��洢�Ƿ��һ�����У�����ʱ��ʼ����
static bool InternalStorage_FirstTime = false;

//�豸���ʻ�����
static SemaphoreHandle_t InternalStorage_Semphr;
//�ļ�
static char UserFile_filename[256];
static FIL UserFile;

/*�ļ���д*/
	/*
		����洢�ļ�
		group_name���ļ��������
		name���ļ�����
		content������
		length�����ݳ���
		TIMEOUT���߳�ͬ����ʱʱ��
	*/
	SS_RESULT InternalStorage_SaveFile( const char* group_name, const char* name , const void* content , uint32_t length , double TIMEOUT )
	{
		if( InternalStorage_Initialized == false )
			return SS_ERR;
		
		//��ȡ������
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( InternalStorage_Semphr , TIMEOUT_Ticks ) )
		{
			//��ȡ�ļ���
			for( uint8_t i = 0 ; i < 4 ; ++i )
				UserFile_filename[i] = FlashPath[i];
			strcat( UserFile_filename , group_name );
			//�������
			if( f_stat( UserFile_filename, 0 ) == FR_NO_FILE )
				f_mkdir(UserFile_filename);
			strcat( UserFile_filename , "/" );
			strcat( UserFile_filename , name );
			strcat( UserFile_filename , ".cfg" );
			
			FRESULT res = f_open(&UserFile, UserFile_filename, FA_CREATE_ALWAYS | FA_WRITE);
			//���ļ�
			if(res == FR_OK) 
			{	//д���ļ�
				uint32_t byteswritten;
				res = f_write(&UserFile, content, length, &byteswritten);
				if( byteswritten < length )
					res = FR_DENIED;
				res = f_close(&UserFile);
			}
			
			//�ͷŻ�����
			xSemaphoreGive(InternalStorage_Semphr);
			if( res == FR_OK )
				return SS_OK;
			else
				return SS_ERR;
		}
		else
			return SS_TimeOut;
	}
	
	/*
		��ȡ�洢�ļ���С
		group_name���ļ��������
		name���ļ�����
		size����ȡ���ļ���С
		TIMEOUT���߳�ͬ����ʱʱ��
	*/
	SS_RESULT InternalStorage_GetFileSize( const char* group_name, const char* name , uint32_t* size , double TIMEOUT )
	{
		if( InternalStorage_Initialized == false )
			return SS_ERR;
		
		//��ȡ������
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( InternalStorage_Semphr , TIMEOUT_Ticks ) )
		{
			//��ȡ�ļ���
			for( uint8_t i = 0 ; i < 4 ; ++i )
				UserFile_filename[i] = FlashPath[i];
			strcat( UserFile_filename , group_name );
			strcat( UserFile_filename , "/" );
			strcat( UserFile_filename , name );
			strcat( UserFile_filename , ".cfg" );
			
			FILINFO info;
			FRESULT res = f_stat( UserFile_filename, &info );
			if( res == FR_OK )
				*size = info.fsize;
			
			//�ͷŻ�����
			xSemaphoreGive(InternalStorage_Semphr);
			if( res == FR_OK )
				return SS_OK;
			else
				return SS_ERR;
		}
		else
			return SS_TimeOut;
	}
	/*
		��ȡ�洢�ļ�
		group_name���ļ��������
		name���ļ�����
		content����ȡ������
		length����ȡ���ļ�����
		TIMEOUT���߳�ͬ����ʱʱ��
	*/
	SS_RESULT InternalStorage_ReadFile( const char* group_name, const char* name , void* content , uint32_t* length , double TIMEOUT )
	{
		if( InternalStorage_Initialized == false )
			return SS_ERR;
		
		//��ȡ������
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( InternalStorage_Semphr , TIMEOUT_Ticks ) )
		{
			//��ȡ�ļ���
			for( uint8_t i = 0 ; i < 4 ; ++i )
				UserFile_filename[i] = FlashPath[i];
			strcat( UserFile_filename , group_name );
			strcat( UserFile_filename , "/" );
			strcat( UserFile_filename , name );
			strcat( UserFile_filename , ".cfg" );
			
			FRESULT res = f_open(&UserFile, UserFile_filename, FA_READ);
			//���ļ�
			if(res == FR_OK) 
			{	//��ȡ�ļ�
				uint32_t bytesread;
				uint32_t file_size = f_size(&UserFile);
				*length = file_size;
				res = f_read(&UserFile, content, file_size, &bytesread);
				if( bytesread < file_size )
					res = FR_DENIED;
				res = f_close(&UserFile);
			}
			
			//�ͷŻ�����
			xSemaphoreGive(InternalStorage_Semphr);
			if( res == FR_OK )
				return SS_OK;
			else
				return SS_ERR;
		}
		else
			return SS_TimeOut;
	}
/*�ļ���д*/

void init_InternalStorage()
{
	while( Get_VDDA_Voltage() < 3.0 )
	{	//��ѹ���Ͳ���ʼ��flash��ֹ�����
		os_delay(0.1);
	}
	
	FRESULT res;
	Aligned_DMABuf uint8_t workBuffer[FF_MAX_SS*2];
	//�����ļ�ϵͳ
	//res = f_mkfs(FlashPath, FM_ANY, 0, workBuffer, sizeof(workBuffer));
	res = f_mount(&FlashFatFS, (TCHAR const*)FlashPath, 1);
	if( res == FR_NO_FILESYSTEM )
	{	//���ļ�ϵͳ�ȸ�ʽ��
		MKFS_PARM mkfs_param;
		mkfs_param.fmt = FM_FAT;
		mkfs_param.au_size = 0;
		mkfs_param.align = 0;
		mkfs_param.n_fat = 2;
		mkfs_param.n_root = 0;
 		res = f_mkfs(FlashPath, &mkfs_param, workBuffer, sizeof(workBuffer));
		InternalStorage_FirstTime = true;
	}
	
	//�½�ConfigĿ¼
	char name[50] = {0};
	strcat( name, FlashPath );
	strcat( name, "Config" );
	if( f_stat( name, 0 ) == FR_NO_FILE )
		f_mkdir(name);
	
	//�½�LogĿ¼
	name[0] = 0;
	strcat( name, FlashPath );
	strcat( name, "Log" );
	if( f_stat( name, 0 ) == FR_NO_FILE )
		f_mkdir(name);
	
	//�����Ƿ��ѳ�ʼ��״̬
	if( res == FR_OK )
	{
		InternalStorage_Semphr = xSemaphoreCreateMutex();
		InternalStorage_Initialized = true;
	}
}