#include "Basic.hpp"
#include "TimeBase.hpp"
#include "stm32h7xx_hal.h"

//��ʼ�����ָʾ
//��ʼ����ɲ��ܽ��г�ʼ������
static uint16_t InitializationStatus_Lock = 0;
static bool InitializationCompleted = false;
bool getInitializationCompleted(){ return InitializationCompleted; }
void setInitializationCompleted()
{ 
	if(InitializationStatus_Lock == 0)
		InitializationCompleted = true; 
}
void LockInitializationStatus(){ ++InitializationStatus_Lock; }
void UnLockInitializationStatus(){ --InitializationStatus_Lock; }

void init_Basic()
{	
	/* Enable I-Cache---------------------------------------------------------*/
	SCB_EnableICache();
  /* Enable D-Cache---------------------------------------------------------*/
	#ifdef DCACHE_SIZE
		SCB_EnableDCache();
	#endif
	
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	init_TimeBase();
	
	/*MPU Region 3����AXI���÷�Cache��������DMA����*/
		MPU_Region_InitTypeDef MPU_Initure;		
		HAL_MPU_Disable();         //����MPU֮ǰ�ȹر�MPU,��������Ժ���ʹ��MPU
	
		MPU_Initure.Enable           = MPU_REGION_ENABLE;               //ʹ�ܸñ�������
		MPU_Initure.Number           = MPU_REGION_NUMBER3;             //���ñ�������
		MPU_Initure.BaseAddress      = 0x24070000;                      //���û�ַ
		MPU_Initure.Size             = MPU_REGION_SIZE_64KB;            //���ñ��������С
		MPU_Initure.SubRegionDisable = 0;                               //��ֹ������
		MPU_Initure.TypeExtField     = MPU_TEX_LEVEL1;                  //����������չ��Ϊlevel1
		MPU_Initure.AccessPermission = MPU_REGION_FULL_ACCESS;          //���÷���Ȩ��,
		MPU_Initure.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;   //����ָ�����(�����ȡָ��)
		MPU_Initure.IsShareable      = MPU_ACCESS_SHAREABLE;            //�Ƿ�������
		MPU_Initure.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;        //�Ƿ�����cache
		MPU_Initure.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;           //�Ƿ�������
		HAL_MPU_ConfigRegion(&MPU_Initure);                             //����MPU
		HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);                         //����MPU
	/*MPU Region 3����AXI���÷�Cache��������DMA����*/
	
	/*MPU Region 4����AXI���÷�Cache��������DMA����*/
		HAL_MPU_Disable();         //����MPU֮ǰ�ȹر�MPU,��������Ժ���ʹ��MPU
	
		MPU_Initure.Enable           = MPU_REGION_ENABLE;               //ʹ�ܸñ�������
		MPU_Initure.Number           = MPU_REGION_NUMBER4;             //���ñ�������
		MPU_Initure.BaseAddress      = 0x30000000;                      //���û�ַ
		MPU_Initure.Size             = MPU_REGION_SIZE_128KB;            //���ñ��������С
		MPU_Initure.SubRegionDisable = 0;                               //��ֹ������
		MPU_Initure.TypeExtField     = MPU_TEX_LEVEL1;                  //����������չ��Ϊlevel1
		MPU_Initure.AccessPermission = MPU_REGION_FULL_ACCESS;          //���÷���Ȩ��,
		MPU_Initure.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;   //����ָ�����(�����ȡָ��)
		MPU_Initure.IsShareable      = MPU_ACCESS_SHAREABLE;            //�Ƿ�������
		MPU_Initure.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;        //�Ƿ�����cache
		MPU_Initure.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;           //�Ƿ�������
		HAL_MPU_ConfigRegion(&MPU_Initure);                             //����MPU
		HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);                         //����MPU
	/*MPU Region 4����AXI���÷�Cache��������DMA����*/
}

/*��дC++ new delete�����*/
	void * operator new( std::size_t size )
	{
		return pvPortMalloc( size );
	}
	void * operator new[]( std::size_t size )
	{
		return pvPortMalloc(size);
	}

	void  operator delete(void* __p) _NOEXCEPT
	{
		return vPortFree ( __p );
	}
	void  operator delete[](void* __p) _NOEXCEPT
	{
		return vPortFree ( __p );
	}
/*��дC++ new delete�����*/