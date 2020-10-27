#include "stm32h743xx.h"
#include "drv_Flash.hpp"
#include "FlashIO.h"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "FatFS.h"
#include "flash_diskio.h"

/*QuadSPI����*/
	#define QuadSPI_Enable (QUADSPI->CR |= (1<<0))
	#define QuadSPI_Disable (QUADSPI->CR &= ~(1<<0))
	
	/*����ģʽ*/
		//����ģʽ
		#define FunctionMode_IndirectWrite (0b00<<26)
		#define FunctionMode_IndirectRead (0b01<<26)
		#define FunctionMode_AutoPolling (0b10<<26)
		
		//����ģʽ
		#define DataMode_None (0b00<<24)
		#define DataMode_1Line (0b01<<24)
		#define DataMode_2Line (0b10<<24)
		#define DataMode_4Line (0b11<<24)
		
		//��ָ��������
		#define DummyCycle(x) (x<<18)
		
		//�����ֽڳ���
		#define AlternateBytesSize8 (0b00<<16)
		#define AlternateBytesSize16 (0b01<<16)
		#define AlternateBytesSize24 (0b10<<16)
		#define AlternateBytesSize32 (0b11<<16)
		
		//�����ֽ�ģʽ
		#define AlternateBytesMode_None (0b00<<14)
		#define AlternateBytesMode_1Line (0b01<<14)
		#define AlternateBytesMode_2Line (0b10<<14)
		#define AlternateBytesMode_4Line (0b11<<14)
		
		//��ַ����
		#define AddressSize8 (0b00<<12)
		#define AddressSize16 (0b01<<12)
		#define AddressSize24 (0b10<<12)
		#define AddressSize32 (0b11<<12)
		
		//��ַģʽ
		#define AddressMode_None (0b00<<10)
		#define AddressMode_1Line (0b01<<10)
		#define AddressMode_2Line (0b10<<10)
		#define AddressMode_4Line (0b11<<10)
		
		//ָ��ģʽ
		#define InstructionMode_None (0b00<<8)
		#define InstructionMode_1Line (0b01<<8)
		#define InstructionMode_2Line (0b10<<8)
		#define InstructionMode_4Line (0b11<<8)
		
		//ָ��
		#define Instruction(x) (x<<0)
	/*����ģʽ*/
	static inline void clear_flags()
	{
		QUADSPI->FCR = (1<<4) | (1<<3) | (1<<1) | (1<<0);
	}
/*QuadSPI����*/

/*Flash����*/
	//Flash�����¼���־
	static EventGroupHandle_t FlashEvents;
	
	static uint16_t SectorsCount = 0;
	static uint16_t PageSize = 256 , PagesInSector = 16;
	static inline bool FlashCMD( uint8_t CMD , double waitT )
	{
		//���״̬
		xEventGroupClearBits( FlashEvents , (1<<0) );
		clear_flags();
		//��ʼ����
		QUADSPI->DLR = 0;
		QUADSPI->CCR = FunctionMode_IndirectWrite | \
			InstructionMode_1Line | Instruction(CMD);
		//�ȴ��������
		EventBits_t result = xEventGroupWaitBits( FlashEvents , (1<<0) , pdTRUE , pdTRUE , waitT*configTICK_RATE_HZ );
		if( (result & (1<<0)) != 0 )
		{
			//�ɹ�
			while( ( QUADSPI->SR & ( (1<<5) ) ) != 0 );
			return true;
		}
		else
		{
			//��ʱ��ֹ����
			QUADSPI->CR |= (1<<1);
			while( ( QUADSPI->CR & (1<<1) ) != 0 );
			return false;
		}
	}
	static inline bool FlashCMD( uint8_t CMD , uint32_t addr , double waitT )
	{
		xEventGroupClearBits( FlashEvents , (1<<0) );
		clear_flags();
		QUADSPI->DLR = 0;
		QUADSPI->CCR = FunctionMode_IndirectWrite | \
			InstructionMode_1Line | Instruction(CMD) | \
			AddressMode_1Line | AddressSize24;
		QUADSPI->AR = addr;
		//�ȴ��������
		EventBits_t result = xEventGroupWaitBits( FlashEvents , (1<<0) , pdTRUE , pdTRUE , waitT*configTICK_RATE_HZ );
		if( (result & (1<<0)) != 0 )
		{
			//�ɹ�
			while( ( QUADSPI->SR & ( (1<<5) ) ) != 0 );
			return true;
		}
		else
		{
			//��ʱ��ֹ����
			QUADSPI->CR |= (1<<1);
			while( ( QUADSPI->CR & (1<<1) ) != 0 );
			return false;
		}
	}
	static inline bool FlashCMDRead( uint8_t* buf , uint16_t length , uint8_t CMD , uint32_t addr , uint8_t dummy_cycles = 0 , double waitT = 1.0 )
	{
		clear_flags();
		xEventGroupClearBits( FlashEvents , (1<<0) );
		QUADSPI->DLR = length-1;
		QUADSPI->CCR = FunctionMode_IndirectRead | \
			InstructionMode_1Line | Instruction(CMD) | \
			AddressMode_1Line | AddressSize24 | \
			DummyCycle(dummy_cycles) | \
			DataMode_1Line;
		QUADSPI->AR = addr;
		
		//QuadSPI����AXI����
		MDMA_Channel0->CIFCR = (1<<4) | (1<<3) | (1<<2) | (1<<1) | (1<<0);
		MDMA_Channel0->CTBR = (0<<17) | (0<<16) | (22<<0);
		MDMA_Channel0->CSAR = (uint32_t)&QUADSPI->DR;
		MDMA_Channel0->CDAR = (uint32_t)buf;
		MDMA_Channel0->CTCR = (0<<10) | (0<<8) | (0<<4) | (0b10<<2) | (0<<0);
		MDMA_Channel0->CBNDTR = (0<<20) | (length<<0);
		MDMA_Channel0->CCR |= (1<<0);
		
		//�ȴ��������
		EventBits_t result = xEventGroupWaitBits( FlashEvents , (1<<0) , pdTRUE , pdTRUE , waitT*configTICK_RATE_HZ );
		if( (result & (1<<0)) != 0 )
		{
			//�ɹ�
			while( ( QUADSPI->SR & ( (1<<5) ) ) != 0 );
			return true;
		}
		else
		{
			//��ʱ��ֹ����
			QUADSPI->CR |= (1<<1);
			while( ( QUADSPI->CR & (1<<1) ) != 0 );
			return false;
		}
	}
	static inline bool FlashCMDRead_QuadOutput( uint8_t* buf , uint16_t length , uint8_t CMD , uint32_t addr , uint8_t dummy_cycles = 0 , double waitT = 1.0 )
	{			
		clear_flags();
		xEventGroupClearBits( FlashEvents , (1<<0) );
		QUADSPI->DLR = length-1;
		QUADSPI->CCR = FunctionMode_IndirectRead | \
			InstructionMode_1Line | Instruction(CMD) | \
			AddressMode_1Line | AddressSize24 | \
			DummyCycle(dummy_cycles) | \
			DataMode_4Line;
		QUADSPI->AR = addr;
		
		//QuadSPI����AXI����
		MDMA_Channel0->CCR &= ~(1<<0);
		MDMA_Channel0->CIFCR = (1<<4) | (1<<3) | (1<<2) | (1<<1) | (1<<0);
		MDMA_Channel0->CTBR = (0<<17) | (0<<16) | (22<<0);
		MDMA_Channel0->CSAR = (uint32_t)&QUADSPI->DR;
		MDMA_Channel0->CDAR = (uint32_t)buf;
		MDMA_Channel0->CTCR = (1<<25) | (3<<18) | (0<<10) | (0<<8) | (0<<4) | (0b10<<2) | (0<<0);
		MDMA_Channel0->CBNDTR = (0<<20) | (length<<0);
		MDMA_Channel0->CCR |= (1<<0);
		//�ȴ��������
		EventBits_t result = xEventGroupWaitBits( FlashEvents, (1<<0), pdTRUE, pdTRUE, waitT*configTICK_RATE_HZ );
		if( (result & (1<<0)) != 0 )
		{
			//�ɹ�
			while( ( QUADSPI->SR & ( (1<<5) ) ) != 0 );
			return true;
		}
		else
		{
			//��ʱ��ֹ����
			QUADSPI->CR |= (1<<1);
			while( ( QUADSPI->CR & (1<<1) ) != 0 );
			return false;
		}
	}
	
	extern "C" bool FlashWriteSectors( const uint8_t* buf , uint32_t sector_addr , uint16_t sectors , double waitT )
	{			
		uint32_t addr = sector_addr * PagesInSector*PageSize;
		EventBits_t result;
		for( uint16_t s = 0 ; s < sectors ; ++s )
		{
			/*����Sector*/
				//дʹ��
				clear_flags();
				xEventGroupClearBits( FlashEvents , (1<<0) );
				QUADSPI->DLR = 0;
				QUADSPI->CCR = FunctionMode_IndirectWrite | \
					InstructionMode_1Line | Instruction(0x06);
				result = xEventGroupWaitBits( FlashEvents , (1<<0) , pdTRUE , pdTRUE , waitT*configTICK_RATE_HZ );
				if( (result & (1<<0)) == 0 )
				{	//��ʱʧ�ܣ���ֹ����
					QUADSPI->CR |= (1<<1);
					while( ( QUADSPI->CR & (1<<1) ) != 0 );
					return false;
				}
				while( ( QUADSPI->SR & ( (1<<5) ) ) != 0 );				
			
				clear_flags();
				xEventGroupClearBits( FlashEvents , (1<<0) );
				QUADSPI->DLR = 0;
				QUADSPI->CCR = FunctionMode_IndirectWrite | \
					InstructionMode_1Line | Instruction(0x20) | \
					AddressMode_1Line | AddressSize24;
				QUADSPI->AR = addr + s*PagesInSector*PageSize;
				//�ȴ�������������				
				result = xEventGroupWaitBits( FlashEvents , (1<<0) , pdTRUE , pdTRUE , waitT*configTICK_RATE_HZ );
				if( (result & (1<<0)) == 0 )
				{	//��ʱʧ�ܣ���ֹ����
					QUADSPI->CR |= (1<<1);
					while( ( QUADSPI->CR & (1<<1) ) != 0 );
					return false;
				}
				//��������ͳɹ�
				while( ( QUADSPI->SR & (1<<5) ) != 0 );
							
				//�ȴ�Flash�����������
				clear_flags();
				xEventGroupClearBits( FlashEvents , (1<<1) );	
				QUADSPI->PSMKR = (1<<0);
				QUADSPI->PSMAR = (0<<0);
				QUADSPI->PIR = 8;
				QUADSPI->DLR = 1-1;
				QUADSPI->CCR = FunctionMode_AutoPolling | \
					InstructionMode_1Line | Instruction(0x05) | \
					DataMode_1Line;
				//�ȴ���ѯƥ��				
				result = xEventGroupWaitBits( FlashEvents , (1<<1) , pdTRUE , pdTRUE , waitT*configTICK_RATE_HZ );
				if( (result & (1<<1)) == 0 )
				{	//��ʱʧ�ܣ���ֹ����
					QUADSPI->CR |= (1<<1);
					while( ( QUADSPI->CR & (1<<1) ) != 0 );
					return false;
				}
				//��ѯƥ��ɹ�
				while( ( QUADSPI->SR & (1<<5) ) != 0 );
			/*����Sector*/
			
			for( uint16_t page = 0 ; page < PagesInSector ; ++page )
			{
				//дʹ��
				clear_flags();
				xEventGroupClearBits( FlashEvents , (1<<0) );
				QUADSPI->DLR = 0;
				QUADSPI->CCR = FunctionMode_IndirectWrite | \
					InstructionMode_1Line | Instruction(0x06);				
				result = xEventGroupWaitBits( FlashEvents , (1<<0) , pdTRUE , pdTRUE , waitT*configTICK_RATE_HZ );
				if( (result & (1<<0)) == 0 )
				{	//��ʱʧ�ܣ���ֹ����
					QUADSPI->CR |= (1<<1);
					while( ( QUADSPI->CR & (1<<1) ) != 0 );
					return false;
				}
				while( ( QUADSPI->SR & ( (1<<5) ) ) != 0 );
				
				//����page programָ��
				clear_flags();
				xEventGroupClearBits( FlashEvents , (1<<0) );
				QUADSPI->DLR = PageSize-1;
				QUADSPI->CCR = FunctionMode_IndirectWrite | \
					InstructionMode_1Line | Instruction(0x32) | \
					AddressMode_1Line | AddressSize24 | \
					DataMode_4Line;
				QUADSPI->AR = addr + s*PagesInSector*PageSize + page*PageSize;
				
				//MDMA��������
				MDMA_Channel0->CIFCR = (1<<4) | (1<<3) | (1<<2) | (1<<1) | (1<<0);
				MDMA_Channel0->CTBR = (0<<17) | (0<<16) | (22<<0);
				MDMA_Channel0->CSAR = (uint32_t)(buf + s*PagesInSector*PageSize + page*PageSize);
				MDMA_Channel0->CDAR = (uint32_t)&QUADSPI->DR;
				MDMA_Channel0->CTCR = (1<<25) | (3<<18) | (0<<10) | (0<<8) | (0<<4) | (0b00<<2) | (0b10<<0);
				MDMA_Channel0->CBNDTR = (0<<20) | (PageSize<<0);
				MDMA_Channel0->CCR |= (1<<0);				
				//�ȴ���������				
				result = xEventGroupWaitBits( FlashEvents , (1<<0) , pdTRUE , pdTRUE , waitT*configTICK_RATE_HZ );
				if( (result & (1<<0)) == 0 )
				{	//��ʱʧ�ܣ���ֹ����
					QUADSPI->CR |= (1<<1);
					while( ( QUADSPI->CR & (1<<1) ) != 0 );
					return false;
				}
				//����ͳɹ�
				while( ( QUADSPI->SR & (1<<5) ) != 0 );
				
				//�ȴ�Flashд�������
				clear_flags();
				xEventGroupClearBits( FlashEvents , (1<<1) );	
				QUADSPI->PSMKR = (1<<0);
				QUADSPI->PSMAR = (0<<0);
				QUADSPI->PIR = 8;
				QUADSPI->DLR = 1-1;
				QUADSPI->CCR = FunctionMode_AutoPolling | \
					InstructionMode_1Line | Instruction(0x05) | \
					DataMode_1Line;
				//�ȴ���ѯƥ��
				result = xEventGroupWaitBits( FlashEvents , (1<<1) , pdTRUE , pdTRUE , waitT*configTICK_RATE_HZ );
				if( (result & (1<<1)) == 0 )
				{	//��ʱʧ�ܣ���ֹ����
					QUADSPI->CR |= (1<<1);
					while( ( QUADSPI->CR & (1<<1) ) != 0 );
					return false;
				}
				//��ѯƥ��ɹ�
				while( ( QUADSPI->SR & (1<<5) ) != 0 );
			}	
		}
		return true;
	}
	
	extern "C" bool FlashRead( uint8_t* buf , uint16_t length , uint32_t addr , double waitT )
	{
		return FlashCMDRead_QuadOutput( buf , length , 0x6b , addr , 8 , waitT );
	}
	
	extern "C" bool FlashReadSectors( uint8_t* buf , uint32_t sector_addr , uint16_t sectors , double waitT )
	{
		return FlashCMDRead_QuadOutput( buf , sectors * PagesInSector*PageSize , 0x6b , sector_addr * PagesInSector*PageSize , 8 , waitT );
	}
	
	extern "C" uint16_t getFlashSectorCount()
	{
		return SectorsCount;
	}
	
	extern "C" uint32_t getFlashSectorSize()
	{
		return PagesInSector*PageSize;
	}
/*Flash����*/

void init_drv_Flash()
{
	/*IO��ʼ��*/
		//PB2(CLK) PD11(IO0) PD12(IO1) PE2(IO2) PD13(IO3) PB6(NCS)
		
		//��GPIOʱ��
		RCC->AHB4ENR |= (1<<4) | (1<<3) | (1<<1);
		os_delay(1e-2);
	
		//������������ٶ�
		set_register( GPIOB->OSPEEDR , 0b11 , 2*2 , 2 );	//PB2(CLK)
		set_register( GPIOD->OSPEEDR , 0b11 , 11*2 , 2 );	//PD11(IO0)
		set_register( GPIOD->OSPEEDR , 0b11 , 12*2 , 2 );	//PD12(IO1)
		set_register( GPIOE->OSPEEDR , 0b11 , 2*2 , 2 );	//PE2(IO2)
		set_register( GPIOD->OSPEEDR , 0b11 , 13*2 , 2 );	//PD13(IO3)
		set_register( GPIOB->OSPEEDR , 0b11 , 6*2 , 2 );	//PB6(NCS)
	
		//���ø��ù���ģʽ
		set_register( GPIOB->MODER , 0b10 , 2*2 , 2 );	//PB2(CLK)
		set_register( GPIOD->MODER , 0b10 , 11*2 , 2 );	//PD11(IO0)
		set_register( GPIOD->MODER , 0b10 , 12*2 , 2 );	//PD12(IO1)
		set_register( GPIOE->MODER , 0b10 , 2*2 , 2 );	//PE2(IO2)
		set_register( GPIOD->MODER , 0b10 , 13*2 , 2 );	//PD13(IO3)
		set_register( GPIOB->MODER , 0b10 , 6*2 , 2 );	//PB6(NCS)
	
		//���ø��ù���
		set_register( GPIOB->AFR[0] , 9 , 2*4 , 4 );	//PB2(CLK)
		set_register( GPIOD->AFR[1] , 9 , 11*4-32 , 4 );	//PD11(IO0)
		set_register( GPIOD->AFR[1] , 9 , 12*4-32 , 4 );	//PD12(IO1)
		set_register( GPIOE->AFR[0] , 9 , 2*4 , 4 );	//PE2(IO2)
		set_register( GPIOD->AFR[1] , 9 , 13*4-32 , 4 );	//PD13(IO3)
		set_register( GPIOB->AFR[0] , 10 , 6*4 , 4 );	//PB6(NCS)
		
		//����WP
		set_register( GPIOE->MODER , 0b01 , 2*2 , 2 );	//PE2(IO2)
		GPIOE->BSRR = 1<<2;
	/*IO��ʼ��*/
	
	/*QuadSPI��ʼ��*/
		//��QuadSPIʱ��
		RCC->AHB3ENR |= (1<<14);
		os_delay(1e-2);
		
		//���÷�Ƶ
		QUADSPI->CR = (10<<24) | (1<<22) | (1<<19) | (1<<17) | (3<<8) | (0<<7) | (1<<0);
		//����flash��С8
		QUADSPI->DCR = (22<<16) | (4<<8) | (0<<0);
		//��ձ�־
		clear_flags();
	/*QuadSPI��ʼ��*/
	
	/*MDMA��ʼ��*/
		//��MDMA��Դ
		RCC->AHB3ENR |= (1<<0);
		os_delay(1e-2);
	/*MDMA��ʼ��*/
	
	//��ʼ���¼���־
	FlashEvents = xEventGroupCreate();
		
	//�򿪴�������ж�
	NVIC_SetPriority( QUADSPI_IRQn , 14 );
	NVIC_EnableIRQ( QUADSPI_IRQn );
	__DSB();
	
	#ifdef DCACHE_SIZE
		Aligned_DMABuf volatile uint8_t buf[DCACHE_SIZE*10];
	#else
		volatile uint8_t buf[30];
	#endif
	
	//��ID�ж�flash���ʹ�С
	#ifdef DCACHE_SIZE
		SCB_InvalidateDCache_by_Addr( (uint32_t*)buf , DCACHE_SIZE );
	#endif
	FlashCMDRead( (uint8_t*)buf , 2 , 0x90 , 0 );
	if( buf[0] != 0xef )
		while(1);
	switch( buf[1] )
	{
		case 0x16: //W25Q64(8mbyte)
			SectorsCount = 8*1024/4;
			break;
		case 0x17: //W25Q128(16mbyte)
			SectorsCount = 16*1024/4;
			break;
	}
	
	//release power down
	FlashCMD( 0xab , 0.1 );
	os_delay(0.01);
	
	/*ʹ��Quad Enable*/
		//дQuad Enable�Ĵ���
		FlashCMD( 0x06 , 0.1 );
		clear_flags();
		xEventGroupClearBits( FlashEvents , (1<<0) );
		QUADSPI->DLR = 2-1;
		QUADSPI->CCR = FunctionMode_IndirectWrite | \
			InstructionMode_1Line | Instruction(0x01) | \
			DataMode_1Line;
		*(volatile uint8_t*)&QUADSPI->DR = 0;
		*(volatile uint8_t*)&QUADSPI->DR = 2;
		EventBits_t result = xEventGroupWaitBits( FlashEvents , (1<<0) , pdTRUE , pdTRUE , 1.0*configTICK_RATE_HZ );
		if( (result & (1<<0)) == 0 )
		{	//��ʱʧ�ܣ���ֹ����
			QUADSPI->CR |= (1<<1);
			while( ( QUADSPI->CR & (1<<1) ) != 0 );
		}
		//�ȴ��������
		while( ( QUADSPI->SR & (1<<5) ) != 0 );
		
		//��ѯ�ȴ��������
		clear_flags();
		xEventGroupClearBits( FlashEvents , (1<<1) );	
		QUADSPI->PSMKR = (1<<0);
		QUADSPI->PSMAR = (0<<0);
		QUADSPI->PIR = 8;
		QUADSPI->DLR = 1-1;
		QUADSPI->CCR = FunctionMode_AutoPolling | \
			InstructionMode_1Line | Instruction(0x05) | \
			DataMode_1Line;
		//�ȴ���ѯƥ��
		result = xEventGroupWaitBits( FlashEvents , (1<<1) , pdTRUE , pdTRUE , 1.0*configTICK_RATE_HZ );
		if( (result & (1<<1)) == 0 )
		{	//��ʱʧ�ܣ���ֹ����
			QUADSPI->CR |= (1<<1);
			while( ( QUADSPI->CR & (1<<1) ) != 0 );
		}
		//��ѯƥ��ɹ�
		while( ( QUADSPI->SR & (1<<5) ) != 0 );
	/*ʹ��Quad Enable*/
	
	//��������ģʽ
	QuadSPI_Disable;
	set_register( GPIOE->MODER , 0b10 , 2*2 , 2 );	//PE2(IO2)
	QuadSPI_Enable;
	os_delay(0.01);
	
	retFlash = FATFS_LinkDriver( &Flash_Driver , FlashPath );
		
	
		
		
//	//дʹ��
//				clear_flags();
//				xEventGroupClearBits( FlashEvents , (1<<0) );
//				QUADSPI->DLR = 0;
//				QUADSPI->CCR = FunctionMode_IndirectWrite | \
//					InstructionMode_1Line | Instruction(0x06);
//				result = xEventGroupWaitBits( FlashEvents , (1<<0) , pdTRUE , pdTRUE , 1*configTICK_RATE_HZ );
//				if( (result & (1<<0)) == 0 )
//				{	//��ʱʧ�ܣ���ֹ����
//					QUADSPI->CR |= (1<<1);
//					while( ( QUADSPI->CR & (1<<1) ) != 0 );
//				}
//				while( ( QUADSPI->SR & ( (1<<5) ) ) != 0 );				
//			
//				clear_flags();
//				xEventGroupClearBits( FlashEvents , (1<<0) );
//				QUADSPI->DLR = 0;
//				QUADSPI->CCR = FunctionMode_IndirectWrite | \
//					InstructionMode_1Line | Instruction(0xc7);
//				//�ȴ�������������				
//				result = xEventGroupWaitBits( FlashEvents , (1<<0) , pdTRUE , pdTRUE , 1*configTICK_RATE_HZ );
//				if( (result & (1<<0)) == 0 )
//				{	//��ʱʧ�ܣ���ֹ����
//					QUADSPI->CR |= (1<<1);
//					while( ( QUADSPI->CR & (1<<1) ) != 0 );
//				}
//				//��������ͳɹ�
//				while( ( QUADSPI->SR & (1<<5) ) != 0 );
//							
//				//�ȴ�Flash�����������
//				clear_flags();
//				xEventGroupClearBits( FlashEvents , (1<<1) );	
//				QUADSPI->PSMKR = (1<<0);
//				QUADSPI->PSMAR = (0<<0);
//				QUADSPI->PIR = 8;
//				QUADSPI->DLR = 1-1;
//				QUADSPI->CCR = FunctionMode_AutoPolling | \
//					InstructionMode_1Line | Instruction(0x05) | \
//					DataMode_1Line;
//				//�ȴ���ѯƥ��				
//				result = xEventGroupWaitBits( FlashEvents , (1<<1) , pdTRUE , pdTRUE , 100*configTICK_RATE_HZ );
//				if( (result & (1<<1)) == 0 )
//				{	//��ʱʧ�ܣ���ֹ����
//					QUADSPI->CR |= (1<<1);
//					while( ( QUADSPI->CR & (1<<1) ) != 0 );
//				}
//				//��ѯƥ��ɹ�
//				while( ( QUADSPI->SR & (1<<5) ) != 0 );	
		
		
	
		
//	Static_AXIDMABuf uint8_t txbuf[4096] = {0x1A};
//	while(1)
//	{
//		FlashReadSectors( (uint8_t*)txbuf , 0x45 , 1 , 1.0 );
//		FlashReadSectors( (uint8_t*)txbuf , 0x45 , 1 , 1.0 );
//		if( txbuf[16] != 0x03 )
//		{
//			FlashReadSectors( (uint8_t*)txbuf , 0x45 , 1 , 1.0 );
//		}
//		os_delay(0.1);
//	}
		
//	
//	FRESULT res;
//	res = f_mount(&FlashFatFS, (TCHAR const*)FlashPath, 1);
//	if( res == FR_NO_FILESYSTEM )
//	{
////		Aligned_DMABuf uint8_t workBuffer[_MAX_SS];
////		res = f_mkfs(FlashPath, FM_ANY, 0, workBuffer, sizeof(workBuffer));
//	}
//	if(res== FR_OK)
//	{
//		char filename[] = "0:/ACFly Prophet.TXT";
//		filename[0] = FlashPath[0];
//		if(f_open(&FlashFile, filename, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) 
//		{
//			char text[] = "ACFly Prophet Flash Test!";
//			uint32_t byteswritten;
//			res = f_write(&FlashFile, text, sizeof(text), &byteswritten);
//			f_close(&FlashFile);
//		}
//		if(f_open(&FlashFile, filename, FA_READ) == FR_OK) 
//		{
//			char text[30];
//			uint32_t byteswritten;
//			res = f_read(&FlashFile, text, sizeof(text), &byteswritten);
//			f_close(&FlashFile);
//		}
//	}
//	Static_AXIDMABuf uint8_t txbuf[4096] = {0x1A};
//	FlashWriteSectors( txbuf , 0 , 1 , 1.0 );
//	while(1)
//	{
//		FlashReadSectors( (uint8_t*)txbuf , 0 , 1 , 1.0 );
//		FlashReadSectors( (uint8_t*)txbuf , 0 , 1 , 1.0 );
//		
//		os_delay(0.2);
//	}
}

extern "C" void QUADSPI_IRQHandler()
{
	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
	if( QUADSPI->SR & (1<<1) )
	{	//�������
		QUADSPI->FCR = (1<<1);	
		xEventGroupSetBitsFromISR( FlashEvents , 1<<0 , &pxHigherPriorityTaskWoken );
	}
	else if( QUADSPI->SR & (1<<3) )
	{	//��ѯƥ��
		QUADSPI->FCR = (1<<3);	
		xEventGroupSetBitsFromISR( FlashEvents , 1<<1 , &pxHigherPriorityTaskWoken );
	}
	__DSB();
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}