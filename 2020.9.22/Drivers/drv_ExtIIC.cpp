#include "Basic.hpp"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"

//IIC��������
	#define IIC_AutoEND (1<<25)
	#define IIC_Reload (1<<24)
	#define IIC_NBytes(x) (x<<16)
	#define IIC_NACK (1<<15)
	#define IIC_STOP (1<<14)
	#define IIC_START (1<<13)
	#define IIC_HEAD10R (1<<12)
	#define IIC_ADD10 (1<<11)
	#define IIC_READ (1<<10)
	#define IIC_ADDR7(x) (x<<1)
//IIC״̬
	#define IIC_ARLO (1<<9)
	#define IIC_BERR (1<<8)
	#define IIC_TCR (1<<7)
	#define IIC_TC (1<<6)
	#define IIC_STOPF (1<<5)
	#define IIC_NACKF (1<<4)
#define IIC_TIMEOUT 1.0*configTICK_RATE_HZ

//IIC������
static SemaphoreHandle_t IICMutex = xSemaphoreCreateRecursiveMutex();
//������ɱ�־
static EventGroupHandle_t events = xEventGroupCreate();
//����������
static uint16_t RMTxLength = 0;
static uint16_t RMRxLength = 0;

/*������֤ͨ��������
	����֮��������
	Sync_waitTime����ʱʱ��
*/
bool Lock_ExtIIC( double Sync_waitTime )
{
	uint32_t Sync_waitTicks;
	if( Sync_waitTicks > 0 )
		Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
	else
		Sync_waitTicks = portMAX_DELAY;
	if( xSemaphoreTakeRecursive( IICMutex , Sync_waitTicks ) == pdTRUE )
		return true;
	return false;
}
void Unlock_ExtIIC()
{
	xSemaphoreGiveRecursive( IICMutex );
}

/*7λ��ַ��������
	addr��7λ��������ַ
	datas��Ҫ���͵�����ָ��
	length�����ݳ���
	Sync_waitTime����ʱʱ��
*/
bool ExtIIC_SendAddr7( uint8_t addr, const uint8_t* datas, uint16_t length, double Sync_waitTime )
{
	if( length == 0 )
		return false;
	
	if( Lock_ExtIIC(Sync_waitTime) )
	{
		//���㵱�η��ͳ���
		RMTxLength = length;
		if( RMTxLength > 255 )
			length = 255;
		else
			length = RMTxLength;
		RMTxLength -= length;
		
		//���״̬
		xEventGroupClearBits( events, IIC_STOPF|IIC_NACKF|IIC_ARLO|IIC_BERR );
		/*����DMA*/
			//���DMA״̬
			DMA1_Stream6->CR &= ~(1<<0);
			DMA1->HIFCR = (1<<21) | (1<<20) | (1<<19)  | (1<<18)  | (1<<16);
			//����DMA�洢����ַ
			DMA1_Stream6->M0AR = (uint32_t)datas;
			//����DMA��������
			DMA1_Stream6->NDTR = RMTxLength + length;
			//ʹ��DMA
			DMA1_Stream6->CR |= (1<<0);	
		/*����DMA*/
		
		//��ʼ����
		if( RMTxLength == 0 )
			I2C1->CR2 = 
				IIC_AutoEND | 
				IIC_NBytes(length) | 
				IIC_ADDR7(addr) | 
				IIC_START;
		else
			I2C1->CR2 = 
				IIC_Reload | 
				IIC_NBytes(length) | 
				IIC_ADDR7(addr) | 
				IIC_START;
		
		//�ȴ��������
		uint32_t revents = xEventGroupWaitBits( events, IIC_STOPF|IIC_NACKF|IIC_ARLO, pdTRUE, pdFALSE, IIC_TIMEOUT );
		//����
		Unlock_ExtIIC();
		
		if( revents & (IIC_NACKF|IIC_ARLO) )
			return false;
		else if( revents & IIC_STOPF )
			return true;
		return false;	
	}
	return false;
}

/*7λ��ַ���Ͳ���������
	addr��7λ��������ַ
	datas��Ҫ���͵�����ָ��
	length�����ݳ���
	Sync_waitTime����ʱʱ��
*/
bool ExtIIC_SendReceiveAddr7( uint8_t addr, const uint8_t* tx_datas, uint16_t tx_length, const uint8_t* rx_datas, uint16_t rx_length, double Sync_waitTime )
{
	if( tx_length==0 || rx_length==0 )
		return false;
	
	if( Lock_ExtIIC(Sync_waitTime) )
	{
		//���㵱�η��ͳ���
		RMTxLength = tx_length;
		if( RMTxLength > 255 )
			tx_length = 255;
		else
			tx_length = RMTxLength;
		RMTxLength -= tx_length;
		
		//���㵱�ν��ճ���
		RMRxLength = rx_length;
		
		//���״̬
		xEventGroupClearBits( events, IIC_STOPF|IIC_NACKF|IIC_ARLO|IIC_BERR );
		/*����DMA*/
			//���DMA״̬
			DMA1_Stream6->CR &= ~(1<<0);
			DMA1->HIFCR = (1<<21) | (1<<20) | (1<<19)  | (1<<18)  | (1<<16);
			//����DMA�洢����ַ
			DMA1_Stream6->M0AR = (uint32_t)tx_datas;
			//����DMA��������
			DMA1_Stream6->NDTR = RMTxLength + tx_length;
			//ʹ��DMA
			DMA1_Stream6->CR |= (1<<0);	
		
			//���DMA״̬
			DMA1_Stream7->CR &= ~(1<<0);
			DMA1->HIFCR = (1<<27) | (1<<26) | (1<<25)  | (1<<24)  | (1<<22);
			//����DMA�洢����ַ
			DMA1_Stream7->M0AR = (uint32_t)rx_datas;
			//����DMA��������
			DMA1_Stream7->NDTR = RMRxLength;
			//ʹ��DMA
			DMA1_Stream7->CR |= (1<<0);	
		/*����DMA*/
		
		//��ʼ����
		if( RMTxLength == 0 )
			I2C1->CR2 = 
				IIC_NBytes(tx_length) | 
				IIC_ADDR7(addr) | 
				IIC_START;
		else
			I2C1->CR2 = 
				IIC_Reload | 
				IIC_NBytes(tx_length) | 
				IIC_ADDR7(addr) | 
				IIC_START;
		
		//�ȴ��������
		uint32_t revents = xEventGroupWaitBits( events, IIC_STOPF|IIC_NACKF|IIC_ARLO, pdTRUE, pdFALSE, IIC_TIMEOUT );
		//����
		Unlock_ExtIIC();
		
		if( revents & (IIC_NACKF|IIC_ARLO) )
			return false;
		else if( revents & IIC_STOPF )
			return true;
		return false;	
	}
	return false;
}

extern "C" void I2C1_EV_IRQHandler()
{	
	BaseType_t HigherPriorityTaskWoken = pdFALSE;
	uint8_t ISR = I2C1->ISR;
	
	if( ISR & IIC_TC )
	{	//��Ҫ����Restart
		if( RMRxLength > 0 )
		{
			//���㵱�η��ͳ���
			uint8_t length;
			if( RMRxLength > 255 )
				length = 255;
			else
				length = RMRxLength;
			RMRxLength -= length;
			
			uint32_t addr = I2C1->CR2 & 0x3ff;
			//��ʼ����
			if( RMRxLength == 0 )
				I2C1->CR2 = 
					IIC_READ | 
					IIC_AutoEND | 
					IIC_NBytes(length) | 
					addr | 
					IIC_START;
			else
				I2C1->CR2 = 
					IIC_READ | 
					IIC_Reload | 
					IIC_NBytes(length) | 
					addr | 
					IIC_START;
		}
		else
			I2C1->CR2 = IIC_STOP;
	}
	else if( ISR & IIC_TCR )
	{	//��������
		if( RMTxLength > 0 )
		{
			//���㵱�η��ͳ���
			uint8_t length;
			if( RMTxLength > 255 )
				length = 255;
			else
				length = RMTxLength;
			RMTxLength -= length;

			//��������
			uint32_t CR2 = I2C1->CR2 & (~(0xff<<16));
			I2C1->CR2 = CR2 | IIC_NBytes(length);
		}
		else if( RMRxLength > 0 )
		{
			//���㵱�η��ͳ���
			uint8_t length;
			if( RMRxLength > 255 )
				length = 255;
			else
				length = RMRxLength;
			RMRxLength -= length;

			//��������
			uint32_t CR2 = I2C1->CR2 & (~(0xff<<16));
			I2C1->CR2 = CR2 | IIC_NBytes(length);
		}
		else
			I2C1->CR2 = IIC_STOP;
	}
	else
		//��λ��ɱ�־	
		xEventGroupSetBitsFromISR( events, ISR, &HigherPriorityTaskWoken );
	
	//����Ѽ�¼�����б�־
	I2C1->ICR = ISR & ( (1<<5) | (1<<4) | (1<<3) );
	
	portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}

extern "C" void I2C1_ER_IRQHandler()
{
	BaseType_t HigherPriorityTaskWoken = pdFALSE;
	uint32_t ISR = I2C1->ISR;
	
	//����Ѽ�¼�Ĵ����־
	I2C1->ICR = ISR & ( (1<<13) | (1<<12) | (1<<11) | (1<<10) | (1<<9) | (1<<8) );
	//��λ�����־
	xEventGroupSetBitsFromISR( events, ISR, &HigherPriorityTaskWoken );	
	
	portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}

void init_drv_ExtIIC()
{
	/*����GPIO PB8(IIC1 SCL) PB9(IIC1 SDA)*/
		//��GPIOB��Դ
		RCC->AHB4ENR |= (1<<1);
		os_delay(0.01);
		
		//���Ÿ��ù���
		set_register( GPIOB->MODER , 0b10 , 8*2 , 2 );	//PB8
		set_register( GPIOB->MODER , 0b10 , 9*2 , 2 );	//PB9
		//���ſ�©����
		GPIOB->OTYPER |= (1<<9) | (1<<8);
		set_register( GPIOB->PUPDR , 0b01 , 8*2 , 2 );	//PB8
		set_register( GPIOB->PUPDR , 0b01 , 9*2 , 2 );	//PB9
		//����
		set_register( GPIOB->OSPEEDR , 0b01 , 8*2 , 2 );	//PB8
		set_register( GPIOB->OSPEEDR , 0b01 , 9*2 , 2 );	//PB9
		//����IIC1
		set_register( GPIOB->AFR[1], 4, 8*4-32, 4 );	//PB8
		set_register( GPIOB->AFR[1], 4, 9*4-32, 4 );	//PB9
	/*����GPIO PB8(IIC1 SCL) PB9(IIC1 SDA)*/
	
	/*����IIC1*/
		//��IIC1��Դ
		RCC->APB1LENR |= (1<<21);
		os_delay(0.01);
		
		//��λIIC
		I2C1->CR1 = 0;
		os_delay(0.005);
	
		//IICʱ������
		#ifdef SYSFREQ480
			I2C1->TIMINGR = 0xb03fdb;
		#else
			I2C1->TIMINGR = 0x9034B6;
		#endif
		
		//ʹ��IIC
		I2C1->CR1 = 
			(1<<15) | (1<<14) | 
			(1<<7) | (1<<6) | (1<<5) | (1<<4) | 
			1;
		NVIC_SetPriority(I2C1_EV_IRQn,5);
		NVIC_EnableIRQ(I2C1_EV_IRQn);
		NVIC_SetPriority(I2C1_ER_IRQn,5);
		NVIC_EnableIRQ(I2C1_ER_IRQn);
	/*����IIC1*/
	
	/*DMA��ʼ��*/
		//��DMA1ʱ��
		RCC->AHB1ENR |= (1<<0);
		os_delay(0.005);
		
		//DMA1_Stream6 ExtIIC Tx
		DMA1_Stream6->PAR = (uint32_t)&I2C1->TXDR;
		DMAMUX1_Channel6->CCR = (34<<0);
		DMA1_Stream6->CR = (1<<16) | (0<<13) | (1<<10) | (0<<9) | (0b01<<6);
		DMA1_Stream6->FCR = (1<<2) | (3<<0);
		
		//DMA1_Stream7 ExtIIC Rx
		DMA1_Stream7->PAR = (uint32_t)&I2C1->RXDR;
		DMAMUX1_Channel7->CCR = (33<<0);
		DMA1_Stream7->CR = (1<<16) | (0<<13) | (1<<10) | (0<<9) | (0b00<<6);
		DMA1_Stream7->FCR = (1<<2) | (3<<0);
	/*DMA��ʼ��*/
}