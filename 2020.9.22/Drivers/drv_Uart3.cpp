#include "Basic.hpp"
#include "drv_Uart3.hpp"
#include "Commulink.hpp"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "event_groups.h"

//���ͻ�����
static SemaphoreHandle_t TxSemphr;
//USB������Ϣ������
#define TxStreamBufferSize 1024
#define TxBufferSize 64
Static_AXIDMABuf uint8_t TxBuffer[TxBufferSize];
static StreamBufferHandle_t TxStreamBuffer;
static bool TxCompleted = true;
//������ɱ�־
static EventGroupHandle_t events = xEventGroupCreate();

//���ջ�����
static SemaphoreHandle_t RxSemphr;
//���ջ�����
#define RxStreamBufferSize 1024
static StreamBufferHandle_t RxStreamBuffer;

static inline void StartSend( const uint8_t* buf, uint16_t len )
{
	//��շ�����ɱ�־
	TxCompleted = false;
	xEventGroupClearBits( events, (1<<0) );
	//�رմ���DMA����
	USART3->CR3 &= ~(1<<7);
	//���DMA״̬
	DMA2->LIFCR = (1<<5) | (1<<4)  | (1<<3)  | (1<<2)  | (1<<0);
	//����DMA�洢����ַ
	DMA2_Stream0->M0AR = (uint32_t)buf;
	//����DMA��������
	DMA2_Stream0->NDTR = len;
	//ʹ��DMA
	DMA2_Stream0->CR |= (1<<0);
	//���Uart״̬
	USART3->ICR = (1<<6);	
	//�򿪴���DMA����
	USART3->CR3 |= (1<<7);
}

static inline void setBaudRate( uint32_t baud )
{
	USART3->CR1 &= ~( (1<<6) | (1<<3) );
	USART3->CR1 &= ~(1<<0);
	USART3->BRR = USART234578CLK / baud;
	USART3->CR1 |= (1<<6) | (1<<3) | (1<<0);
}

void init_drv_Uart3()
{
	/*�˿ڳ�ʼ��*/
		TxSemphr = xSemaphoreCreateRecursiveMutex();
		RxSemphr = xSemaphoreCreateMutex();
		//���ͻ�����
		TxStreamBuffer = xStreamBufferCreate( TxStreamBufferSize , 1 );
		//���ջ�����
		RxStreamBuffer = xStreamBufferCreate( RxStreamBufferSize , 1 );
	/*�˿ڳ�ʼ��*/
	
	/*GPIO��ʼ��*/
		//PD8(USART3 TX) PD9(USART3 RX)
	
		//��GPIODʱ��
		RCC->AHB4ENR|=(1<<3);
		os_delay(1e-2);
	
		//���ù��� TX���� RX��©����
		GPIOD->OTYPER |= (1<<9);
		set_register( GPIOD->PUPDR, 0b01, 9*2 , 2 );
		set_register( GPIOD->AFR[1], 7, 8*4-32, 4 );
		set_register( GPIOD->AFR[1], 7, 9*4-32, 4 );
		set_register( GPIOD->MODER, 0b10, 8*2, 2);
		set_register( GPIOD->MODER, 0b10, 9*2, 2);		
	/*GPIO��ʼ��*/
	
	/*Uart3��ʼ��*/
		//��Uart3ʱ��
		RCC->APB1LENR|=(1<<18);
		os_delay(1e-2);
	
		USART3->CR1 = (1<<29) | (1<<26) | (1<<6) | (1<<3) | (1<<2);
		setBaudRate(115200);
		USART3->RTOR = 10;
		USART3->CR2 = (1<<23);
		USART3->CR3 = (1<<28) | (0b011<<25);		
		USART3->CR1 |= (1<<0);    //USARTʹ��	
		NVIC_SetPriority(USART3_IRQn,5);
		NVIC_EnableIRQ(USART3_IRQn);
		xEventGroupSetBits( events, (1<<0) );
	/*Uart3��ʼ��*/
	
	/*DMA��ʼ��*/
		//��DMA2ʱ��
		RCC->AHB1ENR |= (1<<1);
		delay(1e-5);
		
		//DMA2_Stream0 Uart3 TX
		DMA2_Stream0->PAR = (uint32_t)&USART3->TDR;
		DMAMUX1_Channel8->CCR = (46<<0);
		DMA2_Stream0->CR = (1<<20) | (0<<16) | (0<<13) | (1<<10) | (0<<9) | (0b01<<6);
		DMA2_Stream0->FCR = (1<<2) | (3<<0);
	/*DMA��ʼ��*/
	
	//�˿�ע��
	Port UartPort;
	UartPort.write = Write_Uart3;
	UartPort.lock = Lock_Uart3;
	UartPort.unlock = Unlock_Uart3;
	UartPort.read = Read_Uart3;
	CommuPortRegister(UartPort);
}

static void TxTCB( void *pvParameter1, uint32_t ulParameter2 )
{
	uint16_t length = xStreamBufferReceive( TxStreamBuffer, TxBuffer, TxBufferSize, 0 );
	if( length > 0 )
		StartSend( TxBuffer, length );
	else
	{
		TxCompleted = true;
		xEventGroupSetBits( events, (1<<0) );
	}
}
extern "C" void USART3_IRQHandler()
{
	bool err = false;
	if( USART3->ISR & (1<<3) )
	{	//����fifo���
		err = true;
		USART3->ICR = 1<<3;
	}
	
	BaseType_t HigherPriorityTaskWoken = pdFALSE;
	if( (USART3->ISR & (1<<11)) || (USART3->ISR & (1<<26)) )
	{	//�����ж�
		USART3->ICR = 1<<11;
		
		uint8_t buf[18];
		uint8_t len = 0;
		while( ( USART3->ISR & (1<<5) ) != 0 && len<18 )
			buf[len++] = USART3->RDR;
		if( len > 0 )
			xStreamBufferSendFromISR( RxStreamBuffer , buf , len , &HigherPriorityTaskWoken );
	}
	if( (USART3->ISR & (1<<6)) )
	{	//��������ж�
		USART3->ICR = 1<<6;		
		xTimerPendFunctionCallFromISR( TxTCB, 0, 0, &HigherPriorityTaskWoken );
	}
	portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}

/*���ڽ��պ�������Ҫ���͵�����ѹ�뻺������
	data:��������ָ��
	length:Ҫ���յ����ݳ��ȣ��ֽڣ�
	Sync_waitTime:�߳�ͬ�����ȴ�ʱ�䣨s��
	Rc_waitTime:�ȴ����ݵ����ȴ�ʱ�䣨s��
	����ֵ��ʵ�ʽ��յ����ֽ���
	����:�����ָ���ȴ�ʱ���ڽ��ղ����㹻���ݣ����ջ�������û��ô�����ݣ�
				�ͽ��վ����������
*/
uint16_t Read_Uart3( uint8_t *data, uint16_t length, double Rc_waitTime, double Sync_waitTime )
{
	uint32_t Sync_waitTicks;
	if( Sync_waitTime >= 0 )
		Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
	else
		Sync_waitTicks = portMAX_DELAY;
	uint32_t Rc_waitTicks;
	if( Rc_waitTime >= 0 )
		Rc_waitTicks = Rc_waitTime*configTICK_RATE_HZ;
	else
		Rc_waitTicks = portMAX_DELAY;
	//��ȡ�ź���
	if( xSemaphoreTake( RxSemphr , Sync_waitTicks ) == pdTRUE )
	{	
		uint16_t rc_length = xStreamBufferReceive( RxStreamBuffer , data , length , Rc_waitTicks );
		
		//�ͷ��ź���
		xSemaphoreGive( RxSemphr );
		return rc_length;
	}
	else
		return 0;
}

/*���ڷ��ͺ�������Ҫ���͵�����ѹ�뻺������
	data:Ҫ���͵�����ָ��
	length:Ҫ���͵����ݳ��ȣ��ֽڣ�
	Sync_waitTime:�߳�ͬ�����ȴ�ʱ�䣨s��
	Send_waitTime:�ȴ��������пռ�����ȴ�ʱ�䣨s��
	����ֵ��ʵ�ʷ��͵��ֽ���
	���ͣ������ָ���ȴ�ʱ���ڽ��ղ����㹻���ݣ����ͻ�����λ�ò��㣩
				��ֻ����ǰ������ݣ�ֻ��ǰ�������ѹ�뻺������
*/
uint16_t Write_Uart3( const uint8_t *data, uint16_t length, double Send_waitTime, double Sync_waitTime )
{
	uint32_t Sync_waitTicks;
	if( Sync_waitTicks >= 0 )
		Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
	else
		Sync_waitTicks = portMAX_DELAY;
	uint32_t Send_waitTicks;
	if( Send_waitTime >= 0 )
		Send_waitTicks = Send_waitTime*configTICK_RATE_HZ;
	else
		Send_waitTicks = portMAX_DELAY;
	//��ȡ�ź���
	if( xSemaphoreTakeRecursive( TxSemphr , Sync_waitTicks ) == pdTRUE )
	{	
		uint16_t data_sent = xStreamBufferSend( TxStreamBuffer, data, length, Send_waitTicks );
		if( TxCompleted )
		{
			//USB���пɷ���
			uint16_t sd_length;
			sd_length = xStreamBufferReceive( TxStreamBuffer, TxBuffer, TxBufferSize, 0 );
			if( sd_length > 0 )
				StartSend( TxBuffer, sd_length );			
		}
		//�ͷ��ź���
		xSemaphoreGiveRecursive( TxSemphr );
		return data_sent;
	}
	else
		return 0;
}

/*��������
	������֤����������
	����֮��������
*/
bool Lock_Uart3( double Sync_waitTime )
{
	uint32_t Sync_waitTicks;
	if( Sync_waitTicks > 0 )
		Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
	else
		Sync_waitTicks = portMAX_DELAY;
	if( xSemaphoreTakeRecursive( TxSemphr , Sync_waitTicks ) == pdTRUE )
		return true;
	return false;
}
void Unlock_Uart3()
{
	xSemaphoreGiveRecursive( TxSemphr );
}

/*��ս��ջ�����
	Sync_waitTime:�߳�ͬ�����ȴ�ʱ�䣨s��
*/
bool ResetRx_Uart3( double Sync_waitTime )
{
	uint32_t Sync_waitTicks;
	if( Sync_waitTime >= 0 )
		Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
	else
		Sync_waitTicks = portMAX_DELAY;
	//��ȡ�ź���
	if( xSemaphoreTake( RxSemphr , Sync_waitTicks ) == pdTRUE )
	{
		xStreamBufferReset(RxStreamBuffer);
		xSemaphoreGive(RxSemphr);
		return true;
	}
	return false;
}

/*�ȴ��������
	waitTime:���ȴ�ʱ�䣨s��
*/
bool WaitSent_Uart3( double waitTime )
{
	uint32_t waitTicks;
	if( waitTicks >= 0 )
		waitTicks = waitTime*configTICK_RATE_HZ;
	else
		waitTicks = portMAX_DELAY;
	EventBits_t rtbits = xEventGroupWaitBits( events, (1<<0), pdFALSE, pdTRUE, waitTicks );
	if( rtbits == (1<<0) )
		return true;
	return false;
}

/*���Ĳ�����
	baud_rate:������
	Send_waitTime:�ȴ�������ɵ����ȴ�ʱ�䣨s��
	Sync_waitTime:�߳�ͬ�����ȴ�ʱ�䣨s��
*/
bool SetBaudRate_Uart3( uint32_t baud_rate, double Send_waitTime, double Sync_waitTime )
{
	uint32_t Sync_waitTicks;
	if( Sync_waitTicks >= 0 )
		Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
	else
		Sync_waitTicks = portMAX_DELAY;
	uint32_t Send_waitTicks;
	if( Send_waitTime >= 0 )
		Send_waitTicks = Send_waitTime*configTICK_RATE_HZ;
	else
		Send_waitTicks = portMAX_DELAY;
	//��ȡ�ź���
	if( xSemaphoreTakeRecursive( TxSemphr , Sync_waitTicks ) == pdTRUE )
	{	
		EventBits_t rtbits = xEventGroupWaitBits( events, (1<<0), pdFALSE, pdTRUE, Send_waitTicks );
		bool result = false;
		if( rtbits == (1<<0) )
		{
			setBaudRate(baud_rate);
			result = true;
		}
		xSemaphoreGiveRecursive( TxSemphr );
		return result;
	}
	return false;
}