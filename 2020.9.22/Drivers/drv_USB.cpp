#include "stm32h7xx_hal.h"
#include "drv_USB.hpp"
#include "cmsis_os.h"
#include "message_buffer.h"
#include "Basic.hpp"
#include "CommuLink.hpp"
#include "cdc_device.h"
#include "msc_device.h"
#include "usbd.h"
#include "tusb.h"

#define GLOBAL_BASE(_port)     ((USB_OTG_GlobalTypeDef*) USB_OTG_FS_PERIPH_BASE)
#define USBD_STACK_SIZE     (1024)
static StackType_t  __attribute__ (( section(".AXI_RAM") )) usb_device_stack[USBD_STACK_SIZE];
StaticTask_t usb_device_taskdef;		
	
//MSC�豸�Ƿ�֧��д�붨��
static volatile bool msc_writable = true;

//USB_CDC����д����������
StreamBufferHandle_t usb_rx_streambufferhandle_t;
StreamBufferHandle_t usb_tx_streambufferhandle_t;

//USB_CDC��д��������С
#define USB_RX_BUFFER_LENGHT 1024*10
#define USB_TX_BUFFER_LENGHT 1024*10

//USB_CDC��д������
SemaphoreHandle_t USBD_VCOM_RxSemphr;
SemaphoreHandle_t USBD_VCOM_TxSemphr;

static bool IS_USB_CONNECTED = false;

/*��������
	������֤����������
	����֮��������
*/
bool Lock_USBD_VCOM( double Sync_waitTime )
{
	uint32_t Sync_waitTicks;
	if( Sync_waitTicks > 0 )
		Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
	else
		Sync_waitTicks = portMAX_DELAY;
	if( xSemaphoreTakeRecursive( USBD_VCOM_TxSemphr , Sync_waitTicks ) == pdTRUE )
		return true;
	return false;
}

void Unlock_USBD_VCOM()
{
	xSemaphoreGiveRecursive( USBD_VCOM_TxSemphr );
}

/*USBD���⴮�ڷ��ͺ�������Ҫ���͵�����ѹ�뻺������
	data:Ҫ���͵�����ָ��
	length:Ҫ���͵����ݳ��ȣ��ֽڣ�
	Sync_waitTime:�߳�ͬ�����ȴ�ʱ�䣨s��
	Send_waitTime:�ȴ��������пռ�����ȴ�ʱ�䣨s��
	����ֵ��ʵ�ʷ��͵��ֽ���
	���ͣ������ָ���ȴ�ʱ���ڽ��ղ����㹻���ݣ����ͻ�����λ�ò��㣩
				��ֻ����ǰ������ݣ�ֻ��ǰ�������ѹ�뻺������
*/
uint16_t Write_USBD_VCOM( const uint8_t *data, uint16_t length, double Send_waitTime, double Sync_waitTime )
{
	if( length == 0 )
		return 0;
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
	if( xSemaphoreTakeRecursive( USBD_VCOM_TxSemphr , Sync_waitTicks ) == pdTRUE )
	{
		uint16_t data_sent = xStreamBufferSend( usb_tx_streambufferhandle_t,data,length,0 );
		if(_usbd_dev.ep_status[3][1].busy == false ) 
		{
			tud_cdc_n_write_flush(0);
		}
		xSemaphoreGiveRecursive( USBD_VCOM_TxSemphr );
		return data_sent;
	}
	else
		return 0;			
}

/*USBD���⴮�ڽ��պ�������Ҫ���͵�����ѹ�뻺������
	data:��������ָ��
	length:Ҫ���յ����ݳ��ȣ��ֽڣ�
	Sync_waitTime:�߳�ͬ�����ȴ�ʱ�䣨s��
	Rc_waitTime:�ȴ����ݵ����ȴ�ʱ�䣨s��
	����ֵ��ʵ�ʽ��յ����ֽ���
	����:�����ָ���ȴ�ʱ���ڽ��ղ����㹻���ݣ����ջ�������û��ô�����ݣ�
				�ͽ��վ����������
*/
uint16_t Read_USBD_VCOM( uint8_t *data, uint16_t length, double Rc_waitTime, double Sync_waitTime )
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
	if( xSemaphoreTakeRecursive( USBD_VCOM_RxSemphr , Sync_waitTicks ) == pdTRUE )
	{
		uint16_t rc_length = xStreamBufferReceive(usb_rx_streambufferhandle_t,data,length,Rc_waitTime );
		xSemaphoreGiveRecursive( USBD_VCOM_RxSemphr );
		return rc_length;
	}
	else
		return 0;		
}


void HAL_PCD_MspInit()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**USB_OTG_FS GPIO Configuration    
	PA11     ------> USB_OTG_FS_DM
	PA12     ------> USB_OTG_FS_DP 
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	__HAL_RCC_USB_OTG_FS_CLK_ENABLE();

	/* Peripheral interrupt init */
	HAL_NVIC_SetPriority(OTG_FS_IRQn,5, 0);
}

void HAL_PCD_MspDeInit()
{
   __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
   HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);
   HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
}

static void usb_device_task(void* argument);
void init_drv_USB()
{
	//��λUSB
	USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(0);
	USB_StopDevice(usb_otg); 
	USB_DevDisconnect(usb_otg);
	HAL_PCD_MspDeInit();
	
	//��д���ʻ�����
	USBD_VCOM_RxSemphr = xSemaphoreCreateRecursiveMutex();
	USBD_VCOM_TxSemphr = xSemaphoreCreateRecursiveMutex();

	//��д������
  usb_rx_streambufferhandle_t = xStreamBufferCreate(USB_RX_BUFFER_LENGHT,1);
	usb_tx_streambufferhandle_t = xStreamBufferCreate(USB_TX_BUFFER_LENGHT,1);
	
	//��ʼ��USB�ײ�IO
	HAL_PCD_MspInit();

	//ע��USB�˿�
  Port USBDVCOM;	
	USBDVCOM.write = Write_USBD_VCOM;
	USBDVCOM.lock = Lock_USBD_VCOM;
	USBDVCOM.unlock = Unlock_USBD_VCOM;
	USBDVCOM.read = Read_USBD_VCOM;
	CommuPortRegister(USBDVCOM);	
	
	xTaskCreateStatic( usb_device_task, "usbd",USBD_STACK_SIZE, NULL,SysPriority_UserTask, usb_device_stack, &usb_device_taskdef);
}

static void usb_device_task(void* argument)
{
	(void) argument;
	tusb_init();
  while (1)
  {
    usb_task();
  }	
}


/* �ص����� */

	//CDC���ݽ��ջص�����(�����е���)
	extern "C" void tud_cdc_rx_cb(uint8_t itf,const void* p_data,uint16_t count)
	{
		xStreamBufferSend( usb_rx_streambufferhandle_t,p_data,count,0 );
	}  

	// USB�����¼��ص�����(�����е���)
	// Within 7ms, device must draw an average of current less than 2.5 mA from bus
	extern "C" void tud_suspend_cb(uint8_t rhport,bool remote_wakeup_en)
	{
		void usbd_reset(uint8_t rhport);
		usbd_reset(rhport);
	}

	// USB�����¼��ص�����(�����е���)
	extern "C" void tud_mount_cb(void)
	{

	}

	// USB�γ��¼��ص�����(�����е���)
	extern "C" void tud_umount_cb(uint8_t rhport)
	{
		void usbd_reset(uint8_t rhport);
		usbd_reset(rhport);		
	}
	
	//MSC�豸�Ƿ�ֻ��,true:�ɶ���д,false:ֻ��
  extern "C" bool tud_msc_is_writable_cb(uint8_t lun)
	{
	  return msc_writable;
	}
	
/*�ص�����*/
	
	
//USB�ж�
extern "C"  void OTG_FS_IRQHandler(void)
{
	usb_handler(0);
}
