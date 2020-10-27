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
	
//MSC设备是否支持写入定义
static volatile bool msc_writable = true;

//USB_CDC流读写缓冲区定义
StreamBufferHandle_t usb_rx_streambufferhandle_t;
StreamBufferHandle_t usb_tx_streambufferhandle_t;

//USB_CDC读写缓冲区大小
#define USB_RX_BUFFER_LENGHT 1024*10
#define USB_TX_BUFFER_LENGHT 1024*10

//USB_CDC读写互斥锁
SemaphoreHandle_t USBD_VCOM_RxSemphr;
SemaphoreHandle_t USBD_VCOM_TxSemphr;

static bool IS_USB_CONNECTED = false;

/*发送上锁
	上锁保证数据连续性
	上锁之后必须解锁
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

/*USBD虚拟串口发送函数（将要发送的数据压入缓冲区）
	data:要发送的数据指针
	length:要发送的数据长度（字节）
	Sync_waitTime:线程同步最大等待时间（s）
	Send_waitTime:等待缓冲区有空间的最大等待时间（s）
	返回值：实际发送的字节数
	解释：如果在指定等待时间内接收不到足够数据（发送缓冲区位置不足）
				将只发送前半段数据（只将前半段数据压入缓冲区）
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
	//获取信号量
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

/*USBD虚拟串口接收函数（将要发送的数据压入缓冲区）
	data:接收数据指针
	length:要接收的数据长度（字节）
	Sync_waitTime:线程同步最大等待时间（s）
	Rc_waitTime:等待数据的最大等待时间（s）
	返回值：实际接收到的字节数
	解释:如果在指定等待时间内接收不到足够数据（接收缓冲区里没这么多数据）
				就接收尽量多的数据
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
	//获取信号量	
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
	//复位USB
	USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(0);
	USB_StopDevice(usb_otg); 
	USB_DevDisconnect(usb_otg);
	HAL_PCD_MspDeInit();
	
	//读写访问互斥锁
	USBD_VCOM_RxSemphr = xSemaphoreCreateRecursiveMutex();
	USBD_VCOM_TxSemphr = xSemaphoreCreateRecursiveMutex();

	//读写缓冲区
  usb_rx_streambufferhandle_t = xStreamBufferCreate(USB_RX_BUFFER_LENGHT,1);
	usb_tx_streambufferhandle_t = xStreamBufferCreate(USB_TX_BUFFER_LENGHT,1);
	
	//初始化USB底层IO
	HAL_PCD_MspInit();

	//注册USB端口
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


/* 回调函数 */

	//CDC数据接收回调函数(任务中调用)
	extern "C" void tud_cdc_rx_cb(uint8_t itf,const void* p_data,uint16_t count)
	{
		xStreamBufferSend( usb_rx_streambufferhandle_t,p_data,count,0 );
	}  

	// USB挂起事件回调函数(任务中调用)
	// Within 7ms, device must draw an average of current less than 2.5 mA from bus
	extern "C" void tud_suspend_cb(uint8_t rhport,bool remote_wakeup_en)
	{
		void usbd_reset(uint8_t rhport);
		usbd_reset(rhport);
	}

	// USB挂载事件回调函数(任务中调用)
	extern "C" void tud_mount_cb(void)
	{

	}

	// USB拔出事件回调函数(任务中调用)
	extern "C" void tud_umount_cb(uint8_t rhport)
	{
		void usbd_reset(uint8_t rhport);
		usbd_reset(rhport);		
	}
	
	//MSC设备是否只读,true:可读可写,false:只读
  extern "C" bool tud_msc_is_writable_cb(uint8_t lun)
	{
	  return msc_writable;
	}
	
/*回调函数*/
	
	
//USB中断
extern "C"  void OTG_FS_IRQHandler(void)
{
	usb_handler(0);
}
