#pragma once
#include <stdint.h>


uint8_t Check_USB_SD_Busy();	 
void Update_USB_SD_Busy_Time();	 
extern "C" bool Get_SD_Init_Complete(void);
extern "C" void Set_SD_Init_Complete(void);
extern "C" void Clear_SD_Init_Complete(void);

extern "C" uint8_t Lock_SD( double TIMEOUT );
extern "C" void UnLock_SD(void);

void init_drv_SDMMC(void);
	 
