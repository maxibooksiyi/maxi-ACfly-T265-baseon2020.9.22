#pragma once

//ͼ�񳤿�
#define LCD_W 240
#define LCD_H 240

/*������֤ͨ��������
	����֮��������
	Sync_waitTime����ʱʱ��
*/
bool Lock_Oled( double Sync_waitTime = -1 );
void UnLock_Oled();

/*����ΪColor��ɫ
	Sync_waitTime����ʱʱ��
*/
bool LCD_Clear( uint16_t Color, double Sync_waitTime = -1 );

/*��(x1,y1)-(x2,y2)��������д��ͼ��pic
	Sync_waitTime����ʱʱ��
*/
bool LCD_WritePicture( uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, void* pic, double Sync_waitTime = -1 );

void init_drv_Oled();