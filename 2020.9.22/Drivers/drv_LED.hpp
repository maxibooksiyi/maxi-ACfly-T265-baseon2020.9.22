#pragma once

void init_drv_LED(void);

/*
	LED�����Ⱥ���
	R��G��B�����Ȱٷֱȣ�0-100��
*/
void set_LedBrightness( float R , float G , float B );

/*
	������Ƶ�ʵ��ں���
	freq:������Ƶ��
*/
void set_BuzzerFreq( unsigned short freq );

/*
	���������캯��
	on:�Ƿ�����
*/
void set_BuzzerOnOff( bool on );