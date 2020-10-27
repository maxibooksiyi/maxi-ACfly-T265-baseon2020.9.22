#pragma once

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*Flash��д�ӿ�
	buf:����д������
	sector_addr:��ʼ������ַ��1����ڶ���������
	sectors:����д����������
	waitT:��ʱʱ�䣨�룩

	length:Ҫ�����ֽ���
	addr:����ʼ��ַ��1�����Flash�ڶ����ֽڿ�ʼ����
*/
	//��д����
	bool FlashWriteSectors( const uint8_t* buf , uint32_t sector_addr , uint16_t sectors , double waitT );
	bool FlashReadSectors( uint8_t* buf , uint32_t sector_addr , uint16_t sectors , double waitT );

	//�����
	bool FlashRead( uint8_t* buf , uint16_t length , uint32_t addr , double waitT );
	
	//��ȡFlash������Ŀ
	uint16_t getFlashSectorCount();
	//��ȡFlash������С
	uint32_t getFlashSectorSize();
/*Flash��д�ӿ�*/

#ifdef __cplusplus
	}
#endif