#pragma once

#include <stdint.h>

void init_drv_Uart7();

/*���ڽ��պ�������Ҫ���͵�����ѹ�뻺������
	data:��������ָ��
	length:Ҫ���յ����ݳ��ȣ��ֽڣ�
	Sync_waitTime:�߳�ͬ�����ȴ�ʱ�䣨s��
	Rc_waitTime:�ȴ����ݵ����ȴ�ʱ�䣨s��
	����ֵ��ʵ�ʽ��յ����ֽ���
	����:�����ָ���ȴ�ʱ���ڽ��ղ����㹻���ݣ����ջ�������û��ô�����ݣ�
				�ͽ��վ����������
*/
uint16_t Read_Uart7( uint8_t *data, uint16_t length, double Rc_waitTime, double Sync_waitTime );

/*���ڷ��ͺ�������Ҫ���͵�����ѹ�뻺������
	data:Ҫ���͵�����ָ��
	length:Ҫ���͵����ݳ��ȣ��ֽڣ�
	Sync_waitTime:�߳�ͬ�����ȴ�ʱ�䣨s��
	Send_waitTime:�ȴ��������пռ�����ȴ�ʱ�䣨s��
	����ֵ��ʵ�ʷ��͵��ֽ���
	���ͣ������ָ���ȴ�ʱ���ڽ��ղ����㹻���ݣ����ͻ�����λ�ò��㣩
				��ֻ����ǰ������ݣ�ֻ��ǰ�������ѹ�뻺������
*/
uint16_t Write_Uart7( const uint8_t *data, uint16_t length, double Send_waitTime, double Sync_waitTime );

/*��������
	������֤����������
	����֮��������
*/
bool Lock_Uart7( double Sync_waitTime );
void Unlock_Uart7();

/*��ս��ջ�����
	Sync_waitTime:�߳�ͬ�����ȴ�ʱ�䣨s��
*/
bool ResetRx_Uart7( double Sync_waitTime );

/*�ȴ��������
	waitTime:���ȴ�ʱ�䣨s��
*/
bool WaitSent_Uart7( double waitTime );

/*���Ĳ�����
	baud_rate:������
	Send_waitTime:�ȴ�������ɵ����ȴ�ʱ�䣨s��
	Sync_waitTime:�߳�ͬ�����ȴ�ʱ�䣨s��
*/
bool SetBaudRate_Uart7( uint32_t baud_rate, double Send_waitTime, double Sync_waitTime );