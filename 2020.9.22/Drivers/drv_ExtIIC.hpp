#pragma once

/*������֤ͨ��������
	����֮��������
	Sync_waitTime����ʱʱ��
*/
bool Lock_ExtIIC( double Sync_waitTime = -1 );
void Unlock_ExtIIC();

/*7λ��ַ��������
	addr��7λ��������ַ
	datas��Ҫ���͵�����ָ��
	length�����ݳ���
	Sync_waitTime����ʱʱ��
*/
bool ExtIIC_SendAddr7( uint8_t addr, const uint8_t* datas, uint16_t length, double Sync_waitTime = -1 );

/*7λ��ַ���Ͳ���������
	addr��7λ��������ַ
	datas��Ҫ���͵�����ָ��
	length�����ݳ���
	Sync_waitTime����ʱʱ��
*/
bool ExtIIC_SendReceiveAddr7( uint8_t addr, const uint8_t* tx_datas, uint16_t tx_length, const uint8_t* rx_datas, uint16_t rx_length, double Sync_waitTime = -1 );

void init_drv_ExtIIC();