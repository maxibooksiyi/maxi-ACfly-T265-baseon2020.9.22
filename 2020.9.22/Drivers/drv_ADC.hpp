#pragma once

/*
  ��ȡ��ص�����С,��λ��A
*/
float Get_MainBaterry_Current();

/*
  ��ȡ��ص�ѹ,��λ��V
*/
float Get_MainBaterry_Voltage();

/*
  ��ȡVDDA��׼��ѹ,��λ��V
*/
float Get_VDDA_Voltage();

/*
  ��ȡ�¶ȣ���λ����
*/
float Get_Temperature();

void init_drv_ADC(void);