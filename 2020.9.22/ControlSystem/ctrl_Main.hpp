#pragma once

#define CtrlRateDiv 2
#define CtrlRateHz 400

//����ϵͳ������
bool LockCtrl( double TIMEOUT );
void UnlockCtrl();

//�ϴο���ʱ��
extern TIME last_XYCtrlTime;
extern TIME last_ZCtrlTime;

//MSafe������
extern TaskHandle_t MSafeTaskHandle;
//ǿ��Safe����
extern bool ForceMSafeCtrl;

void init_ControlSystem();