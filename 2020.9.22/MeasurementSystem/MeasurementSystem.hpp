#pragma once

#include "vector2.hpp"
#include "vector3.hpp"
#include "Quaternion.hpp"
#include "map_projection.hpp"

//��ȡ���ֽ�WGAʶ����
void MS_get_WGA( uint32_t* WGA );
//��ȡ������֤���
bool MS_WGA_Correct();

//��ȡ��ǰʹ�õ�������
uint8_t get_current_use_IMUGyroscope();
//��ȡ��ǰʹ�õļ��ٶȼ�
uint8_t get_current_use_IMUAccelerometer();

enum MS_Status
{
	MS_Initializing ,
	MS_Ready ,
	MS_Err ,
};

/*��������Ϣ*/
	struct PosSensorHealthInf1
	{
		//���������
		uint8_t sensor_ind;
		//����λ��
		vector3<double> PositionENU;
		//������λ��
		double sensor_pos;
		//������ƫ�ƣ�����������ʱ���£�
		//HOffset+PositionENU = ����������ֵ
		double HOffset;
		//�ϴν���ʱ��
		TIME last_healthy_TIME;
		//�Ƿ���ã�������ʱ������Ч��
		bool available;
		//�������������½磨������-���㣩
		double NoiseMin, NoiseMax;
		//�ٶ�����
		double VNoise;
	};
	struct PosSensorHealthInf2
	{
		//���������
		uint8_t sensor_ind;
		//�Ƿ�ȫ��λ������
		//�ǲ��ж�λת����Ϣ
		bool global_sensor;
		//��λ����ת����Ϣ
		Map_Projection mp;
		//����λ��
		vector3<double> PositionENU;
		//������λ��
		vector2<double> sensor_pos;
		//������ƫ�ƣ�����������ʱ���£�
		//HOffset+PositionENU = ����������ֵ
		vector2<double> HOffset;
		//�ϴν���ʱ��
		vector2<TIME> last_healthy_TIME;
		//�Ƿ���ã�������ʱ������Ч��
		bool available;
		//�������������½磨������-���㣩
		vector2<double> NoiseMin, NoiseMax;
		//�ٶ�����
		vector2<double> VNoise;
	};
	struct PosSensorHealthInf3
	{
		//���������
		uint8_t sensor_ind;
		//�Ƿ�ȫ��λ������
		//�ǲ��ж�λת����Ϣ
		bool global_sensor;
		//��λ����ת����Ϣ
		Map_Projection mp;
		//����λ��
		vector3<double> PositionENU;
		//������λ��
		vector3<double> sensor_pos;
		//������ƫ�ƣ�����������ʱ���£�
		//HOffset+PositionENU = ����������ֵ
		vector3<double> HOffset;
		//�ϴν���ʱ��
		vector3<TIME> last_healthy_TIME;
		//�Ƿ���ã�������ʱ������Ч��
		bool available;
		//�������������½磨������-���㣩
		vector3<double> NoiseMin, NoiseMax;
		//�ٶ�����
		vector3<double> VNoise;		
	};

	/*XY������������*/
		//��ȡ��ǰXY������
		int8_t get_Current_XYSensor();
	
		//ָ����Ŵ�����������
		bool get_PosSensorHealth_XY( PosSensorHealthInf2* result, uint8_t sensor_ind, double TIMEOUT = -1 );
		//��ǰ������������
		bool get_Health_XY( PosSensorHealthInf2* result, double TIMEOUT = -1 );
		//���Ų�ഫ����������
		bool get_OptimalRange_XY( PosSensorHealthInf2* result, double TIMEOUT = -1 );
		//����ȫ��λ������������
		bool get_OptimalGlobal_XY( PosSensorHealthInf2* result, double TIMEOUT = -1 );
	/*XY������������*/
	
	/*Z������������*/
		//��ȡ��ǰZ������
		int8_t get_Current_ZSensor();
	
		//ָ����Ŵ�����������
		bool get_PosSensorHealth_Z( PosSensorHealthInf1* result, uint8_t sensor_ind, double TIMEOUT = -1 );
		//��ǰ������������
		bool get_Health_Z( PosSensorHealthInf1* result, double TIMEOUT = -1 );
		//���Ų�ഫ����������
		bool get_OptimalRange_Z( PosSensorHealthInf1* result, double TIMEOUT = -1 );
		//����ȫ��λ������������
		bool get_OptimalGlobal_Z( PosSensorHealthInf1* result, double TIMEOUT = -1 );
	/*Z������������*/
	
	/*XYZ������������*/
		//ָ����Ŵ�����������
		bool get_PosSensorHealth_XYZ( PosSensorHealthInf3* result, uint8_t sensor_ind, double TIMEOUT = -1 );
		//���Ų�ഫ����������
		bool get_OptimalRange_XYZ( PosSensorHealthInf3* result, double TIMEOUT = -1 );
		//����ȫ��λ������������
		bool get_OptimalGlobal_XYZ( PosSensorHealthInf3* result, double TIMEOUT = -1 );
	/*XYZ������������*/
/*��������Ϣ*/

/*��̬��Ϣ��ȡ�ӿ�*/
	//��ȡ����ϵͳ״̬
	MS_Status get_Attitude_MSStatus();

	//��ȡ���ڿ��Ƶ��˲���Ľ��ٶ�
	bool get_AngularRate_Ctrl( vector3<double>* result, double TIMEOUT = -1 );
	//��ȡ��̬��Ԫ��
	bool get_Attitude_quat( Quaternion* result, double TIMEOUT = -1 );	
	//��ȡ������Ԫ����ƫ������׼��
	bool get_Airframe_quat( Quaternion* result, double TIMEOUT = -1 );
	//��ȡ������Ԫ����ƫ����׼��
	bool get_AirframeY_quat( Quaternion* result, double TIMEOUT = -1  );
	//��ȡ��ʷ��Ԫ��
	bool get_history_AttitudeQuat( Quaternion* result, double t, double TIMEOUT = -1 );
	//��ȡ��ʷ������Ԫ����ƫ������׼��
	bool get_history_AirframeQuat( Quaternion* result, double t, double TIMEOUT = -1 );
	//��ȡ��ʷ������Ԫ����ƫ����׼��
	bool get_history_AirframeQuatY( Quaternion* result, double t, double TIMEOUT = -1 );
/*��̬��Ϣ��ȡ�ӿ�*/

/*λ����Ϣ��ȡ�ӿ�*/
	//��ȡ����ϵͳ״̬
	MS_Status get_Altitude_MSStatus();
	MS_Status get_Position_MSStatus();
	
	//��ȡʵʱλ��
	bool get_Position( vector3<double>* result, double TIMEOUT = -1 );
	//��ȡʵʱ�ٶȣ������췽��
	bool get_VelocityENU( vector3<double>* result, double TIMEOUT = -1 );
	//��ȡʵʱ����ϵ���ٶȣ������죩
	bool get_AccelerationENU( vector3<double>* result, double TIMEOUT = -1 );
	//��ȡ���ڿ��Ƶ��˲���ĵ���ϵ���ٶ�
	bool get_AccelerationENU_Ctrl( vector3<double>* result, double TIMEOUT = -1 );
	bool get_VelocityENU_Ctrl( vector3<double>* result, double TIMEOUT = -1 );
	bool get_Position_Ctrl( vector3<double>* result, double TIMEOUT = -1 );
	//��ȡ��ͨ�˲����δ��������ƫ�������¶ȣ������ݼ��ٶ�����
	bool get_AccelerationNC_filted( vector3<double>* vec, double TIMEOUT = -1 );
	bool get_AngularRateNC_filted( vector3<double>* vec, double TIMEOUT = -1 );
/*λ����Ϣ��ȡ�ӿ�*/

/*�����Ϣ�ӿ�*/
	struct BatteryCfg
	{
		//��׼��ѹ��V��
		float STVoltage[2];
		//��ѹ�������棨V��
		float VoltMKp[2];
		//�����������棨A��
		float CurrentMKp[2];
		//������W*h��
		float Capacity[2];
		//���ʵ�ѹ��0��0%����ʱ��Ա�׼��ѹ�ĵ�ѹ������б��������
		float VoltP0[2];
		//���ʵ�ѹ��1��10%����ʱ��Ա�׼��ѹ�ĵ�ѹ������б��������
		float VoltP1[2];
		//���ʵ�ѹ��2��20%����ʱ��Ա�׼��ѹ�ĵ�ѹ������б��������
		float VoltP2[2];
		//���ʵ�ѹ��3��30%����ʱ��Ա�׼��ѹ�ĵ�ѹ������б��������
		float VoltP3[2];
		//���ʵ�ѹ��4��40%����ʱ��Ա�׼��ѹ�ĵ�ѹ������б��������
		float VoltP4[2];
		//���ʵ�ѹ��5��50%����ʱ��Ա�׼��ѹ�ĵ�ѹ������б��������
		float VoltP5[2];
		//���ʵ�ѹ��6��60%����ʱ��Ա�׼��ѹ�ĵ�ѹ������б��������
		float VoltP6[2];
		//���ʵ�ѹ��7��70%����ʱ��Ա�׼��ѹ�ĵ�ѹ������б��������
		float VoltP7[2];
		//���ʵ�ѹ��8��80%����ʱ��Ա�׼��ѹ�ĵ�ѹ������б��������
		float VoltP8[2];
		//���ʵ�ѹ��9��90%����ʱ��Ա�׼��ѹ�ĵ�ѹ������б��������
		float VoltP9[2];
		//���ʵ�ѹ��10��100%����ʱ��Ա�׼��ѹ�ĵ�ѹ������б��������
		float VoltP10[2];
	}__PACKED;
	
	//��ѹ
	float get_MainBatteryVoltage();
	//�˲���ѹ��V��
	float get_MainBatteryVoltage_filted();
	//��ʹ�ù��ģ�W*h��
	float get_MainBatteryPowerUsage();
	//�˲����ʣ�W��
	float get_MainBatteryPower_filted();
	//��ص�����A��
	float get_MainBatteryCurrent();
	//CPU�¶ȣ��棩
	float get_CPUTemerature();
	//��ȡ�����Ϣ
	void get_MainBatteryInf( float* Volt, float* Current, float* PowerUsage, float* Power_filted, float* RMPercent );
/*�����Ϣ�ӿ�*/
