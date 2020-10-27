#include "M10_RCCalib.hpp"
#include "Receiver.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"

M10_RCCalib::M10_RCCalib():Mode_Base( "RCCalib", 10 )
{
	
}

//���ջ�У׼
//��У׼������ң��ͨ��˳��
//ӳ�䵽��
//ͨ��1������
//ͨ��2��ƫ��
//ͨ��3������
//ͨ��4�����
//ͨ��5678������
ModeResult M10_RCCalib::main_func( void* param1, uint32_t param2 )
{
	Receiver rc;	SName rc_name;
	if( getReceiver( &rc, &rc_name, 0.1 ) == false )
	{
		sendLedSignal(LEDSignal_Err1);
		os_delay(1.0);
		return MR_Err;
	}
	
	bool Calibration_Started;
	unsigned char Calibration_Stage;
	unsigned char Calibration_Stage2;
	bool RC_Channel_Calibrated[ 16 ];
	float RC_Calibration_Point[ 3 ][ 16 ];
	uint8_t RC_Calibration_Reflection[8];	
	float max_rc[16] , min_rc[16] , avg_rc[16];
	#define avg_time 50	
	#define reset_constant_check for( uint8_t j = 0 ; j < 16 ; ++j )\
														max_rc[j] = min_rc[j] = avg_rc[j] = rc.raw_data[j];\
												 Calibration_Stage2 = 1
	
	//��ʼ����¼��ǰֵ
	for( unsigned char i = 0 ; i < 16 ; ++i )
	{
		max_rc[i] = min_rc[i] = avg_rc[i] = rc.raw_data[i];
		RC_Channel_Calibrated[i] = false;
	}
	Calibration_Stage = Calibration_Stage2 = 0;
	while(1)
	{
		os_delay(0.02);
		
		//��ȡ���ջ�
		SName new_rc_name;
		if( getReceiver( &rc, &new_rc_name, 0.1 ) == false )
		{
			sendLedSignal(LEDSignal_Err1);
			os_delay(1.0);
			return MR_Err;
		}
		if( new_rc_name != rc_name )
		{
			sendLedSignal(LEDSignal_Err1);
			os_delay(1.0);
			return MR_Err;
		}
		
		//���ͨ���Ƿ�ά�ֲ���
		for( uint8_t i = 0; i < 16; ++i )
		{
			if( rc.raw_data[i] > max_rc[i] )
				max_rc[i] = rc.raw_data[i];
			else if( rc.raw_data[i] < min_rc[i] )
				min_rc[i] = rc.raw_data[i];
			
			if( max_rc[i] - min_rc[i] > 1.0f )
			{
				//ͨ���ڱ仯
				//ͨ����һ�α仯��ʼУ׼
				Calibration_Started = true;
				setLedManualCtrl( 0, 0, 0, false, 0 );
				reset_constant_check;
				continue;
			}	
			avg_rc[i] += rc.raw_data[i];
		}
		++Calibration_Stage2;
		
		if( Calibration_Started == false )
		{
			for( unsigned char j = 0; j < 16; ++j )
			{
				avg_rc[j] = rc.raw_data[j];
				Calibration_Stage2 = 1;
			}
			setLedManualCtrl( 100, 0, 0, false, 0 );
			continue;
		}
		
		switch( Calibration_Stage )
		{
			
			case 0:	//��¼�е�
			{
				setLedManualCtrl( 0, 0, Calibration_Stage2 * 100 / avg_time, false, 0 );					
				if( Calibration_Stage2 >= avg_time )
				{
					sendLedSignal(LEDSignal_Continue1);
					Calibration_Stage = 1;
					Calibration_Stage2 = 0;
					for( uint8_t i = 0; i < 16; ++i )
						RC_Calibration_Point[ 1 ][ i ] = avg_rc[i] * ( 1.0f / avg_time );
					for( unsigned char i = 0 ; i < 8 ; ++i )
						RC_Calibration_Reflection[i] = 255;
					reset_constant_check;
				}					
				break;
			}	//case 0
			
			case 1:	//��¼һ����ͨ��Сֵ
			case 2:
			case 3:
			case 4:
			{
				setLedManualCtrl( 0, 0, Calibration_Stage2 * 100 / avg_time, false, 0 );
				if( Calibration_Stage2 >= avg_time )
				{
					//�Ҹı��˵�ͨ��
					uint8_t channel_change_count = 0;
					uint8_t channel_index;
					for( unsigned char i = 0; i < 16; ++i )
					{
						if( fabsf( avg_rc[i] * ( 1.0f / avg_time ) - RC_Calibration_Point[ 1 ][ i ] ) > 15.0f )
						{
							channel_index = i;
							++channel_change_count;
						}
					}
					
					//����ı��ͨ����ֹһ��
					//���߸�ͨ���Ѿ�У׼
					//���¿�ʼ
					if( (channel_change_count != 1) || (RC_Channel_Calibrated[channel_index] == true) )
					{
						reset_constant_check;
						continue;
					}
					else
					{
						RC_Calibration_Point[ 0 ][ channel_index ] = avg_rc[channel_index] * ( 1.0f / avg_time );
						RC_Calibration_Reflection[ Calibration_Stage - 1 ] = channel_index;
						RC_Channel_Calibrated[channel_index] = true;
						
						sendLedSignal(LEDSignal_Continue1);
						reset_constant_check;
						++Calibration_Stage;
					}
				}
				break;			
			}	//case 1 2 3 4
			
			case 5:	//��¼һ����ͨ���ֵ
			case 6:
			case 7:
			case 8:
			{
				setLedManualCtrl( 0, 0, Calibration_Stage2 * 100 / avg_time, false, 0 );
				if( Calibration_Stage2 >= avg_time )
				{
					//Ѱ�Ҹı��ͨ��
					unsigned char channel_change_count = 0;
					unsigned char channel_index;
					for( unsigned char i = 0 ; i < 16 ; ++i )
					{
						if( fabsf( avg_rc[i] * ( 1.0f / avg_time ) - RC_Calibration_Point[ 1 ][ i ] ) > 15.0f )
						{
							channel_index = i;
							++channel_change_count;
						}
					}
					
					//����ı��ͨ����ֹһ��
					//���߸�ͨ���Ѿ�У׼
					//���¿�ʼ
					if( channel_change_count != 1 )
					{
						reset_constant_check;
						continue;
					}
					else
					{
						//���ͨ����У׼��Сֵʱ�Ĳ�һ��
						//���¿�ʼ
						if( channel_index != RC_Calibration_Reflection[ Calibration_Stage - 5 ] )
						{
							reset_constant_check;
							continue;
						}
						
						RC_Calibration_Point[ 2 ][ channel_index ] = avg_rc[channel_index] * ( 1.0f / avg_time );
						
						sendLedSignal(LEDSignal_Continue1);
						reset_constant_check;
						++Calibration_Stage;
					}
				}
				break;			
			}	//case 5 6 7 8
			
			case 9:	//button1
			case 10:	//button2
			{
				setLedManualCtrl( 0, 0, Calibration_Stage2 * 100 / avg_time, false, 0 );
				if( Calibration_Stage2 >= avg_time )
				{
					//Ѱ�Ҹı��ͨ��
					unsigned char channel_change_count = 0;
					unsigned char channel_index;
					for( unsigned char i = 0 ; i < 16 ; ++i )
					{
						if( fabsf( avg_rc[i] * ( 1.0f / avg_time ) - RC_Calibration_Point[ 1 ][ i ] ) > 15.0f )
						{
							channel_index = i;
							++channel_change_count;
						}
					}
					
					//����ı��ͨ����ֹһ��
					//���߸�ͨ���Ѿ�У׼
					//���¿�ʼ
					if( (channel_change_count != 1) || (RC_Channel_Calibrated[channel_index] == true) )
					{
						reset_constant_check;
						continue;
					}
					
					RC_Calibration_Reflection[ Calibration_Stage - 9 + 4 ] = channel_index;
					RC_Calibration_Point[ 0 ][channel_index] = avg_rc[channel_index] * ( 1.0f / avg_time );
					RC_Channel_Calibrated[channel_index] = true;
					
					sendLedSignal(LEDSignal_Continue1);
					reset_constant_check;
					++Calibration_Stage;
				}
				break;
			}
			
			case 11:	//aux3
			case 12:	//aux4
			{
				setLedManualCtrl( 0, 0, Calibration_Stage2 * 100 / avg_time, false, 0 );
				if( Calibration_Stage2 >= avg_time )
				{
					//Ѱ�Ҹı��ͨ��
					uint8_t channel_change_count = 0;
					uint8_t channel_index;
					for( unsigned char i = 0 ; i < 16 ; ++i )
					{
						if( fabsf( avg_rc[i] * ( 1.0f / avg_time ) - RC_Calibration_Point[ 1 ][ i ] ) > 15.0f )
						{
							channel_index = i;
							++channel_change_count;
						}
					}
					
					//����ı��ͨ����ֹһ��
					//���¿�ʼ
					if( (channel_change_count != 1) )
					{
						reset_constant_check;
						continue;
					}
					
					//�ж������������У׼
					if( channel_index == RC_Calibration_Reflection[ 0 ] )
					{
						bool thr_low;
						thr_low = fabsf( avg_rc[channel_index] * ( 1.0f / avg_time ) -  RC_Calibration_Point[ 0 ][ channel_index ] ) < 5.0f;
						
						if( thr_low )
						{
							sendLedSignal(LEDSignal_Continue1);
							reset_constant_check;
							goto CalcRc;
						}
					}
					
					//�����ͨ���Ѿ�У׼
					//���¿�ʼ
					if( (RC_Channel_Calibrated[channel_index] == true) )
					{
						reset_constant_check;
						continue;
					}
					
					RC_Calibration_Reflection[ Calibration_Stage - 9 + 4 ] = channel_index;
					RC_Calibration_Point[ 0 ][channel_index] = avg_rc[channel_index] * ( 1.0f / avg_time );
					RC_Channel_Calibrated[channel_index] = true;
					
					sendLedSignal(LEDSignal_Continue1);
					reset_constant_check;
					++Calibration_Stage;
					if( Calibration_Stage == 13 )
						goto CalcRc;
				}
				break;
			}
		}
	}
	
CalcRc:
	struct
	{
		uint8_t reflections[8];
		float minRcs[8];
		float scales[8];
	}__PACKED RCConfig;
	for( uint8_t i = 0; i < 4; ++i )
	{
		uint8_t reflection_ch = RC_Calibration_Reflection[ i ];
		RCConfig.reflections[i] = reflection_ch;
		
		//failed if mid point not correct
		float m_side = RC_Calibration_Point[ 2 ][ reflection_ch ] - RC_Calibration_Point[ 1 ][ reflection_ch ];
		float l_side = RC_Calibration_Point[ 1 ][ reflection_ch ] - RC_Calibration_Point[ 0 ][ reflection_ch ];
		if( fabsf( m_side - l_side ) > 10.0f )
		{
			sendLedSignal(LEDSignal_Err1);
			os_delay(1.0);
			return MR_Err;
		}
		float side = ( fabsf(m_side) > fabsf(l_side) ) ? m_side : l_side;
		RCConfig.minRcs[i] = RC_Calibration_Point[ 1 ][ reflection_ch ] - side;
		RCConfig.scales[i] = 50.0f / side;
	}
	
	for( uint8_t i = 4; i < 8; ++i )
	{
		if( RC_Calibration_Reflection[ i ] < 16 )
		{
			RCConfig.reflections[i] = RC_Calibration_Reflection[i];
			uint8_t reflection_ch = RC_Calibration_Reflection[ i ];			
			RCConfig.minRcs[i] = RC_Calibration_Point[ 0 ][ reflection_ch ];
			RCConfig.scales[i] = 100.0f / ( RC_Calibration_Point[ 1 ][ reflection_ch ] - RC_Calibration_Point[ 0 ][ reflection_ch ] );
		}
		else
			RCConfig.reflections[i] = 255;
	}
	//���²���
	PR_RESULT res = UpdateParamGroup( SName("RC_")+rc_name, (uint64_t*)&RCConfig, 0, 9 );
	sendLedSignal(LEDSignal_Success1);	

	return MR_OK;
}