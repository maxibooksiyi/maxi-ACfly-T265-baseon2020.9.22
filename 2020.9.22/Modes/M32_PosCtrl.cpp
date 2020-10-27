#include "M32_PosCtrl.hpp"
#include "vector3.hpp"
#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "AC_Math.hpp"
#include "Receiver.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "StorageSystem.hpp"
#include "Missions.hpp"
#include "NavCmdProcess.hpp"
#include "InFlightCmdProcess.hpp"
#include "drv_PWMOut.hpp"
#include "AuxFuncs.hpp"

extern uint16_t mav_mode;
extern uint16_t mav_main_mode;
extern uint16_t mav_sub_mode;

M32_PosCtrl::M32_PosCtrl():Mode_Base( "PosCtrl", 32 )
{
	
}

ModeResult M32_PosCtrl::main_func( void* param1, uint32_t param2 )
{
	mav_mode = MAV_MODE_STABILIZE_ARMED;
	mav_main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
	mav_sub_mode = 0;
	
	double freq = 50;
	setLedMode(LEDMode_Flying1);
	Altitude_Control_Enable();
	Position_Control_Enable();
	uint16_t exit_mode_counter_rs = 0;
	uint16_t exit_mode_counter = 0;
	uint16_t exit_mode_Gcounter = 0;
	
	//����ģʽ
	uint8_t MissionMode = 0;	//0-�ֶ� 1-����	2-Guided
	bool mode_switched = true;
	#define change_MissionMode(x) {MissionMode=x; mode_switched = true;}
	uint16_t current_mission_ind;
	MissionInf current_mission_inf;
	double lastMissionButtonValue = -1;
	double lastRTLButtonValue = -1;
	bool last_SfRTL = false;
	//����״̬��
	NavCmdInf navInf;
	init_NavCmdInf(&navInf);
	//ָ��ִ���Ƿ���ɣ��ֶ�ģʽ��
	bool ManualModeNavCmdInprogress = false;
	ModeMsg ManualModeNavCmd;
	TIME last_ManualModeNavCmdTime;
	
	//�Ѻ�������Ϊ�׸�
	setCurrentMission(0);
	
	//�Ƿ���inFlightCmd
	#define DealInFlightCmd 0
	//��һ��������������м��InFlightCmd������
	#define MissionInc 1
	//�������յ�ǰ���뱶��
	#define CamTriggDistMult 2
	
	/*��ʼ����*/
	
		//��ʼ��������ģʽ
		px4_custom_mode t_mav_mode;
		t_mav_mode.data = param2;
		if( t_mav_mode.main_mode==PX4_CUSTOM_MAIN_MODE_AUTO && t_mav_mode.sub_mode==PX4_CUSTOM_SUB_MODE_AUTO_MISSION )
			change_MissionMode(1)
			
		//
	/*��ʼ����*/
	
	//��ʼ�������������
	CamTriggDist = 0;
	double camTriggDist = CamTriggDist;
		
	//��ʼ��Aux����
	init_process_AuxFuncs();
		
	while(1)
	{
		os_delay(0.02);
		
		//��ȡ���ջ�
		Receiver rc;
		getReceiver( &rc, 0, 0.02 );
		
		//����Auxͨ��
		process_AuxFuncs(&rc);
		
		//��ȡ��Ϣ
		bool msg_available;
		ModeMsg msg;
		msg_available = ModeReceiveMsg( &msg, 0 );
		
		//���ջ�����
		//���������ֶ�ģʽ�л�
		if( rc.available && get_Position_MSStatus()==MS_Ready )
		{
			//ʧ�ػָ����������
			bool sticks_in_neutral = 
				in_symmetry_range_mid( rc.data[0] , 50 , 5 ) &&
				in_symmetry_range_mid( rc.data[1] , 50 , 5 ) &&
				in_symmetry_range_mid( rc.data[2] , 50 , 5 ) &&
				in_symmetry_range_mid( rc.data[3] , 50 , 5 );
			if( !sticks_in_neutral )
				last_SfRTL = false;
			
			//�жϷ���
			if( rc.available_channels >=7 )
			{	
				double RTLButtonValue = rc.data[6];
				if( lastRTLButtonValue < 0 )
					lastRTLButtonValue = RTLButtonValue;
				if( fabs( RTLButtonValue - lastRTLButtonValue ) > 25 || last_SfRTL )
				{	//���밲ȫģʽִ�з���
					lastMissionButtonValue = -1;
					enter_MSafe(true);
					/*�ж��˳�ģʽ*/
						bool inFlight;
						get_is_inFlight(&inFlight);
						if( inFlight==false )
						{
							Attitude_Control_Disable();
							return MR_OK;
						}
					/*�ж��˳�ģʽ*/
					continue;
				}
			}
			else
				lastRTLButtonValue = -1;
			
			//�ж�����/�ֶ�ģʽ
			double MissionButtonValue = rc.data[5];
			if( lastMissionButtonValue < 0 )
				lastMissionButtonValue = MissionButtonValue;
			else if( fabs( MissionButtonValue - lastMissionButtonValue ) > 25 )
			{
				if( MissionMode > 0 )
					change_MissionMode(0)
				else
					change_MissionMode(1)
				init_NavCmdInf(&navInf);
				lastMissionButtonValue = MissionButtonValue;
			}
			if( msg_available && msg.cmd==176 )
			{	//do set mode
				if( msg.params[0] != 0 )
				{	//mavlink����ģʽ
					px4_custom_mode t_mav_mode;
					t_mav_mode.data = msg.params[1];
					if( t_mav_mode.main_mode==PX4_CUSTOM_MAIN_MODE_AUTO && t_mav_mode.sub_mode==PX4_CUSTOM_SUB_MODE_AUTO_MISSION )
					{
						if( MissionMode != 1 )
						{
							change_MissionMode(1);
							init_NavCmdInf(&navInf);
							lastMissionButtonValue = MissionButtonValue;
						}
					}
				}
			}
		}
		else
			lastMissionButtonValue = -1;
		
		if( MissionMode==1 )
		{	//����ģʽ
			mav_mode = MAV_MODE_AUTO_ARMED;
			Position_Control_Enable();
			bool pos_ena;
			is_Position_Control_Enabled(&pos_ena);
			if( pos_ena == false )
			{	//λ�ÿ������޷��򿪷����ֶ�ģʽ
				change_MissionMode(0)
				goto Manual_Mode;
			}
			
			if( mode_switched )
			{	//���л�������ģʽ
				//����ɲ���ȴ�
				
				if( rc.available )
				{
					bool sticks_in_neutral = 
						in_symmetry_range_mid( rc.data[0] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[1] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[2] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[3] , 50 , 5 );
					if( !sticks_in_neutral )
					{	//ҡ�˲����м䷵���ֶ�ģʽ
						init_NavCmdInf(&navInf);
						change_MissionMode(0)
						goto Manual_Mode;
					}
				}
				
				Position_Control_set_XYLock();
				Position_Control_set_ZLock();
				
				//�ȴ�ɲ�����
				Position_ControlMode alt_mode, pos_mode;
				get_Altitude_ControlMode(&alt_mode);
				get_Position_ControlMode(&pos_mode);
				if( alt_mode==Position_ControlMode_Position && pos_mode==Position_ControlMode_Position )
				{	//ɲ�����
					++navInf.counter2;
					//�ȴ�1���ٽ����������
					if( navInf.counter2 >= 1*freq )
					{
						mode_switched = false;
						if( ReadCurrentMission(&current_mission_inf, &current_mission_ind) )
						{	//������һ����ɹ�
							//��ʼ��������Ϣ
							init_NavCmdInf(&navInf);
							//���������������
							camTriggDist = CamTriggDist;
						}
						else
						{	//��ȡ����������Ϣ
							//�����ŰѺ�������Ϊ�׸�
							setCurrentMission(0);
							if( ReadCurrentMission(&current_mission_inf, &current_mission_ind) )
							{	//������һ����ɹ�
								//��ʼ��������Ϣ
								init_NavCmdInf(&navInf);
								//���������������
								camTriggDist = CamTriggDist;
							}
							else
							{	//�޺�����Ϣ�����ֶ�ģʽ
								change_MissionMode(0)
								goto Manual_Mode;
							}
						}
					}
				}
				else
					navInf.counter2 = 0;
			}
			else
			{	//�������
				if( rc.available )
				{
					bool sticks_in_neutral = 
						in_symmetry_range_mid( rc.data[0] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[1] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[2] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[3] , 50 , 5 );
					if( !sticks_in_neutral )
					{
						init_NavCmdInf(&navInf);
						change_MissionMode(0)
						goto Manual_Mode;
					}
				}
				
				//�趨mavlinkģʽ
				mav_mode = MAV_MODE_AUTO_ARMED;
				mav_main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
				mav_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
				
				int16_t res = -100;
				if( Process_InflightCmd( current_mission_inf.cmd, current_mission_inf.params ) == false )
				{
					res = Process_NavCmd(
						current_mission_inf.cmd,
						freq, 
						current_mission_inf.frame,
						current_mission_inf.params,
						&navInf
					);
				}
				
				if( NavCmdRs_SuccessOrFault(res) )
				{	//�����ִ�����
					
					//���߽�������
					if( NavCmdRs_Success(res) )
					{
						if( camTriggDist > 0 )
						{
							os_delay(0.5);
							InflightCmd_CamTakePhoto();
						}
					}
					
					//���Զ�ִ�з����ֶ�ģʽ
					if( current_mission_inf.autocontinue == 0 )
					{
						change_MissionMode(0)
					}
					
					if( res < 0 )
					{	//�л�����һģʽ
						MissionInf chk_inf;
						uint16_t chk_ind;
						if( ReadCurrentMission(&chk_inf, &chk_ind) )
						{	//��ȡ��ǰ������Ϣ�Ƚ�						
							if( chk_ind==current_mission_ind && memcmp( &chk_inf, &current_mission_inf, sizeof(MissionInf) ) == 0 )
							{	//�����ͬ���л���һ������
								if( setCurrentMission( getCurrentMissionInd() + navInf.usr_temp[MissionInc] + 1 ) == false )
								{	//�޺�����Ϣ�����ֶ�ģʽ
									setCurrentMission( 0 );
									change_MissionMode(0)
									if( current_mission_inf.cmd==MAV_CMD_NAV_RETURN_TO_LAUNCH || current_mission_inf.cmd==MAV_CMD_NAV_LAND )
									{
										bool inFlight;
										get_is_inFlight(&inFlight);
										if( inFlight==false )
										{	//������ɼ���
											Attitude_Control_Disable();
											return MR_OK;
										}
									}
								}
								else
								{
									if( ReadCurrentMission(&current_mission_inf, &current_mission_ind) )
									{	//������һ����ɹ�
										//��ʼ��������Ϣ
										init_NavCmdInf(&navInf);
										//���������������
										camTriggDist = CamTriggDist;
									}
									else
									{	//�޺�����Ϣ�����ֶ�ģʽ
										setCurrentMission( 0 );
										change_MissionMode(0)
										if( current_mission_inf.cmd==MAV_CMD_NAV_RETURN_TO_LAUNCH || current_mission_inf.cmd==MAV_CMD_NAV_LAND )
										{
											bool inFlight;
											get_is_inFlight(&inFlight);
											if( inFlight==false )
											{	//������ɼ���
												Attitude_Control_Disable();
												return MR_OK;
											}
										}
									}
								}
							}
							else
							{	//������Ϣ����ͬ���л���һ����
								//ʹ���»�ȡ��������Ϣ
								current_mission_inf = chk_inf;
								//��ʼ��������Ϣ
								init_NavCmdInf(&navInf);
							}
						}
						else
						{	//�޺�����Ϣ�����ֶ�ģʽ
							setCurrentMission( 0 );
							change_MissionMode(0)
							if( current_mission_inf.cmd==MAV_CMD_NAV_RETURN_TO_LAUNCH || current_mission_inf.cmd==MAV_CMD_NAV_LAND )
							{	//������ɼ���
								bool inFlight;
								get_is_inFlight(&inFlight);
								if( inFlight==false )
								{
									Attitude_Control_Disable();
									return MR_OK;
								}
							}
						}
					}
					else
					{	//�л���ָ��ģʽ
						if( setCurrentMission( res ) == false )
						{	//�л�ʧ�ܷ����ֶ�ģʽ
							setCurrentMission( 0 );
							change_MissionMode(0)
						}
					}
				}
				else
				{	//����ִ����			
					if( NavCmdRs_InProgress_CanExInFlightCmd(res) )
					{	//��ִ��InFlightCmd
						
						//��������
						if( camTriggDist > 0 )
						{
							double flightDistance = 0;
							Position_Control_get_LineFlightDistance(&flightDistance);
							int mult = (int)(flightDistance / camTriggDist) + 1;
							if( mult > navInf.usr_temp[CamTriggDistMult] )
							{
								InflightCmd_CamTakePhoto();
								navInf.usr_temp[CamTriggDistMult] = mult;
							}
						}
						
						if( navInf.usr_temp[DealInFlightCmd] == 0 )
						{	//��δִ��inFlightCmd
							//ִ������inFlightCmd
							MissionInf inFlightMs_inf;
							while(1)
							{
								if( ReadMission( getCurrentMissionInd() + navInf.usr_temp[MissionInc] + 1, &inFlightMs_inf ) )
								{
									if( Process_InflightCmd( inFlightMs_inf.cmd, inFlightMs_inf.params ) )
										navInf.usr_temp[MissionInc] += 1;
									else
										break;
								}
								else
									break;
							}
						}
						navInf.usr_temp[DealInFlightCmd] = 1;
					}
				}
			}
		}
		else
		{	//�ֶ�����ģʽ���������߶�����ƣ�
			Manual_Mode:
						
			if(mode_switched)
			{	//�ս����ֶ�ģʽ��ʼ������
				mode_switched = false;
				ManualModeNavCmdInprogress = false;
				last_ManualModeNavCmdTime.set_invalid();
				Attitude_Control_set_YawLock();
			}
			
			mav_mode = MAV_MODE_STABILIZE_ARMED;
			if( rc.available )
			{				
				/*�ж��˳�ģʽ*/
					//��ȡ����״̬
					bool inFlight;
					get_is_inFlight(&inFlight);
					if( rc.data[0] > 30 )
					{
						exit_mode_counter_rs = 400;
						if( exit_mode_counter < exit_mode_counter_rs )
							exit_mode_counter = exit_mode_counter_rs;
					}
					//������Զ�����
					if( inFlight==false && rc.data[0]<30 )
					{
						if( ++exit_mode_counter >= 500 )
						{
							Attitude_Control_Disable();
							return MR_OK;
						}
					}
					else
						exit_mode_counter = exit_mode_counter_rs;
					//���Ƽ���
					if( inFlight==false && (rc.data[0] < 10 && rc.data[1] < 10 && rc.data[2] < 10 && rc.data[3] > 90) )
					{
						if( ++exit_mode_Gcounter >= 50 )
						{
							Attitude_Control_Disable();
							return MR_OK;
						}
					}
					else
						exit_mode_Gcounter = 0;
				/*�ж��˳�ģʽ*/
				
				//�л����߶���
				if( rc.data[4] > 60 )
					Position_Control_Enable();
				else if( rc.data[4] < 40 )
					Position_Control_Disable();
				bool pos_ena;
				is_Position_Control_Enabled(&pos_ena);
								
				//�ж�ҡ���Ƿ����
				bool sticks_in_neutral = 
					in_symmetry_range_mid( rc.data[0] , 50 , 5 ) &&
					in_symmetry_range_mid( rc.data[1] , 50 , 5 ) &&
					in_symmetry_range_mid( rc.data[2] , 50 , 5 ) &&
					in_symmetry_range_mid( rc.data[3] , 50 , 5 );
				
				if( sticks_in_neutral && pos_ena )
				{	//ҡ�����м����ڶ���ģʽ������ִ������
					if(msg_available)
					{
						if( Process_InflightCmd( msg.cmd, msg.params ) == false )
						if( check_NavCmd( msg.cmd, freq, default_NavCmd_frame, msg.params ) )
						{	//ָ��ɱ�ִ��
							init_NavCmdInf(&navInf);
							ManualModeNavCmdInprogress = true;
							ManualModeNavCmd = msg;
						}
					}
					
					if( ManualModeNavCmdInprogress )
					{	//��Ҫִ��NavCmd
						int16_t res = -100;
						res = Process_NavCmd( ManualModeNavCmd.cmd, freq, default_NavCmd_frame, ManualModeNavCmd.params, &navInf );
						if( NavCmdRs_SuccessOrFault(res) )
						{	//NavCmdִ�����
							ManualModeNavCmdInprogress = false;
						}
						last_ManualModeNavCmdTime = TIME::now();
					}
					else
					{
						Position_Control_set_ZLock();
						Position_Control_set_XYLock();
						Attitude_Control_set_YawLock();
					}
				}
				else
				{	//ҡ�˲����м��ֶ�����
											
					//���Ÿ˿��ƴ�ֱ�ٶ�
					if( in_symmetry_range_mid( rc.data[0] , 50 , 5 ) )
						Position_Control_set_ZLock();
					else
					{
						double thr_stick = remove_deadband( rc.data[0] - 50.0 , 5.0 );
						if( thr_stick > 0 )
							thr_stick *= get_maxVelUp() / 50;
						else
							thr_stick *= get_maxVelDown() / 50;
						Position_Control_set_TargetVelocityZ(thr_stick);
					}					
					
					if( pos_ena )
					{
						//�趨mavlinkģʽ
						mav_mode = MAV_MODE_STABILIZE_ARMED;
						mav_main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
						mav_sub_mode = 0;
						
						//��������˿�ˮƽ�ٶ�
						if( in_symmetry_range_mid( rc.data[3] , 50 , 5 ) && in_symmetry_range_mid( rc.data[2] , 50 , 5 ) )
							Position_Control_set_XYLock();
						else
						{
							double RPCtrlScale = atan2( get_maxAccXY() / 50.0, GravityAcc );
							double XYCtrlScale = get_maxVelXY() / 50.0;
							double roll_sitck_d = remove_deadband( rc.data[3] - 50.0, 5.0 );
							double pitch_sitck_d = remove_deadband( rc.data[2] - 50.0, 5.0 );
							Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( \
								pitch_sitck_d * XYCtrlScale ,\
								-roll_sitck_d * XYCtrlScale , \
								( roll_sitck_d == 0 ) ? 0.3 : fabs( roll_sitck_d  )*RPCtrlScale, \
								( pitch_sitck_d == 0 ) ? 0.3 : fabs( pitch_sitck_d )*RPCtrlScale \
							);
						}
					}
					else
					{
						//�趨mavlinkģʽ
						mav_mode = MAV_MODE_STABILIZE_ARMED;
						mav_main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;
						mav_sub_mode = 0;
						
						//���������Ŷ�
						vector3<double> WindDisturbance;
						get_WindDisturbance( &WindDisturbance );
						Quaternion attitude;
						get_Attitude_quat(&attitude);
						double yaw = attitude.getYaw();		
						double sin_Yaw, cos_Yaw;
						fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
						double WindDisturbance_Bodyheading_x = ENU2BodyHeading_x( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
						double WindDisturbance_Bodyheading_y = ENU2BodyHeading_y( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
						
						//��������˿ظ������
						double RPCtrlScale = degree2rad( get_maxLean() / 50.0 );
		//				Attitude_Control_set_Target_RollPitch( 
		//					( rc.data[3] - 50 )*RPCtrlScale - atan2(-WindDisturbance_Bodyheading_y , GravityAcc ),
		//					( rc.data[2] - 50 )*RPCtrlScale - atan2( WindDisturbance_Bodyheading_x , GravityAcc ) 
		//				);				
						Attitude_Control_set_Target_RollPitch( 
							( rc.data[3] - 50 )*RPCtrlScale,
							( rc.data[2] - 50 )*RPCtrlScale
						);
					}
					
					//ƫ�������м���ƫ��
					//�����м����ƫ���ٶ�
					double YCtrlScale = degree2rad( get_maxYawSpeed() / 50.0 );
					if( in_symmetry_range_mid( rc.data[1] , 50 , 5 ) )
						Attitude_Control_set_YawLock();
					else
						Attitude_Control_set_Target_YawRate( ( 50 - rc.data[1] )*YCtrlScale );						
				}
			}
			else
			{
				last_SfRTL = true;
				//��ң���źŽ��밲ȫģʽ
				enter_MSafe();
				/*�ж��˳�ģʽ*/
					bool inFlight;
					get_is_inFlight(&inFlight);
					if( inFlight==false )
					{
						Attitude_Control_Disable();
						return MR_OK;
					}
				/*�ж��˳�ģʽ*/
				
			}
		}
	}
	return MR_OK;
}




