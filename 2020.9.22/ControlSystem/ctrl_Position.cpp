#include "ctrl_Attitude.hpp"
#include "ControlSystem.hpp"
#include "ctrl_Main.hpp"
#include "Parameters.hpp"
#include "MeasurementSystem.hpp"
#include "Sensors.hpp"
#include "TD4.hpp"
#include "ESO_AngularRate.hpp"
#include "ESO_h.hpp"
#include "Filters_LP.hpp"
#include "drv_PWMOut.hpp"
#include "smooth_kp.hpp"
#include "TD3_3D.hpp"
#include "vector2.hpp"
#include "Commulink.hpp"

#include "StorageSystem.hpp"

/*����*/
	//���Ʋ���
	struct PosCtrlCfg
	{
		//Ĭ��XY�����ٶ�
		float AutoVXY[2];
		//Ĭ��Z���Ϻ����ٶ�
		float AutoVZUp[2];
		//Ĭ��Z���º����ٶ�
		float AutoVZDown[2];
		//Ĭ��XYZ�����ٶ�
		float AutoVXYZ[2];
		
		//�߶�ǰ���˲���
		float Z_TD4P1[2];
		float Z_TD4P2[2];
		float Z_TD4P3[2];
		float Z_TD4P4[2];
		
		//λ�÷�������
		float P1[2];
		//�ٶȷ�������
		float P2[2];
		//���ٶȷ�������
		float P3[2];
		//ˮƽ�ٶȿ����ٶȷ�������
		float P2_VelXY[2];
		//ˮƽ�ٶȿ��Ƽ��ٶȷ�������
		float P3_VelXY[2];
		
		//���ˮƽ�ٶ�
		float maxVelXY[2];
		//���ˮƽ���ٶ�
		float maxAccXY[2];
		//���ˮƽ�Ӽ��ٶ�
		float maxJerkXY[2];
		//�Զ�ģʽ���ˮƽ�ٶ�
		float maxAutoVelXY[2];
		//�Զ�ģʽ���ˮƽ���ٶ�
		float maxAutoAccXY[2];
		//�Զ�ģʽ���ˮƽ�Ӽ��ٶ�
		float maxAutoJerkXY[2];
		
		//��������ٶ�
		float maxVelUp[2];
		//��������ٶ�
		float maxVelDown[2];
		//������ϼ��ٶ�
		float maxAccUp[2];
		//������¼��ٶ�
		float maxAccDown[2];
		//������ϼӼ��ٶ�
		float maxJerkUp[2];
		//������¼Ӽ��ٶ�
		float maxJerkDown[2];
		//�Զ�ģʽ��������ٶ�
		float maxAutoVelUp[2];
		//�Զ�ģʽ��������ٶ�
		float maxAutoVelDown[2];
		//�Զ�ģʽ������ϼ��ٶ�
		float maxAutoAccUp[2];
		//�Զ�ģʽ������¼��ٶ�
		float maxAutoAccDown[2];
		//�Զ�ģʽ������ϼӼ��ٶ�
		float maxAutoJerkUp[2];
		//�Զ�ģʽ������¼Ӽ��ٶ�
		float maxAutoJerkDown[2];
	}__PACKED;
	static PosCtrlCfg cfg;
/*����*/	

/*�����ӿ�*/
	float get_maxVelUp()
	{
		return cfg.maxVelUp[0];
	}
	float get_maxVelDown()
	{
		return cfg.maxVelDown[0];
	}
	float get_maxVelXY()
	{
		return cfg.maxVelXY[0];
	}
	float get_maxAccXY()
	{
		return cfg.maxAccXY[0];
	}
/*�����ӿ�*/
	
/*���ƽӿ�*/
	static bool Altitude_Control_Enabled = false;
	static bool Position_Control_Enabled = false;
	
	//λ�ÿ���ģʽ
	static Position_ControlMode Altitude_ControlMode = Position_ControlMode_Position;
	static Position_ControlMode HorizontalPosition_ControlMode = Position_ControlMode_Position;
	
	//����TD4�˲���
	static TD4_SL Target_tracker[3];
	//�����ٶȵ�ͨ�˲���
	static Filter_Butter4_LP TargetVelocityFilter[3];
	
	static vector3<double> target_position;
	static vector3<double> target_velocity;
	static double VelCtrlMaxRoll = -1 , VelCtrlMaxPitch = -1;
	
	/*�߶�*/
		bool is_Altitude_Control_Enabled( bool* ena, double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				*ena = Altitude_Control_Enabled;
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Altitude_Control_Enable( double TIMEOUT )
		{
			if( get_Altitude_MSStatus() != MS_Ready )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == true )
				{	//�������Ѵ�
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//�����û�����
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				Attitude_Control_Enable();
				Altitude_ControlMode = Position_ControlMode_Locking;
				Altitude_Control_Enabled = true;
				
				//���¿���ʱ��
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				//������
				if( ReadParamGroup( "PosCtrl", (uint64_t*)&cfg, 0, TIMEOUT ) != PR_OK )
				{
					UnlockCtrl();
					return false;
				}				
				Position_Control_set_ZAutoSpeed( cfg.AutoVZUp[0], cfg.AutoVZDown[0] );
				Target_tracker[2].P1 = cfg.Z_TD4P1[0];
				Target_tracker[2].P2 = cfg.Z_TD4P2[0];
				Target_tracker[2].P3 = cfg.Z_TD4P3[0];
				Target_tracker[2].P4 = cfg.Z_TD4P4[0];
				
				Position_Control_reset_ZAutoSpeed();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Altitude_Control_Disable( double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( ForceMSafeCtrl && !isMSafe )
				{	//�����û�����
					UnlockCtrl();
					return false;
				}
				Position_Control_Disable();
				Altitude_ControlMode = Position_ControlMode_Null;
				Altitude_Control_Enabled = false;					
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		bool get_Altitude_ControlMode( Position_ControlMode* mode, double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && (Is_2DAutoMode(Altitude_ControlMode) || Is_3DAutoMode(Altitude_ControlMode)) )
				{	//�û����Ʒ�����Ϊ�Զ�ģʽʱ���¿���ʱ��
					last_ZCtrlTime = TIME::now();
				}
				*mode = Altitude_ControlMode;
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		//�趨Z�Զ������ٶ�
		static double AutoVelZUp = 200;
		static double AutoVelZDown = 200;
		bool Position_Control_get_ZAutoSpeed( double* SpUp, double* SpDown, double TIMEOUT )
		{		
			if( LockCtrl(TIMEOUT) )
			{
				*SpUp = AutoVelZUp;
				*SpDown = AutoVelZDown;
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_reset_ZAutoSpeed( double TIMEOUT )
		{
			vector3<double> pos;
			get_Position(&pos);
			if( LockCtrl(TIMEOUT) )
			{
				AutoVelZUp = cfg.AutoVZUp[0];
				AutoVelZDown = cfg.AutoVZDown[0];
				
				//�����ٶ��޷�
				if( AutoVelZUp > cfg.maxAutoVelUp[0] )
					AutoVelZUp = cfg.maxAutoVelUp[0];

				if( AutoVelZDown > cfg.maxAutoVelDown[0] )
					AutoVelZDown = cfg.maxAutoVelDown[0];
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_ZAutoSpeed( double SpUp, double SpDown, double TIMEOUT )
		{
			if( isnan(SpUp) || isinf(SpUp) ||
					isnan(SpDown) || isinf(SpDown) )
				return false;
			
			vector3<double> pos;
			get_Position(&pos);
			if( LockCtrl(TIMEOUT) )
			{
				if( SpUp > 0 )
					AutoVelZUp = SpUp;
				if( SpDown > 0 )
					AutoVelZDown = SpDown;
				
				//�����ٶ��޷�
				if( AutoVelZUp > cfg.maxAutoVelUp[0] )
					AutoVelZUp = cfg.maxAutoVelUp[0];

				if( AutoVelZDown > cfg.maxAutoVelDown[0] )
					AutoVelZDown = cfg.maxAutoVelDown[0];
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		//�ƶ�ZĿ��λ��
		bool Position_Control_move_TargetPositionZRelative( double posz, double TIMEOUT )
		{
			if( isnan(posz) || isinf(posz) )
				return false;
			//����Ϊλ�û����Զ�ģʽ
			if( Altitude_ControlMode!=Position_ControlMode_Position &&
					Is_2DAutoMode(Altitude_ControlMode)==false &&
					Is_3DAutoMode(Altitude_ControlMode)==false )
				return false;
			
			bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
			if( !isMSafe )
			{	//�û����Ʒ��ʸ��¿���ʱ��
				last_ZCtrlTime = TIME::now();
			}
			target_position.z += posz;
			
			return true;
		}
		
		bool Position_Control_set_TargetPositionZ( double posz, double vel, double TIMEOUT )
		{
			if( isnan(posz) || isinf(posz) )
				return false;
			
			vector3<double> pos;
			get_Position(&pos);
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//�����û�����
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				target_position.z = posz;
				if( Is_2DAutoMode(Altitude_ControlMode)==false && Altitude_ControlMode!=Position_ControlMode_Position )
					Target_tracker[2].x1 = pos.z;
				
				//�趨�����ٶ�
				//�����ٶ��޷�
				if( target_position.z > pos.z )
				{
					if( vel > 10 )
						AutoVelZUp = vel;
					else if( vel < 0 )
						AutoVelZUp = cfg.AutoVZUp[0];
					
					if( AutoVelZUp > cfg.maxAutoVelUp[0] )
						AutoVelZUp = cfg.maxAutoVelUp[0];
				}
				else
				{
					if( vel > 10 )
						AutoVelZDown = vel;
					else if( vel < 0 )
						AutoVelZDown = cfg.AutoVZDown[0];
					
					if( AutoVelZDown > cfg.maxAutoVelDown[0] )
						AutoVelZDown = cfg.maxAutoVelDown[0];
				}
				
				//�л�ģʽ
				Altitude_ControlMode = Position_ControlMode_RouteLine;
			
				//���¿���ʱ��
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_TargetPositionZGlobal( double posz, double vel, double TIMEOUT )
		{
			if( isnan(posz) || isinf(posz) )
				return false;
			
			//��ȡ����ȫ��λ��������Ϣ
			PosSensorHealthInf1 global_inf;
			if( get_OptimalGlobal_Z( &global_inf ) == false )
				return false;
			posz -= global_inf.HOffset;
			return Position_Control_set_TargetPositionZ( posz, vel, TIMEOUT );
		}
		bool Position_Control_set_TargetPositionZRelative( double posz, double vel, double TIMEOUT )
		{
			if( isnan(posz) || isinf(posz) )
				return false;
			vector3<double> pos;
			if( get_Position( &pos, TIMEOUT ) == false )
				return false;
			return Position_Control_set_TargetPositionZ( pos.z + posz, vel, TIMEOUT );
		}
		bool Position_Control_set_TargetPositionZRA( double posz, double vel, double TIMEOUT )
		{
			//��ȡ���λ��Z����
			double homeZ;
			getHomeLocalZ(&homeZ);
			return Position_Control_set_TargetPositionZ( homeZ + posz, vel, TIMEOUT );
		}
		
		bool Position_Control_set_TargetVelocityZ( double velz, double TIMEOUT )
		{
			if( isnan(velz) || isinf(velz) )
				return false;
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{	//������δ��
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//�����û�����
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				target_velocity.z = velz;
				Altitude_ControlMode = Position_ControlMode_Velocity;
			
				//���¿���ʱ��
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_ZLock( double TIMEOUT )
		{
			vector3<double> pos;
			if( get_Position( &pos, TIMEOUT ) == false )
				return false;
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{	//������δ��
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//�����û�����
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				if( Altitude_ControlMode != Position_ControlMode_Position )
					Altitude_ControlMode = Position_ControlMode_Locking;
				Target_tracker[2].x1 = pos.z;
				
				//���¿���ʱ��
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		//��ɵ���ǰ�߶��Ϸ���height�߶�
		static double TakeoffHeight;
		bool Position_Control_Takeoff_HeightRelative( double height, double TIMEOUT )
		{
			if( height < 10 )
					return false;
			bool inFlight;	get_is_inFlight(&inFlight);
			if( inFlight == true )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{	//������δ��
					UnlockCtrl();
					return false;
				}	
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//�����û�����
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				
				Altitude_ControlMode = Position_ControlMode_Takeoff;
				TakeoffHeight = height;
			
				//���¿���ʱ��
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_Takeoff_HeightGlobal( double height, double TIMEOUT )
		{
			//��ȡ����ȫ��λ��������Ϣ
			PosSensorHealthInf1 global_inf;
			if( get_OptimalGlobal_Z( &global_inf ) == false )
				return false;
			height -= global_inf.HOffset;
			height -= global_inf.PositionENU.z;
			return Position_Control_Takeoff_HeightRelative( height, TIMEOUT );
		}
		bool Position_Control_Takeoff_Height( double height, double TIMEOUT )
		{
			vector3<double> pos;
			get_Position(&pos);
			height = height - pos.z;
			return Position_Control_Takeoff_HeightRelative( height, TIMEOUT );
		}
	/*�߶�*/
		
	/*ˮƽλ��*/
		bool is_Position_Control_Enabled( bool* ena, double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				*ena = Position_Control_Enabled;
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_Enable( double TIMEOUT )
		{
			if( get_Position_MSStatus() != MS_Ready )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == true )
				{	//�������Ѵ�
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//�����û�����
					last_XYCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				Altitude_Control_Enable();
				if( Altitude_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				HorizontalPosition_ControlMode = Position_ControlMode_Locking;
				Position_Control_Enabled = true;
				
				//���¿���ʱ��
				if(!isMSafe)
					last_XYCtrlTime = TIME::now();
				
				//������	
				Position_Control_reset_XYAutoSpeed();
				Position_Control_reset_XYZAutoSpeed();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_Disable( double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( ForceMSafeCtrl && !isMSafe )
				{	//�����û�����
					UnlockCtrl();
					return false;
				}
				HorizontalPosition_ControlMode = Position_ControlMode_Null;
				Position_Control_Enabled = false;		
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		bool get_Position_ControlMode( Position_ControlMode* mode, double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && (Is_2DAutoMode(HorizontalPosition_ControlMode) || Is_3DAutoMode(HorizontalPosition_ControlMode)) )
				{	//�û����Ʒ�����Ϊ�Զ�ģʽʱ���¿���ʱ��
					last_XYCtrlTime = TIME::now();
				}
				*mode = HorizontalPosition_ControlMode;
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		bool Position_Control_set_TargetVelocityXY_AngleLimit( double velx, double vely, double maxAngle, double TIMEOUT )
		{
			if( isnan(velx) || isinf(velx) ||
					isnan(vely) || isinf(vely) ||
					isnan(maxAngle) || isinf(maxAngle)
				)
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == false )
				{	//������δ��
					UnlockCtrl();
					return false;
				}	
				
				Quaternion attitude;
				if( get_AirframeY_quat( &attitude, TIMEOUT ) == false )
				{
					UnlockCtrl();
					return false;
				}
				
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//�����û�����
					last_XYCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				
				constrain_vector( velx, vely, (double)cfg.maxVelXY[0] );
				target_velocity.x = velx;
				target_velocity.y = vely;	
				HorizontalPosition_ControlMode = Position_ControlMode_Velocity;
				VelCtrlMaxRoll = maxAngle;
				if( VelCtrlMaxRoll>=0 && VelCtrlMaxRoll<0.05 )
					VelCtrlMaxRoll = 0.05;
				VelCtrlMaxPitch = -1;

				//���¿���ʱ��
				if(!isMSafe)
					last_XYCtrlTime = TIME::now();
			
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( double velx, double vely, double maxRoll, double maxPitch, double TIMEOUT )
		{
			if( isnan(velx) || isinf(velx) ||
					isnan(vely) || isinf(vely) ||
					isnan(maxPitch) || isinf(maxPitch) ||
					isnan(maxRoll) || isinf(maxRoll)
				)
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == false )
				{	//������δ��
					UnlockCtrl();
					return false;
				}	
				
				Quaternion attitude;
				if( get_AirframeY_quat( &attitude, TIMEOUT ) == false )
				{
					UnlockCtrl();
					return false;
				}
				
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//�����û�����
					last_XYCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				
				constrain_vector( velx, vely, (double)cfg.maxVelXY[0] );
				
				double yaw = attitude.getYaw();		
				double sin_Yaw, cos_Yaw;
				fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
				double velx_ENU = BodyHeading2ENU_x( velx , vely , sin_Yaw , cos_Yaw );
				double vely_ENU = BodyHeading2ENU_y( velx , vely , sin_Yaw , cos_Yaw );
				
				target_velocity.x = velx_ENU;
				target_velocity.y = vely_ENU;	
				HorizontalPosition_ControlMode = Position_ControlMode_Velocity;
				VelCtrlMaxRoll = maxRoll;
				if( VelCtrlMaxRoll>=0 && VelCtrlMaxRoll<0.05 )
					VelCtrlMaxRoll = 0.05;
				VelCtrlMaxPitch = maxPitch;
				if( VelCtrlMaxPitch>=0 && VelCtrlMaxPitch < 0.05 )
					VelCtrlMaxPitch = 0.05;		

				//���¿���ʱ��
				if(!isMSafe)
					last_XYCtrlTime = TIME::now();
			
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_XYLock( double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//�����û�����
					last_XYCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				if( HorizontalPosition_ControlMode != Position_ControlMode_Position )
					HorizontalPosition_ControlMode = Position_ControlMode_Locking;
				
				//���¿���ʱ��
				if(!isMSafe)
					last_XYCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		
		static double AutoVelXY = 500;
		static double AutoVelXYZ = 500;
		/*ֱ�߷���*/
			//ֱ�߷���ϵ��
			//A=(x1,y1,z1)=target_positionĿ���
			//B=(x2,y2,z2)=���
			static vector3<double> route_line_A_B;	//(B-A)
			static double route_line_m;	// 1/(B-A)^2
			static double line_track_desired = 0;
			static double line_track_desired_vel = 0;
			static double line_track_desired_maxv = 500;
			static double line_track_desired_maxacc = 300;
			static uint16_t line_track_delay_counter = 0;
			#define reset_line_track_state(maxv,maxacc) { line_track_delay_counter = 0;\
																										line_track_desired = 0;\
																										line_track_desired_vel = 0;\
																										line_track_desired_maxv = maxv;\
																										line_track_desired_maxacc = maxacc; }
		
			bool Position_Control_set_TargetPositionXY( double posx, double posy, double vel, double TIMEOUT )
			{
				if( isnan(posx) || isinf(posx) ||
						isnan(posy) || isinf(posy)	)
					return false;
				if( LockCtrl(TIMEOUT) )
				{
					if( Position_Control_Enabled == false )
					{
						UnlockCtrl();
						return false;
					}
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//�����û�����
						last_XYCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}
					target_position.x = posx;
					target_position.y = posy;
					vector3<double> position;
					get_Position(&position);
					//calculate vector B-A
					route_line_A_B = position - target_position;
					route_line_A_B.z = 0;
					double route_line_A_B_sq = route_line_A_B.get_square();
					if( route_line_A_B_sq > 0.1 )
					{
						//calculate 1/(B-A)^2
						route_line_m = 1.0 / route_line_A_B_sq;
						
						//�����ٶ�
						if( vel > 10 )
							AutoVelXY = vel;
						else if( vel < 0 )
							AutoVelXY = cfg.AutoVXY[0];
						
						//�ٶ��޷�
						if( AutoVelXY > cfg.maxAutoVelXY[0] )
							AutoVelXY = cfg.maxAutoVelXY[0];
						
						reset_line_track_state(AutoVelXY,cfg.maxAutoAccXY[0])
						//�л�ģʽ
						HorizontalPosition_ControlMode = Position_ControlMode_RouteLine;
					}
					else
						HorizontalPosition_ControlMode = Position_ControlMode_Position;
				
					//���¿���ʱ��
					if(!isMSafe)
						last_XYCtrlTime = TIME::now();
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			
			bool Position_Control_set_TargetPositionXYZ( double posx, double posy, double posz, double vel, double TIMEOUT )
			{
				if( isnan(posx) || isinf(posx) ||
						isnan(posy) || isinf(posy) ||
						isnan(posz) || isinf(posz) )
					return false;
				if( LockCtrl(TIMEOUT) )
				{
					if( Position_Control_Enabled == false )
					{
						UnlockCtrl();
						return false;
					}
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//�����û�����
						last_XYCtrlTime = last_ZCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}
					target_position.x = posx;
					target_position.y = posy;
					target_position.z = posz;
					vector3<double> position;
					get_Position(&position);
					//calculate vector B-A
					route_line_A_B = position - target_position;
					double route_line_A_B_sq = route_line_A_B.get_square();
					if( route_line_A_B_sq > 0.1 )
					{
						//calculate 1/(B-A)^2
						route_line_m = 1.0 / route_line_A_B_sq;
						
						//�����ٶ�
						if( vel > 10 )
							AutoVelXYZ = vel;
						else if( vel < 0 )
							AutoVelXYZ = cfg.AutoVXYZ[0];
						
						/*�ٶ��޷�*/
							double xyz_length = safe_sqrt( route_line_A_B.get_square() );
							if( xyz_length > 1 )
							{
								double xy_length = safe_sqrt( route_line_A_B.x*route_line_A_B.x + route_line_A_B.y*route_line_A_B.y );
								
								double inv_xyz = 1.0 / xyz_length;
								double xy_scale = xy_length * inv_xyz;
								double z_scale = fabs(route_line_A_B.z) * inv_xyz;
								
								double scale = 1.0;
								if( AutoVelXYZ*xy_scale > cfg.maxAutoVelXY[0] )
									scale = cfg.maxAutoVelXY[0] / (AutoVelXYZ*xy_scale);
								if( route_line_A_B.z < 0 )
								{	//���Ϸ�
									if( AutoVelXYZ*z_scale > cfg.maxAutoVelUp[0] )
									{
										double new_scale = cfg.maxAutoVelUp[0] / (AutoVelXYZ*z_scale);
										if( new_scale < scale )
											scale = new_scale;
									}
								}
								else
								{	//���·�
									if( AutoVelXYZ*z_scale > cfg.maxAutoVelUp[0] )
									{
										double new_scale = cfg.maxAutoVelDown[0] / (AutoVelXYZ*z_scale);
										if( new_scale < scale )
											scale = new_scale;
									}
								}
								
								AutoVelXYZ *= scale;
							}
							else
							{
								if( AutoVelXYZ > cfg.maxAutoVelXY[0] )
									AutoVelXYZ = cfg.maxAutoVelXY[0];
							}
						/*�ٶ��޷�*/
						
						reset_line_track_state(AutoVelXYZ,cfg.maxAutoAccXY[0])
						//�л�ģʽ
						HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_RouteLine3D;
					}
					else
						HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_Position;
				
					//���¿���ʱ��
					if(!isMSafe)
						last_XYCtrlTime = last_ZCtrlTime = TIME::now();
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			
			bool Position_Control_set_TargetPositionXYRelative( double posx, double posy, double vel, double TIMEOUT )
			{
				if( isnan(posx) || isinf(posx) ||
						isnan(posy) || isinf(posy)	)
					return false;
				vector3<double> position;
				get_Position(&position);
				if( LockCtrl(TIMEOUT) )
				{
					if( Position_Control_Enabled == false )
					{
						UnlockCtrl();
						return false;
					}
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//�����û�����
						last_XYCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}					
					target_position.x = position.x + posx;
					target_position.y = position.y + posy;				
					//calculate vector B-A
					route_line_A_B = position - target_position;
					route_line_A_B.z = 0;
					double route_line_A_B_sq = route_line_A_B.get_square();
					if( route_line_A_B_sq > 0.1 )
					{
						//calculate 1/(B-A)^2
						route_line_m = 1.0 / route_line_A_B_sq;
						
						//�����ٶ�
						if( vel > 10 )
							AutoVelXY = vel;
						else if( vel < 0 )
							AutoVelXY = cfg.AutoVXY[0];
						
						//�ٶ��޷�
						if( AutoVelXY > cfg.maxAutoVelXY[0] )
							AutoVelXY = cfg.maxAutoVelXY[0];
						
						reset_line_track_state(AutoVelXY,cfg.maxAutoAccXY[0])
						//�л�ģʽ
						HorizontalPosition_ControlMode = Position_ControlMode_RouteLine;
					}
					else
						HorizontalPosition_ControlMode = Position_ControlMode_Position;
				
					//���¿���ʱ��
					if(!isMSafe)
						last_XYCtrlTime = TIME::now();
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_set_TargetPositionXYZRelative( double posx, double posy, double posz, double vel, double TIMEOUT )
			{
				if( isnan(posx) || isinf(posx) ||
						isnan(posy) || isinf(posy) || 
						isnan(posz) || isinf(posz) )
					return false;
				vector3<double> position;
				get_Position(&position);
				if( LockCtrl(TIMEOUT) )
				{
					if( Position_Control_Enabled == false )
					{
						UnlockCtrl();
						return false;
					}
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//�����û�����
						last_XYCtrlTime = last_ZCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}				
					target_position.x = position.x + posx;
					target_position.y = position.y + posy;
					target_position.z = position.z + posz;
					//calculate vector B-A
					route_line_A_B = position - target_position;
					double route_line_A_B_sq = route_line_A_B.get_square();
					if( route_line_A_B_sq > 0.1 )
					{
						//calculate 1/(B-A)^2
						route_line_m = 1.0 / route_line_A_B_sq;
						
						//�����ٶ�
						if( vel > 10 )
							AutoVelXYZ = vel;
						else if( vel < 0 )
							AutoVelXYZ = cfg.AutoVXYZ[0];
						
						/*�ٶ��޷�*/
							double xyz_length = safe_sqrt( route_line_A_B.get_square() );
							if( xyz_length > 1 )
							{
								double xy_length = safe_sqrt( route_line_A_B.x*route_line_A_B.x + route_line_A_B.y*route_line_A_B.y );
								
								double inv_xyz = 1.0 / xyz_length;
								double xy_scale = xy_length * inv_xyz;
								double z_scale = fabs(route_line_A_B.z) * inv_xyz;
								
								double scale = 1.0;
								if( AutoVelXYZ*xy_scale > cfg.maxAutoVelXY[0] )
									scale = cfg.maxAutoVelXY[0] / (AutoVelXYZ*xy_scale);
								if( route_line_A_B.z < 0 )
								{	//���Ϸ�
									if( AutoVelXYZ*z_scale > cfg.maxAutoVelUp[0] )
									{
										double new_scale = cfg.maxAutoVelUp[0] / (AutoVelXYZ*z_scale);
										if( new_scale < scale )
											scale = new_scale;
									}
								}
								else
								{	//���·�
									if( AutoVelXYZ*z_scale > cfg.maxAutoVelUp[0] )
									{
										double new_scale = cfg.maxAutoVelDown[0] / (AutoVelXYZ*z_scale);
										if( new_scale < scale )
											scale = new_scale;
									}
								}
								
								AutoVelXYZ *= scale;
							}
							else
							{
								if( AutoVelXYZ > cfg.maxAutoVelXY[0] )
									AutoVelXYZ = cfg.maxAutoVelXY[0];
							}
						/*�ٶ��޷�*/
						
						//�л�ģʽ
						reset_line_track_state(AutoVelXYZ,cfg.maxAutoAccXY[0])
						HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_RouteLine3D;
					}
					else
						HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_Position;
				
					//���¿���ʱ��
					if(!isMSafe)
						last_XYCtrlTime = last_ZCtrlTime = TIME::now();
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			
			bool Position_Control_set_TargetPositionXYRelativeBodyheading( double posx, double posy, double vel, double TIMEOUT )
			{
				Quaternion att;
				get_Attitude_quat( &att );
				double yaw = att.getYaw();
				double sin_Yaw, cos_Yaw;
				fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
				double posx_enu = BodyHeading2ENU_x( posx , posy , sin_Yaw , cos_Yaw );
				double posy_enu = BodyHeading2ENU_y( posx , posy , sin_Yaw , cos_Yaw );
				return Position_Control_set_TargetPositionXYRelative( posx_enu, posy_enu, vel, TIMEOUT );
			}
			bool Position_Control_set_TargetPositionXYZRelativeBodyheading( double posx, double posy, double posz, double vel, double TIMEOUT )
			{
				Quaternion att;
				get_Attitude_quat( &att );
				double yaw = att.getYaw();
				double sin_Yaw, cos_Yaw;
				fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
				double posx_enu = BodyHeading2ENU_x( posx , posy , sin_Yaw , cos_Yaw );
				double posy_enu = BodyHeading2ENU_y( posx , posy , sin_Yaw , cos_Yaw );
				return Position_Control_set_TargetPositionXYZRelative( posx_enu, posy_enu, posz, vel, TIMEOUT );
			}
			
			bool Position_Control_set_TargetPositionXY_LatLon( double Lat, double Lon, double vel, double TIMEOUT )
			{
				if( isnan(Lat) || isinf(Lat) ||
						isnan(Lon) || isinf(Lon)	)
					return false;
				
				//��ȡ����ȫ��λ��������Ϣ
				PosSensorHealthInf2 global_inf;
				if( get_OptimalGlobal_XY( &global_inf ) == false )
					return false;
				//��ȡָ����γ��ƽ������
				double x, y;
				map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
				x -= global_inf.HOffset.x;
				y -= global_inf.HOffset.y;
				return Position_Control_set_TargetPositionXY( x, y, vel, TIMEOUT );
			}
			bool Position_Control_set_TargetPositionXYZ_LatLon( double Lat, double Lon, double posz, double vel, double TIMEOUT )
			{
				if( isnan(Lat) || isinf(Lat) ||
						isnan(Lon) || isinf(Lon) || 
						isnan(posz) || isinf(posz) )
					return false;
				
				//��ȡ����ȫ��λ��������Ϣ
				PosSensorHealthInf3 global_inf;
				if( get_OptimalGlobal_XYZ( &global_inf ) == false )
					return false;
				//��ȡָ����γ��ƽ������
				double x, y;
				map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
				x -= global_inf.HOffset.x;
				y -= global_inf.HOffset.y;
				posz -= global_inf.HOffset.z;
				return Position_Control_set_TargetPositionXYZ( x, y, posz, vel, TIMEOUT );
			}
			bool Position_Control_set_TargetPositionXYZRA_LatLon( double Lat, double Lon, double posz, double vel, double TIMEOUT )
			{
				if( isnan(Lat) || isinf(Lat) ||
						isnan(Lon) || isinf(Lon) || 
						isnan(posz) || isinf(posz) )
					return false;
				
				//��ȡ����ȫ��λ��������Ϣ
				PosSensorHealthInf2 global_inf;
				if( get_OptimalGlobal_XY( &global_inf ) == false )
					return false;
				//��ȡָ����γ��ƽ������
				double x, y;
				map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
				x -= global_inf.HOffset.x;
				y -= global_inf.HOffset.y;
				//��ȡ���λ��Z����
				double homeZ;
				getHomeLocalZ(&homeZ);
				return Position_Control_set_TargetPositionXYZ( x, y, homeZ + posz, vel, TIMEOUT );
			}
			bool Position_Control_set_TargetPositionXYZRelative_LatLon( double Lat, double Lon, double posz, double vel, double TIMEOUT )
			{
				if( isnan(Lat) || isinf(Lat) ||
						isnan(Lon) || isinf(Lon) || 
						isnan(posz) || isinf(posz) )
					return false;
				
				//��ȡ����ȫ��λ��������Ϣ
				PosSensorHealthInf2 global_inf;
				if( get_OptimalGlobal_XY( &global_inf ) == false )
					return false;
				//��ȡָ����γ��ƽ������
				double x, y, z;
				map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
				x -= global_inf.HOffset.x;
				y -= global_inf.HOffset.y;
				z = global_inf.PositionENU.z + posz;
				return Position_Control_set_TargetPositionXYZ( x, y, z, vel, TIMEOUT );
			}
			
			bool Position_Control_get_LineFlightDistance( double* distance, double TIMEOUT )
			{
				if( distance == 0 )
					return false;
				if( HorizontalPosition_ControlMode!=Position_ControlMode_RouteLine3D && HorizontalPosition_ControlMode!=Position_ControlMode_RouteLine )
					return false;
				
				vector3<double> position;
				get_Position(&position);
				if( LockCtrl(TIMEOUT) )
				{
					//���㴹��
					vector3<double> A_C = position - target_position;
					double k = (A_C * route_line_A_B) * route_line_m;
					vector3<double> foot_point = (route_line_A_B * k) + target_position;
					
					//����ƫ��
					vector3<double> B = target_position + route_line_A_B;
					vector3<double> dis = foot_point - B;
					*distance = safe_sqrt(dis.get_square());
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
		/*ֱ�߷���*/
			
			
		/*�趨�Զ������ٶ�*/
			bool Position_Control_get_XYAutoSpeed( double* AtVelXY, double TIMEOUT )
			{
				if( LockCtrl(TIMEOUT) )
				{
					*AtVelXY = AutoVelXY;
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_reset_XYAutoSpeed( double TIMEOUT )
			{
				if( LockCtrl(TIMEOUT) )
				{
					AutoVelXY = cfg.AutoVXY[0];
					if( AutoVelXY > cfg.maxAutoVelXY[0] )
						AutoVelXY = cfg.maxAutoVelXY[0];
					line_track_desired_maxv = AutoVelXY;
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_set_XYAutoSpeed( double AtVelXY, double TIMEOUT )
			{
				if( isnan(AtVelXY) || isinf(AtVelXY) )
					return false;
				
				if( LockCtrl(TIMEOUT) )
				{
					AutoVelXY = AtVelXY;
					if( AutoVelXY > cfg.maxAutoVelXY[0] )
						AutoVelXY = cfg.maxAutoVelXY[0];
					line_track_desired_maxv = AutoVelXY;
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			
			bool Position_Control_get_XYZAutoSpeed( double* AtVelXYZ, double TIMEOUT )
			{
				if( LockCtrl(TIMEOUT) )
				{
					*AtVelXYZ = AutoVelXYZ;
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_reset_XYZAutoSpeed( double TIMEOUT )
			{		
				if( LockCtrl(TIMEOUT) )
				{
					AutoVelXYZ = cfg.AutoVXYZ[0];
					
					//�ٶ��޷�
					double xyz_length = safe_sqrt( route_line_A_B.get_square() );
					if( xyz_length > 1 )
					{
						double xy_length = safe_sqrt( route_line_A_B.x*route_line_A_B.x + route_line_A_B.y*route_line_A_B.y );
						
						double inv_xyz = 1.0 / xyz_length;
						double xy_scale = xy_length * inv_xyz;
						double z_scale = fabs(route_line_A_B.z) * inv_xyz;
						
						double scale = 1.0;
						if( AutoVelXYZ*xy_scale > cfg.maxAutoVelXY[0] )
							scale = cfg.maxAutoVelXY[0] / (AutoVelXYZ*xy_scale);
						if( route_line_A_B.z < 0 )
						{	//���Ϸ�
							if( AutoVelXYZ*z_scale > cfg.maxAutoVelUp[0] )
							{
								double new_scale = cfg.maxAutoVelUp[0] / (AutoVelXYZ*z_scale);
								if( new_scale < scale )
									scale = new_scale;
							}
						}
						else
						{	//���·�
							if( AutoVelXYZ*z_scale > cfg.maxAutoVelDown[0] )
							{
								double new_scale = cfg.maxAutoVelDown[0] / (AutoVelXYZ*z_scale);
								if( new_scale < scale )
									scale = new_scale;
							}
						}
						
						AutoVelXYZ *= scale;
					}
					else
					{
						if( AutoVelXYZ > cfg.maxAutoVelXY[0] )
							AutoVelXYZ = cfg.maxAutoVelXY[0];
					}
					line_track_desired_maxv = AutoVelXYZ;
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_set_XYZAutoSpeed( double AtVelXYZ, double TIMEOUT )
			{
				if( isnan(AtVelXYZ) || isinf(AtVelXYZ) )
					return false;
				
				if( LockCtrl(TIMEOUT) )
				{
					AutoVelXYZ = AtVelXYZ;
					
					//�ٶ��޷�
					double xyz_length = safe_sqrt( route_line_A_B.get_square() );
					if( xyz_length > 1 )
					{
						double xy_length = safe_sqrt( route_line_A_B.x*route_line_A_B.x + route_line_A_B.y*route_line_A_B.y );
						
						double inv_xyz = 1.0 / xyz_length;
						double xy_scale = xy_length * inv_xyz;
						double z_scale = fabs(route_line_A_B.z) * inv_xyz;
						
						double scale = 1.0;
						if( AutoVelXYZ*xy_scale > cfg.maxAutoVelXY[0] )
							scale = cfg.maxAutoVelXY[0] / (AutoVelXYZ*xy_scale);
						if( route_line_A_B.z < 0 )
						{	//���Ϸ�
							if( AutoVelXYZ*z_scale > cfg.maxAutoVelUp[0] )
							{
								double new_scale = cfg.maxAutoVelUp[0] / (AutoVelXYZ*z_scale);
								if( new_scale < scale )
									scale = new_scale;
							}
						}
						else
						{	//���·�
							if( AutoVelXYZ*z_scale > cfg.maxAutoVelDown[0] )
							{
								double new_scale = cfg.maxAutoVelDown[0] / (AutoVelXYZ*z_scale);
								if( new_scale < scale )
									scale = new_scale;
							}
						}
						
						AutoVelXYZ *= scale;
					}
					else
					{
						if( AutoVelXYZ > cfg.maxAutoVelXY[0] )
							AutoVelXYZ = cfg.maxAutoVelXY[0];
					}
					line_track_desired_maxv = AutoVelXYZ;
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
		/*�趨�Զ������ٶ�*/
	/*ˮƽλ��*/
/*���ƽӿ�*/
		
/*�˲���*/
	static Filter_Butter4_LP ThrOut_Filters[3];
/*�˲���*/
			
void ctrl_Position()
{	
	bool Attitude_Control_Enabled;	is_Attitude_Control_Enabled(&Attitude_Control_Enabled);
	if( Attitude_Control_Enabled == false )
	{
		Altitude_Control_Enabled = false;
		Position_Control_Enabled = false;
		return;
	}
	
	double h = 1.0/CtrlRateHz;
	
	double e_1_n;
	double e_1;
	double e_2_n;
	double e_2;
	
	bool inFlight;	get_is_inFlight(&inFlight);
	vector3<double> Position;	get_Position_Ctrl(&Position);
	vector3<double> VelocityENU;	get_VelocityENU_Ctrl(&VelocityENU);
	vector3<double> AccelerationENU;	get_AccelerationENU_Ctrl(&AccelerationENU);
	get_throttle_force(&AccelerationENU.z);	AccelerationENU.z-=GravityAcc;
	
	//λ���ٶ��˲�
	double Ps = cfg.P1[0];
	double Pv = cfg.P2[0];
	double Pa = cfg.P3[0];
	
	static vector3<double> TAcc;
	vector3<double> TargetVelocity;
	vector3<double> TargetVelocity_1;
	vector3<double> TargetVelocity_2;
	
	//XY��Z����һ��Ϊ��3Dģʽ���˳�3Dģʽ
	if( Is_3DAutoMode(HorizontalPosition_ControlMode) && Is_3DAutoMode(Altitude_ControlMode)==false )
		HorizontalPosition_ControlMode = Position_ControlMode_Locking;
	else if( Is_3DAutoMode(HorizontalPosition_ControlMode)==false && Is_3DAutoMode(Altitude_ControlMode) )
		Altitude_ControlMode = Position_ControlMode_Locking;
	
	if( Position_Control_Enabled )
	{	//ˮƽλ�ÿ���
		if( get_Position_MSStatus() != MS_Ready )
		{
			Position_Control_Enabled = false;
			goto PosCtrl_Finish;
		}
		
		switch( HorizontalPosition_ControlMode )
		{
			case Position_ControlMode_Position:
			{
				if( inFlight )
				{
					vector2<double> e1;
					e1.x = target_position.x - Position.x;
					e1.y = target_position.y - Position.y;
					vector2<double> e1_1;
					e1_1.x = - VelocityENU.x;
					e1_1.y = - VelocityENU.y;
					vector2<double> e1_2;
					e1_2.x = - TAcc.x;
					e1_2.y = - TAcc.y;
					double e1_length = safe_sqrt(e1.get_square());
					e_1_n = e1.x*e1_1.x + e1.y*e1_1.y;
					if( !is_zero(e1_length) )
						e_1 = e_1_n / e1_length;
					else
						e_1 = 0;
					e_2_n = ( e1.x*e1_2.x + e1.y*e1_2.y + e1_1.x*e1_1.x + e1_1.y*e1_1.y )*e1_length - e_1*e_1_n;
					if( !is_zero(e1_length*e1_length) )
						e_2 = e_2_n / (e1_length*e1_length);
					else
						e_2 = 0;
					smooth_kp_d2 d1 = smooth_kp_2( e1_length, e_1, e_2 , Ps, 200 );
					vector2<double> T2;
					vector2<double> T2_1;
					vector2<double> T2_2;
					if( !is_zero(e1_length*e1_length*e1_length) )
					{
						vector2<double> n = e1 * (1.0/e1_length);
						vector2<double> n_1 = (e1_1*e1_length - e1*e_1) / (e1_length*e1_length);
						vector2<double> n_2 = ( (e1_2*e1_length-e1*e_2)*e1_length - (e1_1*e1_length-e1*e_1)*(2*e_1) ) / (e1_length*e1_length*e1_length);
						T2 = n*d1.d0;
						T2_1 = n*d1.d1 + n_1*d1.d0;
						T2_2 = n*d1.d2 + n_1*(2*d1.d1) + n_2*d1.d0;
					}
					TargetVelocity.x = T2.x;	TargetVelocity.y = T2.y;
					TargetVelocity_1.x = T2_1.x;	TargetVelocity_1.y = T2_1.y;
					TargetVelocity_2.x = T2_2.x;	TargetVelocity_2.y = T2_2.y;
				}
				else
				{
					//û���ǰ��λ�ÿ���ģʽ
					//��������λ��
					target_position.x = Position.x;
					target_position.y = Position.y;
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				break;
			}		
			case Position_ControlMode_Velocity:
			{
				if( !inFlight )
				{
					//û���ʱ���������ٶ�
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				else
				{
					TargetVelocity.x = target_velocity.x;
					TargetVelocity.y = target_velocity.y;
					Pv = cfg.P2_VelXY[0];
				}
				break;
			}
			
			case Position_ControlMode_RouteLine:
			{
				if( inFlight )
				{						
					//���㴹��	
					vector2<double> A( target_position.x, target_position.y );					
					vector2<double> C( Position.x, Position.y );
					vector2<double> A_C = C - A;
					vector2<double> A_B( route_line_A_B.x, route_line_A_B.y );
					double k = (A_C * A_B) * route_line_m;
					vector2<double> foot_point = (A_B * k) + A;
					
					//�������A��λ��
					vector2<double> A_desired = A;
					double A_B_length = safe_sqrt(A_B.get_square());					
					if( A_B_length > 0.1 )
					{						
						if( 0.5*line_track_desired_vel*line_track_desired_vel/line_track_desired_maxacc >= A_B_length - line_track_desired )
						{	//��Ҫ����
							line_track_desired_vel -= line_track_desired_maxacc*h;
							if( line_track_desired_vel < 0 )
								line_track_desired_vel = 0;
						}
						else
						{	//ǰ�����ٵ�����ٶ�
							if( line_track_desired_vel < line_track_desired_maxv )
								line_track_desired_vel += line_track_desired_maxacc*h;
							if( line_track_desired_vel > line_track_desired_maxv )
								line_track_desired_vel = line_track_desired_maxv;
						}
						line_track_desired += line_track_desired_vel*h;
						if( line_track_desired > A_B_length )
							line_track_desired = A_B_length;
						
						vector2<double> B = A + A_B;
						A_desired = B - A_B*(line_track_desired/A_B_length);
					}
					
					//����ƫ��
					vector2<double> e1r = A_desired - foot_point;
					vector2<double> e1d = foot_point - C;
					double e1r_length = safe_sqrt(e1r.get_square());
					double e1d_length = safe_sqrt(e1d.get_square());
					
					//����route����λ����
					vector2<double> route_n;
					if( e1r_length > 0.001 )
						route_n = e1r * (1.0/e1r_length);
					
					//����d����λ����
					vector2<double> d_n;
					if( e1d_length > 0.001 )
						d_n = e1d * (1.0/e1d_length);
					
					//����e1����
					vector2<double> e1_1( VelocityENU.x, VelocityENU.y );
					double e1r_1 = -(e1_1 * route_n);
					double e1d_1 = -(e1_1 * d_n);
					//e1���׵�
					vector2<double> e1_2( TAcc.x, TAcc.y );
					double e1r_2 = -(e1_2 * route_n);
					double e1d_2 = -(e1_2 * d_n);
					
					/*route����*/
						smooth_kp_d2 d1r = smooth_kp_2( e1r_length, e1r_1, e1r_2 , Ps, AutoVelXY+100 );
						vector2<double> T2r = route_n * d1r.d0;
						vector2<double> T2r_1 = route_n * d1r.d1;
						vector2<double> T2r_2 = route_n * d1r.d2;
					/*route����*/
					
					/*d����*/
						smooth_kp_d2 d1d = smooth_kp_2( e1d_length, e1d_1, e1d_2 , Ps, AutoVelXY );
						vector2<double> T2d = d_n * d1d.d0;
						vector2<double> T2d_1 = d_n * d1d.d1;
						vector2<double> T2d_2 = d_n * d1d.d2;
					/*d����*/
						
					TargetVelocity.x = T2r.x+T2d.x;	TargetVelocity.y = T2r.y+T2d.y;
					TargetVelocity_1.x = T2r_1.x+T2d_1.x;	TargetVelocity_1.y = T2r_1.y+T2d_1.y;
					TargetVelocity_2.x = T2r_2.x+T2d_2.x;	TargetVelocity_2.y = T2r_2.y+T2d_2.y;
						
					//�жϵ���
					if( e1r.get_square()<50*50 && e1d.get_square() < 80*80 && 
							A_B_length - line_track_desired < 1 ) 
					{
						++line_track_delay_counter;
						if( line_track_delay_counter >= 3*CtrlRateHz )
							HorizontalPosition_ControlMode = Position_ControlMode_Position;
					}
					else
						line_track_delay_counter = 0;
				}
				else
				{
					//û���ʱ���������ٶ�
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				break;
			}
			
			case Position_ControlMode_RouteLine3D:
			{
				if( inFlight )
				{
					//���㴹��
					vector3<double> A_C = Position - target_position;
					double k = (A_C * route_line_A_B) * route_line_m;
					vector3<double> foot_point = (route_line_A_B * k) + target_position;
					
					//�������A��λ��
					vector3<double> A_desired = target_position;
					double A_B_length = safe_sqrt(route_line_A_B.get_square());					
					if( A_B_length > 0.1 )
					{
						if( 0.5*line_track_desired_vel*line_track_desired_vel/line_track_desired_maxacc >= A_B_length - line_track_desired )
						{	//��Ҫ����
							line_track_desired_vel -= line_track_desired_maxacc*h;
							if( line_track_desired_vel < 0 )
								line_track_desired_vel = 0;
						}
						else
						{	//ǰ�����ٵ�����ٶ�
							if( line_track_desired_vel < line_track_desired_maxv )
								line_track_desired_vel += line_track_desired_maxacc*h;
							if( line_track_desired_vel > line_track_desired_maxv )
								line_track_desired_vel = line_track_desired_maxv;
						}
						line_track_desired += line_track_desired_vel*h;
						if( line_track_desired > A_B_length )
							line_track_desired = A_B_length;
						
						vector3<double> B = target_position + route_line_A_B;
						A_desired = B - route_line_A_B*(line_track_desired/A_B_length);
					}
					
					//����ƫ��
					vector3<double> e1r = A_desired - foot_point;
					vector3<double> e1d = foot_point - Position;
					double e1r_length = safe_sqrt(e1r.get_square());
					double e1d_length = safe_sqrt(e1d.get_square());
					
					//����route����λ����
					vector3<double> route_n;
					if( e1r_length > 0.001 )
						route_n = e1r * (1.0/e1r_length);
					
					//����e1����
					double e1r_1_length = -(VelocityENU * route_n);	
					vector3<double> e1r_1 = route_n * e1r_1_length;					
					vector3<double> e1d_1 = -(VelocityENU + e1r_1);
					//e1���׵�
					vector3<double> e1_2( TAcc.x, TAcc.y, AccelerationENU.z );
					double e1r_2_length = -(e1_2 * route_n);
					vector3<double> e1r_2 = route_n * e1r_2_length;					
					vector3<double> e1d_2 = -(e1_2 + e1r_2);
					
					/*route����*/
						smooth_kp_d2 d1r = smooth_kp_2( e1r_length, e1r_1_length, e1r_2_length , Ps, AutoVelXYZ+100 );
						vector3<double> T2r = route_n * d1r.d0;
						vector3<double> T2r_1 = route_n * d1r.d1;
						vector3<double> T2r_2 = route_n * d1r.d2;
					/*route����*/
					
					/*d����*/
						e_1_n = e1d.x*e1d_1.x + e1d.y*e1d_1.y + e1d.z*e1d_1.z;
						if( !is_zero(e1d_length) )
							e_1 = e_1_n / e1d_length;
						else
							e_1 = 0;
						e_2_n = ( e1d.x*e1d_2.x + e1d.y*e1d_2.y + e1d.z*e1d_2.z + e1d_1.x*e1d_1.x + e1d_1.y*e1d_1.y + e1d_1.z*e1d_1.z )*e1d_length - e_1*e_1_n;
						if( !is_zero(e1d_length*e1d_length) )
							e_2 = e_2_n / (e1d_length*e1d_length);
						else
							e_2 = 0;
						smooth_kp_d2 d1d = smooth_kp_2( e1d_length, e_1, e_2 , Ps, AutoVelXYZ );
						vector3<double> T2d;
						vector3<double> T2d_1;
						vector3<double> T2d_2;
						if( !is_zero(e1d_length*e1d_length*e1d_length) )
						{
							vector3<double> n = e1d * (1.0/e1d_length);
							vector3<double> n_1 = (e1d_1*e1d_length - e1d*e_1) / (e1d_length*e1d_length);
							vector3<double> n_2 = ( (e1d_2*e1d_length-e1d*e_2)*e1d_length - (e1d_1*e1d_length-e1d*e_1)*(2*e_1) ) / (e1d_length*e1d_length*e1d_length);
							T2d = n*d1d.d0;
							T2d_1 = n*d1d.d1 + n_1*d1d.d0;
							T2d_2 = n*d1d.d2 + n_1*(2*d1d.d1) + n_2*d1d.d0;
						}
					/*d����*/
						
					TargetVelocity = T2r + T2d;
					TargetVelocity_1 = T2r_1 + T2d_1;
					TargetVelocity_2 = T2r_2 + T2d_2;
						
					//�жϵ���
					if( e1r.get_square()<50*50 && e1d.get_square() < 80*80 && 
							A_B_length - line_track_desired < 1.0 ) 
					{
						++line_track_delay_counter;
						if( line_track_delay_counter >= 3*CtrlRateHz )
							HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_Position;
					}
					else
						line_track_delay_counter = 0;
				}
				else
				{
					//û���ʱ���������ٶ�
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				break;
			}
			
			case Position_ControlMode_Locking:
			default:
			{	//ɲ����λ��
				static uint16_t lock_counter = 0;
				if( inFlight )
				{					
					TargetVelocity.x = 0;
					TargetVelocity.y = 0;
					if( VelocityENU.x*VelocityENU.x + VelocityENU.y*VelocityENU.y < 10*10 )
					{
						if( ++lock_counter >= CtrlRateHz*0.7 )
						{
							lock_counter = 0;
							target_position.x = Position.x;
							target_position.y = Position.y;
							HorizontalPosition_ControlMode = Position_ControlMode_Position;
						}
					}
					else
						lock_counter = 0;
				}
				else
				{
					lock_counter = 0;
					target_position.x = Position.x;
					target_position.y = Position.y;
					HorizontalPosition_ControlMode = Position_ControlMode_Position;
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				break;
			}
		}	
		
		//�����������ٶ�
		vector2<double> e2;
		e2.x = TargetVelocity.x - VelocityENU.x;
		e2.y = TargetVelocity.y - VelocityENU.y;
		vector2<double> e2_1;
		e2_1.x = TargetVelocity_1.x - TAcc.x;
		e2_1.y = TargetVelocity_1.y - TAcc.y;
		double e2_length = safe_sqrt(e2.get_square());
		e_1_n = e2.x*e2_1.x + e2.y*e2_1.y;
		if( !is_zero(e2_length) )
			e_1 = e_1_n / e2_length;
		else
			e_1 = 0;
		smooth_kp_d1 d2;
		if( Is_AutoMode(HorizontalPosition_ControlMode) )
			d2 = smooth_kp_1( e2_length, e_1 , Pv, cfg.maxAutoAccXY[0] );
		else
			d2 = smooth_kp_1( e2_length, e_1 , Pv, cfg.maxAccXY[0] );
		vector2<double> T3;
		vector2<double> T3_1;
		if( !is_zero(e2_length*e2_length) )
		{
			vector2<double> n = e2 * (1.0/e2_length);
			vector2<double> n_1 = (e2_1*e2_length - e2*e_1) / (e2_length*e2_length);
			T3 = n*d2.d0;
			T3_1 = n*d2.d1 + n_1*d2.d0;
		}
		T3 += vector2<double>( TargetVelocity_1.x, TargetVelocity_1.y );
		T3_1 += vector2<double>( TargetVelocity_2.x, TargetVelocity_2.y );
		
		vector2<double> e3;
		e3.x = T3.x - TAcc.x;
		e3.y = T3.y - TAcc.y;
		double e3_length = safe_sqrt(e3.get_square());
		double d3;
		if( Is_AutoMode(HorizontalPosition_ControlMode) )
			d3 = smooth_kp_0( e3_length , Pa, cfg.maxAutoJerkXY[0] );
		else
			d3 = smooth_kp_0( e3_length , Pa, cfg.maxJerkXY[0] );
		vector2<double> T4;
		if( !is_zero(e3_length) )
		{
			vector2<double> n = e3 * (1.0/e3_length);
			T4 = n*d3;
		}
		if( Is_AutoMode(HorizontalPosition_ControlMode) )
			T4.constrain( cfg.maxAutoJerkXY[0] );
		else
			T4.constrain( cfg.maxJerkXY[0] );
		T4 += T3_1;
		
		TAcc.x += T4.x*h;
		TAcc.y += T4.y*h;	
		
		//ȥ�������Ŷ�
		vector3<double> WindDisturbance;
		get_WindDisturbance( &WindDisturbance );
		vector2<double> target_acceleration;
//		target_acceleration.x = TAcc.x - WindDisturbance.x;
//		target_acceleration.y = TAcc.y - WindDisturbance.y;
		target_acceleration.x = T3.x - WindDisturbance.x;
		target_acceleration.y = T3.y - WindDisturbance.y;
		
		//��ת��Bodyheading
		Quaternion attitude;
		get_Attitude_quat(&attitude);
		double yaw = attitude.getYaw();		
		double sin_Yaw, cos_Yaw;
		fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
		double target_acceleration_x_bodyheading = ENU2BodyHeading_x( target_acceleration.x , target_acceleration.y , sin_Yaw , cos_Yaw );
		double target_acceleration_y_bodyheading = ENU2BodyHeading_y( target_acceleration.x , target_acceleration.y , sin_Yaw , cos_Yaw );
//		target_acceleration_x_bodyheading = ThrOut_Filters[0].run(target_acceleration_x_bodyheading);
//		target_acceleration_y_bodyheading = ThrOut_Filters[1].run(target_acceleration_y_bodyheading);
		
		//������������Ƕ�
		double WindDisturbance_Bodyheading_x = ENU2BodyHeading_x( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
		double WindDisturbance_Bodyheading_y = ENU2BodyHeading_y( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
		//����Ƕ�
		double AccUp = GravityAcc + AccelerationENU.z;
		double AntiDisturbancePitch = atan2( -WindDisturbance_Bodyheading_x , AccUp );
		double AntiDisturbanceRoll = atan2( WindDisturbance_Bodyheading_y , AccUp );
		
		//����Ŀ��Ƕ�
		double target_Roll = atan2( -target_acceleration_y_bodyheading , AccUp );
		double target_Pitch = atan2( target_acceleration_x_bodyheading , AccUp );
		if( HorizontalPosition_ControlMode==Position_ControlMode_Velocity )
		{	//�Ƕ��޷�
			if( VelCtrlMaxRoll>0 && VelCtrlMaxPitch>0 )
			{
				target_Roll = constrain( target_Roll , AntiDisturbanceRoll - VelCtrlMaxRoll, AntiDisturbanceRoll + VelCtrlMaxRoll );
				target_Pitch = constrain( target_Pitch , AntiDisturbancePitch - VelCtrlMaxPitch, AntiDisturbancePitch + VelCtrlMaxPitch );
			}
			else if( VelCtrlMaxRoll>0 )
			{
				vector2<double> Tangle( target_Roll - AntiDisturbanceRoll, target_Pitch - AntiDisturbancePitch );
				Tangle.constrain(VelCtrlMaxRoll);
				target_Roll = AntiDisturbanceRoll + Tangle.x;
				target_Pitch = AntiDisturbancePitch + Tangle.y;
			}
		}

		//�趨Ŀ��Ƕ�
		Attitude_Control_set_Target_RollPitch( target_Roll, target_Pitch );
		
		//��ȡ��ʵĿ��Ƕ�����TAcc
		Attitude_Control_get_Target_RollPitch( &target_Roll, &target_Pitch );
		target_acceleration_x_bodyheading = tan(target_Pitch)*GravityAcc;
		target_acceleration_y_bodyheading = -tan(target_Roll)*GravityAcc;
		target_acceleration.x = BodyHeading2ENU_x( target_acceleration_x_bodyheading, target_acceleration_y_bodyheading , sin_Yaw, cos_Yaw );
		target_acceleration.y = BodyHeading2ENU_y( target_acceleration_x_bodyheading, target_acceleration_y_bodyheading , sin_Yaw, cos_Yaw );
		TAcc.x = target_acceleration.x + WindDisturbance.x;
		TAcc.y = target_acceleration.y + WindDisturbance.y;
	}//ˮƽλ�ÿ���
	else
	{
		ThrOut_Filters[0].reset(0);
		ThrOut_Filters[1].reset(0);
	}
	
PosCtrl_Finish:	
	if( Altitude_Control_Enabled )
	{//�߶ȿ���
			
		//���ÿ������޷�
		Target_tracker[2].r2p = cfg.maxVelUp[0];
		Target_tracker[2].r2n = cfg.maxVelDown[0];
		Target_tracker[2].r3p = cfg.maxAccUp[0];
		Target_tracker[2].r3n = cfg.maxAccDown[0];
		Target_tracker[2].r4p = cfg.maxJerkUp[0];
		Target_tracker[2].r4n = cfg.maxJerkDown[0];
		
		if( !Is_3DAutoMode(Altitude_ControlMode) )
		{
			switch( Altitude_ControlMode )
			{
				case Position_ControlMode_Position:
				{	//����λ��
					if( inFlight )
					{
						Target_tracker[2].r2p = 0.3*cfg.maxVelUp[0];
						Target_tracker[2].r2n = 0.3*cfg.maxVelDown[0];
						Target_tracker[2].track4( target_position.z , 1.0 / CtrlRateHz );
					}
					else
					{
						//û���ǰ��λ�ÿ���ģʽ
						//��Ҫ���
						Target_tracker[2].reset();
						target_position.z = Target_tracker[2].x1 = Position.z;
						Attitude_Control_set_Throttle( get_STThrottle() );
						goto AltCtrl_Finish;
					}
					break;
				}
				case Position_ControlMode_Velocity:
				{	//�����ٶ�
					if( inFlight )
					{
						double TVel;
						if( target_velocity.z > cfg.maxVelUp[0] )
							TVel = cfg.maxVelUp[0];
						else if( target_velocity.z < -cfg.maxVelDown[0] )
							TVel = -cfg.maxVelDown[0];
						else
							TVel = target_velocity.z;
						Target_tracker[2].track3( TVel , 1.0 / CtrlRateHz );
					}
					else
					{
						//û����������ٶ�Ϊ��
						//��Ҫ���
						Target_tracker[2].reset();
						Target_tracker[2].x1 = Position.z;						
						if( target_velocity.z <= 0 )
						{
							Attitude_Control_set_Throttle( get_STThrottle() );
							goto AltCtrl_Finish;
						}
					}
					break;
				}
				
				case Position_ControlMode_Takeoff:
				{	//���
					
					//���ÿ��������ֵ
					Target_tracker[2].r3p = cfg.maxAutoAccUp[0];
					Target_tracker[2].r3n = cfg.maxAutoAccDown[0];
					Target_tracker[2].r4p = cfg.maxAutoJerkUp[0];
					Target_tracker[2].r4n = cfg.maxAutoJerkDown[0];
					
					if( inFlight )
					{
						//�����
						//���ƴﵽĿ��߶�
						double homeZ;
						getHomeLocalZ(&homeZ);
						if( Position.z - homeZ < 100 )
							Target_tracker[2].r2n = Target_tracker[2].r2p = 50;
						else
							Target_tracker[2].r2n = Target_tracker[2].r2p = AutoVelZUp;
						Target_tracker[2].track4( target_position.z , 1.0 / CtrlRateHz );
						if( fabs( target_position.z - Position.z ) < 10 && \
								in_symmetry_range( Target_tracker[2].x2 , 0.1 ) && \
								in_symmetry_range( Target_tracker[2].x3 , 0.1 )	)
							Altitude_ControlMode = Position_ControlMode_Position;
					}
					else
					{
						//δ���
						//�ȴ����
						target_position.z =  Position.z + TakeoffHeight;
						Target_tracker[2].reset();
						Target_tracker[2].x1 = Position.z;
					}
					break;
				}
				case Position_ControlMode_RouteLine:
				{	//�ɵ�ָ���߶�
					
					//���ÿ��������ֵ
					Target_tracker[2].r3p = cfg.maxAutoAccUp[0];
					Target_tracker[2].r3n = cfg.maxAutoAccDown[0];
					Target_tracker[2].r4p = cfg.maxAutoJerkUp[0];
					Target_tracker[2].r4n = cfg.maxAutoJerkDown[0];
					
					if( inFlight )
					{
						//�����
						//���ƴﵽĿ��߶�
						Target_tracker[2].r2n = AutoVelZUp;
						Target_tracker[2].r2p = AutoVelZDown;
						Target_tracker[2].track4( target_position.z , 1.0f / CtrlRateHz );
						if( fabs( target_position.z - Position.z ) < 10 && \
								in_symmetry_range( VelocityENU.z , 10.0f ) &&  \
								in_symmetry_range( AccelerationENU.z , 50.0f ) && \
								in_symmetry_range( Target_tracker[2].x2 , 0.1f ) && \
								in_symmetry_range( Target_tracker[2].x3 , 0.1f )	)
							Altitude_ControlMode = Position_ControlMode_Position;
					}
					else
					{
						//δ���
						//��Ҫ���
						Target_tracker[2].reset();
						Target_tracker[2].x1 = Position.z;
						Attitude_Control_set_Throttle( 0 );
						goto AltCtrl_Finish;
					}
					break;
				}
				
				case Position_ControlMode_Locking:
				default:
				{	//��λ�ã����ٵ�0Ȼ����ס�߶ȣ�
					if( inFlight )
					{
						Target_tracker[2].track3( 0 , 1.0 / CtrlRateHz );
						if( in_symmetry_range( VelocityENU.z , 10.0 ) && \
								in_symmetry_range( Target_tracker[2].x2 , 0.1 ) && \
								in_symmetry_range( Target_tracker[2].x3 , 0.1 ) && \
								in_symmetry_range( Target_tracker[2].x4 , 0.1 )	)
						{
							target_position.z = Target_tracker[2].x1 = Position.z;
							Altitude_ControlMode = Position_ControlMode_Position;
						}
					}
					else
					{
						Altitude_ControlMode = Position_ControlMode_Position;
						Attitude_Control_set_Throttle( get_STThrottle() );
						goto AltCtrl_Finish;
					}
					break;
				}
			}
		}
		
		if( inFlight )
		{
			//���������ٶ�
			double target_velocity_z;
			//������ֱ�ٶȵĵ���
			double Tvz_1, Tvz_2;
			if( Is_3DAutoMode(Altitude_ControlMode) )
			{
				target_velocity_z = TargetVelocity.z;
				Tvz_1 = TargetVelocity_1.z;
				Tvz_2 = TargetVelocity_2.z;
				Target_tracker[2].reset();
				Target_tracker[2].x1 = target_position.z;
			}
			else
			{
				if( Target_tracker[2].get_tracking_mode() == 4 )
				{
					double max_fb_vel = ( Target_tracker[2].x1 - Position.z ) > 0 ? cfg.maxAutoVelUp[0] : cfg.maxAutoVelDown[0];
					smooth_kp_d2 TvFb = smooth_kp_2(
							Target_tracker[2].x1 - Position.z,
							Target_tracker[2].x2 - VelocityENU.z, 
							Target_tracker[2].x3 - AccelerationENU.z, 
							Ps, max_fb_vel );
					target_velocity_z = TvFb.d0 + Target_tracker[2].x2;
					Tvz_1 = TvFb.d1 + Target_tracker[2].x3;
					Tvz_2 = TvFb.d2 + Target_tracker[2].x4;
				}
				else
				{
					target_velocity_z = Target_tracker[2].x2;
					Tvz_1 = Target_tracker[2].x3;
					Tvz_2 = Target_tracker[2].x4;
				}
			}
			
			//�����������ٶ�
			double max_fb_acc = ( target_velocity_z - VelocityENU.z ) > 0 ? cfg.maxAutoAccUp[0] : cfg.maxAutoAccDown[0];
			smooth_kp_d1 TaFb = smooth_kp_1(
					target_velocity_z - VelocityENU.z,
					Tvz_1 - AccelerationENU.z, 
					Pv, max_fb_acc );
			double target_acceleration_z = TaFb.d0 + Tvz_1;
			double target_acceleration_z_1 = TaFb.d1 + Tvz_2;
			//target_acceleration_z = TargetVelocityFilter[2].run( target_acceleration_z );
			//���ٶ����
			double acceleration_z_error = target_acceleration_z - AccelerationENU.z;
			
			//��ȡ���cosin
			Quaternion quat;
			get_Airframe_quat(&quat);
			double lean_cosin = quat.get_lean_angle_cosin();
			
			//��ȡ�����ת����
			double MotorStartThrottle = get_STThrottle();
			//��ȡ��ͣ���� - �����ת����
			double hover_throttle;
			get_hover_throttle(&hover_throttle);
			hover_throttle = hover_throttle - MotorStartThrottle;	
			
			//�����������
			double force, T, b;
			get_throttle_force(&force);			
			get_ESO_height_T(&T);
			get_throttle_b(&b);
			if( force < 1 )
				force = 1;
			double throttle = ( force + T * ( acceleration_z_error * Pa + target_acceleration_z_1 ) )/b;
			//��ǲ���
			if( lean_cosin > 0.1 )				
				throttle /= lean_cosin;
			else	//���̫��
				throttle = (100 - MotorStartThrottle) / 2;
			
			if( inFlight )
			{
				double logbuf[10];
				logbuf[0] = throttle;
				logbuf[1] = hover_throttle;
				logbuf[2] = force;
				logbuf[3] = target_acceleration_z;
				logbuf[4] = AccelerationENU.z;
				SDLog_Msg_DebugVect( "thr", logbuf, 5 );
			}
			
			//�����޷�
			throttle += MotorStartThrottle;
			if( throttle > 90 )
				throttle = 90;
			if( inFlight )
			{
				if( throttle < MotorStartThrottle )
					throttle = MotorStartThrottle;
			}
			
			//�෭����
			static uint32_t RollOverProtectCounter = 0;
			if( lean_cosin < 0 )
			{
				if( ++RollOverProtectCounter >= CtrlRateHz*3 )
				{
					RollOverProtectCounter = CtrlRateHz*3;
					throttle = 0;
				}
			}
			else
				RollOverProtectCounter = 0;
			
//			throttle = ThrOut_Filters[2].run(throttle);
			//���
			Attitude_Control_set_Throttle( throttle );
		}
		else
		{
			//û���
			//���������������
			double throttle;
			get_Target_Throttle(&throttle);
			ThrOut_Filters[2].reset(throttle);
			Attitude_Control_set_Throttle( throttle + h * 15 );
		}
		
	}//�߶ȿ���
AltCtrl_Finish:
	return;
}

void init_Ctrl_Position()
{
	//��ʼ������TD4�˲���
	Target_tracker[0] = TD4_SL( 15 , 15 , 15 , 15 );
	Target_tracker[1] = TD4_SL( 15 , 15 , 15 , 15 );
	Target_tracker[2] = TD4_SL( 1 , 2 , 50 , 60 );	
	Target_tracker[2].r3p = 900;
	Target_tracker[2].r3n = 600;
	
	//��ʼ�������ٶȵ�ͨ�˲���
	TargetVelocityFilter[0].set_cutoff_frequency( CtrlRateHz , 10 );
	TargetVelocityFilter[1].set_cutoff_frequency( CtrlRateHz , 10 );
	TargetVelocityFilter[2].set_cutoff_frequency( CtrlRateHz , 10 );
	
	/*��ʼ������*/
		PosCtrlCfg initial_cfg;
		//Ĭ��XY�����ٶ�
		initial_cfg.AutoVXY[0] = 500;
		//Ĭ��Z���Ϻ����ٶ�
		initial_cfg.AutoVZUp[0] = 200;
		//Ĭ��Z���º����ٶ�
		initial_cfg.AutoVZDown[0] = 200;
		//Ĭ��XYZ�����ٶ�
		initial_cfg.AutoVXYZ[0] = 500;	
		
		//�߶�ǰ���˲���
		initial_cfg.Z_TD4P1[0] = 1;
		initial_cfg.Z_TD4P2[0] = 5;
		initial_cfg.Z_TD4P3[0] = 50;
		initial_cfg.Z_TD4P4[0] = 60;
		
		//λ�÷�������
		initial_cfg.P1[0] = 1;
		//�ٶȷ�������
		initial_cfg.P2[0] = 2;
		//���ٶȷ�������
		initial_cfg.P3[0] = 10;
		//ˮƽ�ٶȿ����ٶȷ�������
		initial_cfg.P2_VelXY[0] = 4;
		//ˮƽ�ٶȿ��Ƽ��ٶȷ�������
		initial_cfg.P3_VelXY[0] = 25;
		
		//���ˮƽ�ٶ�
		initial_cfg.maxVelXY[0] = 1500;
		//���ˮƽ���ٶ�
		initial_cfg.maxAccXY[0] = 300;
		//���ˮƽ�Ӽ��ٶ�
		initial_cfg.maxJerkXY[0] = 3000;
		//�Զ�ģʽ���ˮƽ�ٶ�
		initial_cfg.maxAutoVelXY[0] = 1200;
		//�Զ�ģʽ���ˮƽ���ٶ�
		initial_cfg.maxAutoAccXY[0] = 300;
		//�Զ�ģʽ���ˮƽ�Ӽ��ٶ�
		initial_cfg.maxAutoJerkXY[0] = 600;
		
		//��������ٶ�
		initial_cfg.maxVelUp[0] = 250;
		//��������ٶ�
		initial_cfg.maxVelDown[0] = 200;
		//������ϼ��ٶ�
		initial_cfg.maxAccUp[0] = 800;
		//������¼��ٶ�
		initial_cfg.maxAccDown[0] = 600;
		//������ϼӼ��ٶ�
		initial_cfg.maxJerkUp[0] = 5000;
		//������¼Ӽ��ٶ�
		initial_cfg.maxJerkDown[0] = 3000;		
		//�Զ�ģʽ��������ٶ�
		initial_cfg.maxAutoVelUp[0] = 200;
		//�Զ�ģʽ��������ٶ�
		initial_cfg.maxAutoVelDown[0] = 200;
		//�Զ�ģʽ������ϼ��ٶ�
		initial_cfg.maxAutoAccUp[0] = 250;
		//�Զ�ģʽ������¼��ٶ�
		initial_cfg.maxAutoAccDown[0] = 200;
		//�Զ�ģʽ������ϼӼ��ٶ�
		initial_cfg.maxAutoJerkUp[0] = 800;
		//�Զ�ģʽ������¼Ӽ��ٶ�
		initial_cfg.maxAutoJerkDown[0] = 800;
	/*��ʼ������*/

	SName param_names[] = {
		//Ĭ��XY�����ٶ�
		"PC_AutoVXY" ,	
		//Ĭ��Z���Ϻ����ٶ�
		"PC_AutoVZUp" ,	
		//Ĭ��Z���º����ٶ�
		"PC_AutoVZDn" ,	
		//Ĭ��XYZ�����ٶ�
		"PC_AutoVXYZ" ,	
		
		//�߶�ǰ���˲���
		"PC_Z_TD4P1" ,
		"PC_Z_TD4P2" ,
		"PC_Z_TD4P3" ,
		"PC_Z_TD4P4" ,
		
		//λ�÷�������
		"PC_P1" ,
		//�ٶȷ�������
		"PC_P2" ,
		//���ٶȷ�������
		"PC_P3" ,
		//ˮƽ�ٶȿ����ٶȷ�������
		"PC_P2VelXY" ,
		//ˮƽ�ٶȿ��Ƽ��ٶȷ�������
		"PC_P3VelXY" ,
		
		//���ˮƽ�ٶ�
		"PC_maxVelXY" ,
		//���ˮƽ���ٶ�
		"PC_maxAccXY" ,
		//���ˮƽ�Ӽ��ٶ�
		"PC_maxJerkXY" ,
		//�Զ�ģʽ���ˮƽ�ٶ�
		"PC_maxAutoVelXY" ,
		//�Զ�ģʽ���ˮƽ���ٶ�
		"PC_maxAutoAccXY" ,
		//�Զ�ģʽ���ˮƽ�Ӽ��ٶ�
		"PC_maxAutoJerkXY" ,
		
		//��������ٶ�
		"PC_maxVelUp" ,
		//��������ٶ�
		"PC_maxVelDn" ,
		//������ϼ��ٶ�
		"PC_maxAccUp" ,
		//������¼��ٶ�
		"PC_maxAccDn" ,
		//������ϼӼ��ٶ�
		"PC_maxJerkUp" ,
		//������¼Ӽ��ٶ�
		"PC_maxJerkDn" ,
		//�Զ�ģʽ��������ٶ�
		"PC_maxAutoVelUp" ,
		//�Զ�ģʽ��������ٶ�
		"PC_maxAutoVelDn" ,
		//�Զ�ģʽ������ϼ��ٶ�
		"PC_maxAutoAccUp" ,
		//�Զ�ģʽ������¼��ٶ�
		"PC_maxAutoAccDn" ,
		//�Զ�ģʽ������ϼӼ��ٶ�
		"PC_maxAutoJerkUp" ,
		//�Զ�ģʽ������¼Ӽ��ٶ�
		"PC_maxAutoJerkDn" ,
	};

	MAV_PARAM_TYPE param_types[] = {
		//Ĭ��XY�����ٶ�
		MAV_PARAM_TYPE_REAL32 ,	
		//Ĭ��Z���Ϻ����ٶ�
		MAV_PARAM_TYPE_REAL32 ,	
		//Ĭ��Z���º����ٶ�
		MAV_PARAM_TYPE_REAL32 ,	
		//Ĭ��XYZ�����ٶ�
		MAV_PARAM_TYPE_REAL32 ,	
		//�߶�ǰ���˲���
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		
		//λ�÷�������
		MAV_PARAM_TYPE_REAL32 ,
		//�ٶȷ�������
		MAV_PARAM_TYPE_REAL32 ,
		//���ٶȷ�������
		MAV_PARAM_TYPE_REAL32 ,
		//ˮƽ�ٶȿ����ٶȷ�������
		MAV_PARAM_TYPE_REAL32 ,
		//ˮƽ�ٶȿ��Ƽ��ٶȷ�������
		MAV_PARAM_TYPE_REAL32 ,
		
		//���ˮƽ�ٶ�
		MAV_PARAM_TYPE_REAL32 ,
		//���ˮƽ���ٶ�
		MAV_PARAM_TYPE_REAL32 ,
		//���ˮƽ�Ӽ��ٶ�
		MAV_PARAM_TYPE_REAL32 ,
		//�Զ�ģʽ���ˮƽ�ٶ�
		MAV_PARAM_TYPE_REAL32 ,
		//�Զ�ģʽ���ˮƽ���ٶ�
		MAV_PARAM_TYPE_REAL32 ,
		//�Զ�ģʽ���ˮƽ�Ӽ��ٶ�
		MAV_PARAM_TYPE_REAL32 ,
		
		//��������ٶ�
		MAV_PARAM_TYPE_REAL32 ,
		//��������ٶ�
		MAV_PARAM_TYPE_REAL32 ,
		//������ϼ��ٶ�
		MAV_PARAM_TYPE_REAL32 ,
		//������¼��ٶ�
		MAV_PARAM_TYPE_REAL32 ,
		//������ϼӼ��ٶ�
		MAV_PARAM_TYPE_REAL32 ,
		//������¼Ӽ��ٶ�
		MAV_PARAM_TYPE_REAL32 ,	
		//�Զ�ģʽ��������ٶ�
		MAV_PARAM_TYPE_REAL32 ,
		//�Զ�ģʽ��������ٶ�
		MAV_PARAM_TYPE_REAL32 ,
		//�Զ�ģʽ������ϼ��ٶ�
		MAV_PARAM_TYPE_REAL32 ,
		//�Զ�ģʽ������¼��ٶ�
		MAV_PARAM_TYPE_REAL32 ,
		//�Զ�ģʽ������ϼӼ��ٶ�
		MAV_PARAM_TYPE_REAL32 ,
		//�Զ�ģʽ������¼Ӽ��ٶ�
		MAV_PARAM_TYPE_REAL32 ,
	};

	ThrOut_Filters[0].set_cutoff_frequency( CtrlRateHz, 20 );
	ThrOut_Filters[1].set_cutoff_frequency( CtrlRateHz, 20 );
	ThrOut_Filters[2].set_cutoff_frequency( CtrlRateHz, 20 );
	
	ParamGroupRegister( "PosCtrl", 3, 31, param_types, param_names, (uint64_t*)&initial_cfg );
}