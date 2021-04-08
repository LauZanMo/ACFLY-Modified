#include "Modes.hpp"
#include "M35_Auto1.hpp"
#include "vector3.hpp"
#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "AC_Math.hpp"
#include "Receiver.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "NavCmdProcess.hpp"

M35_Auto1::M35_Auto1():Mode_Base( "Auto1", 35 )
{
	
}

ModeResult M35_Auto1::main_func( void* param1, uint32_t param2 )
{
	double freq = 50;
	setLedMode(LEDMode_Flying1);
	Altitude_Control_Enable();
	Position_Control_Enable();
	uint16_t exit_mode_counter_rs = 0;
	uint16_t exit_mode_counter = 0;
	uint16_t exit_mode_Gcounter = 0;
	uint32_t RollOverProtectCounter = 0;
	
	//��ȡģʽ����
	ModeFuncCfg MFunc_cfg;
	ReadParamGroup( "MFunc", (uint64_t*)&MFunc_cfg, 0 );
	
	//����ģʽ
	bool mode_switched = true;
	#define change_Mode(x) {cMode=x; mode_switched = true;}
	uint16_t current_mission_ind;
	uint8_t ModeButtonZone = 255;
	uint8_t MissionButtonZone = 255;
	uint8_t RTLButtonZone = 255;
	uint8_t cMode = AFunc_PosHold;
	
	//��ǰִ����������
	uint16_t mission_ind = 0;
	//����״̬��
	NavCmdInf navInf;
	init_NavCmdInf(&navInf);
	while(1)
	{
		os_delay(0.02);
		
		if( get_CrashedState() )
		{	//�෭����
			Attitude_Control_Disable();
			return MR_Err;
		}
		
		//��ȡ���ջ�
		Receiver rc;
		getReceiver( &rc, 0, 0.02 );
		
		//��ȡ��Ϣ
		bool msg_available;
		ModeMsg msg;
		msg_available = ModeReceiveMsg( &msg, 0 );
		bool msg_handled = false;
		
		if( msg_available && msg.cmd==MAV_CMD_COMPONENT_ARM_DISARM )
		{	//����վ����
			if( msg.params[0] == 0 )
			{
				if( (msg.cmd_type & CMD_TYPE_MASK) == CMD_TYPE_MAVLINK )
				{
					Attitude_Control_Disable();
					set_mav_mode( 
							MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
							PX4_CUSTOM_MAIN_MODE_ALTCTL,
							0 );
					os_delay(1.0);
					
					uint8_t port_index = msg.cmd_type & CMD_TYPE_PORT_MASK;
					const Port* port = get_CommuPort( port_index );
					if( port->write )
					{
						mavlink_message_t msg_sd;
						if( mavlink_lock_chan( port_index, 0.01 ) )
						{
							mavlink_msg_command_ack_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								port_index ,
								&msg_sd,
								msg.cmd,	//command
								MAV_RESULT_ACCEPTED ,	//result
								100 ,	//progress
								0 ,	//param2
								msg.sd_sysid ,	//target system
								msg.sd_compid //target component
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(port_index);
						}
					}
				}
				return MR_OK;
			}
			msg_handled = 2;
		}
		if( get_CrashedState() )
		{	//�෭����
			Attitude_Control_Disable();
			return MR_Err;
		}
		

		uint8_t reqMode = cMode;
		if( msg_available && msg.cmd==176 )
		{	//ָ�����ģʽ
			if( (int)msg.params[0] & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED )
			{	//mavlink����ģʽ
				px4_custom_mode t_mav_mode;
				t_mav_mode.data = msg.params[1];
				if( t_mav_mode.main_mode==PX4_CUSTOM_MAIN_MODE_POSCTL )
				{	//ָ������ֶ�
					reqMode = AFunc_PosHold;
					msg_handled = 1;
				}
				else if( (t_mav_mode.main_mode==PX4_CUSTOM_MAIN_MODE_AUTO || t_mav_mode.main_mode==0) && t_mav_mode.sub_mode==PX4_CUSTOM_SUB_MODE_AUTO_MISSION )
				{	//ָ���������ģʽ
					reqMode = AFunc_Mission;
					msg_handled = 1;
				}
				else if( (t_mav_mode.main_mode==PX4_CUSTOM_MAIN_MODE_AUTO || t_mav_mode.main_mode==0) && t_mav_mode.sub_mode==PX4_CUSTOM_SUB_MODE_AUTO_RTL )
				{	//ָ����뷵��ģʽ
					reqMode = AFunc_RTL;
					msg_handled = 1;
				}
			}
		}
		if( rc.available )
		{	//���ջ�����
			
			uint8_t new_ModeButtonZone = get_RcButtonZone( rc.data[4], ModeButtonZone );
			if( ModeButtonZone<=5 && new_ModeButtonZone!=ModeButtonZone )
			{	//ģʽ��ť�ı����ģʽ
				reqMode = MFunc_cfg.Bt1AFunc1[8*new_ModeButtonZone];			
			}
			ModeButtonZone = new_ModeButtonZone;
			
			//ʹ��ң�������·���ģʽ
			bool sticks_in_neutral = 
				in_symmetry_range_mid( rc.data[0] , 50 , MFunc_cfg.NeutralZone[0] ) &&
				in_symmetry_range_mid( rc.data[1] , 50 , MFunc_cfg.NeutralZone[0] ) &&
				in_symmetry_range_mid( rc.data[2] , 50 , MFunc_cfg.NeutralZone[0] ) &&
				in_symmetry_range_mid( rc.data[3] , 50 , MFunc_cfg.NeutralZone[0] );
			if( !sticks_in_neutral )
			{	//ҡ��û���в������Զ�����
				if( is_AFunc_auto(cMode) )
				{
					if( is_AFunc_auto(MFunc_cfg.Bt1AFunc1[8*ModeButtonZone]) == false )
						reqMode = MFunc_cfg.Bt1AFunc1[8*ModeButtonZone];
					else
						reqMode = AFunc_PosHold;
					MissionButtonZone = RTLButtonZone = 255;
				}					
			}
			else
			{	//ҡ�˻��п�ִ���Զ�����	
				
				/*�ж�ִ������*/
					if( MFunc_cfg.MissionBt[0]>=2 && MFunc_cfg.MissionBt[0]<=4 )
					{	//��ť����ִ������
						if( rc.available_channels >= MFunc_cfg.MissionBt[0]+4 )
						{			
							//��ȡ��ť״̬
							double btn_value = rc.data[MFunc_cfg.MissionBt[0]-1+4];
							uint8_t new_MissionButtonZone = get_RcButtonZone( btn_value, MissionButtonZone );									
							if( new_MissionButtonZone!=MissionButtonZone )
							{	//��ť״̬�����仯
								if( new_MissionButtonZone>=4 )
									reqMode = AFunc_Mission;
								else
									reqMode = 0;
							}
							MissionButtonZone = new_MissionButtonZone;
						}
					}
					else if( MFunc_cfg.MissionBt[0]>=12 && MFunc_cfg.MissionBt[0]<=14 )
					{	//��ť�仯ִ������
						if( rc.available_channels >= MFunc_cfg.MissionBt[0]-10+4 )
						{
							//��ȡ��ť״̬
							double btn_value = rc.data[MFunc_cfg.MissionBt[0]-11+4];
							uint8_t new_MissionButtonZone = get_RcButtonZone( btn_value, MissionButtonZone );
							if( MissionButtonZone<=5 && new_MissionButtonZone!=MissionButtonZone )
							{	//��ť״̬�����仯
								if( cMode != AFunc_Mission )
									reqMode = AFunc_Mission;
								else
									reqMode = 0;
							}
							MissionButtonZone = new_MissionButtonZone;
						}
					}
				/*�ж�ִ������*/
				
				/*�жϷ���*/
					if( MFunc_cfg.RTLBt[0]>=2 && MFunc_cfg.RTLBt[0]<=4 )
					{	//��ť���·���
						if( rc.available_channels >= MFunc_cfg.RTLBt[0]+4 )
						{
							//��ȡ��ť״̬
							double btn_value = rc.data[MFunc_cfg.RTLBt[0]-1+4];
							uint8_t new_RTLButtonZone = get_RcButtonZone( btn_value, RTLButtonZone );		
							if( new_RTLButtonZone!=RTLButtonZone )
							{	//��ť״̬�����仯	
								if( new_RTLButtonZone>=4 )
									reqMode = AFunc_RTL;
								else
									reqMode = 0;
							}
							RTLButtonZone = new_RTLButtonZone;
						}
					}
					else if( MFunc_cfg.RTLBt[0]>=12 && MFunc_cfg.RTLBt[0]<=14 )
					{	//��ť�仯����
						if( rc.available_channels >= MFunc_cfg.RTLBt[0]-10+4 )
						{
							//��ȡ��ť״̬
							double btn_value = rc.data[MFunc_cfg.RTLBt[0]-11+4];
							uint8_t new_RTLButtonZone = get_RcButtonZone( btn_value, RTLButtonZone );	
							if( RTLButtonZone<=5 && new_RTLButtonZone!=RTLButtonZone )
							{	//��ť״̬�����仯
								if( cMode != AFunc_RTL )
									reqMode = AFunc_RTL;
								else
									reqMode = 0;
							}
							RTLButtonZone = new_RTLButtonZone;
						}
					}
				/*�жϷ���*/
					
				if( reqMode == 0 )
				{	//�а�ť�ɿ����¼�ⰴťλ��
					
					/*�ж�ִ������*/
						if( MFunc_cfg.MissionBt[0]>=2 && MFunc_cfg.MissionBt[0]<=4 )
						{	//��ť����ִ������
							if( MissionButtonZone>=4 )
								reqMode = AFunc_Mission;
						}
					/*�ж�ִ������*/
						
					/*�жϷ���*/
						if( MFunc_cfg.RTLBt[0]>=2 && MFunc_cfg.RTLBt[0]<=4 )
						{	//��ť���·���
							if( RTLButtonZone>=4 )
								reqMode = AFunc_RTL;
						}
					/*�жϷ���*/
						
					if( reqMode == 0 )
						reqMode = MFunc_cfg.Bt1AFunc1[8*ModeButtonZone];
				}
			}
		}
		else
		{	//���ջ�����������ң��״̬
			ModeButtonZone = MissionButtonZone = RTLButtonZone = 255;
			//��������Զ�ģʽ���л�������ģʽ
			if( is_AFunc_auto(cMode)==false )
				reqMode = AFunc_RTL;
		}
		
		if( is_AFunc_auto(reqMode) || is_AFunc_auto(cMode) )
		{	//�����Զ�ģʽ��λmode_swithced
			if( cMode != reqMode )
			{
				cMode = reqMode;
				mode_switched = true;
			}
		}
		else
			cMode = reqMode;
		#define swManualMode if( is_AFunc_auto(MFunc_cfg.Bt1AFunc1[8*ModeButtonZone]) == false )\
														reqMode = MFunc_cfg.Bt1AFunc1[8*ModeButtonZone];\
													else\
														reqMode = AFunc_PosHold;\
		
		if( cMode==AFunc_RTL )
		{	//���밲ȫģʽ����
RTL:
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
		}
		else if( cMode==AFunc_Mission )
		{	//����ģʽ
			Position_Control_Enable();
			bool pos_ena;
			is_Position_Control_Enabled(&pos_ena);
			if( pos_ena == false )
			{	//λ�ÿ������޷��򿪷����ֶ�ģʽ
				swManualMode
				goto Manual_Mode;
			}
			
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
					swManualMode
					goto Manual_Mode;
				}
			}
			
			if( mode_switched )
			{	//���л�������ģʽ
				//����ɲ���ȴ�			
				
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
					{	//�����������ģʽ
						mode_switched = false;
					}
				}
				else
					navInf.counter2 = 0;
			}
			else
			{	//�������				
				
				//�趨mavlinkģʽ
				set_mav_mode( 
					MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
					PX4_CUSTOM_MAIN_MODE_AUTO,
					PX4_CUSTOM_SUB_MODE_AUTO_MISSION );
				
				//����mission_ind״̬�жϵ�ǰ��Ҫִ��ʲô���ж���
				switch( mission_ind )
				{
					case 0:
					{	//���
						double params[7];
						params[0] = 0;
						params[3] = nan("");
						params[4] = 0;	params[5] = 0;
						params[6] = 0.5;
						int16_t res = Process_NavCmd( MAV_CMD_NAV_TAKEOFF, freq, MAV_FRAME_BODY_FLU, params, &navInf );
						if( NavCmdRs_SuccessOrFault(res) )
						{	//������
							init_NavCmdInf(&navInf);
							++mission_ind;
						}
						break;
					}
					
					case 1:
					{	//Ѳ��
//						double vec[3];
//						vec[0] = msg.cmd;
//						vec[1] = msg.params[0];
//						vec[2] = msg.params[1];
//						SDLog_Msg_DebugVect("xun",vec,3);
						
						Position_Control_set_ZLock();
						if(msg_available)
						{		
							if( msg.cmd == 2 )
							{
								if(msg.params[0]!=200)
								{
//									//roll��
//									Quaternion quat;
//									get_history_AirframeQuatY(&quat,0.1);
//									double roll = quat.getRoll();
//									//�Եظ߶�
//									double height = 100;
//									PosSensorHealthInf1 ZRange_inf;
//									if( get_OptimalRange_Z( &ZRange_inf ) )
//									{	//��ഫ��������
//										if( ZRange_inf.last_healthy_TIME.is_valid() && ZRange_inf.last_healthy_TIME.get_pass_time() < 50 )
//										{	//���50���ڽ���
//											//��ȡ�߶�
//											height = ZRange_inf.HOffset + ZRange_inf.PositionENU.z;
//										}
//									}
//									//��ǲ���
//									double
									
									//��ʱ��Ϊ���Ƕ����ȣ�
									double angle_err = -msg.params[0];
									double angle_err_rad = degree2rad(angle_err);
									//�����������Ϊ�����޵�λ��
									double pos_err = -msg.params[1];
									//����ƫ��ͷ��ֱ��
									Attitude_Control_set_Target_YawRate( degree2rad(constrain( angle_err, 70.0 ) ));
									//��Ѳ��ѹ�������ٶ�
									vector2<double> d_vel;
									d_vel.y = pos_err*cos(angle_err_rad);
									d_vel.x = pos_err*sin(-angle_err_rad);
									d_vel *= 2.5;
									d_vel.constrain(150);
									//��Ѳ��ǰ���ٶ�
									vector2<double> f_vel;
									f_vel.y = sin(angle_err_rad);
									f_vel.x = cos(angle_err_rad);
									f_vel *= constrain( ( 30 - fabs(angle_err) ), 3.0, 12.0 );
									Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( d_vel.x+f_vel.x, d_vel.y+f_vel.y , 0.05, 0.05);
								}
								if(msg.params[0] == 200)
								{
									Attitude_Control_set_YawLock();
									Position_Control_set_XYLock();
								}
							}
						}
						break;
					}
					
//					case 1:
//					{	//��ֱ��
//						double params[7];
//						params[0] = 2;
//						params[1] = 0;
//						params[2] = 0;
//						params[3] = nan("");
//						params[4] = 0.5;	params[5] = 0;
//						params[6] = 0.5;
//						int16_t res = Process_NavCmd( MAV_CMD_NAV_WAYPOINT, freq, MAV_FRAME_BODY_FLU, params, &navInf );
//						if( NavCmdRs_SuccessOrFault(res) )
//						{	//��ֱ�����
//							init_NavCmdInf(&navInf);
//							++mission_ind;
//						}
//						break;
//					}
					
					case 2:
					{	//��ֱ��
						double params[7];
						params[0] = 2;
						params[1] = 0;
						params[2] = 0;
						params[3] = nan("");
						params[4] = 0;	params[5] = 0.5;
						params[6] = 0;
						int16_t res = Process_NavCmd( MAV_CMD_NAV_WAYPOINT, freq, MAV_FRAME_BODY_FLU, params, &navInf );
						if( NavCmdRs_SuccessOrFault(res) )
						{	//��ֱ�����
							init_NavCmdInf(&navInf);
							++mission_ind;
						}
						break;
					}
					
					case 3:
					{	//��ֱ��
						double params[7];
						params[0] = 2;
						params[1] = 0;
						params[2] = 0;
						params[3] = nan("");
						params[4] = 0;	params[5] = 0.5;
						params[6] = 0;
						int16_t res = Process_NavCmd( MAV_CMD_NAV_WAYPOINT, freq, MAV_FRAME_BODY_FLU, params, &navInf );
						if( NavCmdRs_SuccessOrFault(res) )
						{	//��ֱ�����
							init_NavCmdInf(&navInf);
							++mission_ind;
						}
						break;
					}
					
					case 4:
					{	//��ֱ��
						double params[7];
						params[0] = 2;
						params[1] = 0;
						params[2] = 0;
						params[3] = nan("");
						params[4] = 0;	params[5] = 0.5;
						params[6] = -0.5;
						int16_t res = Process_NavCmd( MAV_CMD_NAV_WAYPOINT, freq, MAV_FRAME_BODY_FLU, params, &navInf );
						if( NavCmdRs_SuccessOrFault(res) )
						{	//��ֱ�����
							init_NavCmdInf(&navInf);
							++mission_ind;
						}
						break;
					}
					
					case 5:
					{	//����
						Position_Control_set_XYLock();
						Position_Control_set_TargetVelocityZ(-50);
						break;
					}
					
					default:
					{
						swManualMode
						mission_ind = 0;
						goto Manual_Mode;
						break;
					}
				}
			}
		}
		else
		{	//�ֶ�����ģʽ���������߶�����ƣ�
			Manual_Mode:
			if( rc.available )
			{				
				/*�ж��˳�ģʽ*/
					//��ȡ����״̬
					bool inFlight;
					get_is_inFlight(&inFlight);
					if( rc.data[0] > 30 )
					{
						exit_mode_counter_rs = 480;
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
					//����ǿ�Ƽ���
					if( rc.data[0] < 10 && rc.data[1] < 10 && rc.data[2] < 10 && rc.data[3] > 90 )
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
				
				/*�ж�ģʽ*/
					uint8_t MF_mode = 0;
					if( rc.data[4]<1.0/6*100 )
						MF_mode = MFunc_cfg.Bt1AFunc1[0];
					else if( rc.data[4]<2.0/6*100 )
						MF_mode = MFunc_cfg.Bt1AFunc2[0];
					else if( rc.data[4]<3.0/6*100 )
						MF_mode = MFunc_cfg.Bt1AFunc3[0];
					else if( rc.data[4]<4.0/6*100 )
						MF_mode = MFunc_cfg.Bt1AFunc4[0];
					else if( rc.data[4]<5.0/6*100 )
						MF_mode = MFunc_cfg.Bt1AFunc5[0];
					else if( rc.data[4]<6.0/6*100 )
						MF_mode = MFunc_cfg.Bt1AFunc6[0];
				/*�ж�ģʽ*/
					
				//�л����߶���
				if( MF_mode==2 )
					Position_Control_Enable();
				else if( MF_mode==1 )
					Position_Control_Disable();
				bool pos_ena;
				is_Position_Control_Enabled(&pos_ena);
					
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
					set_mav_mode( 
						MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
						PX4_CUSTOM_MAIN_MODE_POSCTL,
						0 );
					
					//��������˿�ˮƽ�ٶ�
					if( in_symmetry_range_mid( rc.data[3] , 50 , 5 ) && in_symmetry_range_mid( rc.data[2] , 50 , 5 ) )
						Position_Control_set_XYLock();
					else
					{
						double RPCtrlScale = atan2( get_maxAccXY() / 50.0, GravityAcc );
						double XYCtrlScale = get_maxVelXY() / 50.0;						
						double roll_sitck_d = remove_deadband( rc.data[3] - 50.0, 5.0 );
						double pitch_sitck_d = remove_deadband( rc.data[2] - 50.0, 5.0 );
						vector3<double> velocityFLU;
						get_VelocityFLU_Ctrl(&velocityFLU);
						double vel_stick_err_pitch = velocityFLU.x/XYCtrlScale - pitch_sitck_d;
						double vel_stick_err_roll = velocityFLU.y/XYCtrlScale - -roll_sitck_d;
						Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( \
							pitch_sitck_d * XYCtrlScale ,\
							-roll_sitck_d * XYCtrlScale , \
							fabs( vel_stick_err_roll  )*RPCtrlScale, \
							fabs( vel_stick_err_pitch )*RPCtrlScale \
						);
					}
				}
				else
				{
					//�趨mavlinkģʽ
					set_mav_mode( 
						MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
						PX4_CUSTOM_MAIN_MODE_ALTCTL,
						0 );
					
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
			else
			{	//��ң���źŽ��밲ȫģʽ
				change_Mode(AFunc_RTL)
				goto RTL;				
			}
		}
	}
	return MR_OK;
}