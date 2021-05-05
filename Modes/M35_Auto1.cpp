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

void mavlink_send_command_ack(ModeMsg msg,
															uint8_t result,
															uint8_t progress,
															int32_t result_param2);

M35_Auto1::M35_Auto1():Mode_Base( "Auto1", 35 )
{
	
}

ModeResult M35_Auto1::main_func( void* param1, uint32_t param2 )
{
	//��������
	const uint8_t mavlink_control = 0;
	const uint8_t custom_nav = 1;
	const uint8_t altitude_adjust = 2;
	const uint8_t yaw_adjust = 3;
	const uint8_t land = 4;

	//���ݼ�¼
	ModeMsg last_nav_takeoff_local_msg;
	ModeMsg last_user1_msg;
	ModeMsg last_condition_change_alt_msg;
	ModeMsg last_condition_yaw_msg;
	ModeMsg last_nav_land_local_msg;
	
	//altitude_adjust����ָ���־��0Ϊ��ɣ�1Ϊ�����߶�
	uint8_t altitude_adjust_flag;

	//MAV_CMD_USER_1��ʱ����
	const uint8_t max_delay_counter = 25;
	uint8_t delay_counter = 0;
	
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
					case mavlink_control:
					{
						if(msg_available)
						{
								bool inFlight;
								get_is_inFlight(&inFlight);
							
								//�ж��Ϳ�
								if( inFlight==false )
								{
									//δ�Ϳ�
									if(msg.cmd == MAV_CMD_NAV_TAKEOFF_LOCAL)
									{
										/*
										* MAV_CMD_NAV_TAKEOFF_LOCAL:
										* params[0]:Minimum pitch (if airspeed sensor present), desired pitch without sensor
										* params[1]:Empty
										* params[2]:Takeoff ascend rate
										* params[3]:Yaw angle (if magnetometer or another yaw estimation source present), 
													ignored without one of these
										* params[4]:Y-axis position
										* params[5]:X-axis position
										* params[6]:Z-axis position
										*/
										
										//mavlink����
										mavlink_send_command_ack(msg, MAV_RESULT_IN_PROGRESS, 0, 0);
										
										//����Z��Ĭ���ٶ�
										if (msg.params[2] != 0)
											Position_Control_set_ZAutoSpeed(msg.params[2] * 100, msg.params[2] * 100);
										
										//������Ը߶�
										Position_Control_Takeoff_HeightRelative(msg.params[6]*100);
										
										//��¼���ݣ����ڽ�������
										last_nav_takeoff_local_msg = msg;
										altitude_adjust_flag = 0;
										
										//ת���߶ȵ�������
										mission_ind = altitude_adjust;
									}
								}
								else
								{
									//�Ϳ�
									if(msg.cmd == MAV_CMD_USER_1)
									{
										/*
										* MAV_CMD_USER_1:
										* params[0]:X-axis linear velocity
										* params[1]:Y-axis linear velocity
										* params[2]:Z-axis linear velocity
										* params[3]:Z-axis angular velocity
										* params[4]:Maximum roll
										* params[5]:Maximum pitch
										* params[6]:Mission conversion
										*/
										
										//����ת��
										if (msg.params[6] != 0)
										{
											//ɲ��
											Position_Control_set_XYLock();
											Attitude_Control_set_YawLock();
											Position_Control_set_ZLock();
											
											//mavlink����
											mavlink_send_command_ack(msg, MAV_RESULT_ACCEPTED, 0, 0);
										}
										else
										{
											//mavlink����
											mavlink_send_command_ack(msg, MAV_RESULT_IN_PROGRESS, 0, 0);
											
											//����XY�����ٶ���roll��pitch�Ƕ�����
											Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit(msg.params[0]*100,
																																									msg.params[1]*100,
																																									msg.params[4],
																																									msg.params[5]);
											
											//����Z����ٶ�
											Attitude_Control_set_Target_YawRate(msg.params[3]);
											
											//������ʱΪ2D
											Position_Control_set_ZLock();
											
											//��¼���ݣ���֤���������������ڽ�������
											last_user1_msg = msg;

											//ת����������
											mission_ind = custom_nav;
										}
									}
									
									else if(msg.cmd == MAV_CMD_CONDITION_CHANGE_ALT)
									{
										/*
										* MAV_CMD_CONDITION_CHANGE_ALT:
										* params[0]:Descent/Ascend rate
										* params[1]:Empty
										* params[2]:Empty
										* params[3]:Empty
										* params[4]:Empty
										* params[5]:Empty
										* params[6]:Target altitude
										*/
										
										//mavlink����
										mavlink_send_command_ack(msg, MAV_RESULT_IN_PROGRESS, 0, 0);
										
										//����Ŀ��߶�(���)
										if (msg.params[0] != 0)
											//�ٶ��޷���
											Position_Control_set_TargetPositionZRelative(msg.params[6]*100, msg.params[0]*100);
										else
											Position_Control_set_TargetPositionZRelative(msg.params[6]*100);
										
										//��¼���ݣ����ڽ�������
										last_condition_change_alt_msg = msg;
										altitude_adjust_flag = 1;
										
										//ת�������߶�����
										mission_ind = altitude_adjust;
									}
									
									else if(msg.cmd == MAV_CMD_CONDITION_YAW)
									{
										/*
										* MAV_CMD_CONDITION_YAW:
										* params[0]:Target angle, 0 is east
										* params[1]:Angular speed
										* params[2]:Direction: -1: counter clockwise, 1: clockwise
										* params[3]:0: absolute angle, 1: relative offset
										* params[4]:Empty
										* params[5]:Empty
										* params[6]:Empty
										*/
										
										//mavlink����
										mavlink_send_command_ack(msg, MAV_RESULT_IN_PROGRESS, 0, 0);
										
										//A9���ṩƫ�����ٶ��Զ����ƽӿ�
										//����mavlink��Ϣ�������ƽǶ�����
										if (msg.params[3])
											Attitude_Control_set_Target_YawRelative(msg.params[0]);
										else
											Attitude_Control_set_Target_Yaw(msg.params[0]);
										
										//��¼���ݣ����ڽ�������
										last_condition_yaw_msg = msg;
										
										//ת������ƫ������
										mission_ind = yaw_adjust;
									}
									
									else if(msg.cmd == MAV_CMD_NAV_LAND_LOCAL)
									{
										/*
										* MAV_CMD_NAV_LAND_LOCAL:
										* params[0]:Landing target number (if available)
										* params[1]:Maximum accepted offset from desired landing position - computed magnitude
													from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the
													maximum accepted distance between the desired landing position and the 
													position where the vehicle is about to land
										* params[2]:Landing descend rate
										* params[3]:Desired yaw angle
										* params[4]:Y-axis position
										* params[5]:X-axis position
										* params[6]:Z-axis / ground level position
										*/
										
										//mavlink����
										mavlink_send_command_ack(msg, MAV_RESULT_IN_PROGRESS, 0, 0);
										
										//��¼���ݣ����ڽ�������
										last_nav_land_local_msg = msg;
										
										//ת����������
										mission_ind = land;
									}
								}
						}
						else
						{
							//��ָ�������ɲ��
							Position_Control_set_XYLock();
							Attitude_Control_set_YawLock();
							Position_Control_set_ZLock();
						}
						
						break;
					}
					
					case custom_nav:
					{
						if(msg_available)
						{
								if(msg.cmd == MAV_CMD_USER_1)
								{
									/*
									* MAV_CMD_USER_1:
									* params[0]:X-axis linear velocity
									* params[1]:Y-axis linear velocity
									* params[2]:Z-axis linear velocity
									* params[3]:Z-axis angular velocity
									* params[4]:Maximum roll
									* params[5]:Maximum pitch
									* params[6]:Mission conversion
									*/
									
									//����ת��
									if (msg.params[6] != 0)
									{
										//ɲ��
										Position_Control_set_XYLock();
										Attitude_Control_set_YawLock();
										Position_Control_set_ZLock();

										//ת��mavlink��������
										mission_ind = mavlink_control;
										
										//mavlink����
										mavlink_send_command_ack(msg, MAV_RESULT_ACCEPTED, 0, 0);
									}
									else
									{
										//��������
										//����XY�����ٶ���roll��pitch�Ƕ�����
										Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit(msg.params[0]*100,
																																								msg.params[1]*100,
																																								msg.params[4],
																																								msg.params[5]);
										
										//����Z����ٶ�
										Attitude_Control_set_Target_YawRate(msg.params[3]);
										
										//������ʱΪ2D
										Position_Control_set_ZLock();
										
										//��¼���ݣ���֤��������
										last_user1_msg = msg;
									}
									
									//��ռ���
									delay_counter = 0;
								}
						}
						else if(delay_counter < max_delay_counter)
						{
							//��������
							//����XY�����ٶ���roll��pitch�Ƕ�����
							Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit(last_user1_msg.params[0]*100,
																																					last_user1_msg.params[1]*100,
																																					last_user1_msg.params[4],
																																					last_user1_msg.params[5]);
							
							//����Z����ٶ�
							Attitude_Control_set_Target_YawRate(last_user1_msg.params[3]);
							
							//������ʱΪ2D
							Position_Control_set_ZLock();
							
							//��ʱ����
							delay_counter++;
						}
						else
						{
							//��ʱ������ת����ɲ��
							Position_Control_set_XYLock();
							Attitude_Control_set_YawLock();
							Position_Control_set_ZLock();

							//ת��mavlink��������
							mission_ind = mavlink_control;

							//��ռ���
							delay_counter = 0;
							
							//mavlink����
							mavlink_send_command_ack(last_user1_msg, MAV_RESULT_ACCEPTED, 0, 0);
						}
						
						break;
					}
					
					case altitude_adjust:
					{
						//�����Ʒ�����������
						Position_Control_set_XYLock();
						Attitude_Control_set_YawLock();
						
						//���ʿ��Ʒ��������
						Position_ControlMode alt_mode;
						get_Altitude_ControlMode(&alt_mode);
						if( alt_mode == Position_ControlMode_Position )
						{
							//ת��mavlink��������
							mission_ind = mavlink_control;
							
							//mavlink����
							if(altitude_adjust_flag == 0)
								mavlink_send_command_ack(last_nav_takeoff_local_msg, MAV_RESULT_ACCEPTED, 0, 0);
							else
								mavlink_send_command_ack(last_condition_change_alt_msg, MAV_RESULT_ACCEPTED, 0, 0);
						}
						
						break;
					}
					
					case yaw_adjust:
					{
						//�����Ʒ�����������
						Position_Control_set_XYLock();
						Position_Control_set_ZLock();
						
						//���ʿ��Ʒ��������
						double yaw_err;
						Attitude_Control_get_YawTrackErr(&yaw_err);
						if( yaw_err <= 0.01 )
						{
							//�ص�mavlink����ģʽ
							mission_ind = mavlink_control;
							
							//mavlink����
							mavlink_send_command_ack(last_condition_yaw_msg, MAV_RESULT_ACCEPTED, 0, 0);
						}
						
						break;
					}
					
					case land:
					{
						//�����Ʒ�����������
						Position_Control_set_XYLock();
						Attitude_Control_set_YawLock();
						
						//���ý����ٶ�
						if(last_nav_land_local_msg.params[2] != 0)
							Position_Control_set_TargetVelocityZ(last_nav_land_local_msg.params[2]*100);
						else
							Position_Control_set_TargetVelocityZ(-40);
						
						//�ж��Ϳ�
						bool inFlight;
						get_is_inFlight(&inFlight);
						if( inFlight==false )
						{
							//�رսǶȿ�����
							Attitude_Control_Disable();
							
							//�ص�mavlink����ģʽ
							mission_ind = mavlink_control;
							
							//mavlink����
							mavlink_send_command_ack(last_nav_land_local_msg, MAV_RESULT_ACCEPTED, 0, 0);
						}
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

void mavlink_send_command_ack(ModeMsg msg,
															uint8_t result,
															uint8_t progress,
															int32_t result_param2)
{
	uint8_t port_index = msg.cmd_type & CMD_TYPE_PORT_MASK;
	const Port* port = get_CommuPort( port_index );
	if( (msg.cmd_type & CMD_TYPE_MASK) == CMD_TYPE_MAVLINK && port->write )
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
				result ,	//result
				progress ,	//progress
				result_param2 ,	//param2
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
