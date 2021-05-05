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
	//任务索引
	const uint8_t mavlink_control = 0;
	const uint8_t custom_nav = 1;
	const uint8_t altitude_adjust = 2;
	const uint8_t yaw_adjust = 3;
	const uint8_t land = 4;

	//数据记录
	ModeMsg last_nav_takeoff_local_msg;
	ModeMsg last_user1_msg;
	ModeMsg last_condition_change_alt_msg;
	ModeMsg last_condition_yaw_msg;
	ModeMsg last_nav_land_local_msg;
	
	//altitude_adjust控制指令标志：0为起飞，1为调整高度
	uint8_t altitude_adjust_flag;

	//MAV_CMD_USER_1延时计数
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
	
	//读取模式配置
	ModeFuncCfg MFunc_cfg;
	ReadParamGroup( "MFunc", (uint64_t*)&MFunc_cfg, 0 );
	
	//任务模式
	bool mode_switched = true;
	#define change_Mode(x) {cMode=x; mode_switched = true;}
	uint16_t current_mission_ind;
	uint8_t ModeButtonZone = 255;
	uint8_t MissionButtonZone = 255;
	uint8_t RTLButtonZone = 255;
	uint8_t cMode = AFunc_PosHold;
	
	//当前执行任务的序号
	uint16_t mission_ind = 0;
	//任务状态机
	NavCmdInf navInf;
	init_NavCmdInf(&navInf);
	while(1)
	{
		os_delay(0.02);
		
		if( get_CrashedState() )
		{	//侧翻加锁
			Attitude_Control_Disable();
			return MR_Err;
		}
		
		//获取接收机
		Receiver rc;
		getReceiver( &rc, 0, 0.02 );
		
		//获取消息
		bool msg_available;
		ModeMsg msg;
		msg_available = ModeReceiveMsg( &msg, 0 );
		bool msg_handled = false;
		
		if( msg_available && msg.cmd==MAV_CMD_COMPONENT_ARM_DISARM )
		{	//地面站加锁
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
		{	//侧翻加锁
			Attitude_Control_Disable();
			return MR_Err;
		}
		

		uint8_t reqMode = cMode;
		if( msg_available && msg.cmd==176 )
		{	//指令更改模式
			if( (int)msg.params[0] & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED )
			{	//mavlink定义模式
				px4_custom_mode t_mav_mode;
				t_mav_mode.data = msg.params[1];
				if( t_mav_mode.main_mode==PX4_CUSTOM_MAIN_MODE_POSCTL )
				{	//指令进入手动
					reqMode = AFunc_PosHold;
					msg_handled = 1;
				}
				else if( (t_mav_mode.main_mode==PX4_CUSTOM_MAIN_MODE_AUTO || t_mav_mode.main_mode==0) && t_mav_mode.sub_mode==PX4_CUSTOM_SUB_MODE_AUTO_MISSION )
				{	//指令进入任务模式
					reqMode = AFunc_Mission;
					msg_handled = 1;
				}
				else if( (t_mav_mode.main_mode==PX4_CUSTOM_MAIN_MODE_AUTO || t_mav_mode.main_mode==0) && t_mav_mode.sub_mode==PX4_CUSTOM_SUB_MODE_AUTO_RTL )
				{	//指令进入返航模式
					reqMode = AFunc_RTL;
					msg_handled = 1;
				}
			}
		}
		if( rc.available )
		{	//接收机可用
			
			uint8_t new_ModeButtonZone = get_RcButtonZone( rc.data[4], ModeButtonZone );
			if( ModeButtonZone<=5 && new_ModeButtonZone!=ModeButtonZone )
			{	//模式按钮改变更改模式
				reqMode = MFunc_cfg.Bt1AFunc1[8*new_ModeButtonZone];			
			}
			ModeButtonZone = new_ModeButtonZone;
			
			//使用遥控器更新飞行模式
			bool sticks_in_neutral = 
				in_symmetry_range_mid( rc.data[0] , 50 , MFunc_cfg.NeutralZone[0] ) &&
				in_symmetry_range_mid( rc.data[1] , 50 , MFunc_cfg.NeutralZone[0] ) &&
				in_symmetry_range_mid( rc.data[2] , 50 , MFunc_cfg.NeutralZone[0] ) &&
				in_symmetry_range_mid( rc.data[3] , 50 , MFunc_cfg.NeutralZone[0] );
			if( !sticks_in_neutral )
			{	//摇杆没回中不允许自动操作
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
			{	//摇杆回中可执行自动操作	
				
				/*判断执行任务*/
					if( MFunc_cfg.MissionBt[0]>=2 && MFunc_cfg.MissionBt[0]<=4 )
					{	//按钮按下执行任务
						if( rc.available_channels >= MFunc_cfg.MissionBt[0]+4 )
						{			
							//获取按钮状态
							double btn_value = rc.data[MFunc_cfg.MissionBt[0]-1+4];
							uint8_t new_MissionButtonZone = get_RcButtonZone( btn_value, MissionButtonZone );									
							if( new_MissionButtonZone!=MissionButtonZone )
							{	//按钮状态发生变化
								if( new_MissionButtonZone>=4 )
									reqMode = AFunc_Mission;
								else
									reqMode = 0;
							}
							MissionButtonZone = new_MissionButtonZone;
						}
					}
					else if( MFunc_cfg.MissionBt[0]>=12 && MFunc_cfg.MissionBt[0]<=14 )
					{	//按钮变化执行任务
						if( rc.available_channels >= MFunc_cfg.MissionBt[0]-10+4 )
						{
							//获取按钮状态
							double btn_value = rc.data[MFunc_cfg.MissionBt[0]-11+4];
							uint8_t new_MissionButtonZone = get_RcButtonZone( btn_value, MissionButtonZone );
							if( MissionButtonZone<=5 && new_MissionButtonZone!=MissionButtonZone )
							{	//按钮状态发生变化
								if( cMode != AFunc_Mission )
									reqMode = AFunc_Mission;
								else
									reqMode = 0;
							}
							MissionButtonZone = new_MissionButtonZone;
						}
					}
				/*判断执行任务*/
				
				/*判断返航*/
					if( MFunc_cfg.RTLBt[0]>=2 && MFunc_cfg.RTLBt[0]<=4 )
					{	//按钮按下返航
						if( rc.available_channels >= MFunc_cfg.RTLBt[0]+4 )
						{
							//获取按钮状态
							double btn_value = rc.data[MFunc_cfg.RTLBt[0]-1+4];
							uint8_t new_RTLButtonZone = get_RcButtonZone( btn_value, RTLButtonZone );		
							if( new_RTLButtonZone!=RTLButtonZone )
							{	//按钮状态发生变化	
								if( new_RTLButtonZone>=4 )
									reqMode = AFunc_RTL;
								else
									reqMode = 0;
							}
							RTLButtonZone = new_RTLButtonZone;
						}
					}
					else if( MFunc_cfg.RTLBt[0]>=12 && MFunc_cfg.RTLBt[0]<=14 )
					{	//按钮变化返航
						if( rc.available_channels >= MFunc_cfg.RTLBt[0]-10+4 )
						{
							//获取按钮状态
							double btn_value = rc.data[MFunc_cfg.RTLBt[0]-11+4];
							uint8_t new_RTLButtonZone = get_RcButtonZone( btn_value, RTLButtonZone );	
							if( RTLButtonZone<=5 && new_RTLButtonZone!=RTLButtonZone )
							{	//按钮状态发生变化
								if( cMode != AFunc_RTL )
									reqMode = AFunc_RTL;
								else
									reqMode = 0;
							}
							RTLButtonZone = new_RTLButtonZone;
						}
					}
				/*判断返航*/
					
				if( reqMode == 0 )
				{	//有按钮松开重新检测按钮位置
					
					/*判断执行任务*/
						if( MFunc_cfg.MissionBt[0]>=2 && MFunc_cfg.MissionBt[0]<=4 )
						{	//按钮按下执行任务
							if( MissionButtonZone>=4 )
								reqMode = AFunc_Mission;
						}
					/*判断执行任务*/
						
					/*判断返航*/
						if( MFunc_cfg.RTLBt[0]>=2 && MFunc_cfg.RTLBt[0]<=4 )
						{	//按钮按下返航
							if( RTLButtonZone>=4 )
								reqMode = AFunc_RTL;
						}
					/*判断返航*/
						
					if( reqMode == 0 )
						reqMode = MFunc_cfg.Bt1AFunc1[8*ModeButtonZone];
				}
			}
		}
		else
		{	//接收机不可用重置遥控状态
			ModeButtonZone = MissionButtonZone = RTLButtonZone = 255;
			//如果不是自动模式则切换到返航模式
			if( is_AFunc_auto(cMode)==false )
				reqMode = AFunc_RTL;
		}
		
		if( is_AFunc_auto(reqMode) || is_AFunc_auto(cMode) )
		{	//进出自动模式置位mode_swithced
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
		{	//进入安全模式返航
RTL:
			enter_MSafe(true);
			/*判断退出模式*/
				bool inFlight;
				get_is_inFlight(&inFlight);
				if( inFlight==false )
				{
					Attitude_Control_Disable();
					return MR_OK;
				}
			/*判断退出模式*/
		}
		else if( cMode==AFunc_Mission )
		{	//任务模式
			Position_Control_Enable();
			bool pos_ena;
			is_Position_Control_Enabled(&pos_ena);
			if( pos_ena == false )
			{	//位置控制器无法打开返回手动模式
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
				{	//摇杆不在中间返回手动模式
					init_NavCmdInf(&navInf);
					swManualMode
					goto Manual_Mode;
				}
			}
			
			if( mode_switched )
			{	//刚切换到任务模式
				//首先刹车等待			
				
				Position_Control_set_XYLock();
				Position_Control_set_ZLock();
				
				//等待刹车完成
				Position_ControlMode alt_mode, pos_mode;
				get_Altitude_ControlMode(&alt_mode);
				get_Position_ControlMode(&pos_mode);
				if( alt_mode==Position_ControlMode_Position && pos_mode==Position_ControlMode_Position )
				{	//刹车完成
					++navInf.counter2;
					//等待1秒再进入任务飞行
					if( navInf.counter2 >= 1*freq )
					{	//进入任务飞行模式
						mode_switched = false;
					}
				}
				else
					navInf.counter2 = 0;
			}
			else
			{	//任务飞行				
				
				//设定mavlink模式
				set_mav_mode( 
					MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
					PX4_CUSTOM_MAIN_MODE_AUTO,
					PX4_CUSTOM_SUB_MODE_AUTO_MISSION );
				
				//根据mission_ind状态判断当前需要执行什么飞行动作
				switch( mission_ind )
				{
					case mavlink_control:
					{
						if(msg_available)
						{
								bool inFlight;
								get_is_inFlight(&inFlight);
							
								//判断滞空
								if( inFlight==false )
								{
									//未滞空
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
										
										//mavlink反馈
										mavlink_send_command_ack(msg, MAV_RESULT_IN_PROGRESS, 0, 0);
										
										//设置Z轴默认速度
										if (msg.params[2] != 0)
											Position_Control_set_ZAutoSpeed(msg.params[2] * 100, msg.params[2] * 100);
										
										//控制相对高度
										Position_Control_Takeoff_HeightRelative(msg.params[6]*100);
										
										//记录数据，用于结束反馈
										last_nav_takeoff_local_msg = msg;
										altitude_adjust_flag = 0;
										
										//转到高度调整任务
										mission_ind = altitude_adjust;
									}
								}
								else
								{
									//滞空
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
										
										//任务转换
										if (msg.params[6] != 0)
										{
											//刹车
											Position_Control_set_XYLock();
											Attitude_Control_set_YawLock();
											Position_Control_set_ZLock();
											
											//mavlink反馈
											mavlink_send_command_ack(msg, MAV_RESULT_ACCEPTED, 0, 0);
										}
										else
										{
											//mavlink反馈
											mavlink_send_command_ack(msg, MAV_RESULT_IN_PROGRESS, 0, 0);
											
											//设置XY轴线速度与roll、pitch角度限制
											Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit(msg.params[0]*100,
																																									msg.params[1]*100,
																																									msg.params[4],
																																									msg.params[5]);
											
											//设置Z轴角速度
											Attitude_Control_set_Target_YawRate(msg.params[3]);
											
											//导航暂时为2D
											Position_Control_set_ZLock();
											
											//记录数据，保证控制连续，并用于结束反馈
											last_user1_msg = msg;

											//转到导航任务
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
										
										//mavlink反馈
										mavlink_send_command_ack(msg, MAV_RESULT_IN_PROGRESS, 0, 0);
										
										//设置目标高度(相对)
										if (msg.params[0] != 0)
											//速度无方向
											Position_Control_set_TargetPositionZRelative(msg.params[6]*100, msg.params[0]*100);
										else
											Position_Control_set_TargetPositionZRelative(msg.params[6]*100);
										
										//记录数据，用于结束反馈
										last_condition_change_alt_msg = msg;
										altitude_adjust_flag = 1;
										
										//转到调整高度任务
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
										
										//mavlink反馈
										mavlink_send_command_ack(msg, MAV_RESULT_IN_PROGRESS, 0, 0);
										
										//A9不提供偏航角速度自动控制接口
										//根据mavlink信息决定控制角度类型
										if (msg.params[3])
											Attitude_Control_set_Target_YawRelative(msg.params[0]);
										else
											Attitude_Control_set_Target_Yaw(msg.params[0]);
										
										//记录数据，用于结束反馈
										last_condition_yaw_msg = msg;
										
										//转到调整偏航任务
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
										
										//mavlink反馈
										mavlink_send_command_ack(msg, MAV_RESULT_IN_PROGRESS, 0, 0);
										
										//记录数据，用于结束反馈
										last_nav_land_local_msg = msg;
										
										//转到降落任务
										mission_ind = land;
									}
								}
						}
						else
						{
							//无指令控制则刹车
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
									
									//任务转换
									if (msg.params[6] != 0)
									{
										//刹车
										Position_Control_set_XYLock();
										Attitude_Control_set_YawLock();
										Position_Control_set_ZLock();

										//转到mavlink控制任务
										mission_ind = mavlink_control;
										
										//mavlink反馈
										mavlink_send_command_ack(msg, MAV_RESULT_ACCEPTED, 0, 0);
									}
									else
									{
										//更新数据
										//设置XY轴线速度与roll、pitch角度限制
										Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit(msg.params[0]*100,
																																								msg.params[1]*100,
																																								msg.params[4],
																																								msg.params[5]);
										
										//设置Z轴角速度
										Attitude_Control_set_Target_YawRate(msg.params[3]);
										
										//导航暂时为2D
										Position_Control_set_ZLock();
										
										//记录数据，保证控制连续
										last_user1_msg = msg;
									}
									
									//清空计数
									delay_counter = 0;
								}
						}
						else if(delay_counter < max_delay_counter)
						{
							//保持连续
							//设置XY轴线速度与roll、pitch角度限制
							Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit(last_user1_msg.params[0]*100,
																																					last_user1_msg.params[1]*100,
																																					last_user1_msg.params[4],
																																					last_user1_msg.params[5]);
							
							//设置Z轴角速度
							Attitude_Control_set_Target_YawRate(last_user1_msg.params[3]);
							
							//导航暂时为2D
							Position_Control_set_ZLock();
							
							//延时计数
							delay_counter++;
						}
						else
						{
							//超时，任务转换，刹车
							Position_Control_set_XYLock();
							Attitude_Control_set_YawLock();
							Position_Control_set_ZLock();

							//转到mavlink控制任务
							mission_ind = mavlink_control;

							//清空计数
							delay_counter = 0;
							
							//mavlink反馈
							mavlink_send_command_ack(last_user1_msg, MAV_RESULT_ACCEPTED, 0, 0);
						}
						
						break;
					}
					
					case altitude_adjust:
					{
						//除控制方向其余锁定
						Position_Control_set_XYLock();
						Attitude_Control_set_YawLock();
						
						//访问控制方向控制器
						Position_ControlMode alt_mode;
						get_Altitude_ControlMode(&alt_mode);
						if( alt_mode == Position_ControlMode_Position )
						{
							//转到mavlink控制任务
							mission_ind = mavlink_control;
							
							//mavlink反馈
							if(altitude_adjust_flag == 0)
								mavlink_send_command_ack(last_nav_takeoff_local_msg, MAV_RESULT_ACCEPTED, 0, 0);
							else
								mavlink_send_command_ack(last_condition_change_alt_msg, MAV_RESULT_ACCEPTED, 0, 0);
						}
						
						break;
					}
					
					case yaw_adjust:
					{
						//除控制方向其余锁定
						Position_Control_set_XYLock();
						Position_Control_set_ZLock();
						
						//访问控制方向控制器
						double yaw_err;
						Attitude_Control_get_YawTrackErr(&yaw_err);
						if( yaw_err <= 0.01 )
						{
							//回到mavlink控制模式
							mission_ind = mavlink_control;
							
							//mavlink反馈
							mavlink_send_command_ack(last_condition_yaw_msg, MAV_RESULT_ACCEPTED, 0, 0);
						}
						
						break;
					}
					
					case land:
					{
						//除控制方向其余锁定
						Position_Control_set_XYLock();
						Attitude_Control_set_YawLock();
						
						//设置降落速度
						if(last_nav_land_local_msg.params[2] != 0)
							Position_Control_set_TargetVelocityZ(last_nav_land_local_msg.params[2]*100);
						else
							Position_Control_set_TargetVelocityZ(-40);
						
						//判断滞空
						bool inFlight;
						get_is_inFlight(&inFlight);
						if( inFlight==false )
						{
							//关闭角度控制器
							Attitude_Control_Disable();
							
							//回到mavlink控制模式
							mission_ind = mavlink_control;
							
							//mavlink反馈
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
		{	//手动飞行模式（包含定高定点控制）
			Manual_Mode:
			if( rc.available )
			{				
				/*判断退出模式*/
					//获取飞行状态
					bool inFlight;
					get_is_inFlight(&inFlight);
					if( rc.data[0] > 30 )
					{
						exit_mode_counter_rs = 480;
						if( exit_mode_counter < exit_mode_counter_rs )
							exit_mode_counter = exit_mode_counter_rs;
					}
					//降落后自动加锁
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
					//手势强制加锁
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
				/*判断退出模式*/
				
				/*判断模式*/
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
				/*判断模式*/
					
				//切换定高定点
				if( MF_mode==2 )
					Position_Control_Enable();
				else if( MF_mode==1 )
					Position_Control_Disable();
				bool pos_ena;
				is_Position_Control_Enabled(&pos_ena);
					
				//油门杆控制垂直速度
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
					//设定mavlink模式
					set_mav_mode( 
						MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
						PX4_CUSTOM_MAIN_MODE_POSCTL,
						0 );
					
					//俯仰横滚杆控水平速度
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
					//设定mavlink模式
					set_mav_mode( 
						MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
						PX4_CUSTOM_MAIN_MODE_ALTCTL,
						0 );
					
					//补偿风力扰动
					vector3<double> WindDisturbance;
					get_WindDisturbance( &WindDisturbance );
					Quaternion attitude;
					get_Attitude_quat(&attitude);
					double yaw = attitude.getYaw();		
					double sin_Yaw, cos_Yaw;
					fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
					double WindDisturbance_Bodyheading_x = ENU2BodyHeading_x( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
					double WindDisturbance_Bodyheading_y = ENU2BodyHeading_y( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
					//俯仰横滚杆控俯仰横滚
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
				
				//偏航杆在中间锁偏航
				//不在中间控制偏航速度
				double YCtrlScale = degree2rad( get_maxYawSpeed() / 50.0 );
				if( in_symmetry_range_mid( rc.data[1] , 50 , 5 ) )
					Attitude_Control_set_YawLock();
				else
					Attitude_Control_set_Target_YawRate( ( 50 - rc.data[1] )*YCtrlScale );
			}
			else
			{	//无遥控信号进入安全模式
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
