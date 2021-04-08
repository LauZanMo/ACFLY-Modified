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
#include "ctrl_Main.hpp"
M32_PosCtrl::M32_PosCtrl():Mode_Base( "PosCtrl", 32 )
{
	
}

void M32_PosCtrl::get_MavlinkMode( ModeFuncCfg cfg, Receiver rc, 
																	uint8_t btn_zones[4], AFunc* mode )
{	//��ȡ����ģʽ
	if( get_Position_MSStatus() != MS_Ready )
	{
		*mode = AFunc_AltHold;
		return;
	}
	
	if( rc.available )
	{	//���ջ����ø���ģʽ��ť״̬		
		uint8_t MF_mode = *mode;
		
		uint8_t new_btn_zones[4];
		new_btn_zones[0] = get_RcButtonZone( rc.data[4], btn_zones[0] );
		new_btn_zones[1] = get_RcButtonZone( rc.data[5], btn_zones[1] );
		new_btn_zones[2] = get_RcButtonZone( rc.data[6], btn_zones[2] );
		new_btn_zones[3] = get_RcButtonZone( rc.data[7], btn_zones[3] );
		
		if( new_btn_zones[0]<=5 && new_btn_zones[0]!=btn_zones[0] )
		{	//ģʽ��ť�ı����ģʽ
			MF_mode = cfg.Bt1AFunc1[8*new_btn_zones[0]];			
		}
		
		/*�ж�ִ������*/
			if( cfg.MissionBt[0]>=2 && cfg.MissionBt[0]<=4 )
			{	//��ť����ִ������
				if( rc.available_channels >= cfg.MissionBt[0]+4 )
				{
					uint8_t new_btn_zone = new_btn_zones[cfg.MissionBt[0]-1];
					uint8_t old_btn_zone = btn_zones[cfg.MissionBt[0]-1];
					if( new_btn_zone != old_btn_zone )
					{	//��ť״̬�����仯
						if( new_btn_zone>=4 )
							MF_mode = AFunc_Mission;
						else if( old_btn_zone<=5 )
							MF_mode = 0;
					}
				}
			}
			else if( cfg.MissionBt[0]>=12 && cfg.MissionBt[0]<=14 )
			{	//��ť�仯ִ������
				if( rc.available_channels >= cfg.MissionBt[0]-10+4 )
				{
					//��ȡ��ť״̬
					uint8_t new_btn_zone = new_btn_zones[cfg.MissionBt[0]-11];
					uint8_t old_btn_zone = btn_zones[cfg.MissionBt[0]-11];
					if( old_btn_zone<=5 && new_btn_zone!=old_btn_zone )
					{	//��ť״̬�����仯
						if( *mode != AFunc_Mission )
							MF_mode = AFunc_Mission;
						else
							MF_mode = 0;
					}
				}
			}
		/*�ж�ִ������*/
		
		/*�жϷ���*/
			if( cfg.RTLBt[0]>=2 && cfg.RTLBt[0]<=4 )
			{	//��ť���·���
				if( rc.available_channels >= cfg.RTLBt[0]+4 )
				{
					uint8_t new_btn_zone = new_btn_zones[cfg.RTLBt[0]-1];
					uint8_t old_btn_zone = btn_zones[cfg.RTLBt[0]-1];	
					if( new_btn_zone!=old_btn_zone )
					{	//��ť״̬�����仯	
						if( new_btn_zone>=4 )
							MF_mode = AFunc_RTL;
						else if( old_btn_zone<=5 )
							MF_mode = 0;
					}
				}
			}
			else if( cfg.RTLBt[0]>=12 && cfg.RTLBt[0]<=14 )
			{	//��ť�仯����
				if( rc.available_channels >= cfg.RTLBt[0]-10+4 )
				{
					//��ȡ��ť״̬
					uint8_t new_btn_zone = new_btn_zones[cfg.RTLBt[0]-11];
					uint8_t old_btn_zone = btn_zones[cfg.RTLBt[0]-11];
					if( old_btn_zone<=5 && new_btn_zone!=old_btn_zone )
					{	//��ť״̬�����仯
						if( *mode != AFunc_RTL )
							MF_mode = AFunc_RTL;
						else
							MF_mode = 0;
					}
				}
			}
		/*�жϷ���*/
		if( MF_mode == 0 )
			MF_mode = cfg.Bt1AFunc1[8*new_btn_zones[0]];
		
		*mode = (AFunc)MF_mode;
		
		btn_zones[0] = new_btn_zones[0];
		btn_zones[1] = new_btn_zones[1];
		btn_zones[2] = new_btn_zones[2];
		btn_zones[3] = new_btn_zones[3];
	}
	else
	{
		*mode = AFunc_RTL;
	}
}

ModeResult M32_PosCtrl::main_func( void* param1, uint32_t param2 )
{
	set_mav_mode_arm();
	
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
	
	//��ȡ��ʼ������Ϣ
	CurrentWpInf currentWpInf;
	ReadParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0 );
	
	if( MFunc_cfg.RstWp0[0] != 0 )
		//��Ҫ������ʼ������Ϊ0
		setCurrentMission(0);

	//����ģʽ
	bool mode_switched = true;
	#define change_Mode(x) {*resMode=cMode=x; mode_switched = true;}
	uint16_t current_mission_ind;
	MissionInf current_mission_inf;
	uint8_t ModeButtonZone = 255;
	uint8_t MissionButtonZone = 255;
	uint8_t RTLButtonZone = 255;
	AFunc cMode = AFunc_PosHold;
	AFunc* resMode;
	if(param1)
	{
		resMode = (AFunc*)param1;
		cMode = *resMode;
	}
	else
		resMode = &cMode;
	
	//����״̬��
	NavCmdInf navInf; 
	init_NavCmdInf(&navInf);
	//ָ��ִ���Ƿ���ɣ��ֶ�ģʽ��
	bool ManualModeNavCmdInprogress = false;
	ModeMsg ManualModeNavCmd;
	
	//����ģʽ�ɵ��ϴ������״̬
	uint8_t MissionMode_BackToLastWp = 0;
	
	//�Ƿ���inFlightCmd
	#define DealInFlightCmd 0
	//��һ��������������м��InFlightCmd������
	#define MissionInc 1
	//�������յ�ǰ���뱶��
	#define CamTriggDistMult 2
	
	//��ʼ�������������
	CamTriggDist = 0;
	double camTriggDist = CamTriggDist;
		
	//��ʼ��Aux����
	init_process_AuxFuncs();
		
	while(1)
	{
		os_delay(1.0/freq);
		
		//��ȡ���ջ�
		Receiver rc;
		getReceiver( &rc, 0, 0.02 );
		
		//����Auxͨ��
		process_AuxFuncs(&rc);
		
		//��ȡ��Ϣ
		bool msg_available;
		ModeMsg msg;
		msg_available = ModeReceiveMsg( &msg, 0 );
		uint8_t msg_handled = 0;	//1-����acceped 2-����denied
		
		bool inFlight;
		get_is_inFlight(&inFlight);
		if( inFlight == false && ForceMSafeCtrl == true)
		{//�ж��˳�ģʽ
			Attitude_Control_Disable();
			return MR_OK;
		}		
		
		if( msg_available && msg.cmd==MAV_CMD_COMPONENT_ARM_DISARM )
		{	//����վ����
			if( msg.params[0] == 0 )
			{
				if( (msg.cmd_type & CMD_TYPE_MASK) == CMD_TYPE_MAVLINK )
				{
					Attitude_Control_Disable();
					set_mav_mode_disarm();
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
				//���浱ǰ������Ϣ
				UpdateParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0, sizeof(CurrentWpInf)/8 );
				return MR_OK;
			}
			msg_handled = 2;
		}
		if( get_CrashedState() )
		{	//�෭����
			Attitude_Control_Disable();
			//���浱ǰ������Ϣ
			UpdateParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0, sizeof(CurrentWpInf)/8 );
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
				else if( t_mav_mode.main_mode==PX4_CUSTOM_MAIN_MODE_ALTCTL )
				{	//ָ������ֶ�
					reqMode = AFunc_AltHold;
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
								else if( MissionButtonZone<=5 )
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
								else if( RTLButtonZone<=5 )
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
				cMode = (AFunc)reqMode;
				mode_switched = true;
			}
		}
		else
			cMode = (AFunc)reqMode;
		#define swManualMode if( is_AFunc_auto(MFunc_cfg.Bt1AFunc1[8*ModeButtonZone]) == false )\
														*resMode = cMode = (AFunc)MFunc_cfg.Bt1AFunc1[8*ModeButtonZone];\
													else\
														*resMode = cMode = AFunc_PosHold;\
		
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
					//���浱ǰ������Ϣ
					UpdateParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0, sizeof(CurrentWpInf)/8 );
					return MR_OK;
				}
			/*�ж��˳�ģʽ*/
		}
		else if( cMode==AFunc_TakeOff )
		{	//���ģʽ
			if( mode_switched )
			{
				mode_switched = false;
				if( inFlight ){
					swManualMode}
				else
				{
					Position_Control_Enable();
					bool pos_ena;
					is_Position_Control_Enabled(&pos_ena);
					if( pos_ena )
					{
						Position_Control_set_XYLock();
						Position_Control_Takeoff_HeightGlobal(param2);
					}
					else{
						swManualMode}
				}
			}
			else
			{
				Position_Control_Enable();
				bool pos_ena;
				is_Position_Control_Enabled(&pos_ena);
				if( pos_ena == false )
				{	//λ�ÿ������޷��򿪷����ֶ�ģʽ
					swManualMode
					goto Manual_Mode;
				}
				
				set_mav_mode( 
					MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
					PX4_CUSTOM_MAIN_MODE_AUTO,
					PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF );
				
				Position_Control_set_XYLock();
				Position_ControlMode alt_mode;
				get_Altitude_ControlMode(&alt_mode);
				if( alt_mode == Position_ControlMode_Position )
				{
					swManualMode
				}
			}			
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
			
			if( mode_switched )
			{	//���л�������ģʽ
				//����ɲ���ȴ�								
				Position_Control_set_XYLock();
				Position_Control_set_ZLock();
				Attitude_Control_set_YawLock();
				
				camTriggDist = CamTriggDist = 0;
				
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
							if( current_mission_ind == currentWpInf.CurrentWp[0] )
								//�ָ����߷���
								MissionMode_BackToLastWp = 0;
							else
								//��ǰ�ͼ�¼�ĺ��㲻ͬ���ָ�����
								MissionMode_BackToLastWp = 10;
						}
						else
						{	//��ȡ����������Ϣ
							//�����ŰѺ�������Ϊ�׸�
							setCurrentMission(0);
							//��λ��ǰ������Ϣ
							currentWpInf.CurrentWp[0] = 0;
							currentWpInf.line_x = 0;
							currentWpInf.line_y = 0;
							currentWpInf.line_z = 0;
							currentWpInf.line_fs = -1;
							currentWpInf.CamTrigDist = 0;
							MissionMode_BackToLastWp = 10;
							if( ReadCurrentMission(&current_mission_inf, &current_mission_ind) )
							{	//������һ����ɹ�
								//��ʼ��������Ϣ
								init_NavCmdInf(&navInf);
								//���������������
								camTriggDist = CamTriggDist;
							}
							else
							{	//�޺�����Ϣ�����ֶ�ģʽ
								swManualMode
								goto Manual_Mode;
							}
						}
					}
				}
				else
					navInf.counter2 = 0;
			}
			else if( MissionMode_BackToLastWp != 10 )
			{	//���ȷɵ��ϴκ��߷���λ��
				
				//�趨mavlinkģʽ
				set_mav_mode( 
					MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
					PX4_CUSTOM_MAIN_MODE_AUTO,
					PX4_CUSTOM_SUB_MODE_AUTO_MISSION );
				
				double AB_length = safe_sqrt( sq(currentWpInf.line_x) + sq(currentWpInf.line_y) + sq(currentWpInf.line_z) );
				if( current_mission_inf.cmd!=MAV_CMD_NAV_WAYPOINT || currentWpInf.CurrentWp[0]==0 || currentWpInf.line_fs<0 )
				{	//����Ҫ�ָ�ֱ�ӽ����������							
					MissionMode_BackToLastWp = 10;
				}
				else
				{	//��Ҫ�ָ����ϴη���λ��
					switch( MissionMode_BackToLastWp )
					{
						case 0:
						{	//û��������
							if(inFlight)
							{
								camTriggDist = CamTriggDist = currentWpInf.CamTrigDist;
								++MissionMode_BackToLastWp;
							}
							else
							{
								Position_Control_set_XYLock();
								Position_Control_set_TargetVelocityZ(50);
							}
							break;
						}
						case 1:
						{	//�߶ȵ���
							//����xy
							Position_Control_set_XYLock();							
							//��Zƫ�ƾ���
							double z_offset = 0;
							if( AB_length > 0.1 )
							{
								double inv_AB_length = 1.0 / AB_length;
								double ufs = AB_length - currentWpInf.line_fs;
								if( ufs < 0 )
									ufs = 0;
								z_offset = ufs * currentWpInf.line_z*inv_AB_length;
							}
							//ִ�н��
							bool res = false;
							switch(current_mission_inf.frame)
							{
								case MAV_FRAME_GLOBAL_INT:
								case MAV_FRAME_GLOBAL:
								{
									res = Position_Control_set_TargetPositionZGlobal( current_mission_inf.params[6]*100 + z_offset, 0 );
									break;
								}
								case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
								case MAV_FRAME_GLOBAL_RELATIVE_ALT:
								{
									res = Position_Control_set_TargetPositionZRA( current_mission_inf.params[6]*100 + z_offset, 0 );
									break;
								}
								case MAV_FRAME_GLOBAL_TERRAIN_ALT:
								case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
								{
									res = Position_Control_set_TargetPositionZRA( current_mission_inf.params[6]*100 + z_offset, 0 );
									break;
								}
								
								case MAV_FRAME_LOCAL_NED:
									res = Position_Control_set_TargetPositionZ( -(current_mission_inf.params[6]*100 + z_offset), 0 );
									break;
								
								case MAV_FRAME_LOCAL_ENU:
									res = Position_Control_set_TargetPositionZ( current_mission_inf.params[6]*100 + z_offset, 0 );
									break;
								
								case MAV_FRAME_BODY_NED:
								case MAV_FRAME_BODY_FRD:
								case MAV_FRAME_BODY_OFFSET_NED:
								case MAV_FRAME_LOCAL_OFFSET_NED:
									res = Position_Control_set_TargetPositionZRelative( -(current_mission_inf.params[6]*100 + z_offset), 0 );
									break;
								
								
								case MAV_FRAME_BODY_FLU:
								{
									res = Position_Control_set_TargetPositionZRelative( current_mission_inf.params[6]*100 + z_offset, 0 );
									break;
								}
								
								default:
									MissionMode_BackToLastWp = 10;
							}
							if(res)
								++MissionMode_BackToLastWp;
							else
								MissionMode_BackToLastWp = 10;
							break;
						}
						
						case 2:
						{	//�ȴ��߶ȵ������
							//����xy
							Position_Control_set_XYLock();
							Position_ControlMode alt_mode;
							get_Altitude_ControlMode(&alt_mode);
							if( alt_mode == Position_ControlMode_Position )
								++MissionMode_BackToLastWp;
							break;
						}
						
						case 3:
						{	//��תƫ��
														
							//����XYZ
							Position_Control_set_XYLock();
							Position_Control_set_ZLock();
							//��AB�������ȵ���
							double offset_x=0, offset_y=0;
							if( AB_length > 0.1 )
							{
								double inv_AB_length = 1.0 / AB_length;
								double ufs = AB_length - currentWpInf.line_fs;
								if( ufs < 0 )
									ufs = 0;
								offset_x = ufs * currentWpInf.line_x*inv_AB_length;
								offset_y = ufs * currentWpInf.line_y*inv_AB_length;
							}
							
							double LA, LB;
							switch(current_mission_inf.frame)
							{
								case MAV_FRAME_GLOBAL:
								case MAV_FRAME_GLOBAL_RELATIVE_ALT:
								case MAV_FRAME_GLOBAL_INT:
								case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
								case MAV_FRAME_GLOBAL_TERRAIN_ALT:
								case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
								{	//ȫ��λ
									if( current_mission_inf.params[4]<-90 ||  current_mission_inf.params[4]> 90 
										|| current_mission_inf.params[5]<-180 || current_mission_inf.params[5]>180 )
									{	//��γ��Ϊ����ȷ��תƫ��
										MissionMode_BackToLastWp += 2;
										goto ModeLoopFin;
									}
									
									//��ȡ����ȫ��λ��������Ϣ
									PosSensorHealthInf2 global_inf;
									if( get_OptimalGlobal_XY( &global_inf ) == false )
									{
										MissionMode_BackToLastWp = 10;
										goto ModeLoopFin;
									}
									//��ȡָ����γ��ƽ������
									double x, y;
									map_projection_project( &global_inf.mp, current_mission_inf.params[4], current_mission_inf.params[5], &x, &y );
									x -= global_inf.HOffset.x;
									y -= global_inf.HOffset.y;
									x += offset_x;
									y += offset_y;
									LA = y - global_inf.PositionENU.y;
									LB = x - global_inf.PositionENU.x;
									break;
								}
								
								case MAV_FRAME_LOCAL_NED:
								{
									vector3<double> position;
									get_Position_Ctrl(&position);
									double x, y;
									x = current_mission_inf.params[5]*100;
									y = current_mission_inf.params[4]*100;
									x += offset_x;
									y += offset_y;
									LA = y - position.y;
									LB = x - position.x;
									break;
								}
								
								case MAV_FRAME_LOCAL_ENU:
								{
									vector3<double> position;
									get_Position_Ctrl(&position);
									double x, y;
									x = current_mission_inf.params[4]*100;
									y = current_mission_inf.params[5]*100;
									x += offset_x;
									y += offset_y;
									LA = y - position.y;
									LB = x - position.x;
									break;
								}
								
								default:
									MissionMode_BackToLastWp = 10;
									goto ModeLoopFin;
							}
							
							if( sq(LA) + sq(LB) > 5 )
								Attitude_Control_set_Target_Yaw( atan2(LA,LB) );
							++MissionMode_BackToLastWp;
								
							break;
						}
						
						case 4:
						{	//�ȴ�ƫ����ת��ʼ�������
							Position_Control_set_XYLock();
							Position_Control_set_ZLock();
							double yawTrackErr;
							Attitude_Control_get_YawTrackErr(&yawTrackErr);
							if( yawTrackErr < 0.01 )
							{						
								//��AB�������ȵ���
								double offset_x=0, offset_y=0;
								if( AB_length > 0.1 )
								{
									double inv_AB_length = 1.0 / AB_length;
									double ufs = AB_length - currentWpInf.line_fs;
									if( ufs < 0 )
										ufs = 0;
									offset_x = ufs * currentWpInf.line_x*inv_AB_length;
									offset_y = ufs * currentWpInf.line_y*inv_AB_length;
								}
								double Tx, Ty;
								switch(current_mission_inf.frame)
								{
									case MAV_FRAME_GLOBAL:
									case MAV_FRAME_GLOBAL_RELATIVE_ALT:
									case MAV_FRAME_GLOBAL_INT:
									case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
									case MAV_FRAME_GLOBAL_TERRAIN_ALT:
									case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
									{	//ȫ��λ
										if( current_mission_inf.params[4]<-90 ||  current_mission_inf.params[4]> 90 
											|| current_mission_inf.params[5]<-180 || current_mission_inf.params[5]>180 )
										{	//��γ��Ϊ����ȷ�˳�
											MissionMode_BackToLastWp = 10;
											goto ModeLoopFin;
										}
										
										//��ȡ����ȫ��λ��������Ϣ
										PosSensorHealthInf2 global_inf;
										if( get_OptimalGlobal_XY( &global_inf ) == false )
										{
											MissionMode_BackToLastWp = 10;
											goto ModeLoopFin;
										}
										//��ȡָ����γ��ƽ������
										double x, y;
										map_projection_project( &global_inf.mp, current_mission_inf.params[4], current_mission_inf.params[5], &x, &y );
										x -= global_inf.HOffset.x;
										y -= global_inf.HOffset.y;
										x += offset_x;
										y += offset_y;
										Tx = x;	Ty = y;
										break;
									}
									
									case MAV_FRAME_LOCAL_NED:
									{
										double x, y;
										x = current_mission_inf.params[5]*100;
										y = current_mission_inf.params[4]*100;
										x += offset_x;
										y += offset_y;
										Tx = x;	Ty = y;
										break;
									}
									
									case MAV_FRAME_LOCAL_ENU:
									{
										double x, y;
										x = current_mission_inf.params[4]*100;
										y = current_mission_inf.params[5]*100;
										x += offset_x;
										y += offset_y;
										Tx = x;	Ty = y;
										break;
									}
									
									default:
										MissionMode_BackToLastWp = 10;
										goto ModeLoopFin;
								}
								
								bool res = Position_Control_set_TargetPositionXY( Tx, Ty, 0 );
								if(res)
									++MissionMode_BackToLastWp;
								else
									MissionMode_BackToLastWp = 10;
							}
							break;
						}
						
						case 5:
						{	//�ȴ����߷������
							Position_Control_set_ZLock();
							Position_ControlMode pos_mode;
							get_Position_ControlMode(&pos_mode);
							if( pos_mode == Position_ControlMode_Position )
							{	//�ѳɹ��ƶ����ϴκ���λ��
								MissionMode_BackToLastWp = 10;
							}
							break;
						}
						
					}
				}
			}
			else
			{	//�������
				
				//�趨mavlinkģʽ
				set_mav_mode( 
					MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
					PX4_CUSTOM_MAIN_MODE_AUTO,
					PX4_CUSTOM_SUB_MODE_AUTO_MISSION );
				
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
						swManualMode
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
									swManualMode
									//��λ��ǰ������Ϣ
									currentWpInf.CurrentWp[0] = 0;
									currentWpInf.line_x = 0;
									currentWpInf.line_y = 0;
									currentWpInf.line_z = 0;
									currentWpInf.line_fs = -1;
									currentWpInf.CamTrigDist = 0;
									if( current_mission_inf.cmd==MAV_CMD_NAV_RETURN_TO_LAUNCH || current_mission_inf.cmd==MAV_CMD_NAV_LAND )
									{
										bool inFlight;
										get_is_inFlight(&inFlight);
										if( inFlight==false )
										{	//������ɼ���
											Attitude_Control_Disable();
											UpdateParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0, sizeof(CurrentWpInf)/8 );
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
										//��������״̬
										if( current_mission_inf.cmd == MAV_CMD_NAV_WAYPOINT )
										{
											vector3<double> AB;
											NavCmd16_WAYPOINT_GetAB( current_mission_inf.frame, current_mission_inf.params, &AB );
											currentWpInf.CurrentWp[0] = current_mission_ind;
											currentWpInf.line_x = AB.x;
											currentWpInf.line_y = AB.y;
											currentWpInf.line_z = AB.z;
											currentWpInf.line_fs = 0;
											currentWpInf.CamTrigDist = camTriggDist;
										}
										else
										{
											currentWpInf.CurrentWp[0] = current_mission_ind;
											currentWpInf.line_x = 0;
											currentWpInf.line_y = 0;
											currentWpInf.line_z = 0;
											currentWpInf.line_fs = 0;
											currentWpInf.CamTrigDist = camTriggDist;
										}
									}
									else
									{	//�޺�����Ϣ�����ֶ�ģʽ
										setCurrentMission( 0 );
										swManualMode						
										//��λ��ǰ������Ϣ
										currentWpInf.CurrentWp[0] = 0;
										currentWpInf.line_x = 0;
										currentWpInf.line_y = 0;
										currentWpInf.line_z = 0;
										currentWpInf.line_fs = -1;
										currentWpInf.CamTrigDist = 0;
										if( current_mission_inf.cmd==MAV_CMD_NAV_RETURN_TO_LAUNCH || current_mission_inf.cmd==MAV_CMD_NAV_LAND )
										{
											bool inFlight;
											get_is_inFlight(&inFlight);
											if( inFlight==false )
											{	//������ɼ���
												Attitude_Control_Disable();
												UpdateParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0, sizeof(CurrentWpInf)/8 );
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
								//��������״̬																	
								if( current_mission_inf.cmd == MAV_CMD_NAV_WAYPOINT )
								{
									vector3<double> AB;
									NavCmd16_WAYPOINT_GetAB( current_mission_inf.frame, current_mission_inf.params, &AB );
									currentWpInf.CurrentWp[0] = current_mission_ind;
									currentWpInf.line_x = AB.x;
									currentWpInf.line_y = AB.y;
									currentWpInf.line_z = AB.z;
									currentWpInf.line_fs = 0;
									currentWpInf.CamTrigDist = camTriggDist;
								}
								else
								{
									currentWpInf.CurrentWp[0] = current_mission_ind;
									currentWpInf.line_x = 0;
									currentWpInf.line_y = 0;
									currentWpInf.line_z = 0;
									currentWpInf.line_fs = 0;
									currentWpInf.CamTrigDist = camTriggDist;
								}
							}
						}
						else
						{	//�޺�����Ϣ�����ֶ�ģʽ
							setCurrentMission( 0 );
							swManualMode
							//��λ��ǰ������Ϣ
							currentWpInf.CurrentWp[0] = 0;
							currentWpInf.line_x = 0;
							currentWpInf.line_y = 0;
							currentWpInf.line_z = 0;
							currentWpInf.line_fs = -1;
							currentWpInf.CamTrigDist = 0;
							if( current_mission_inf.cmd==MAV_CMD_NAV_RETURN_TO_LAUNCH || current_mission_inf.cmd==MAV_CMD_NAV_LAND )
							{	//������ɼ���
								bool inFlight;
								get_is_inFlight(&inFlight);
								if( inFlight==false )
								{
									Attitude_Control_Disable();
									UpdateParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0, sizeof(CurrentWpInf)/8 );
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
							swManualMode
							//��λ��ǰ������Ϣ
							currentWpInf.CurrentWp[0] = 0;
							currentWpInf.line_x = 0;
							currentWpInf.line_y = 0;
							currentWpInf.line_z = 0;
							currentWpInf.line_fs = -1;
							currentWpInf.CamTrigDist = 0;
						}
						if( ReadCurrentMission(&current_mission_inf, &current_mission_ind) )
						{	//������һ����ɹ�
							//��ʼ��������Ϣ
							init_NavCmdInf(&navInf);
							//���������������
							camTriggDist = CamTriggDist;
							//��������״̬														
							if( current_mission_inf.cmd == MAV_CMD_NAV_WAYPOINT )
							{
								vector3<double> AB;
								NavCmd16_WAYPOINT_GetAB( current_mission_inf.frame, current_mission_inf.params, &AB );
								currentWpInf.CurrentWp[0] = current_mission_ind;
								currentWpInf.line_x = AB.x;
								currentWpInf.line_y = AB.y;
								currentWpInf.line_z = AB.z;
								currentWpInf.line_fs = 0;
								currentWpInf.CamTrigDist = camTriggDist;
							}
							else
							{
								currentWpInf.CurrentWp[0] = current_mission_ind;
								currentWpInf.line_x = 0;
								currentWpInf.line_y = 0;
								currentWpInf.line_z = 0;
								currentWpInf.line_fs = 0;
								currentWpInf.CamTrigDist = camTriggDist;
							}
						}
						else
						{	//�޺�����Ϣ�����ֶ�ģʽ
							setCurrentMission( 0 );
							swManualMode							
							//��λ��ǰ������Ϣ
							currentWpInf.CurrentWp[0] = 0;
							currentWpInf.line_x = 0;
							currentWpInf.line_y = 0;
							currentWpInf.line_z = 0;
							currentWpInf.line_fs = -1;
							currentWpInf.CamTrigDist = 0;
							if( current_mission_inf.cmd==MAV_CMD_NAV_RETURN_TO_LAUNCH || current_mission_inf.cmd==MAV_CMD_NAV_LAND )
							{
								bool inFlight;
								get_is_inFlight(&inFlight);
								if( inFlight==false )
								{	//������ɼ���
									Attitude_Control_Disable();
									UpdateParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0, sizeof(CurrentWpInf)/8 );
									return MR_OK;
								}
							}
						}					
					}
				}
				else
				{	//����ִ����			
					if( NavCmdRs_InProgress_CanExInFlightCmd(res) )
					{	//��ִ��InFlightCmd
						
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
						
						//�������״̬
						vector3<double> line_AB;
						double flightDistance = -1;
						if( Position_Control_get_LineFlightABDistance( &line_AB, &flightDistance ) )
						{
							currentWpInf.CurrentWp[0] = current_mission_ind;
							currentWpInf.line_x = line_AB.x;
							currentWpInf.line_y = line_AB.y;
							currentWpInf.line_z = line_AB.z;
							currentWpInf.line_fs = flightDistance;
							currentWpInf.CamTrigDist = camTriggDist;
						}
						
						//��������
						if( camTriggDist > 0 )
						{
							Position_Control_get_LineFlightDistance(&flightDistance);
							int mult = (int)(flightDistance / camTriggDist) + 1;
							if( mult > navInf.usr_temp[CamTriggDistMult] )
							{
								InflightCmd_CamTakePhoto();
								navInf.usr_temp[CamTriggDistMult] = mult;
							}
						}
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
				Attitude_Control_set_YawLock();
			}
			
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
							//���浱ǰ������Ϣ
							UpdateParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0, sizeof(CurrentWpInf)/8 );
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
							//���浱ǰ������Ϣ
							UpdateParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0, sizeof(CurrentWpInf)/8 );
							return MR_OK;
						}
					}
					else
						exit_mode_Gcounter = 0;
				/*�ж��˳�ģʽ*/
					
				//�л����߶���
				if( cMode==AFunc_AltHold )
					Position_Control_Disable();
				else
					Position_Control_Enable();				
				bool pos_ena;
				is_Position_Control_Enabled(&pos_ena);
				
				//�趨mavlinkģʽ
				if( pos_ena )						
					set_mav_mode( 
						MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
						PX4_CUSTOM_MAIN_MODE_POSCTL,
						0 );
				else
					set_mav_mode( 
						MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
						PX4_CUSTOM_MAIN_MODE_ALTCTL,
						0 );

				
				//�ж�ҡ���Ƿ����
				bool sticks_in_neutral = 
					in_symmetry_range_mid( rc.data[0] , 50 , MFunc_cfg.NeutralZone[0] ) &&
					in_symmetry_range_mid( rc.data[1] , 50 , MFunc_cfg.NeutralZone[0] ) &&
					in_symmetry_range_mid( rc.data[2] , 50 , MFunc_cfg.NeutralZone[0] ) &&
					in_symmetry_range_mid( rc.data[3] , 50 , MFunc_cfg.NeutralZone[0] );
				
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
							msg_handled = 1;
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
								
					ManualModeNavCmdInprogress = false;
					
					//���Ÿ˿��ƴ�ֱ�ٶ�
					if( in_symmetry_range_mid( rc.data[0] , 50 , MFunc_cfg.NeutralZone[0] ) )
						Position_Control_set_ZLock();
					else
					{
						double thr_stick = remove_deadband( rc.data[0] - 50.0 , (double)MFunc_cfg.NeutralZone[0] );
						if( thr_stick > 0 )
							thr_stick *= get_maxVelUp() / 50;
						else
							thr_stick *= get_maxVelDown() / 50;
						Position_Control_set_TargetVelocityZ(thr_stick);
					}					
					
					if( pos_ena )
					{
						
						//��������˿�ˮƽ�ٶ�
						if( in_symmetry_range_mid( rc.data[3] , 50 , MFunc_cfg.NeutralZone[0] ) && in_symmetry_range_mid( rc.data[2] , 50 , MFunc_cfg.NeutralZone[0] ) )							
							Position_Control_set_XYLock();
						else
						{
							double RPCtrlScale = atan2( get_maxAccXY() / 50.0, GravityAcc );
							double XYCtrlScale = get_maxVelXY() / 50.0;						
							double roll_sitck_d = remove_deadband( rc.data[3] - 50.0, (double)MFunc_cfg.NeutralZone[0] );
							double pitch_sitck_d = remove_deadband( rc.data[2] - 50.0, (double)MFunc_cfg.NeutralZone[0] );
							vector3<double> velocityFLU;
							get_VelocityFLU_Ctrl(&velocityFLU);
							double vel_stick_err_pitch = velocityFLU.x/XYCtrlScale - pitch_sitck_d;
							double vel_stick_err_roll = velocityFLU.y/XYCtrlScale - -roll_sitck_d;
							constrain_vector( vel_stick_err_roll, vel_stick_err_pitch, 50 );
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
					if( in_symmetry_range_mid( rc.data[1] , 50 , MFunc_cfg.NeutralZone[0] ) )
						Attitude_Control_set_YawLock();
					else
						Attitude_Control_set_Target_YawRate( ( 50 - rc.data[1] )*YCtrlScale );						
				}
			}
			else
			{
				//��ң���źŽ��밲ȫģʽ
				change_Mode(AFunc_RTL)
				goto RTL;
			}
		}
		
ModeLoopFin:
		/*������Ϣ������*/
			if( msg_available )
			{
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
							msg_handled==1 ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED ,	//result
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
		/*������Ϣ������*/
	}
	return MR_OK;
}
