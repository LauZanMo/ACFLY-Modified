/*
	MSafe��ȫģʽ
	�û�ģʽһ��ʱ�䲻���п���
	�����enter_MSafe������ģʽ
	
	���밲ȫģʽ��ForceMSafeCtrl��
	�û�ģʽ���ƽ�ʧЧ
	
	�û�ģʽͬʱ����ˮƽ��XYλ�û���̬��
	�ʹ�ֱ�߶ȿ��ƿ��˳�MSafe��ÿ���Ȩ
	
	����������һ���û����Ĵ��ļ�����
	�������ļ��辭����֤�ٷ�������
*/

#include "Basic.hpp"
#include "Modes.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "Receiver.hpp"
#include "ControlSystem.hpp"
#include "ctrl_Main.hpp"
#include "MeasurementSystem.hpp"
#include "NavCmdProcess.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"

//��ȫģʽ������
TaskHandle_t MSafeTaskHandle;
//ǿ�Ʒ��������䣩
bool ForceRTL = false;

//��ȫģʽ����
struct MSafeCfg
{
	//�Զ�����ģʽ
	//0-���Զ�����
	//1-�Զ�ģʽ���Զ��жϵ������㷵����
	uint8_t SfRtMode[8];
	
	//�����ٶ�
	float RtSpeed[2];
	
	//��γ�ȶ�λ���߷�����Χ
	float GbRtHRange[2];
	//���ض�λ���޾�γ�ȣ����߷�����Χ
	float LcRtHRange[2];
	
	//��γ�ȶ�λ���߸߶ȣ��Եأ�
	float GbRtHeight[2];
	//���ض�λ���޾�γ�ȣ����߸߶ȣ��Եأ�
	float LcRtHeight[2];
	
	//ǿ�Ʒ�����ѹ
	float FcRTLVolt[2];
	//ǿ�ƽ����ѹ
	float FcLandVolt[2];
}__PACKED;

#define MSafeRate 20
static void MSafe_Server(void* pvParameters)
{
	//׼ȷ������ʱ
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	//����
	MSafeCfg cfg;
	cfg.SfRtMode[0] = 1;	//SfRtMode
	cfg.RtSpeed[0] = 1000;	//RtSpeed
	cfg.GbRtHRange[0] = 3000;	//GbRtHRange
	cfg.LcRtHRange[0] = 3000;	//LcRtHRange
	cfg.GbRtHeight[0] = 5000;	//GbRtHeight
	cfg.LcRtHeight[0] = 2000;	//LcRtHeight
	cfg.FcRTLVolt[0] = 0;	//ǿ�Ʒ�����ѹ	
	cfg.FcLandVolt[0] = 0;	//ǿ�ƽ����ѹ
	uint16_t cfg_update_counter = 60000;
	
	//�͵���״̬
	uint8_t lowPowerState = 0;
	uint16_t lowPowerState1_counter = 0;
	uint16_t lowPowerState2_counter = 0;
	
	//�Ƿ�ս����Զ�ģʽ
	//16-32�ж�λȫ�Զ�ģʽ
	//32-48�޶�λ�Զ�ģʽ
	uint8_t firstAuto = 0;
	NavCmdInf navInf;
	init_NavCmdInf(&navInf);
	uint16_t current_mission_ind;
	//�����߶�
	double RtHeight = -1;
	while(1)
	{
		vTaskDelayUntil( &xLastWakeTime, (1.0/MSafeRate)*configTICK_RATE_HZ );
		
		//��ȡ��ѹ
		float mainBatVolt = get_MainBatteryVoltage_filted();
		
		//5����¶�ȡһ������
		if( ++cfg_update_counter >= 5*MSafeRate )
		{
			ReadParamGroup( "MSafe", (uint64_t*)&cfg, 0 );
		}
		
		//��ȡ�Ƿ�򿪿�����
		//������û�򿪲����н�һ������
		bool attCtrlEna;
		is_Attitude_Control_Enabled(&attCtrlEna);
		if( attCtrlEna==false )
		{
			//�˳���ȫģʽ
			ForceMSafeCtrl = false;
			//��λ�Զ����Ƽ�ʱ��
			firstAuto = 0;
			
			//��λ�͵������
			if( cfg.FcRTLVolt[0]<=5 || mainBatVolt<=7 || mainBatVolt>cfg.FcRTLVolt[0] )
				lowPowerState = 0;
			continue;
		}		
		
		//�жϵ͵���
		if( mainBatVolt > 7 ) 
		{
			if( cfg.FcRTLVolt[0] > 5 )
			{	//�жϷ�����ѹ
				if( mainBatVolt < cfg.FcRTLVolt[0] )
				{
					if( lowPowerState < 1 )
					{
						if( ++lowPowerState1_counter >= 3*MSafeRate )
						{
							lowPowerState1_counter = 0;
							lowPowerState = 1;
						}
					}
					else
						lowPowerState1_counter = 0;
				}
				else
					lowPowerState1_counter = 0;
			}
			else if( cfg.FcLandVolt[0]>5 )
			{	//�жϽ����ѹ
				if( mainBatVolt < cfg.FcLandVolt[0] )
				{
					if( lowPowerState < 2 )
					{
						if( ++lowPowerState2_counter >= 3*MSafeRate )
						{
							lowPowerState2_counter = 0;
							lowPowerState = 2;
						}
					}
					else
						lowPowerState2_counter = 0;
				}
				else
					lowPowerState2_counter = 0;
			}
		}
		else
			lowPowerState1_counter = lowPowerState2_counter = 0;
		
		//��ȡ�ϴο���ʱ��
		TIME lastXYCtrlTime, lastZCtrlTime;
		get_lastXYCtrlTime(&lastXYCtrlTime);
		get_lastZCtrlTime(&lastZCtrlTime);
		
		if( lowPowerState>0 || lastXYCtrlTime.get_pass_time()>1 || lastZCtrlTime.get_pass_time()>1 )
		{	//�͵�������Ƴ�ʱ
			//ǿ�ƽ���MSafe����
			ForceMSafeCtrl = true;			
			//�򿪸߶ȿ�����
			Altitude_Control_Enable();
			
			//��ȡ���ջ�
			Receiver rc;
			getReceiver(&rc);
			
			//�ж��Ƿ�ʹ��ң��������
			//��ң������ForceRTL��ң�ػ���ʱ
			//ִ�з���
			bool UseRcCtrl = false;
			if( rc.available )
			{
				bool sticks_in_neutral = 
						in_symmetry_range_mid( rc.data[0] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[1] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[2] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[3] , 50 , 5 );
				if( ForceRTL || lowPowerState>0 )
				{
					if(!sticks_in_neutral)
						UseRcCtrl = true;
				}
				else
					UseRcCtrl = true;
			}
			
			if( UseRcCtrl )
			{	//ʹ��ң��������
		
				//��λ�Զ����Ƽ�ʱ��
				firstAuto = 0;
				
				//����5ͨ״̬ѡ�񶨵㶨��				
				if( rc.data[4] > 60 )
				{
					Position_Control_Enable();
				}
				else if( rc.data[4] < 40 )
				{
					Position_Control_Disable();
				}
				
				bool posCtrlEna;
				is_Position_Control_Enabled(&posCtrlEna);
				
				//���Ÿ˿��ƴ�ֱ�ٶ�
				if( in_symmetry_range_mid( rc.data[0] , 50 , 5 ) )
					Position_Control_set_ZLock();
				else
					Position_Control_set_TargetVelocityZ( ( remove_deadband( rc.data[0] - 50.0 , 5.0 ) ) * 6 );
				//ƫ�������м���ƫ��
				//�����м����ƫ���ٶ�
				if( in_symmetry_range_mid( rc.data[1] , 50 , 5 ) )
					Attitude_Control_set_YawLock();
				else
					Attitude_Control_set_Target_YawRate( ( 50 - rc.data[1] )*0.05 );
				//ˮƽ����
				if( posCtrlEna )
				{
					//��������˿�ˮƽ�ٶ�
					if( in_symmetry_range_mid( rc.data[3] , 50 , 5 ) && in_symmetry_range_mid( rc.data[2] , 50 , 5 ) )
						Position_Control_set_XYLock();
					else
					{
						double roll_sitck_d = remove_deadband( rc.data[3] - 50.0, 5.0 );
						double pitch_sitck_d = remove_deadband( rc.data[2] - 50.0, 5.0 );
						Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit(
							pitch_sitck_d * 25 ,
							-roll_sitck_d * 25 ,
							fabs( roll_sitck_d  )*0.017,
							fabs( pitch_sitck_d )*0.017
						);
					}
				}
				else
				{
					//��������˿ظ������
					Attitude_Control_set_Target_RollPitch( 
						( rc.data[3] - 50 )*0.015,
						( rc.data[2] - 50 )*0.015
					);
				}
			}
			else
			{	//��ң���ź�
				//���Դ�ˮƽλ�ÿ�����
				Position_Control_Enable();
				
				bool posCtrlEna;
				is_Position_Control_Enabled(&posCtrlEna);
				
				if( posCtrlEna )
				{	//�Ѵ�ˮƽλ�ÿ�����
					
					if( firstAuto != 31 )
					{	//�ս���ʧ��״̬
						//ɲ����λ��
						Position_Control_set_XYLock();
						Position_Control_set_ZLock();
						if( firstAuto != 16 )
						{	//��ʼ���ȴ�
							init_NavCmdInf(&navInf);
							firstAuto = 16;
						}
						else
						{	//ɲ����ȴ�2��
							Position_ControlMode alt_mode, pos_mode;
							get_Altitude_ControlMode(&alt_mode);
							get_Position_ControlMode(&pos_mode);
							if( alt_mode==Position_ControlMode_Position && pos_mode==Position_ControlMode_Position )
							{
								if( ++navInf.counter2 >= 2*MSafeRate )
								{
									init_NavCmdInf(&navInf);								
									current_mission_ind = 0;
									firstAuto = 31;
								}
							}
						}
					}
					else
					{	//��ʼ�Զ����У�������
						
						//�����ѹǿ�ƽ���
						if( lowPowerState == 2 )
						{
							set_mav_mode( 
								MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
								PX4_CUSTOM_MAIN_MODE_AUTO,
								PX4_CUSTOM_SUB_MODE_AUTO_LAND );
							current_mission_ind = 3;
						}
						else
							set_mav_mode( 
								MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
								PX4_CUSTOM_MAIN_MODE_AUTO,
								PX4_CUSTOM_SUB_MODE_AUTO_RTL );
						
						switch( current_mission_ind )
						{
							
							case 0:
							{	//�жϷ��������Ƿ���Ҫ����																
								vector2<double> homeP;
								if( getHomeLatLon(&homeP) )
								{	//��������γ��
									
									//��ȡ����ȫ��λ��������Ϣ
									PosSensorHealthInf2 global_inf;
									if( get_OptimalGlobal_XY( &global_inf ) == false )
									{
										if( getHomePoint(&homeP) )
											goto RtLocal;
										else
										{	//�޷�����
											//ֱ�ӽ���
											current_mission_ind = 3;
											break;
										}
									}
									//��ȡָ����γ��ƽ������
									double x, y;
									map_projection_project( &global_inf.mp, homeP.x, homeP.y, &x, &y );
									x -= global_inf.HOffset.x;
									y -= global_inf.HOffset.y;
									double RtDistanceSq = sq(y - global_inf.PositionENU.y) + sq(x - global_inf.PositionENU.x);
									
									//�ж��Ƿ�������߷�������
									if( RtDistanceSq > sq(cfg.GbRtHRange[0]) )
									{	//���߷���
										RtHeight = cfg.GbRtHeight[0];
										double homeZ;
										getHomeLocalZ(&homeZ);
										vector3<double> pos;
										get_Position_Ctrl(&pos);
										if( RtHeight>0 && homeZ+RtHeight>pos.z )
										{	//�����߶ȴ��ڵ�ǰ�߶ȲŽ�������
											
										}
										else
											RtHeight = -1;
									}
									else
										RtHeight = -1;
									
									//�л�������״̬
									init_NavCmdInf(&navInf);								
									++current_mission_ind;
								}
								else if( getHomePoint(&homeP) )
								{	//������Local����
RtLocal:
									vector3<double> position;
									get_Position_Ctrl(&position);
									double RtDistanceSq = sq(homeP.y - position.y) + sq(homeP.x - position.x);
									
									//�ж��Ƿ�������߷�������
									if( RtDistanceSq > sq(cfg.LcRtHRange[0]) )
									{	//���߷���
										RtHeight = cfg.LcRtHeight[0];
										double homeZ;
										getHomeLocalZ(&homeZ);
										vector3<double> pos;
										get_Position_Ctrl(&pos);
										if( RtHeight>0 && homeZ+RtHeight>pos.z )
										{	//�����߶ȴ��ڵ�ǰ�߶ȲŽ�������
											
										}
										else
											RtHeight = -1;
									}
									else
										RtHeight = -1;
									
									//�л�������״̬
									init_NavCmdInf(&navInf);								
									++current_mission_ind;
								}
								else
								{	//�޷�����
									//ֱ�ӽ���
									current_mission_ind = 3;
									break;
								}
								break;
							}
							
							case 1:
							{	//���ߵ�ָ���Եظ߶�
								double homeZ;
								getHomeLocalZ(&homeZ);
								vector3<double> pos;
								get_Position_Ctrl(&pos);
								if( RtHeight>0 )
								{
									double params[7];
									params[0] = 0;
									params[1] = 0;
									params[2] = 0;
									params[3] = nan("");
									params[4] = 230;	params[5] = 230;
									params[6] = RtHeight*0.01;
									int16_t res = Process_NavCmd( MAV_CMD_NAV_WAYPOINT, MSafeRate, MAV_FRAME_GLOBAL_RELATIVE_ALT, params, &navInf );
									if( NavCmdRs_SuccessOrFault(res) )
									{
										init_NavCmdInf(&navInf);								
										++current_mission_ind;
									}
								}
								else
								{
									init_NavCmdInf(&navInf);								
									++current_mission_ind;
								}							
								break;
							}
							
							case 2:
							{	//�ص�Home��
								double params[7];
								params[0] = 0;
								params[1] = 0;
								params[2] = 0;
								params[3] = 0;
								params[4] = 0;	params[5] = 0;
								params[6] = 0;
								int16_t res = Process_NavCmd( MAV_CMD_NAV_RETURN_TO_LAUNCH, MSafeRate, MAV_FRAME_GLOBAL_RELATIVE_ALT, params, &navInf );
								if( NavCmdRs_SuccessOrFault(res) )
								{
									init_NavCmdInf(&navInf);								
									++current_mission_ind;
								}
								break;
							}
							
							default:
							case 3:
							{	//����
								Position_Control_set_XYLock();
								Position_Control_set_TargetVelocityZ(-50);
								break;
							}
							
						}
					}
				}
				else
				{	//�޷���λ�ÿ�����
					//����̬�ظ�ˮƽ
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					//�½�
					Position_Control_set_TargetVelocityZ(-50);
					//��λ�Զ����Ƽ�ʱ��
					firstAuto = 0;
				}
			}
			
			//�ѽ���رտ��������˳�MSafe
			bool inFlight;
			get_is_inFlight(&inFlight);
			if( inFlight==false )
			{
				Attitude_Control_Disable();
				os_delay(0.5);
				ForceMSafeCtrl = false;
			}
		}
		else
		{
			ForceMSafeCtrl = false;
			
			//��λ�Զ����Ƽ�ʱ��
			firstAuto = 0;
		}
	}
}

void init_MSafe()
{
	//ע�����
	MSafeCfg initial_cfg;
	initial_cfg.SfRtMode[0] = 1;	//SfRtMode
	initial_cfg.RtSpeed[0] = 1000;	//RtSpeed
	initial_cfg.GbRtHRange[0] = 3000;	//GbRtHRange
	initial_cfg.LcRtHRange[0] = 1500;	//LcRtHRange
	initial_cfg.GbRtHeight[0] = 5000;	//GbRtHeight
	initial_cfg.LcRtHeight[0] = 2000;	//LcRtHeight
	initial_cfg.FcRTLVolt[0] = 0;	//ǿ�Ʒ�����ѹ	
	initial_cfg.FcLandVolt[0] = 0;	//ǿ�ƽ����ѹ
	
	MAV_PARAM_TYPE param_types[] = {
		MAV_PARAM_TYPE_UINT8 ,	//SfRtMode
		MAV_PARAM_TYPE_REAL32 ,	//RtSpeed
		
		MAV_PARAM_TYPE_REAL32 ,	//GbRtHRange
		MAV_PARAM_TYPE_REAL32 ,	//LcRtHRange
		
		MAV_PARAM_TYPE_REAL32 ,	//GbRtHeight
		MAV_PARAM_TYPE_REAL32 ,	//LcRtHeight
		
		MAV_PARAM_TYPE_REAL32 ,	//ǿ�Ʒ�����ѹ		
		MAV_PARAM_TYPE_REAL32 ,	//ǿ�ƽ����ѹ
	};
	SName param_names[] = {
		"Sf_SfRtMode" ,	//SfRtMode		
		"Sf_RtSpeed" ,	//RtSpeed
		
		"Sf_GbRtHRange" ,	//GbRtHRange
		"Sf_LcRtHRange" ,	//LcRtHRange
		
		"Sf_GbRtHeight" ,	//GbRtHeight		
		"Sf_LcRtHeight" ,	//LcRtHeight
		
		"Sf_FcRTLVolt" , //ǿ�Ʒ�����ѹ
		"Sf_FcLandVolt" , //ǿ�ƽ����ѹ
	};
	ParamGroupRegister( "MSafe", 3, sizeof(MSafeCfg)/8, param_types, param_names, (uint64_t*)&initial_cfg );
	
	xTaskCreate( MSafe_Server, "MSafe", 1024, NULL, SysPriority_SafeTask, &MSafeTaskHandle);
}