#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "drv_LED.hpp"
#include "mavlink.h"
#include "MavlinksendFuncs.hpp"
#include "MavlinkRCProcess.hpp"
#include "MeasurementSystem.hpp"
#include "Sensors.hpp"
#include "Parameters.hpp"
#include <map>
#include "ctrl_Main.hpp"

using namespace std;

/*������ʾ*/
	static float ledSignalCounter = -1;	static LEDSignal ledSignal;
	static LEDMode ledmode = LEDMode_Processing1;
	static float ledR = 0, ledG = 0, ledB = 0;
	static bool buzzerOn = false;	static uint16_t buzzerFreq;

	void sendLedSignal( LEDSignal signal )
	{
		ledSignalCounter = 0;
		ledSignal = signal;
	}
	void setLedMode( LEDMode mode )
	{
		ledmode = mode;
	}
	void setLedManualCtrl( float R, float G, float B, bool BuzzerOn, uint16_t BuzzerFreq )
	{
		ledR = R;	ledG = G;	ledB = B;
		buzzerOn = BuzzerOn;	buzzerFreq = BuzzerFreq;
		ledmode = LEDMode_Manual;
	}
	static inline void LEDRefresh(float dt)
	{
		if( ledSignalCounter >= 0 )
		{
			switch(ledSignal)
			{
				case LEDSignal_Start1:
				{
					if( ledSignalCounter > 0.3 )
					{
						ledSignalCounter = -1;
						return;
					}
					if( ledSignalCounter < 0.15f )
					{
						set_BuzzerFreq(900);
						set_BuzzerOnOff(true);
						set_LedBrightness( 0, 0, 100 );
					}
					else
					{
						set_BuzzerFreq(1500);
						set_BuzzerOnOff(true);
						set_LedBrightness( 0, 100, 0 );
					}
					break;
				}
				case LEDSignal_Start2:
				{
					if( ledSignalCounter > 0.45 )
					{
						ledSignalCounter = -1;
						return;
					}
					if( ledSignalCounter < 0.15 )
					{
						set_BuzzerFreq(800);
						set_BuzzerOnOff(true);
						set_LedBrightness( 100, 0, 0 );
					}
					else if( ledSignalCounter < 0.3 )
					{
						set_BuzzerFreq(1000);
						set_BuzzerOnOff(true);
						set_LedBrightness( 0, 100, 0 );
					}
					else
					{
						set_BuzzerFreq(1200);
						set_BuzzerOnOff(true);
						set_LedBrightness( 0, 0, 100 );
					}
					break;
				}
				
				case LEDSignal_Continue1:
				{
					if( ledSignalCounter > 0.8 )
					{
						ledSignalCounter = -1;
						return;
					}
					set_BuzzerFreq(1500);				
					if( ledSignalCounter < 0.2f )
					{
						set_LedBrightness( 0, 0, 100 );
						set_BuzzerOnOff(true);
					}
					else if( ledSignalCounter < 0.4f )
					{
						set_LedBrightness( 0, 0, 0 );
						set_BuzzerOnOff(false);
					}
					else if( ledSignalCounter < 0.6f )
						set_LedBrightness( 0, 0, 100 );
					else
						set_LedBrightness( 0, 0, 0 );
					break;
				}
				
				case LEDSignal_Success1:
				{
					if( ledSignalCounter > 0.8 )
					{
						ledSignalCounter = -1;
						return;
					}
					set_BuzzerFreq(1500);
					if( ledSignalCounter < 0.2f )
					{						
						set_LedBrightness( 0, 100, 0 );
						set_BuzzerOnOff(true);
					}
					else if( ledSignalCounter < 0.4f )
					{
						set_LedBrightness( 0, 0, 0 );
						set_BuzzerOnOff(false);
					}
					else if( ledSignalCounter < 0.6f )
					{
						set_LedBrightness( 0, 100, 0 );
						set_BuzzerOnOff(true);
					}
					else
					{
						set_LedBrightness( 0, 0, 0 );
						set_BuzzerOnOff(false);
					}
					break;
				}
				
				
				case LEDSignal_Err1:
				{
					if( ledSignalCounter > 1.0 )
					{
						ledSignalCounter = -1;
						return;
					}
					set_BuzzerFreq(800);
					set_BuzzerOnOff(true);
					if( ledSignalCounter < 0.25f )
						set_LedBrightness( 100, 0, 0 );
					else if( ledSignalCounter < 0.5f )
						set_LedBrightness( 0, 0, 0 );
					else if( ledSignalCounter < 0.75f )
						set_LedBrightness( 100, 0, 0 );
					else
						set_LedBrightness( 0, 0, 0 );
					break;
				}
			}
			ledSignalCounter += dt;
			return;
		}
		
		static float counter = 0;
		switch(ledmode)
		{
			/*����ģʽ*/
				case LEDMode_Normal1:
				{
					if( counter > 2 )
						counter = 0;
					set_BuzzerOnOff(false);
					if( counter < 1 )
						set_LedBrightness( 0 , counter*100 , 0 );
					else
						set_LedBrightness( 0 , 200 - counter*100 , 0 );
					break;
				}
				case LEDMode_Normal2:
				{
					if( counter > 2 )
						counter = 0;
					set_BuzzerOnOff(false);
					if( counter < 1 )
						set_LedBrightness( 0 , 0 , counter*100 );
					else
						set_LedBrightness( 0 , 200 - counter*100 , 0 );
					break;
				}
			/*����ģʽ*/
				
			/*����ģʽ*/
				case LEDMode_Flying1:
				{
					if( counter > 1.4 )
						counter = 0;
					set_BuzzerOnOff(false);
					if( counter < 1 )
						set_LedBrightness( 0 , counter*30 , 30-counter*30 );
					else if( counter < 1.1 )
						set_LedBrightness( 0 , 100 , 0 );
					else if( counter < 1.2 )
						set_LedBrightness( 0 , 0 , 0 );
					else if( counter < 1.3 )
						set_LedBrightness( 0 , 100 , 0 );
					else
						set_LedBrightness( 0 , 0 , 0 );
					break;
				}
				case LEDMode_Flying2:
				{
					if( counter > 1.4 )
						counter = 0;
					set_BuzzerOnOff(false);
					if( counter < 1 )
						set_LedBrightness( 0 , counter*30 , 30-counter*30 );
					else if( counter < 1.1 )
						set_LedBrightness( 0 , 0 , 100 );
					else if( counter < 1.2 )
						set_LedBrightness( 0 , 0 , 0 );
					else if( counter < 1.3 )
						set_LedBrightness( 0 , 0 , 100 );
					else
						set_LedBrightness( 0 , 0 , 0 );
					break;
				}
			/*����ģʽ*/	
				
			/*������*/
				case LEDMode_Processing1:
				{
					if( counter > 0.5 )
						counter = 0;
					set_BuzzerOnOff(false);
					set_LedBrightness( 0 , 0 , counter*200 );
					break;
				}
				case LEDMode_Processing2:
				{
					if( counter > 0.5 )
						counter = 0;
					set_BuzzerOnOff(false);
					if( counter < 0.25 )
						set_LedBrightness( 0 , 100-counter*400 , counter*400 );
					else
						set_LedBrightness( 0 , (counter-0.25)*400 , 100-(counter-0.25)*400 );
					break;
				}
			/*������*/
				
			default:
			{	//�û��ֶ�����
				if(buzzerOn)
					set_BuzzerFreq(buzzerFreq);
				set_BuzzerOnOff(buzzerOn);
				set_LedBrightness( ledR, ledG, ledB );
				break;
			}
		}
		counter += dt;
	}
/*������ʾ*/

/*���ܽӿ�*/
	//���ܽӿڶ���
	typedef struct
	{
		//��ʼ������
		bool (*init)( Port port, uint32_t param );
	}PortFunc;
	static PortFunc PortFuncs[256] = {0};
	
	bool PortFunc_Register( uint8_t FuncInd, bool (*init)( Port port, uint32_t param ) )
	{
		if( FuncInd < 8 )
			return false;
		if( PortFuncs[FuncInd].init )
			return false;
		PortFuncs[FuncInd].init = init;
		return true;
	}
/*���ܽӿ�*/
	
/*�˿�*/
	static Port Ports[MAXPorts] = {0};
	//ע��˿�
	bool PortRegister( uint8_t ind, Port port )
	{
		if( ind==0 || ind>=MAXPorts )
			return false;
		if( Ports[ind].read!=0 || Ports[ind].write!=0 )
			return false;
		
		Ports[ind] = port;
		return true;
	}
/*�˿�*/
	
/*ͨ�Ŷ˿�*/
	//�˿�
	static uint8_t CommuPorts[MAVLINK_COMM_NUM_BUFFERS] = {0};
	//������Ϣ�б�
	struct SDMsg
	{
		uint16_t counter;
		uint16_t rate;
	};
	static map<uint16_t,SDMsg> SDMessages[MAVLINK_COMM_NUM_BUFFERS];
	static SemaphoreHandle_t SDMessagesMutex[MAVLINK_COMM_NUM_BUFFERS];
	
	//��ָ���˿�������Ϣ����
	bool SetMsgRate( uint8_t port_index, uint16_t Msg, float RateHz, double TIMEOUT )
	{
		if( port_index >= MAVLINK_COMM_NUM_BUFFERS )
			return false;
		if( Msg >= Mavlink_Send_Funcs_Count )
			return false;
		if( Mavlink_Send_Funcs[ Msg ] == 0 )
			return false;
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( SDMessagesMutex[port_index], TIMEOUT_Ticks ) == pdTRUE )
		{
			uint16_t Rate = 0;
			if( RateHz > 0.01f )
				Rate = 100.0f / RateHz;
			
			if( Rate==0 && RateHz!=0 )
				Rate = 1;
			map<uint16_t,SDMsg>::iterator it = SDMessages[port_index].find(Msg);
			if( it == SDMessages[port_index].end() )
			{	//�޴���Ϣ �����Ϣ
				if( Rate != 0 )
				{
					SDMsg sdmsg;
					sdmsg.rate = Rate;
					sdmsg.counter = 0;
					SDMessages[port_index].insert( pair<uint16_t,SDMsg>(Msg, sdmsg) );
				}
			}
			else
			{	//��Ϣ���� ��������
				if( Rate != 0 )
					it->second.rate = Rate;
				else
					SDMessages[port_index].erase(it);
			}
			
			xSemaphoreGive(SDMessagesMutex[port_index]);
			return true;
		}
		return false;
	}
	
	//��ָ���˿ڷ�����Ϣ�б�
	static bool sendParamListReset = false;
	void sendParamList()
	{
		sendParamListReset = true;
		ResetParametersIterator();
	}
	
	//ע��˿�����Э��ͨ��
	static bool CommuPortRegister( uint8_t ind )
	{
		if( ind==0 || ind>=MAXPorts )
			return false;
		if( Ports[ind].read==0 || Ports[ind].write==0 )
			return false;
		
		//Ѱ�ҿ��õ�λ��
		int8_t p_index = -1;
		for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
		{
			if( CommuPorts[i] == 0 )
			{
				p_index = i;
				break;
			}
		}
		//������
		if( p_index < 0 )
			return false;
		
		mavlink_init_chan( p_index );
		CommuPorts[ p_index ] = ind;
		mavlink_set_proto_version( p_index , 1 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_ATTITUDE, 200 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_GPS_RAW_INT, 2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_GPS2_RAW, 2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_SYS_STATUS, 1 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_VFR_HUD, 2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_SCALED_IMU, 200 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_SCALED_IMU2, 2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_SCALED_IMU3, 2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_RC_CHANNELS, 2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_RC_CHANNELS_SCALED, 2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_MISSION_CURRENT, 0.2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, 0.2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_SCALED_PRESSURE, 1 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_HOME_POSITION, 0.2 );
		return true;
	}
	//��ȡ�˿�
	const Port* get_CommuPort( uint8_t ind )
	{
		if( ind < MAVLINK_COMM_NUM_BUFFERS )
			return &Ports[CommuPorts[ind]];
		else
			return 0;
	}
	
	//�豸Id
	static uint8_t CommulinkSysId = 1;
	static uint8_t CommulinkCompId = 1;
	uint8_t get_CommulinkSysId(){ return CommulinkSysId; }
	uint8_t get_CommulinkCompId(){ return CommulinkCompId; }
/*ͨ�Ŷ˿�*/
	
/*Rtk�˿�*/
	//�˿�
	#define MaxRtkPortsCount 3
	static RtkPort RtkPorts[ MaxRtkPortsCount ] = {0};

	//ע��Rtk�˿�
	int8_t RtkPortRegister( RtkPort port )
	{
		if( port.write==0 || port.lock==0 || port.unlock==0 )
			return false;
		
		//Ѱ�ҿ��õ�λ��
		int8_t p_index = -1;
		for( uint8_t i = 0 ; i < MaxRtkPortsCount ; ++i )
		{
			if( RtkPorts[ i ].write == 0 )
			{
				p_index = i;
				break;
			}
		}
		//������
		if( p_index < 0 )
			return -1;
		
		RtkPorts[ p_index ] = port;
		return p_index;
	}
	//ʹ��ʧ��Rtk�˿�
	bool RtkPort_setEna( uint8_t port, bool ena )
	{
		if( port < MaxRtkPortsCount )
		{
			RtkPorts[port].ena = ena;
			return true;
		}
		return false;
	}
	//��ȡ�˿�
	const RtkPort* get_RtkPort( uint8_t port )
	{
		if( port < MaxRtkPortsCount )
			return &RtkPorts[port];
		else
			return 0;
	}
	//��rtk�˿ڷ���ע������
	void inject_RtkPorts( const uint8_t data[], uint16_t length )
	{
		for( uint8_t i = 0 ; i < MaxRtkPortsCount ; ++i )
		{
			if( RtkPorts[i].write!=0 && RtkPorts[i].ena )
				RtkPorts[i].write( data, length, 0.02, 0.02 );
		}
	}
/*Rtk�˿�*/
	
static uint16_t mav_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
static uint16_t mav_main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
static uint16_t mav_sub_mode = 0;
bool set_mav_mode_arm()
{
	//�����û�����
	bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
	if( !isMSafe && ForceMSafeCtrl )
		return false;
	
	mav_mode |= MAV_MODE_STABILIZE_ARMED;
	return true;
}
bool set_mav_mode_disarm()
{
	//�����û�����
	bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
	if( !isMSafe && ForceMSafeCtrl )
		return false;
	
	mav_mode &= ~MAV_MODE_STABILIZE_ARMED;
	return true;
}
bool set_mav_mode( uint16_t req_mav_mode, uint16_t req_mav_main_mode, uint16_t req_mav_sub_mode )
{
	//�����û�����
	bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
	if( !isMSafe && ForceMSafeCtrl )
		return false;
	
	mav_mode = req_mav_mode;
	mav_main_mode = req_mav_main_mode;
	mav_sub_mode = req_mav_sub_mode;
	return true;
}
bool get_mav_mode( uint16_t* req_mav_mode, uint16_t* req_mav_main_mode, uint16_t* req_mav_sub_mode )
{
	*req_mav_mode = mav_mode;
	*req_mav_main_mode = mav_main_mode;
	*req_mav_sub_mode = mav_sub_mode;
	return true;
}
static void Commulink_Server(void* pvParameters)
{
	//��ʼ����ʼ����
	sendLedSignal(LEDSignal_Start1);
	//�ȴ���ʼ�����
	while( getInitializationCompleted() == false )
	{
		//ˢ��led������ʾ
		LEDRefresh(0.01f);
		os_delay(0.01);
	}

	//����˿ڹ���
	if( Port_isBasicFunc(Ports[MAXPorts-1]) )
		CommuPortRegister(MAXPorts-1);
	for( uint8_t i = 1; i < MAXPorts - 1; ++i )
	{
		if( Port_isFullFunc(Ports[i]) )
		{
			char FuncName_buf[17];
			char ParamName_buf[17];
			sprintf( FuncName_buf, "Lk_Uart%dFunc", i );
			sprintf( ParamName_buf, "Lk_Uart%dParam", i );
			
			uint8_t Func_temp[8];
			uint32_t Param_temp[2];			
			if( ReadParam( FuncName_buf, 0, 0, (uint64_t*)Func_temp, 0 )==PR_OK &&
					ReadParam( ParamName_buf, 0, 0, (uint64_t*)Param_temp, 0 )==PR_OK
			)
			{
				if( Func_temp[0] == 1 )
				{	//����ΪMavlinkͨ�Ŷ˿�
					if( Param_temp[0] > 1000 )
						Ports[i].SetBaudRate(Param_temp[0],1,1);
					CommuPortRegister(i);
				}
				else if( Func_temp[0] >= 8 )
				{	//����Ϊ�Զ��幦��
					if( PortFuncs[Func_temp[0]].init )
						PortFuncs[Func_temp[0]].init( Ports[i], Param_temp[0] );
				}
			}		
		}
	}
	
	uint8_t CommulinkId_temp[8];
	ReadParam( "Lk_SysId", 0, 0, (uint64_t*)CommulinkId_temp, 0 );
	CommulinkSysId = CommulinkId_temp[0];
	ReadParam( "Lk_CompId", 0, 0, (uint64_t*)CommulinkId_temp, 0 );
	CommulinkCompId = CommulinkId_temp[0];
	
	//������������
	uint16_t HeartBeat_counter = 0;
	
	//׼ȷ������ʱ
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		//ˢ��led������ʾ
		LEDRefresh(0.01f);

		bool sendHB = false;
		if( ++HeartBeat_counter >= 100 )
		{
			HeartBeat_counter = 0;
			sendHB = true;
		}
		for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
		{	//�������ж˿�
			mavlink_message_t msg_sd;
			if( Ports[CommuPorts[i]].write != 0 )
			{	//
				if( sendHB )
				{	//����������
					if( mavlink_lock_chan(i,0.01) )
					{
						extern bool GCS_is_MP;
						px4_custom_mode custom_mode;
						custom_mode.reserved = 0;
						custom_mode.main_mode = mav_main_mode;
						if( (mav_mode&MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) && GCS_is_MP && mav_main_mode==PX4_CUSTOM_MAIN_MODE_AUTO )
							custom_mode.main_mode = 0;
						custom_mode.sub_mode = mav_sub_mode;						
						mavlink_msg_heartbeat_pack_chan( 
							get_CommulinkSysId() ,	//system id
							get_CommulinkCompId() ,	//component id
							i	,	//chan
							&msg_sd,
							MAV_TYPE_QUADROTOR ,	//type
							MAV_AUTOPILOT_PX4 ,	//autopilot
							mav_mode > 0 ? MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mav_mode : 0 ,	//base mode
							custom_mode.data ,	//custom mode
							MAV_STATE_STANDBY	//sys status
						);
						mavlink_msg_to_send_buffer( Ports[CommuPorts[i]].write, 
																			  Ports[CommuPorts[i]].lock,
																			  Ports[CommuPorts[i]].unlock,
																			 &msg_sd, 0, 0.01);
						mavlink_unlock_chan(i);
					}
				}
				
				/*������Ϣ�б��е���Ϣ*/
					#define MAX_SDMsgs 20
					uint16_t sdmsgs[MAX_SDMsgs];
					uint16_t sdmsgs_count = 0;
					if( xSemaphoreTake( SDMessagesMutex[i], 0.01*configTICK_RATE_HZ ) == pdTRUE )
					{					
						for( map<uint16_t,SDMsg>::iterator it = SDMessages[i].begin(); it != SDMessages[i].end(); ++it )
						{
							if( ++(it->second.counter) >= it->second.rate )
							{
								it->second.counter = 0;
								if( sdmsgs_count < MAX_SDMsgs )
									sdmsgs[sdmsgs_count++] = it->first;
								else
									break;
							}
						}
						xSemaphoreGive(SDMessagesMutex[i]);
					}
					for( uint16_t k = 0; k < sdmsgs_count; ++k )
					{
						if( sdmsgs[k]<Mavlink_Send_Funcs_Count && Mavlink_Send_Funcs[sdmsgs[k]]!=0 )
						{
							if( mavlink_lock_chan( i, 0.01 ) )
							{
								if( Mavlink_Send_Funcs[sdmsgs[k]]( i, &msg_sd ) )
									mavlink_msg_to_send_buffer( Ports[CommuPorts[i]].write, 
																							Ports[CommuPorts[i]].lock,
																							Ports[CommuPorts[i]].unlock,
																						  &msg_sd, 0, 0.01 );
								mavlink_unlock_chan(i);
							}
						}
					}
				/*������Ϣ�б��е���Ϣ*/
				
				/*���Ͳ����б�*/
					uint32_t param_ind; SName param_name; MAV_PARAM_TYPE param_type; uint64_t param_value;
					if( ReadCurrentParameter( &param_name, &param_ind, &param_type, &param_value, 0 ) == PR_OK )
					{
						//������
						char pname[17];
						param_name.get_CharStr(pname);
						//����ֵ
						float value = *(float*)&param_value;
						//��������
						uint32_t params_count;
						GetParametersCount(&params_count);
						
						if( mavlink_lock_chan( i, 0.01 ) )
						{
							mavlink_msg_param_value_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								i ,	//chan
								&msg_sd,
								pname,	//param id
								value ,	//param value
								param_type ,	//param type
								params_count ,	//param count
								param_ind	//param index
							);
							mavlink_msg_to_send_buffer( Ports[CommuPorts[i]].write, 
																				  Ports[CommuPorts[i]].lock,
																				  Ports[CommuPorts[i]].unlock,
																				  &msg_sd, 0, 0.01 );
							mavlink_unlock_chan(i);
						}
					}
				}
			/*���Ͳ����б�*/
			
			/*���ͺ�������*/
				//����ʱ�ٴ��������
				extern bool RqMissionInt[MAVLINK_COMM_NUM_BUFFERS];
				extern int32_t RqMissionInd[MAVLINK_COMM_NUM_BUFFERS];
				extern int32_t RqMissionCounter[MAVLINK_COMM_NUM_BUFFERS];
				extern uint8_t RqMissiontarget_sysid[MAVLINK_COMM_NUM_BUFFERS];
				extern uint8_t RqMissiontarget_compid[MAVLINK_COMM_NUM_BUFFERS];
				
				for( uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i )
				{
					if( RqMissionCounter[i] > 0 )
					{
						--RqMissionCounter[i];
						if( (RqMissionCounter[i] & 0xf) == 0 )
						{	//��ʱ��������
							if( RqMissionInd[i] == 0 )
							{	//0�ź���ͬʱ����int����ͨ����
								const Port* port = get_CommuPort(i);
								if( port->write != 0 )
								{
									mavlink_message_t msg_sd;
									if( mavlink_lock_chan( i, 0.01 ) )
									{
										mavlink_msg_mission_request_int_pack_chan( 
											get_CommulinkSysId() ,	//system id
											get_CommulinkCompId() ,	//component id
											i ,	//chan
											&msg_sd,
											RqMissiontarget_sysid[i] ,	//target system
											RqMissiontarget_compid[i] ,	//target component
											0 ,	//seq
											MAV_MISSION_TYPE_MISSION	//mission type
										
										);
										mavlink_msg_to_send_buffer(port->write, 
																							 port->lock,
																							 port->unlock,
																							 &msg_sd, 0, 0.01);
										mavlink_unlock_chan(i);
									}
									
									if( mavlink_lock_chan( i, 0.01 ) )
									{
										mavlink_msg_mission_request_pack_chan( 
											get_CommulinkSysId() ,	//system id
											get_CommulinkCompId() ,	//component id
											i ,	//chan
											&msg_sd,
											RqMissiontarget_sysid[i] ,	//target system
											RqMissiontarget_compid[i] ,	//target component
											0 ,	//seq
											MAV_MISSION_TYPE_MISSION	//mission type
										
										);
										mavlink_msg_to_send_buffer(port->write, 
																							 port->lock,
																							 port->unlock,
																							 &msg_sd, 0, 0.01);
										mavlink_unlock_chan(i);
									}
								}
							}
							else if( RqMissionInt[i] )
							{
								const Port* port = get_CommuPort(i);
								if( port->write != 0 )
								{
									mavlink_message_t msg_sd;
									if( mavlink_lock_chan( i, 0.01 ) )
									{
										mavlink_msg_mission_request_int_pack_chan( 
											get_CommulinkSysId() ,	//system id
											get_CommulinkCompId() ,	//component id
											i ,	//chan
											&msg_sd,
											RqMissiontarget_sysid[i] ,	//target system
											RqMissiontarget_compid[i] ,	//target component
											RqMissionInd[i] ,	//seq
											MAV_MISSION_TYPE_MISSION	//mission type
										
										);
										mavlink_msg_to_send_buffer(port->write, 
																							 port->lock,
																							 port->unlock,
																							 &msg_sd, 0, 0.01);
										mavlink_unlock_chan(i);
									}
								}
							}
							else
							{
								const Port* port = get_CommuPort(i);
								if( port->write != 0 )
								{
									mavlink_message_t msg_sd;									
									if( mavlink_lock_chan( i, 0.01 ) )
									{
										mavlink_msg_mission_request_pack_chan( 
											get_CommulinkSysId() ,	//system id
											get_CommulinkCompId() ,	//component id
											i ,	//chan
											&msg_sd,
											RqMissiontarget_sysid[i] ,	//target system
											RqMissiontarget_compid[i] ,	//target component
											RqMissionInd[i] ,	//seq
											MAV_MISSION_TYPE_MISSION	//mission type
										
										);
										mavlink_msg_to_send_buffer(port->write, 
																							 port->lock,
																							 port->unlock,
																							 &msg_sd, 0, 0.01);
										mavlink_unlock_chan(i);
									}
								}
							}
						}
					}
				}
			/*���ͺ�������*/
				
			if( Ports[CommuPorts[i]].read != 0 )
			{	//�������ݴ���
				//ÿ�ν���64���ֽڽ��д���
				mavlink_message_t msg;
				uint8_t buf[64];
				uint8_t length;			
				do
				{
					length = Ports[CommuPorts[i]].read( buf, 64, 0, 0.01 );
					for( uint8_t k = 0; k < length; ++k )
					{
						//��Ϣ���
						if( mavlink_parse_char( i, buf[k], &msg, NULL ) == MAVLINK_FRAMING_OK )
						{
							//��Ϣ������
							
							//�����Ϣ����������
							//������Ϣ
							if( msg.msgid < Mavlink_RC_Process_Count )
							{
								if( Mavlink_RC_Process[ msg.msgid ] != 0 )
									Mavlink_RC_Process[ msg.msgid ]( i , &msg );
							}
						}
					}
				}while( length > 10 );
			}
		}
		if( sendParamListReset == false )
			ParameterIteratorMoveNext();
		else
			sendParamListReset = false;
		vTaskDelayUntil( &xLastWakeTime, 0.01*configTICK_RATE_HZ );
	}
}

void init_Commulink()
{
	//��ʼ��������
	for( uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i )
	{
		SDMessagesMutex[i] = xSemaphoreCreateMutex();
	}
	
	//ע��ͨ�Ų���	
	CommulinkConfig initial_cfg;
	initial_cfg.sys_id[0] = 1;
	initial_cfg.comp_id[0] = 1;
	initial_cfg.Uart1_Func[0] = 1;
	initial_cfg.Uart1_Param[0] = 115200;
	initial_cfg.Uart3_Func[0] = 100;
	initial_cfg.Uart3_Param[0] = 0;
	initial_cfg.Uart5_Func[0] = 32;
	initial_cfg.Uart5_Param[0] = 0;
	initial_cfg.Uart7_Func[0] = 52;
	initial_cfg.Uart7_Param[0] = 0;
	initial_cfg.Uart8_Func[0] = 12;
	initial_cfg.Uart8_Param[0] = 0;
	MAV_PARAM_TYPE param_types[] = {
		MAV_PARAM_TYPE_UINT8 ,	//sysid
		MAV_PARAM_TYPE_UINT8 ,	//compid
		MAV_PARAM_TYPE_UINT8 ,	//Uart1_Func
		MAV_PARAM_TYPE_UINT32 ,	//Uart1_Param
		MAV_PARAM_TYPE_UINT8 ,	//Uart3_Func
		MAV_PARAM_TYPE_UINT32 ,	//Uart3_Param
		MAV_PARAM_TYPE_UINT8 ,	//Uart5_Func
		MAV_PARAM_TYPE_UINT32 ,	//Uart5_Param
		MAV_PARAM_TYPE_UINT8 ,	//Uart7_Func
		MAV_PARAM_TYPE_UINT32 ,	//Uart7_Param
		MAV_PARAM_TYPE_UINT8 ,	//Uart8_Func
		MAV_PARAM_TYPE_UINT32 ,	//Uart8_Param
	};
	SName param_names[] = {
		"Lk_SysId" ,	//sysid
		"Lk_CompId" ,	//compid
		"Lk_Uart1Func" ,
		"Lk_Uart1Param" ,
		"Lk_Uart3Func" ,
		"Lk_Uart3Param" ,
		"Lk_Uart5Func" ,
		"Lk_Uart5Param" ,
		"Lk_Uart7Func" ,
		"Lk_Uart7Param" ,
		"Lk_Uart8Func" ,
		"Lk_Uart8Param" ,
	};
	ParamGroupRegister( "Commulink", 5, sizeof(initial_cfg)/8, param_types, param_names, (uint64_t*)&initial_cfg );
	
	xTaskCreate( Commulink_Server, "Commulink", 2048, NULL, SysPriority_UserTask, NULL);
}