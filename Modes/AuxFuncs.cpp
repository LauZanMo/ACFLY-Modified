#include "AuxFuncs.hpp"
#include "Basic.hpp"
#include "Parameters.hpp"
#include "Receiver.hpp"
#include "drv_PWMOut.hpp"
#include "Commulink.hpp"
#include "StorageSystem.hpp"
#include "MeasurementSystem.hpp"
#include "ControlSystem.hpp"
#include "event_groups.h"
#include "semphr.h"
//保存之前通道值用于触发
static double last_Channel_values[16];
//云台是否自动控制
static bool GimbalCtrl_atLocked[16];

//拍照次数
static uint16_t PhotoCnt = 0;
//相片序号
static uint16_t PhotoIndex = 1;

//发送完成标志
static EventGroupHandle_t IO_events = xEventGroupCreate();

/*热靴触发*/
	static SemaphoreHandle_t PosLogMutex = xSemaphoreCreateMutex();
	static bool SD_Pos_Record()
	{
		//获取时间
		RTC_TimeStruct RTC_Time;
		RTC_Time = Get_RTC_Time();
		//获取速度
		vector3<double> vel;
		get_VelocityENU_Ctrl(&vel);	
		
		//获取姿态
		Quaternion airframe_quat;
		get_Attitude_quat(&airframe_quat);
		airframe_quat.Enu2Ned();	
		
		static int8_t global_pos_ind = -1;
		PosSensorHealthInf3 posInf;
		if( global_pos_ind < 0 )
		{	//第一次获取最优位置传感器
			if( get_OptimalGlobal_XYZ(&posInf) )
				global_pos_ind = posInf.sensor_ind;
		}
		else
			get_PosSensorHealth_XYZ( &posInf, global_pos_ind );
		
		double lat = 0;
		double lon = 0;
		double alt = 0;
		double accN = 999999;
		double accE = 999999;
		double accD = 999999;
		uint8_t fix_type = 0;
		uint16_t week = 0;
		double TOW = 0;
		
		if( global_pos_ind >= 0 )
		{
			Position_Sensor gps_sensor;
			if( GetPositionSensor( global_pos_ind, &gps_sensor ) )
			{
				if( gps_sensor.available && gps_sensor.sensor_type==Position_Sensor_Type_GlobalPositioning )
				{
					//计算经纬度
					map_projection_reproject( &posInf.mp, 
						posInf.PositionENU.x+posInf.HOffset.x, 
						posInf.PositionENU.y+posInf.HOffset.y,
						&lat, &lon );
					//高度
					alt = posInf.PositionENU.z + posInf.HOffset.z;
					alt *= 0.01;
					//精度
					accN = gps_sensor.addition_inf[4]*0.01;
					accE = gps_sensor.addition_inf[4]*0.01;
					accD = gps_sensor.addition_inf[5]*0.01;
					//week
					week = gps_sensor.addition_inf[2];
					//TOW
					TOW = gps_sensor.addition_inf[3];
					//fix
					if( gps_sensor.addition_inf[1]==1 )
						fix_type=0;
					else if( gps_sensor.addition_inf[1]==5 )
						fix_type=34;
					else if( gps_sensor.addition_inf[1]==6 )
						fix_type=50;
					else
						fix_type=16;
				}
			}
		}		

		xSemaphoreTake( PosLogMutex, portMAX_DELAY );
		
			uint16_t photo_ind = PhotoIndex;
			char pos_txt_buf[200];
			int n = sprintf(
				pos_txt_buf,
				"%4d\t%6.6f\t[%4d]\t%4d,N\t%4d,E\t%4d,V \t%11.8f,Lat \t%11.8f,Lon \t%6.3f,Ellh \t%9.6f,    %9.6f,    %9.6f,    %2d,Q\r\n",
				PhotoIndex++,
				TOW,
				week,
				0,
				0,
				0,
				lat,
				lon,
				alt,
				accN,
				accE,
				accD,	
				fix_type
			);			
			
		xSemaphoreGive(PosLogMutex);
		
		if(!SDLog_Txt1( pos_txt_buf, n ))
			return false;
		
		//发送拍照信息到地面站
		double Altitude_Local=0;
		vector3<double> Position;
		get_Position_Ctrl(&Position);
		double homeZ;
		double heightAboveGround = 0;
		if( getHomeLocalZ( &homeZ, 0.01 ) )
			heightAboveGround = Position.z - homeZ;
		
		mavlink_message_t msg_sd;
		for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
		{	//遍历所有端口		
			if( mavlink_lock_chan( i, 2/configTICK_RATE_HZ ) )
			{
				mavlink_msg_camera_feedback_pack_chan(
					get_CommulinkSysId() ,	// system id
					get_CommulinkCompId() ,	// component id
					i ,
					&msg_sd,	  
					TIME::get_System_Run_Time() * 1e3 , // boot ms					
					1, //target_system  System ID
					global_pos_ind, // camera_id
					photo_ind, //Image index
					lat*1e7, // lat [degE7]
					lon*1e7, // lon [degE7]
					alt,  // alt_msl [m] Altitude (MSL).
					heightAboveGround*0.01, //alt_rel [m] Altitude (Relative to HOME location).
					0, //Camera Roll
					0, //Camera Pitch 
					0, //Camera Yaw
					30, //foc_len [mm] Focal Length
					0,  //0:Shooting photos, not video
					photo_ind	//Completed image captures
				);
				const Port* port = get_CommuPort(i);
				if(port->write){
					mavlink_msg_to_send_buffer(port->write, 
																	 port->lock,
																	 port->unlock,
																	 &msg_sd, 0, 2/configTICK_RATE_HZ);
				}
				mavlink_unlock_chan(i);	
			}	
		}
		return true;
	}

/*热靴触发*/

void init_process_AuxFuncs()
{
	Receiver rc;
	if( getReceiver(&rc) )
	{
		//复位保存之前通道
		for( uint8_t i = 0; i < rc.raw_available_channels; ++i )
			last_Channel_values[i] = rc.raw_data[i];
		for( uint8_t i = rc.raw_available_channels; i < 16; ++i )
			last_Channel_values[i] = -200;
	}
	else
	{
		//复位保存之前通道
		for( uint8_t i = 0; i < 16; ++i )
			last_Channel_values[i] = -200;
	}
	//复位运动自动控制标志
	for( uint8_t i = 0; i < 16; ++i )
		GimbalCtrl_atLocked[i] = false;
}

static SemaphoreHandle_t CamMutex = xSemaphoreCreateRecursiveMutex();
bool AuxCamTakePhoto()
{
	if( getInitializationCompleted() == false )
		return false;
	uint8_t cam_chans = 0;
	AuxFuncsConfig aux_configs;
	ReadParamGroup( "AuxCfg", (uint64_t*)&aux_configs, 0 );
	if( xSemaphoreTakeRecursive( CamMutex, 0.1*configTICK_RATE_HZ ) )
	{
		uint8_t MainMotorCount = get_MainMotorCount();		
		
		//拉低（拉高）相机PWM
		for( uint8_t i = MainMotorCount; i < PWMChannelsCount; ++i )
		{
			uint8_t aux_cfg = ((uint8_t*)&aux_configs)[i*8];
			if( aux_cfg>=25 && aux_cfg<=48 )
			{
				Aux_PWM_Out( aux_configs.Aux_CamOnPwm[0]*0.1-100, i );
				++cam_chans;
			}
		}
		
		//无相机返回
		if(cam_chans==0)
		{
			xSemaphoreGiveRecursive(CamMutex);
			return false;
		}
		
		//拉高（拉低）相机PWM
		os_delay(aux_configs.Aux_CamShTime[0]);
		for( uint8_t i = MainMotorCount; i < PWMChannelsCount; ++i )
		{
			uint8_t aux_cfg = ((uint8_t*)&aux_configs)[i*8];
			if( aux_cfg>=25 && aux_cfg<=40 )
				Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
		}
		
		xSemaphoreTake( PosLogMutex, portMAX_DELAY );
			++PhotoCnt;
		xSemaphoreGive(PosLogMutex);
		
		mavlink_message_t msg_sd;
		for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
		{	//遍历所有端口		
			if( mavlink_lock_chan( i, 2/configTICK_RATE_HZ ) )
			{
				mavlink_msg_camera_status_pack_chan(
					get_CommulinkSysId() ,	// system id
					get_CommulinkCompId() ,	// component id
					i ,
					&msg_sd,	  
					TIME::get_System_Run_Time() * 1e6 , // boot us	
					255,	//target_system
					1, //cam index
					PhotoCnt, //image index
					1	,	//event id
					0, // p1
					0, // p2
					0, // p3
					0 //p4
				);
				const Port* port = get_CommuPort(i);
				if(port->write){
					mavlink_msg_to_send_buffer(port->write, 
																	 port->lock,
																	 port->unlock,
																	 &msg_sd, 0, 2/configTICK_RATE_HZ);
				}
				mavlink_unlock_chan(i);	
			}	
		}
		SD_Pos_Record();
		
		xSemaphoreGiveRecursive(CamMutex);
		return true;
	}
	else
		return false;
}

void process_AuxFuncs(const Receiver* rc)
{
	AuxFuncsConfig aux_configs;
	ReadParamGroup( "AuxCfg", (uint64_t*)&aux_configs, 0 );

	uint8_t MainMotorCount = get_MainMotorCount();
	if( xSemaphoreTakeRecursive( CamMutex, 0 ) )	
	{
		for( uint8_t i = MainMotorCount; i < PWMChannelsCount; ++i )
		{
			uint8_t aux_cfg = ((uint8_t*)&aux_configs)[i*8];
			if( aux_cfg>=1 && aux_cfg<=16 )
			{	//映射遥控器通道
				if( rc->available )
				{
					uint8_t ref_chan = aux_cfg - 1;
					if( rc->raw_available_channels > ref_chan )
						Aux_PWM_Out( rc->raw_data[ref_chan], i );
				}
			}
			else if( aux_cfg>=25 && aux_cfg<=48 )
			{	//用遥控器对应通道进行相机快门触发（raw_data）
				if( rc->available )
				{
					uint8_t ref_chan = aux_cfg - 25;
					if( rc->raw_available_channels > ref_chan )
					{
						if( fabs(rc->raw_data[ref_chan] - last_Channel_values[ref_chan]) > 15 )
						{	//触发相机
							if( last_Channel_values[ref_chan] > -100 )							
							{
								AuxCamTakePhoto();					
							}							
							last_Channel_values[ref_chan] = rc->raw_data[ref_chan];
						}
						else
							Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
					}
					else
						Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
				}
				else
					Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
			}
			else if( aux_cfg>=49 && aux_cfg<=72 )
			{	//用遥控器对应通道进行云台控制（raw_data）
				if( rc->available )
				{
					uint8_t ref_chan = aux_cfg - 49;
					if( rc->raw_available_channels > ref_chan )
					{
						if( GimbalCtrl_atLocked[i] )
						{	//云台自动控制
							//通道变化大于阈值才调整云台
							if( fabs(rc->raw_data[ref_chan] - last_Channel_values[ref_chan]) > 10 )
							{
								Aux_PWM_Out( rc->raw_data[ref_chan], i );
								last_Channel_values[ref_chan] = rc->raw_data[ref_chan];
								GimbalCtrl_atLocked[i] = false;
							}
						}
						else
						{	//云台手动控制
							Aux_PWM_Out( rc->raw_data[ref_chan], i );
							last_Channel_values[ref_chan] = rc->raw_data[ref_chan];
						}
					}
				}
			}
		}
		xSemaphoreGiveRecursive(CamMutex);
	}
}


bool AuxGimbalSetAngle( double angle )
{
	AuxFuncsConfig aux_configs;
	ReadParamGroup( "AuxCfg", (uint64_t*)&aux_configs, 0 );
	
	uint8_t AuxChannelsCount = get_AuxChannelCount();
	for( uint8_t i = 0; i < AuxChannelsCount; ++i )
	{
		uint8_t aux_cfg = ((uint8_t*)&aux_configs)[i*8];
		if( aux_cfg>=49 && aux_cfg<=72 )
		{
			Aux_PWM_Out( angle*100/90, i );
			GimbalCtrl_atLocked[i] = true;
		}
	}
	return true;
}

void init_AuxFuncs()
{
	//注册通信参数	
	AuxFuncsConfig initial_cfg;
	initial_cfg.Aux1Func[0] = 0;
	initial_cfg.Aux2Func[0] = 0;
	initial_cfg.Aux3Func[0] = 0;
	initial_cfg.Aux4Func[0] = 0;
	initial_cfg.Aux5Func[0] = 0;
	initial_cfg.Aux6Func[0] = 0;
	initial_cfg.Aux7Func[0] = 0;
	initial_cfg.Aux8Func[0] = 0;
	initial_cfg.Aux_CamOnPwm[0] = 2000;
	initial_cfg.Aux_CamOffPwm[0] = 1000;
	initial_cfg.Aux_CamShTime[0] = 0.1;
	MAV_PARAM_TYPE param_types[] = {
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_REAL32 ,
	};
	SName param_names[] = {
		"Aux_1Func" ,
		"Aux_2Func" ,
		"Aux_3Func" ,
		"Aux_4Func" ,
		"Aux_5Func" ,
		"Aux_6Func" ,
		"Aux_7Func" ,
		"Aux_8Func" ,
		"Aux_CamOnPwm" ,
		"Aux_CamOffPwm" ,
		"Aux_CamShTime" ,
	};
	ParamGroupRegister( "AuxCfg", 1,sizeof(initial_cfg)/8, param_types, param_names, (uint64_t*)&initial_cfg );
}