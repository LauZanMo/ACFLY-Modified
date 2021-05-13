#include "MavlinkRCProcess.hpp"
#include "mavlink.h"
#include "Commulink.hpp"
#include "Parameters.hpp"
#include "MavlinkCMDProcess.hpp"

#include "Basic.hpp"
#include "AC_Math.hpp"
#include "MeasurementSystem.hpp"
#include "Modes.hpp"
#include "Missions.hpp"
#include "ControlSystem.hpp"
#include "Sensors.hpp"
#include "AuxFuncs.hpp"
#include "SensorsBackend.hpp"
bool GCS_is_MP = false;
static void Msg0_HEARTBEAT( uint8_t Port_index , const mavlink_message_t* msg )
{
	const mavlink_heartbeat_t* msg_rd = (mavlink_heartbeat_t*)msg->payload64;
	GCS_is_MP = msg_rd->base_mode == 0;
	//�Է���mavlink1����mavlink1Э��
	if( msg->magic == MAVLINK_STX )
		mavlink_set_proto_version( Port_index , 2 );
	else
		mavlink_set_proto_version( Port_index , 1 );
}

//�л�ģʽ
static void Msg11_SET_MODE( uint8_t Port_index , const mavlink_message_t* msg )
{
	const mavlink_set_mode_t* msg_rd = (mavlink_set_mode_t*)msg->payload64;
	ModeMsg mode_msg;
	mode_msg.cmd_type = CMD_TYPE_MAVLINK | Port_index;
	mode_msg.sd_sysid = msg->sysid;
	mode_msg.sd_compid = msg->compid;
	mode_msg.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	mode_msg.cmd = 176;
	mode_msg.params[0] = msg_rd->base_mode;
	mode_msg.params[1] = msg_rd->custom_mode;
	mode_msg.params[2] = 0;
	mode_msg.params[3] = 0;
	mode_msg.params[4] = 0;
	mode_msg.params[5] = 0;
	mode_msg.params[6] = 0;
	SendMsgToMode( mode_msg, 0.01 );
}

/*����Э��*/
	static void Msg20_PARAM_REQUEST_READ( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_param_request_read_t* msg_rd = (mavlink_param_request_read_t*)msg->payload64;
			
		if( msg_rd->param_index < 0 )
		{
			uint32_t index;	MAV_PARAM_TYPE param_type; uint64_t param_value;
			if( ReadParam( msg_rd->param_id, &index, &param_type, &param_value, 0 ) == PR_OK )
			{
				const Port* port = get_CommuPort( Port_index );
				
				if( port != 0 )
				{
					//����ֵ
					float value = *(float*)&param_value;
					//��������
					uint32_t params_count;
					GetParametersCount(&params_count);
					
					mavlink_message_t msg_sd;
					if( mavlink_lock_chan( Port_index, 0.01 ) )
					{
						mavlink_msg_param_value_pack_chan( 
							get_CommulinkSysId() ,	//system id
							get_CommulinkCompId() ,	//component id
							Port_index ,	//chan
							&msg_sd,
							msg_rd->param_id,	//param id
							value ,	//param value
							param_type ,	//param type
							params_count ,	//param count
							index	//param index
						);
						mavlink_msg_to_send_buffer(port->write, 
																			 port->lock,
																			 port->unlock,
																			 &msg_sd, 0, 0.01);
						mavlink_unlock_chan(Port_index);
					}
				}
			}
		}
		else
		{
			SName param_name;	MAV_PARAM_TYPE param_type; uint64_t param_value;
			if( ReadParam( (uint32_t)msg_rd->param_index, &param_name, &param_type, &param_value, 0 ) == PR_OK )
			{
				const Port* port = get_CommuPort( Port_index );
				
				if( port && port->write )
				{
					//������
					char pname[17];
					param_name.get_CharStr(pname);
					//����ֵ					
					float value = *(float*)&param_value;
					//��������
					uint32_t params_count;
					GetParametersCount(&params_count);
					
					mavlink_message_t msg_sd;
					if( mavlink_lock_chan( Port_index, 0.01 ) )
					{
						mavlink_msg_param_value_pack_chan( 
							get_CommulinkSysId() ,	//system id
							get_CommulinkCompId() ,	//component id
							Port_index ,	//chan
							&msg_sd,
							pname,	//param id
							value ,	//param value
							param_type ,	//param type
							params_count ,	//param count
							msg_rd->param_index	//param index
						);
						mavlink_msg_to_send_buffer(port->write, 
																			 port->lock,
																			 port->unlock,
																			 &msg_sd, 0, 0.01);
						mavlink_unlock_chan(Port_index);
					}
				}
			}
		}
	}

  
	static TIME message_send_time(1);
	static void Msg21_PARAM_REQUEST_LIST( uint8_t Port_index , const mavlink_message_t* msg )
	{	
		char text[100];
		IMU_Sensor imu_sensors;
		Position_Sensor pos_sensors;
		if(	message_send_time.get_pass_time() > 2	){
			for( uint8_t j = 0 ; j < 5 ; ++j )
			{
				if( j == 0 ){
					sprintf(text,"Welcome to ACFLY!");
				}else if( j == 1 ){
					float Firmware_V[2]={0};					
					ReadParam( "Init_Firmware_V", 0, 0, (uint64_t*)Firmware_V, 0 );
					sprintf(text,"Firware Version:A9 %g - official",Firmware_V[0]);
				}else if( j == 2 ){
					if( !GetAccelerometer(0, &imu_sensors) )
						continue;	
					sprintf(text,"IMU:");
					imu_sensors.name.get_CharStr(&text[4]);				
				}else if( j == 3 ){
					if( !GetMagnetometer(Internal_Magnetometer_Index, &imu_sensors) )
						continue;
					sprintf(text,"MagInternal:");
					imu_sensors.name.get_CharStr(&text[12]);
				}else if( j == 4 ){
					if( !GetMagnetometer(External_Magnetometer_Index, &imu_sensors) )
						continue;
					sprintf(text,"MagExternal:");
					imu_sensors.name.get_CharStr(&text[12]);				
				}
										
				const Port* port = get_CommuPort(Port_index);
				if( port && port->write )
				{	
					mavlink_message_t msg_sd;
					if(mavlink_lock_chan( Port_index, 0.01 )){
						mavlink_msg_statustext_pack_chan( 
						get_CommulinkSysId() ,	//system id
						get_CommulinkCompId() ,	//component id
						Port_index ,	//chan
						&msg_sd,
						MAV_SEVERITY_INFO,
						text
						);				
						mavlink_msg_to_send_buffer(port->write, 
							 port->lock,
							 port->unlock,
							 &msg_sd, 0, 0.01);
						mavlink_unlock_chan(Port_index);	
					}
				}
			}
			message_send_time = TIME::now();
		}
		sendParamList();		
	}

	static void Msg23_PARAM_SET( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_param_set_t* msg_rd = (mavlink_param_set_t*)msg->payload64;
		uint32_t index;	MAV_PARAM_TYPE param_type; uint64_t param_value;
		if( ReadParam( msg_rd->param_id, &index, &param_type, &param_value, 0 ) == PR_OK )
		{
			float value_f = msg_rd->param_value;
			uint32_t value = *(uint32_t*)&value_f;
			
			if(!strcmp(msg_rd->param_id,"Init_Boot_Count") || !strcmp(msg_rd->param_id,"Init_Firmware_V"))
			{
				return;
			}
			
			if( UpdateParam( msg_rd->param_id, value ) == PR_OK )
			{		
				//��������
				uint32_t params_count;
				GetParametersCount(&params_count);
				
				//��ÿ���˿ڷ���PARAM_VALUE
				for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
				{
					const Port* port = get_CommuPort(i);
					if( port->write != 0 )
					{	
						mavlink_message_t msg_sd;
						if( mavlink_lock_chan( i, 0.01 ) )
						{
							mavlink_msg_param_value_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								i ,	//chan
								&msg_sd,
								msg_rd->param_id,	//param id
								msg_rd->param_value ,	//param value
								param_type ,	//param type
								params_count ,	//param count
								index	//param index
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
/*����Э��*/
	
/*��������Э��*/
	static void Msg43_MISSION_REQUEST_LIST( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_mission_request_list_t* msg_rd = (mavlink_mission_request_list_t*)msg->payload64;
		
		const Port* port = get_CommuPort(Port_index);
		if( port->write != 0 )
		{
			mavlink_message_t msg_sd;
			if( mavlink_lock_chan( Port_index, 0.01 ) )
			{
				mavlink_msg_mission_count_pack_chan( 
					get_CommulinkSysId() ,	//system id
					get_CommulinkCompId() ,	//component id
					Port_index ,	//chan
					&msg_sd,
					msg->sysid ,	//target system
					msg->compid ,	//target component
					getMissionsCount() ,	//count
					MAV_MISSION_TYPE_MISSION	//mission type
				);				
				mavlink_msg_to_send_buffer(port->write, 
																	 port->lock,
																	 port->unlock,
																	 &msg_sd, 0, 0.01);
				mavlink_unlock_chan(Port_index);
			}
		}
	}
	
	static void Msg40_MISSION_REQUEST( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_mission_request_t* msg_rd = (mavlink_mission_request_t*)msg->payload64;
		
		const Port* port = get_CommuPort(Port_index);
		if( port->write != 0 )
		{
			MissionInf wp_inf;
			if( ReadMission( msg_rd->seq, &wp_inf ) )
			{		
//				mavlink_message_t msg_sd;
//				mavlink_lock_chan( Port_index, 0.01 );
//				mavlink_msg_mission_item_pack_chan( 
//					get_CommulinkSysId() ,	//system id
//					get_CommulinkCompId() ,	//component id
//					Port_index ,	//chan
//					&msg_sd,
//					msg->sysid ,	//target system
//					msg->compid ,	//target component
//					msg_rd->seq ,	//seq
//					wp_inf.frame, //frame
//					wp_inf.cmd , //command
//					(msg_rd->seq==getCurrentMissionInd()) ? 1:0 ,	//current
//					wp_inf.autocontinue ,	//autocontinue
//					wp_inf.params[0] ,	//hold time(decimal seconds)
//					wp_inf.params[1] ,	//acceptance radius in meters(hit waypoint)
//					wp_inf.params[2] ,	//radius in meters to pass the waypoint
//					wp_inf.params[3] ,	//desired yaw angle
//					wp_inf.params[4], wp_inf.params[5], wp_inf.params[6], //x y z
//					MAV_MISSION_TYPE_MISSION
//				);				
//				mavlink_msg_to_send_buffer(port->write, 
//																	 port->lock,
//																	 port->unlock,
//																	 &msg_sd, 0, 0.01);
//				mavlink_unlock_chan(Port_index);


				double x, y;
				switch(wp_inf.frame)
				{
					case MAV_FRAME_GLOBAL:
					case MAV_FRAME_GLOBAL_RELATIVE_ALT:
					case MAV_FRAME_GLOBAL_INT:
					case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
					case MAV_FRAME_GLOBAL_TERRAIN_ALT:
					case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
						x = wp_inf.params[4]*1e7;
						y = wp_inf.params[5]*1e7;
						break;
					default:
						x = wp_inf.params[4]*1e4;
						y = wp_inf.params[5]*1e4;
						break;
				}		
				
				mavlink_message_t msg_sd;
				if( mavlink_lock_chan( Port_index, 0.01 ) )
				{
					mavlink_msg_mission_item_int_pack_chan( 
						get_CommulinkSysId() ,	//system id
						get_CommulinkCompId() ,	//component id
						Port_index ,	//chan
						&msg_sd,
						msg->sysid ,	//target system
						msg->compid ,	//target component
						msg_rd->seq ,	//seq
						wp_inf.frame , //frame
						wp_inf.cmd , //command
						(msg_rd->seq==getCurrentMissionInd()) ? 1:0 ,	//current
						wp_inf.autocontinue ,	//autocontinue
						wp_inf.params[0] ,	//hold time(decimal seconds)
						wp_inf.params[1] ,	//acceptance radius in meters(hit waypoint)
						wp_inf.params[2] ,	//radius in meters to pass the waypoint
						wp_inf.params[3] ,	//desired yaw angle
						x, y, wp_inf.params[6], //x y z
						MAV_MISSION_TYPE_MISSION
					);				
					mavlink_msg_to_send_buffer(port->write, 
																		 port->lock,
																		 port->unlock,
																		 &msg_sd, 0, 0.01);
					mavlink_unlock_chan(Port_index);
				}
			}
		}
	}
	
	static void Msg51_MISSION_REQUEST_INT( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_mission_request_int_t* msg_rd = (mavlink_mission_request_int_t*)msg->payload64;
		
		const Port* port = get_CommuPort(Port_index);
		if( port->write != 0 )
		{
			MissionInf wp_inf;
			if( ReadMission( msg_rd->seq, &wp_inf ) )
			{		
				double x, y;
				switch(wp_inf.frame)
				{
					case MAV_FRAME_GLOBAL:
					case MAV_FRAME_GLOBAL_RELATIVE_ALT:
					case MAV_FRAME_GLOBAL_INT:
					case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
					case MAV_FRAME_GLOBAL_TERRAIN_ALT:
					case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
						x = wp_inf.params[4]*1e7;
						y = wp_inf.params[5]*1e7;
						break;
					default:
						x = wp_inf.params[4]*1e4;
						y = wp_inf.params[5]*1e4;
						break;
				}		
				
				mavlink_message_t msg_sd;
				if( mavlink_lock_chan( Port_index, 0.01 ) )
				{
					mavlink_msg_mission_item_int_pack_chan( 
						get_CommulinkSysId() ,	//system id
						get_CommulinkCompId() ,	//component id
						Port_index ,	//chan
						&msg_sd,
						msg->sysid ,	//target system
						msg->compid ,	//target component
						msg_rd->seq ,	//seq
						wp_inf.frame , //frame
						wp_inf.cmd , //command
						(msg_rd->seq==getCurrentMissionInd()) ? 1:0 ,	//current
						wp_inf.autocontinue ,	//autocontinue
						wp_inf.params[0] ,	//hold time(decimal seconds)
						wp_inf.params[1] ,	//acceptance radius in meters(hit waypoint)
						wp_inf.params[2] ,	//radius in meters to pass the waypoint
						wp_inf.params[3] ,	//desired yaw angle
						x, y, wp_inf.params[6], //x y z
						MAV_MISSION_TYPE_MISSION
					);				
					mavlink_msg_to_send_buffer(port->write, 
																		 port->lock,
																		 port->unlock,
																		 &msg_sd, 0, 0.01);
					mavlink_unlock_chan(Port_index);
				}
			}
		}
	}
	
	static void Msg41_MISSION_SET_CURRENT( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_mission_set_current_t* msg_rd = (mavlink_mission_set_current_t*)msg->payload64;
		
		if( setCurrentMission(msg_rd->seq) )
		{
			//��ÿ���˿ڷ���MISSION_CURRENT
			for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
			{
				const Port* port = get_CommuPort(i);
				if( port->write != 0 )
				{
					mavlink_message_t msg_sd;
					if( mavlink_lock_chan( i, 0.01 ) )
					{
						mavlink_msg_mission_current_pack_chan(
							get_CommulinkSysId() ,	//system id
							get_CommulinkCompId() ,	//component id
							i ,	//chan
							&msg_sd,
							msg_rd->seq
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
	
	//���ص���������
	static uint16_t DnlMissionsCount = 0;
	//����ʱ�ٴ��������
	bool RqMissionInt[MAVLINK_COMM_NUM_BUFFERS];
	int32_t RqMissionInd[MAVLINK_COMM_NUM_BUFFERS];
	int32_t RqMissionCounter[MAVLINK_COMM_NUM_BUFFERS] = {0};
	uint8_t RqMissiontarget_sysid[MAVLINK_COMM_NUM_BUFFERS];
	uint8_t RqMissiontarget_compid[MAVLINK_COMM_NUM_BUFFERS];
	
	static void Msg44_MISSION_COUNT( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_mission_count_t* msg_rd = (mavlink_mission_count_t*)msg->payload64;
		
		const Port* port = get_CommuPort(Port_index);
		if( port->write != 0 )
		{
			DnlMissionsCount = msg_rd->count;
			if( DnlMissionsCount > 0 )
			{
				clearMissions();
				
				mavlink_message_t msg_sd;
				if( mavlink_lock_chan( Port_index, 0.01 ) )
				{
					mavlink_msg_mission_request_int_pack_chan( 
						get_CommulinkSysId() ,	//system id
						get_CommulinkCompId() ,	//component id
						Port_index ,	//chan
						&msg_sd,
						msg->sysid ,	//target system
						msg->compid ,	//target component
						0 ,	//seq
						MAV_MISSION_TYPE_MISSION	//mission type
					
					);
					mavlink_msg_to_send_buffer(port->write, 
																		 port->lock,
																		 port->unlock,
																		 &msg_sd, 0, 0.01);

					mavlink_msg_mission_request_pack_chan( 
						get_CommulinkSysId() ,	//system id
						get_CommulinkCompId() ,	//component id
						Port_index ,	//chan
						&msg_sd,
						msg->sysid ,	//target system
						msg->compid ,	//target component
						0 ,	//seq
						MAV_MISSION_TYPE_MISSION	//mission type
					
					);
					mavlink_msg_to_send_buffer(port->write, 
																		 port->lock,
																		 port->unlock,
																		 &msg_sd, 0, 0.01);
					mavlink_unlock_chan(Port_index);
				}
				
//				//��ʱ����
//				RqMissionCounter[Port_index] = 100;
//				RqMissionInd[Port_index] = 0;
//				RqMissiontarget_sysid[Port_index] = msg->sysid;
//				RqMissiontarget_compid[Port_index] = msg->compid;
				
				--DnlMissionsCount;
			}
		}
	}
	
	static void Msg39_MISSION_ITEM( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_mission_item_t* msg_rd = (mavlink_mission_item_t*)msg->payload64;
		
		if( msg_rd->seq == getUploadingMissionsCount() )
		{
			//��Ӻ�������
			MissionInf wp_inf;
			wp_inf.cmd = msg_rd->command;
			wp_inf.frame = msg_rd->frame;
			wp_inf.autocontinue = msg_rd->autocontinue;
			wp_inf.params[0] = msg_rd->param1;
			wp_inf.params[1] = msg_rd->param2;
			wp_inf.params[2] = msg_rd->param3;
			wp_inf.params[3] = msg_rd->param4;
			wp_inf.params[4] = msg_rd->x;
			wp_inf.params[5] = msg_rd->y;
			wp_inf.params[6] = msg_rd->z;
			addMission( wp_inf, false );
			
			const Port* port = get_CommuPort(Port_index);
			if( port->write != 0 )
			{		
				if( DnlMissionsCount > 0 )
				{	
					--DnlMissionsCount;
					//���ͺ�������
					mavlink_message_t msg_sd;
					if( mavlink_lock_chan( Port_index, 0.01 ) )
					{
						mavlink_msg_mission_request_pack_chan( 
							get_CommulinkSysId() ,	//system id
							get_CommulinkCompId() ,	//component id
							Port_index ,	//chan
							&msg_sd,
							msg->sysid ,	//target system
							msg->compid ,	//target component
							getUploadingMissionsCount() ,	//seq
							MAV_MISSION_TYPE_MISSION	//mission type					
						);
						mavlink_msg_to_send_buffer(port->write, 
																			 port->lock,
																			 port->unlock,
																			 &msg_sd, 0, 0.01);
						mavlink_unlock_chan(Port_index);
					}

					//��ʱ����
					RqMissionCounter[Port_index] = 100;
					RqMissionInd[Port_index] = getUploadingMissionsCount();
					RqMissionInt[Port_index] = false;
					RqMissiontarget_sysid[Port_index] = msg->sysid;
					RqMissiontarget_compid[Port_index] = msg->compid;
				}
				else
				{	//����ACK			
					mavlink_message_t msg_sd;
					if( mavlink_lock_chan( Port_index, 0.01 ) )
					{
						mavlink_msg_mission_ack_pack_chan( 
							get_CommulinkSysId() ,	//system id
							get_CommulinkCompId() ,	//component id
							Port_index ,	//chan
							&msg_sd,
							msg->sysid ,	//target system
							msg->compid ,	//target component
							MAV_MISSION_ACCEPTED ,	//type
							MAV_MISSION_TYPE_MISSION	//mission type						
						);
						mavlink_msg_to_send_buffer(port->write, 
																			 port->lock,
																			 port->unlock,
																			 &msg_sd, 0, 0.01);
						mavlink_unlock_chan(Port_index);
					}
					
					//��λ��ʱ����
					RqMissionCounter[Port_index] = 0;
					
					//�������񵽴洢��
					saveMissions();
				}
			}
		}
	}
	
	static void Msg73_MISSION_ITEM_INT( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_mission_item_int_t* msg_rd = (mavlink_mission_item_int_t*)msg->payload64;
		
		if( msg_rd->seq == getUploadingMissionsCount() )
		{
			//��Ӻ�������
			MissionInf wp_inf;
			wp_inf.cmd = msg_rd->command;
			wp_inf.frame = msg_rd->frame;
			wp_inf.autocontinue = msg_rd->autocontinue;
			wp_inf.params[0] = msg_rd->param1;
			wp_inf.params[1] = msg_rd->param2;
			wp_inf.params[2] = msg_rd->param3;
			wp_inf.params[3] = msg_rd->param4;
			switch(wp_inf.frame)
			{
				case MAV_FRAME_GLOBAL:
				case MAV_FRAME_GLOBAL_RELATIVE_ALT:
				case MAV_FRAME_GLOBAL_INT:
				case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
				case MAV_FRAME_GLOBAL_TERRAIN_ALT:
				case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
					wp_inf.params[4] = msg_rd->x*1e-7;
					wp_inf.params[5] = msg_rd->y*1e-7;
					break;
				default:
					wp_inf.params[4] = msg_rd->x*1e-4;
					wp_inf.params[5] = msg_rd->y*1e-4;
					break;
			}		
			wp_inf.params[6] = msg_rd->z;
			addMission( wp_inf, false );
			
			const Port* port = get_CommuPort(Port_index);
			if( port->write != 0 )
			{		
				if( DnlMissionsCount > 0 )
				{	
					--DnlMissionsCount;
					//���ͺ�������
					mavlink_message_t msg_sd;
					if( mavlink_lock_chan( Port_index, 0.01 ) )
					{
						mavlink_msg_mission_request_int_pack_chan( 
							get_CommulinkSysId() ,	//system id
							get_CommulinkCompId() ,	//component id
							Port_index ,	//chan
							&msg_sd,
							msg->sysid ,	//target system
							msg->compid ,	//target component
							getUploadingMissionsCount() ,	//seq
							MAV_MISSION_TYPE_MISSION	//mission type
						
						);
						mavlink_msg_to_send_buffer(port->write, 
																			 port->lock,
																			 port->unlock,
																			 &msg_sd, 0, 0.01);
						mavlink_unlock_chan(Port_index);
					}

					//��ʱ����
					RqMissionCounter[Port_index] = 100;
					RqMissionInd[Port_index] = getUploadingMissionsCount();
					RqMissionInt[Port_index] = true;
					RqMissiontarget_sysid[Port_index] = msg->sysid;
					RqMissiontarget_compid[Port_index] = msg->compid;
				}
				else
				{	//����ACK			
					mavlink_message_t msg_sd;
					if( mavlink_lock_chan( Port_index, 0.01 ) )
					{
						mavlink_msg_mission_ack_pack_chan( 
							get_CommulinkSysId() ,	//system id
							get_CommulinkCompId() ,	//component id
							Port_index ,	//chan
							&msg_sd,
							msg->sysid ,	//target system
							msg->compid ,	//target component
							MAV_MISSION_ACCEPTED ,	//type
							MAV_MISSION_TYPE_MISSION	//mission type						
						);
						mavlink_msg_to_send_buffer(port->write, 
																			 port->lock,
																			 port->unlock,
																			 &msg_sd, 0, 0.01);
						mavlink_unlock_chan(Port_index);
					}
					
					//��λ��ʱ����
					RqMissionCounter[Port_index] = 0;
					
					//�������񵽴洢��
					saveMissions();
				}
			}
		}
	}
	
	static void Msg45_MISSION_CLEAR_ALL( uint8_t Port_index , const mavlink_message_t* msg )
	{
		const mavlink_mission_item_int_t* msg_rd = (mavlink_mission_item_int_t*)msg->payload64;
		
		clearMissions();
		
		const Port* port = get_CommuPort(Port_index);
		if( port->write != 0 )
		{	//����ACK			
			mavlink_message_t msg_sd;
			if( mavlink_lock_chan( Port_index, 0.01 ) )
			{
				mavlink_msg_mission_ack_pack_chan( 
					get_CommulinkSysId() ,	//system id
					get_CommulinkCompId() ,	//component id
					Port_index ,	//chan
					&msg_sd,
					msg->sysid ,	//target system
					msg->compid ,	//target component
					MAV_MISSION_ACCEPTED ,	//type
					MAV_MISSION_TYPE_MISSION	//mission type						
				);
				mavlink_msg_to_send_buffer(port->write, 
																	 port->lock,
																	 port->unlock,
																	 &msg_sd, 0, 0.01);
				mavlink_unlock_chan(Port_index);
			}
		}
	}
/*��������Э��*/

static void Msg66_REQUEST_DATA_STREAM( uint8_t Port_index , const mavlink_message_t* msg )
{
	const mavlink_request_data_stream_t* msg_rd = (mavlink_request_data_stream_t*)msg->payload64;
	if( msg_rd->req_message_rate == 0 || msg_rd->start_stop == 0 )
		SetMsgRate( Port_index , msg_rd->req_stream_id , 0 );
	else
		SetMsgRate( Port_index , msg_rd->req_stream_id , msg_rd->req_message_rate );
}

static void Msg76_COMMAND_LONG( uint8_t Port_index , const mavlink_message_t* msg )
{
	const mavlink_command_long_t* msg_rd = (mavlink_command_long_t*)msg->payload64;
	if( msg_rd->command < Mavlink_CMD_Process_Count )
	{
		if( Mavlink_CMD_Process[ msg_rd->command ] != 0 )
			Mavlink_CMD_Process[ msg_rd->command ]( Port_index , msg );
		else
		{
			const mavlink_command_long_t* msg_rd = (mavlink_command_long_t*)msg->payload64;
			ModeMsg mode_msg;
			mode_msg.cmd_type = CMD_TYPE_MAVLINK | Port_index;
			mode_msg.sd_sysid = msg->sysid;
			mode_msg.sd_compid = msg->compid;
			mode_msg.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
			mode_msg.cmd = msg_rd->command;
			mode_msg.params[0] = msg_rd->param1;
			mode_msg.params[1] = msg_rd->param2;
			mode_msg.params[2] = msg_rd->param3;
			mode_msg.params[3] = msg_rd->param4;
			mode_msg.params[4] = msg_rd->param5;
			mode_msg.params[5] = msg_rd->param6;
			mode_msg.params[6] = msg_rd->param7;
			SendMsgToMode( mode_msg, 0.01 );
		}
	}
	else
	{
		const mavlink_command_long_t* msg_rd = (mavlink_command_long_t*)msg->payload64;
		ModeMsg mode_msg;
		mode_msg.cmd_type = CMD_TYPE_MAVLINK | Port_index;
		mode_msg.sd_sysid = msg->sysid;
		mode_msg.sd_compid = msg->compid;
		mode_msg.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
		mode_msg.cmd = msg_rd->command;
		mode_msg.params[0] = msg_rd->param1;
		mode_msg.params[1] = msg_rd->param2;
		mode_msg.params[2] = msg_rd->param3;
		mode_msg.params[3] = msg_rd->param4;
		mode_msg.params[4] = msg_rd->param5;
		mode_msg.params[5] = msg_rd->param6;
		mode_msg.params[6] = msg_rd->param7;
		SendMsgToMode( mode_msg, 0.01 );
	}
}

const uint8_t default_vslam_pos_index = 11;
const uint8_t default_vslam_vel_index = 12;
const uint8_t default_lidar_slam_pos_index = 13;

bool vslam_pos_register_flag = false;
bool vslam_vel_register_flag = false;
bool lidar_slam_pos_register_flag = false;

static void Msg102_VISION_POSITION_ESTIMATE( uint8_t Port_index , const mavlink_message_t* msg )
{
	static double initYaw, sinYaw, cosYaw;
	
	if(!vslam_pos_register_flag)
	{
		if(get_Attitude_MSStatus() != MS_Ready)
			return;
		
		Quaternion quat;
		get_Attitude_quat(&quat);
		initYaw = quat.getYaw();
		fast_sin_cos(initYaw, &sinYaw, &cosYaw);
		
		vslam_pos_register_flag = PositionSensorRegister( default_vslam_pos_index,
																											Position_Sensor_Type_RelativePositioning,
																											Position_Sensor_DataType_s_xy,
																											Position_Sensor_frame_ENU,
																											0.1,
																											50
																											);
	}
	
	const mavlink_vision_position_estimate_t* msg_rd = (mavlink_vision_position_estimate_t*)msg->payload64;
	
	//�����׼
	vector3<double> posVSlam;
	posVSlam.x = BodyHeading2ENU_x(msg_rd->x * 100 , msg_rd->y * 100 , sinYaw , cosYaw);
	posVSlam.y = BodyHeading2ENU_y(msg_rd->x * 100 , msg_rd->y * 100 , sinYaw , cosYaw);
//	posVSlam.z = msg_rd->z * 100;
	PositionSensorUpdatePosition(default_vslam_pos_index, posVSlam, true);
}

static void Msg103_VISION_SPEED_ESTIMATE( uint8_t Port_index , const mavlink_message_t* msg )
{
	static double initYaw, sinYaw, cosYaw;
	
	if(!vslam_vel_register_flag)
	{
		if(get_Attitude_MSStatus() != MS_Ready)
			return;
		
		Quaternion quat;
		get_Attitude_quat(&quat);
		initYaw = quat.getYaw();
		fast_sin_cos(initYaw, &sinYaw, &cosYaw);
		
		vslam_vel_register_flag = PositionSensorRegister( default_vslam_vel_index,
																											Position_Sensor_Type_RelativePositioning,
																											Position_Sensor_DataType_v_xy,
																											Position_Sensor_frame_ENU,
																											0.1,
																											50
																											);
	}
	const mavlink_vision_speed_estimate_t* msg_rd = (mavlink_vision_speed_estimate_t*)msg->payload64;
	
	vector3<double> velVSlam;
	velVSlam.x = BodyHeading2ENU_x(msg_rd->x * 100 , msg_rd->y * 100 , sinYaw , cosYaw);
	velVSlam.y = BodyHeading2ENU_y(msg_rd->x * 100 , msg_rd->y * 100 , sinYaw , cosYaw);
//	velVSlam.z = msg_rd->z * 100;
	PositionSensorUpdateVel(default_vslam_vel_index, velVSlam, true);
}

static void Msg138_ATT_POS_MOCAP( uint8_t Port_index , const mavlink_message_t* msg )
{
	static double initYaw, sinYaw, cosYaw;
	
	if(!lidar_slam_pos_register_flag)
	{
		if(get_Attitude_MSStatus() != MS_Ready)
			return;
		
		Quaternion quat;
		get_Attitude_quat(&quat);
		initYaw = quat.getYaw();
		fast_sin_cos(initYaw, &sinYaw, &cosYaw);
		
		lidar_slam_pos_register_flag = PositionSensorRegister( default_lidar_slam_pos_index,
																													 Position_Sensor_Type_RelativePositioning,
																													 Position_Sensor_DataType_s_xy,
																													 Position_Sensor_frame_ENU,
																													 0.1,
																													 25
																													 );
	}
	
	const mavlink_att_pos_mocap_t* msg_rd = (mavlink_att_pos_mocap_t*)msg->payload64;
	
	//�����׼
	vector3<double> posLidarSlam;
	posLidarSlam.x = BodyHeading2ENU_x(msg_rd->x * 100 , msg_rd->y * 100 , sinYaw , cosYaw);
	posLidarSlam.y = BodyHeading2ENU_y(msg_rd->x * 100 , msg_rd->y * 100 , sinYaw , cosYaw);
	PositionSensorUpdatePosition(default_lidar_slam_pos_index, posLidarSlam, true);
}

static void Msg233_GPS_RTCM_DATA( uint8_t Port_index , const mavlink_message_t* msg )
{
	const mavlink_gps_rtcm_data_t* msg_rd = (mavlink_gps_rtcm_data_t*)msg->payload64;
	
	static uint8_t RTCM_buffer[MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN*4];
	static uint8_t bf_sequence = 0;
	static uint8_t bf_fragments_received = 0;
	
	if( (msg_rd->flags & 1) == 0 )
	{	// it is not fragmented, pass direct
		inject_RtkPorts( msg_rd->data, msg_rd->len );
		bf_fragments_received = 0xf;
		return;
	}
	
	uint8_t fragment = (msg_rd->flags >> 1U) & 0x03;
	uint8_t sequence = (msg_rd->flags >> 3U) & 0x1F;
	
	if ( bf_sequence!=sequence || bf_fragments_received&(1U<<fragment) ) 
	{	// we have one or more partial fragments already received
		// which conflict with the new fragment, discard previous fragments
		bf_fragments_received = 0;
	}
	
	// add this fragment
	bf_sequence = sequence;
	bf_fragments_received |= (1U << fragment);
	
	// copy the data
	memcpy(&RTCM_buffer[MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN*(uint16_t)fragment], msg_rd->data, msg_rd->len);
	
	// when we get a fragment of less than max size then we know the
	// number of fragments. Note that this means if you want to send a
	// block of RTCM data of an exact multiple of the buffer size you
	// need to send a final packet of zero length
	uint8_t fragment_count = 0;
	uint16_t total_length = 0;
	if (msg_rd->len < MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN) {
			fragment_count = fragment+1;
			total_length = (MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN*fragment) + msg_rd->len;
	} else if (bf_fragments_received == 0x0F) {
			// special case of 4 full fragments
			fragment_count = 4;
			total_length = MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN*4;
	}
	
	// see if we have all fragments
	if (fragment_count != 0 &&
			bf_fragments_received == (1U << fragment_count) - 1) {
			// we have them all, inject
			inject_RtkPorts(RTCM_buffer, total_length);
	}
}


	
static void Msg243_SET_HOME_POSITION( uint8_t Port_index , const mavlink_message_t* msg )
{
//	const mavlink_set_home_position_t* msg_rd = (mavlink_set_home_position_t*)msg->payload64;
//	vector2<double> HomeLatLon;
//	HomeLatLon.y = msg_rd->longitude*1e-7;
//	HomeLatLon.x = msg_rd->latitude*1e-7;
////	setHomeLatLon(&HomeLatLon);
}

void (*const Mavlink_RC_Process[])( uint8_t Port_index , const mavlink_message_t* msg_sd ) = 
{
	/*000-*/	Msg0_HEARTBEAT	,
	/*001-*/	0	,
	/*002-*/	0	,
	/*003-*/	0	,
	/*004-*/	0	,
	/*005-*/	0	,
	/*006-*/	0	,
	/*007-*/	0	,
	/*008-*/	0	,
	/*009-*/	0	,
	/*010-*/	0	,
	/*011-*/	Msg11_SET_MODE	,
	/*012-*/	0	,
	/*013-*/	0	,
	/*014-*/	0	,
	/*015-*/	0	,
	/*016-*/	0	,
	/*017-*/	0	,
	/*018-*/	0	,
	/*019-*/	0	,
	/*020-*/	Msg20_PARAM_REQUEST_READ	,
	/*021-*/	Msg21_PARAM_REQUEST_LIST	,
	/*022-*/	0	,
	/*023-*/	Msg23_PARAM_SET	,
	/*024-*/	0	,
	/*025-*/	0	,
	/*026-*/	0	,
	/*027-*/	0	,
	/*028-*/	0	,
	/*029-*/	0	,
	/*030-*/	0	,
	/*031-*/	0	,
	/*032-*/	0	,
	/*033-*/	0	,
	/*034-*/	0	,
	/*035-*/	0	,
	/*036-*/	0	,
	/*037-*/	0	,
	/*038-*/	0	,
	/*039-*/	Msg39_MISSION_ITEM	,
	/*040-*/	Msg40_MISSION_REQUEST	,
	/*041-*/	Msg41_MISSION_SET_CURRENT	,
	/*042-*/	0	,
	/*043-*/	Msg43_MISSION_REQUEST_LIST	,
	/*044-*/	Msg44_MISSION_COUNT	,
	/*045-*/	Msg45_MISSION_CLEAR_ALL	,
	/*046-*/	0	,
	/*047-*/	0	,
	/*048-*/	0	,
	/*049-*/	0	,
	/*050-*/	0	,
	/*051-*/	Msg51_MISSION_REQUEST_INT	,
	/*052-*/	0	,
	/*053-*/	0	,
	/*054-*/	0	,
	/*055-*/	0	,
	/*056-*/	0	,
	/*057-*/	0	,
	/*058-*/	0	,
	/*059-*/	0	,
	/*060-*/	0	,
	/*061-*/	0	,
	/*062-*/	0	,
	/*063-*/	0	,
	/*064-*/	0	,
	/*065-*/	0	,
	/*066-*/	Msg66_REQUEST_DATA_STREAM	,
	/*067-*/	0	,
	/*068-*/	0	,
	/*069-*/	0	,
	/*070-*/	0	,
	/*071-*/	0	,
	/*072-*/	0	,
	/*073-*/	Msg73_MISSION_ITEM_INT	,
	/*074-*/	0	,
	/*075-*/	0	,
	/*076-*/	Msg76_COMMAND_LONG	,
	/*077-*/	0	,
	/*078-*/	0	,
	/*079-*/	0	,
	/*080-*/	0	,
	/*081-*/	0	,
	/*082-*/	0	,
	/*083-*/	0	,
	/*084-*/	0	,
	/*085-*/	0	,
	/*086-*/	0	,
	/*087-*/	0	,
	/*088-*/	0	,
	/*089-*/	0	,
	/*090-*/	0	,
	/*091-*/	0	,
	/*092-*/	0	,
	/*093-*/	0	,
	/*094-*/	0	,
	/*095-*/	0	,
	/*096-*/	0	,
	/*097-*/	0	,
	/*098-*/	0	,
	/*099-*/	0	,
	/*100-*/	0	,
	/*101-*/	0	,
	/*102-*/	Msg102_VISION_POSITION_ESTIMATE	,
	/*103-*/	Msg103_VISION_SPEED_ESTIMATE	,
	/*104-*/	0	,
	/*105-*/	0	,
	/*106-*/	0	,
	/*107-*/	0	,
	/*108-*/	0	,
	/*109-*/	0	,
	/*110-*/	0	,
	/*111-*/	0	,
	/*112-*/	0	,
	/*113-*/	0	,
	/*114-*/	0	,
	/*115-*/	0	,
	/*116-*/	0	,
	/*117-*/	0	,
	/*118-*/	0	,
	/*119-*/	0	,
	/*120-*/	0	,
	/*121-*/	0	,
	/*122-*/	0	,
	/*123-*/	0	,
	/*124-*/	0	,
	/*125-*/	0	,
	/*126-*/	0	,
	/*127-*/	0	,
	/*128-*/	0	,
	/*129-*/	0	,
	/*130-*/	0	,
	/*131-*/	0	,
	/*132-*/	0	,
	/*133-*/	0	,
	/*134-*/	0	,
	/*135-*/	0	,
	/*136-*/	0	,
	/*137-*/	0	,
	/*138-*/	Msg138_ATT_POS_MOCAP	,
	/*139-*/	0	,
	/*140-*/	0	,
	/*141-*/	0	,
	/*142-*/	0	,
	/*143-*/	0	,
	/*144-*/	0	,
	/*145-*/	0	,
	/*146-*/	0	,
	/*147-*/	0	,
	/*148-*/	0	,
	/*149-*/	0	,
	/*150-*/	0	,
	/*151-*/	0	,
	/*152-*/	0	,
	/*153-*/	0	,
	/*154-*/	0	,
	/*155-*/	0	,
	/*156-*/	0	,
	/*157-*/	0	,
	/*158-*/	0	,
	/*159-*/	0	,
	/*160-*/	0	,
	/*161-*/	0	,
	/*162-*/	0	,
	/*163-*/	0	,
	/*164-*/	0	,
	/*165-*/	0	,
	/*166-*/	0	,
	/*167-*/	0	,
	/*168-*/	0	,
	/*169-*/	0	,
	/*170-*/	0	,
	/*171-*/	0	,
	/*172-*/	0	,
	/*173-*/	0	,
	/*174-*/	0	,
	/*175-*/	0	,
	/*176-*/	0	,
	/*177-*/	0	,
	/*178-*/	0	,
	/*179-*/	0	,
	/*180-*/	0	,
	/*181-*/	0	,
	/*182-*/	0	,
	/*183-*/	0	,
	/*184-*/	0	,
	/*185-*/	0	,
	/*186-*/	0	,
	/*187-*/	0	,
	/*188-*/	0	,
	/*189-*/	0	,
	/*190-*/	0	,
	/*191-*/	0	,
	/*192-*/	0	,
	/*193-*/	0	,
	/*194-*/	0	,
	/*195-*/	0	,
	/*196-*/	0	,
	/*197-*/	0	,
	/*198-*/	0	,
	/*199-*/	0	,
	/*200-*/	0	,
	/*201-*/	0	,
	/*202-*/	0	,
	/*203-*/	0	,
	/*204-*/	0	,
	/*205-*/	0	,
	/*206-*/	0	,
	/*207-*/	0	,
	/*208-*/	0	,
	/*209-*/	0	,
	/*210-*/	0	,
	/*211-*/	0	,
	/*212-*/	0	,
	/*213-*/	0	,
	/*214-*/	0	,
	/*215-*/	0	,
	/*216-*/	0	,
	/*217-*/	0	,
	/*218-*/	0	,
	/*219-*/	0	,
	/*220-*/	0	,
	/*221-*/	0	,
	/*222-*/	0	,
	/*223-*/	0	,
	/*224-*/	0	,
	/*225-*/	0	,
	/*226-*/	0	,
	/*227-*/	0	,
	/*228-*/	0	,
	/*229-*/	0	,
	/*230-*/	0	,
	/*231-*/	0	,
	/*232-*/	0	,
	/*233-*/	Msg233_GPS_RTCM_DATA	,
	/*234-*/	0	,
	/*235-*/	0	,
	/*236-*/	0	,
	/*237-*/	0	,
	/*238-*/	0	,
	/*239-*/	0	,
	/*240-*/	0	,
	/*241-*/	0	,
	/*242-*/	0	,
	/*243-*/	Msg243_SET_HOME_POSITION ,
	/*244-*/	0	,
	/*245-*/	0	,
	/*246-*/	0	,
	/*247-*/	0	,
	/*248-*/	0	,
	/*249-*/	0	,
	/*250-*/	0	,
	/*251-*/	0	,
	/*252-*/	0	,
	/*253-*/	0	,
	/*254-*/	0	,
	/*255-*/	0	,
	/*256-*/	0	,
	/*257-*/	0	,
	/*258-*/	0	,
	/*259-*/	0	,
	/*260-*/	0	,
	/*261-*/	0	,
	/*262-*/	0	,
	/*263-*/	0	,
	/*264-*/	0	,
	/*265-*/	0	,
	/*266-*/	0	,
	/*267-*/	0	,
	/*268-*/	0	,
	/*269-*/	0	,
	/*270-*/	0	,
	/*271-*/	0	,
	/*272-*/	0	,
	/*273-*/	0	,
	/*274-*/	0	,
	/*275-*/	0	,
	/*276-*/	0	,
	/*277-*/	0	,
	/*278-*/	0	,
	/*279-*/	0	,
	/*280-*/	0	,
	/*281-*/	0	,
	/*282-*/	0	,
	/*283-*/	0	,
	/*284-*/	0	,
	/*285-*/	0	,
	/*286-*/	0	,
	/*287-*/	0	,
	/*288-*/	0	,
	/*289-*/	0	,
};
const uint16_t Mavlink_RC_Process_Count = sizeof( Mavlink_RC_Process ) / sizeof( void* );