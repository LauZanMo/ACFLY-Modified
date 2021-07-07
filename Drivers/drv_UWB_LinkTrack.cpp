#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"

struct DriverInfo
{
	uint32_t param;
	Port port;
};

typedef struct
{
	uint8_t id;
	uint8_t role;
	int pos_x:24;	int pos_y:24;	int pos_z:24;		
	int vel_x:24;	int vel_y:24;	int vel_z:24;	
	int dis_0:24;	int dis_1:24;	int dis_2:24;	int dis_3:24;	int dis_4:24;	int dis_5:24;	int dis_6:24;	int dis_7:24;
	float imuGyro[3];
	float imuAcc[3];
	uint8_t reserved1[12];
	int16_t angle[3];
	float q[4];
	uint8_t reserved2[4];
	uint32_t localTime;
	uint32_t systemTime;
	uint8_t reserved3[1];
	uint8_t eop[3];
	uint16_t voltage;
	uint8_t reserved4[5];
}__PACKED _Uwb;
static const unsigned char packet_ID[2] = { 0x55 , 0x01 };

static void OpticalFlow_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	/*状态机*/
		_Uwb  Uwb;
		unsigned char rc_counter = 0;
		signed char sum = 0;
	/*状态机*/
	
	//等待初始化完成
	while( get_Altitude_MSStatus() != MS_Ready )
		os_delay(1);

	//记录初始偏航
	Quaternion quat;
	get_Attitude_quat(&quat);
	double iniYaw = quat.getYaw();
	double sin_Yaw, cos_Yaw;
	fast_sin_cos( iniYaw, &sin_Yaw, &cos_Yaw );
	
	while(1)
	{
		uint8_t rdata;
		if( driver_info.port.read( &rdata, 1, 2, 0.5 ) )
		{
			if( rc_counter < sizeof(packet_ID) )
			{
				//接收包头
				if( rdata != packet_ID[ rc_counter ] )
				{
					rc_counter = 0;
					sum = 0;
				}
				else
				{
					++rc_counter;
					sum += rdata;
				}
			}
			else if( rc_counter < sizeof(packet_ID) + sizeof(_Uwb) )
			{	//接收数据
				( (unsigned char*)&Uwb )[ rc_counter - sizeof(packet_ID) ] = rdata;
				sum += rdata;
				++rc_counter;
			}
			else
			{	//校验
				if( sum == rdata )
				{
					
					if( Uwb.eop[0]>200 || Uwb.eop[1]>200 )
						PositionSensorSetInavailable(default_uwb_sensor_index);
					else 
					{					
						vector3<double> pos, vel;
						pos.x = ENU2BodyHeading_x( Uwb.pos_x*0.1 , Uwb.pos_y*0.1 , sin_Yaw , cos_Yaw );
						pos.y = -ENU2BodyHeading_y( Uwb.pos_x*0.1 , Uwb.pos_y*0.1 , sin_Yaw , cos_Yaw );
						pos.z = Uwb.pos_z * 0.1;
						vel.x = ENU2BodyHeading_x( Uwb.vel_x*0.1 , Uwb.vel_y*0.1 , sin_Yaw , cos_Yaw );
						vel.y = -ENU2BodyHeading_y( Uwb.vel_x*0.1 , Uwb.vel_y*0.1 , sin_Yaw , cos_Yaw );
						vel.z = Uwb.vel_z * 0.01;
//						if( Uwb.eop[2] > 200 )
//							PositionSensorChangeDataType( default_uwb_sensor_index, Position_Sensor_DataType_s_xy );
//						else
							PositionSensorChangeDataType( default_uwb_sensor_index, Position_Sensor_DataType_s_xyz );
						double eop_xy = sqrtf( Uwb.eop[0]*Uwb.eop[0] + Uwb.eop[1]*Uwb.eop[1] );
						PositionSensorUpdatePosition( default_uwb_sensor_index, pos, true, -1, eop_xy, 20 );
					}
				}
				rc_counter = 0;
				sum = 0;
			}
		}
	}
}

static bool UWB_LinkTrack_DriverInit( Port port, uint32_t param )
{
	//波特率115200
	port.SetBaudRate( 460800, 2, 2 );
	//注册传感器
	bool res = PositionSensorRegister( default_uwb_sensor_index , \
																			Position_Sensor_Type_RelativePositioning , \
																			Position_Sensor_DataType_v_xy , \
																			Position_Sensor_frame_BodyHeading , \
																			0.1, 100 );
	if(!res)
		return false;
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( OpticalFlow_Server, "OptFlowGL9306", 1024, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_UWB_LinkTrack()
{
	PortFunc_Register( 41, UWB_LinkTrack_DriverInit );
}