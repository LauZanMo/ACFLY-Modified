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
	unsigned char  Reserved;
	unsigned char  Quality;
	signed char Delta_X;
	signed char Delta_Y;
	unsigned char  Reference;
	unsigned char  Lightness;
	signed char Delta_Z;
	signed char Sub_X;
	signed char Sub_Y;
	unsigned char  Timestamp[4];
	signed char  Checksum;
	unsigned char  Packet_footer[2];
}__PACKED _Flow;
static const unsigned char packet_ID[4] = { '#' , 'J' , 'B' , '#' };

static void OpticalFlow_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	/*状态机*/
		_Flow  Flow;
		unsigned char rc_counter = 0;
		signed char sum = 0;
	/*状态机*/
	
	while(1)
	{
		uint8_t rdata;
		if( driver_info.port.read( &rdata, 1, 2, 0.5 ) )
		{
			if( rc_counter < 4 )
			{
				//接收包头
				if( rdata != packet_ID[ rc_counter ] )
					rc_counter = 0;
				else
				{
					++rc_counter;
					sum = 0;
				}
			}
			else if( rc_counter <= 17 )
			{	//接收数据
				( (unsigned char*)&Flow )[ rc_counter - 4 ] = rdata;
				sum += (signed char)rdata;
				++rc_counter;
			}
			else if( rc_counter == 18 )
			{	//校验
				if( sum != 0 || rdata != '\r' )
					rc_counter = 0;
				else
					++rc_counter;
			}
			else
			{	//接收包尾
				if( rdata == '\n' )
				{
					PosSensorHealthInf1 ZRange_inf;
					if( get_OptimalRange_Z( &ZRange_inf ) )
					{	//测距传感器可用
						if( ZRange_inf.last_healthy_TIME.is_valid() && ZRange_inf.last_healthy_TIME.get_pass_time() < 50 )
						{	//测距50秒内健康
							//获取高度
							double height = ZRange_inf.HOffset + ZRange_inf.PositionENU.z;
							//获取角速度
							vector3<double> AngularRate;
							get_AngularRate_Ctrl( &AngularRate );
							//补偿光流
							vector3<double> vel;
							float ultra_deadband = height - 1;
              if( ultra_deadband < 0 )
						    ultra_deadband = 0;							
					    double rotation_compensation_x = -constrain( rad2degree( AngularRate.y ) * 0.3 , 45.0 );
					    double rotation_compensation_y = constrain( rad2degree( AngularRate.x ) * 0.3 , 45.0 );
							
							double temp_flow_x, temp_flow_y;
							double flow_x, flow_y;
							temp_flow_x = Flow.Delta_X;
							temp_flow_y = Flow.Delta_Y;
							switch(driver_info.param)
							{
								case 0:
								default:
								{
									flow_x = temp_flow_x;
									flow_y = temp_flow_y;
									break;
								}
								case 1:
								{
									flow_x = temp_flow_y;
									flow_y = -temp_flow_x;
									break;
								}
								case 2:
								{
									flow_x = -temp_flow_x;
									flow_y = -temp_flow_y;
									break;
								}
								case 3:
								{
									flow_x = -temp_flow_y;
									flow_y = temp_flow_x;
									break;
								}
							}
							vel.x = ( flow_x - rotation_compensation_x ) * 2.0f * (1 + ultra_deadband/40) ;
							vel.y = ( flow_y - rotation_compensation_y ) * 2.0f * (1 + ultra_deadband/40) ;
							PositionSensorUpdateVel( default_optical_flow_index , vel , true );
						}
						else
							PositionSensorSetInavailable( default_optical_flow_index );
					}
					else
						PositionSensorSetInavailable( default_optical_flow_index );
				}
				rc_counter = 0;
			}
			
		}
	}
}

static bool OpticalFlow_JL32xx_DriverInit( Port port, uint32_t param )
{
	//波特率19200
	port.SetBaudRate( 115200, 2, 2 );
	//注册传感器
	bool res = PositionSensorRegister( default_optical_flow_index , \
																			Position_Sensor_Type_RelativePositioning , \
																			Position_Sensor_DataType_v_xy , \
																			Position_Sensor_frame_BodyHeading , \
																			0.1, 100 );
	if(!res)
		return false;
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( OpticalFlow_Server, "OptFlowJL32xx", 1024, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_OpticalFlow_JL32xx()
{
	PortFunc_Register( 34, OpticalFlow_JL32xx_DriverInit );
}