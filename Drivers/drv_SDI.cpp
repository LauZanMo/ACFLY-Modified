#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "Modes.hpp"

struct DriverInfo
{
	uint32_t param;
	Port port;
};

static void SDI_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	/*状态机变量*/
		static uint8_t rc_step1 = 0;	//0：接收包头'A' 'C'
																	//1：接收1字节消息类别
																	//2：接收1字节消息长度
																	//3：接收数据包内容
																	//4：接收2字节校验
		static uint8_t rc_step2 = 0;
	
		#define MAX_SDI_PACKET_SIZE 4*6
		static uint8_t msg_type;
		static uint8_t msg_length;
		ALIGN4 static uint8_t msg_pack[MAX_SDI_PACKET_SIZE];
		static uint8_t sumA;
		static uint8_t sumB;
		
		#define reset_SDI_RC ( rc_step1 = rc_step2 = 0 )
	/*状态机变量*/
	
	while(1)
	{
		uint8_t r_data;
		if( driver_info.port.read( &r_data, 1, 2, 0.5 ) )
		{
			switch( rc_step1 )
			{
				case 0 :
					//接收包头'A''C'
					if( rc_step2 == 0 )
					{
						if( r_data == 'A' )
							rc_step2 = 1;
					}
					else
					{
						if( r_data == 'C' )
						{
							rc_step1 = 1;
							rc_step2 = 0;
							sumA = sumB = 0;
						}
						else
							rc_step2 = 0;
					}
					break;
					
				case 1:
					//接收消息类别
					msg_type = r_data;
					sumA += r_data;
					sumB += sumA;
					rc_step1 = 2;
					rc_step2 = 0;
					break;
				
				case 2:
					//接收消息长度
					if( r_data > MAX_SDI_PACKET_SIZE )
					{
						reset_SDI_RC;
						break;
					}
					msg_length = r_data;
					sumA += r_data;
					sumB += sumA;
					if( msg_length == 0 )
						rc_step1 = 4;
					else
						rc_step1 = 3;
					rc_step2 = 0;
					break;
					
				case 3:
					//接收数据包
					msg_pack[ rc_step2 ] = r_data;				
					sumA += r_data;
					sumB += sumA;
					++rc_step2;
					if( rc_step2 >= msg_length )
					{
						rc_step1 = 4;
						rc_step2 = 0;
					}
					break;
					
				case 4:
					//接收校验位
					if( rc_step2 == 0 )
					{
						if( sumA != r_data )
						{
							reset_SDI_RC;
							break;
						}
						rc_step2 = 1;
					}
					else
					{
						if( sumB == r_data )
						{
							ModeMsg msg;
							msg.cmd = msg_type;
							for( uint16_t i = 4; i <= msg_length; i+=4 )
							{
								uint8_t p_ind = (i-4) >> 2;
								if( p_ind < 8 )
									msg.params[p_ind] = *(float*)&msg_pack[i-4];
							}
							SendMsgToMode( msg, 0 );
						}
						reset_SDI_RC;
					}
					break;					
			}					
		}
	}
}

static bool SDI_DriverInit( Port port, uint32_t param )
{
	//波特率115200
	port.SetBaudRate( 115200, 2, 2 );
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( SDI_Server, "SDI", 1024, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_SDI()
{
	PortFunc_Register( 100, SDI_DriverInit );
}