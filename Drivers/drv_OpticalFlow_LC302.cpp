#include "drv_OpticalFlow_LC302.hpp"
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
	int16_t flow_x_integral;	// X ���ص��ۼ�ʱ���ڵ��ۼ�λ��(radians*10000)
														// [���� 10000 ���Ը߶�(mm)��Ϊʵ��λ��(mm)]
	int16_t flow_y_integral;	// Y ���ص��ۼ�ʱ���ڵ��ۼ�λ��(radians*10000)
														// [���� 10000 ���Ը߶�(mm)��Ϊʵ��λ��(mm)]
	uint16_t integration_timespan;	// ��һ�η��͹������ݵ����η��͹������ݵ��ۼ�ʱ�䣨us��
	uint16_t ground_distance; // Ԥ���� Ĭ��Ϊ 999��0x03E7��
	uint8_t valid;	// ״ֵ̬:0(0x00)Ϊ�������ݲ�����
									//245(0xF5)Ϊ�������ݿ���
	uint8_t version; //�汾��
}__PACKED _Flow;
static const unsigned char packet_ID[2] = { 0xfe , 0x0a };

static void OpticalFlow_Server(void* pvParameters)
{
	/*״̬��*/
		_Flow  Flow;
		unsigned char rc_counter = 0;
		signed char sum = 0;
	/*״̬��*/
	
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	while(1)
	{
		uint8_t rdata;
		if( driver_info.port.read( &rdata, 1, 2, 0.5 ) )
		{
			if( rc_counter < 2 )
			{
				//���հ�ͷ
				if( rdata != packet_ID[ rc_counter ] )
					rc_counter = 0;
				else
				{
					++rc_counter;
					sum = 0;
				}
			}
			else if( rc_counter < 12 )
			{	//��������
				( (unsigned char*)&Flow )[ rc_counter - 2 ] = rdata;
				sum ^= (signed char)rdata;
				++rc_counter;
			}
			else if( rc_counter == 12 )
			{	//У��
				if( sum != rdata )
					rc_counter = 0;
				else
					++rc_counter;
			}
			else
			{	//���հ�β
				if( rdata == 0x55 )
				{
					PosSensorHealthInf1 ZRange_inf;
					if( get_OptimalRange_Z( &ZRange_inf ) )
					{	//��ഫ��������
						if( ZRange_inf.last_healthy_TIME.is_valid() && ZRange_inf.last_healthy_TIME.get_pass_time() < 50 )
						{	//���50���ڽ���
							//��ȡ�߶�
							double height = ZRange_inf.HOffset + ZRange_inf.PositionENU.z;
							//��ȡ���ٶ�
							vector3<double> AngularRate;
							get_AngularRate_Ctrl( &AngularRate );
							//��������
							vector3<double> vel;
							double rotation_compensation_x = -constrain( AngularRate.y * 10000 , 4500000000.0 );
							double rotation_compensation_y = constrain(  AngularRate.x * 10000 , 4500000000.0 );
							double integral_time = (Flow.integration_timespan * 1e-6f);
							if( integral_time > 1e-3 )
							{
								double temp_flow_x, temp_flow_y;
								double flow_x, flow_y;
								temp_flow_x = Flow.flow_x_integral;
								temp_flow_y = -Flow.flow_y_integral;
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
								
								integral_time = 1.0 / integral_time;
								vel.x = ( flow_x*integral_time - rotation_compensation_x ) * 1e-4f * ( 1 + height );
								vel.y = ( flow_y*integral_time - rotation_compensation_y ) * 1e-4f * ( 1 + height ) ;
								PositionSensorUpdateVel( default_optical_flow_index , vel , true );
							}
							else
								PositionSensorSetInavailable( default_optical_flow_index );
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

static bool OpticalFlow_LC302_DriverInit( Port port, uint32_t param )
{
	//������19200
	port.SetBaudRate( 19200, 2, 2 );
	//ע�ᴫ����
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
	xTaskCreate( OpticalFlow_Server, "OptFlowLC302", 1024, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_OpticalFlow_LC302()
{
	PortFunc_Register( 32, OpticalFlow_LC302_DriverInit );
}