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
{   //�������ģ������40ms����һ�Σ����������и���ʱ�����ݴ�UART-TX����
	int16_t flow_x_integral;	// X������һ�����ݸ��µ���ǰ���µ�λ��
	int16_t flow_y_integral;	// Y������һ�����ݸ��µ���ǰ���µ�λ��
	uint8_t flow_sum;	// ���������ۼӺ�ȡ��8λ
	uint8_t quality;	//ͼ������0��100��30�������ݽϲ������
}__PACKED _Flow;

static const unsigned char packet_ID[2] = { 0xfe , 0x04 };
static void OpticalFlow_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	/*״̬��*/
		_Flow  Flow;
		unsigned char rc_counter = 0;
		signed char sum = 0;
	/*״̬��*/
	
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
			else if( rc_counter < 8 )
			{	//��������
				( (unsigned char*)&Flow )[ rc_counter - 2 ] = rdata;
				if(rc_counter < 6)
				sum += (signed char)rdata;
				++rc_counter;
			}
			else
			{	//���հ�β
				if( rdata == 0xAA && Flow.quality > 30 )
				{
					sum = 0 ;
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
							//��ȡ���
							Quaternion quat_flow;
							get_Airframe_quat( &quat_flow );
							double lean_cosin = quat_flow.get_lean_angle_cosin();
							//��������
							vector3<double> vel;
							#define OKp 400
							double rotation_compensation_x = -constrain( AngularRate.y * OKp , 4500000000.0 );
							double rotation_compensation_y = constrain(  AngularRate.x * OKp , 4500000000.0 );						
							//�����ʱ��
							Position_Sensor sensor;
							GetPositionSensor( default_optical_flow_index, &sensor );
							double integral_time = sensor.last_update_time.get_pass_time();
							if( integral_time > 1e-3 )
							{
								double temp_flow_x, temp_flow_y;
								double flow_x, flow_y;
								temp_flow_x = -Flow.flow_x_integral;
								temp_flow_y = Flow.flow_y_integral;
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
								
								double freq = 1.0 / integral_time;
								vel.x = ( flow_x*freq - rotation_compensation_x ) * (1.0/OKp) * ( 1 + height );
								vel.y = ( flow_y*freq - rotation_compensation_y ) * (1.0/OKp) * ( 1 + height );
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

static bool OpticalFlow_GL9306_DriverInit( Port port, uint32_t param )
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
	xTaskCreate( OpticalFlow_Server, "OptFlowGL9306", 1024, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_OpticalFlow_GL9306()
{
	PortFunc_Register( 35, OpticalFlow_GL9306_DriverInit );
}