#include "drv_ISTRA24.hpp"
#include "drv_Uart7.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "StorageSystem.hpp"
#include "ControlSystem.hpp"
#define SensorInd 2

typedef struct
{
	uint16_t Message_Id;//ID
	uint16_t Strength;//�ź�ǿ��
	uint16_t Range;	//Ŀ�����
	uint8_t Rsv[5];	//Ԥ��
}__PACKED _ISTRA24;
static const unsigned char packet_ID[2] = { 0xAA , 0xAA };
float ISTRA24 =0;
static void ISTRA24_Server(void* pvParameters)
{
	/*״̬��*/
		__attribute__ ((aligned (4))) _ISTRA24  SensorD;
		unsigned char rc_counter = 0;
		unsigned short sum = 0;
	/*״̬��*/
	while(1)
	{
		uint8_t rdata=0;
		if( Read_Uart7( &rdata, 1, 2, 0.5 ) )
		{
			if( rc_counter == 0 )
				sum = 0;
			if( rc_counter < 2 )
			{
				//���հ�ͷ
				if( rdata != packet_ID[ rc_counter ] )
					rc_counter = 0;
				else
				{
					++rc_counter;
				}
			}
			else if( rc_counter < 11 )
			{	//��������
				((unsigned char*)&SensorD)[rc_counter - 2] = rdata;
				if( rc_counter > 3 )//ǰ7���ֽ�֮��
					sum += (unsigned char)rdata;
				++rc_counter;
			}
			else
			{	//У��
				if( uint8_t(sum&0x00ff) == rdata )
				{	//У��ɹ�  					
					//��С��ת��					
					__attribute__ ((aligned (4))) uint16_t Range = SensorD.Range;
					((uint16_t*)&Range)[0] = __REV16(Range);						
					if( SensorD.Strength > 5 && Range > 20 && Range < 9000 )
					{
						vector3<double> position;
						position.z = Range;								
						//��ȡ���
						Quaternion quat;
						get_Airframe_quat( &quat ); 
						double lean_cosin = quat.get_lean_angle_cosin();
						//����
						position.z *= lean_cosin;
						PositionSensorUpdatePosition( SensorInd, position, true );
					}
					else
					{
						SensorD.Range = 0;
						PositionSensorSetInavailable( SensorInd );
					}
					
					bool inFlight=false;
					get_is_inFlight(&inFlight);
					if(inFlight)
					{
						double ISTRA24_Height = Range * 0.01f;
						SDLog_Msg_DebugVect( "ISTRA24", &ISTRA24_Height, 1 );	
					}
				}else
					ISTRA24 = 0;
				
				rc_counter = 0;
			}
		}
	}
}

void init_drv_TFMini()
{
	//������115200
	SetBaudRate_Uart7( 115200, 2, 2 );
	//ע�ᴫ����
	bool res = PositionSensorRegister( SensorInd , \
																			Position_Sensor_Type_RangePositioning , \
																			Position_Sensor_DataType_s_z , \
																			Position_Sensor_frame_ENU , \
																			0.05 , //��ʱ
																			0 ,	//xy���ζ�
																			0 //z���ζ�
																			);
	if(res)
		xTaskCreate( ISTRA24_Server, "ISTRA24", 1024, NULL, SysPriority_ExtSensor, NULL);
}



