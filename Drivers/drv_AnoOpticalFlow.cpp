#include "drv_AnoOpticalFlow.hpp"
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

static const unsigned char packet_ID[2] = {0xAA, 0x22};
const uint8_t default_ano_laser_index = 9;
const uint8_t default_ano_optical_flow_index = 10;

static void AnoOpticalFlow_Server(void* pvParameters)
{
		DriverInfo driver_info = *(DriverInfo*)pvParameters;
		delete (DriverInfo*)pvParameters;
	
		_AnoFrame Frame;
		_AnoFlow Flow;
		_AnoHeight Height;
		_AnoInertia Inertia;
		_AnoQuat Quat;
		uint8_t datatemp[20];
		unsigned char rc_counter = 0;
		unsigned char sum = 0;
		
		while(1)
		{
				uint8_t rdata;
				if(driver_info.port.read( &rdata, 1, 2, 0.5 ))
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
										sum += rdata;
										++rc_counter;
								}
						}
						else if( rc_counter < 5 )
						{	
								//����֡
								( (unsigned char*)&Frame )[ rc_counter - 2 ] = rdata;
								sum += (unsigned char)rdata;
								++rc_counter;
						}
						else if( rc_counter < 5 + Frame.LENGTH )
						{
								( (unsigned char*)&datatemp )[ rc_counter - 5 ] = rdata;
								
								sum += (unsigned char)rdata;
								++rc_counter;
						}
						else
						{
								//У��
								if( sum == rdata )
								{
										//У��ɹ�
										if( Frame.FUNC == FlowFrame )
										{
												//��������������
												Flow.MODE = datatemp[0];
												Flow.STATE = datatemp[1];
												Flow.DX = (datatemp[2] << 8) + datatemp[3];
												Flow.DY = (datatemp[4] << 8) + datatemp[5];
												Flow.DX_FIX = (datatemp[6] << 8) + datatemp[7];
												Flow.DY_FIX = (datatemp[8] << 8) + datatemp[9];
												Flow.DIS_X = (datatemp[10] << 8) + datatemp[11];
												Flow.DIS_Y = (datatemp[12] << 8) + datatemp[13];
												Flow.QUALITY = datatemp[14];
											
												//��������������
												//������������50
												if (Flow.QUALITY > 50)
												{
													vector3<double> vel;
													vel.x = (double)Flow.DX;
													vel.y = (double)Flow.DY;
													PositionSensorUpdateVel(default_ano_optical_flow_index, vel, true);
												}
												else
													PositionSensorSetInavailable(default_ano_optical_flow_index);
										}
										else if( Frame.FUNC == HeightFrame )
										{
												//��������������
												Height.MODE = datatemp[0];
												Height.ALT = (datatemp[1] << 8) + datatemp[2];
												
												//��������������
												//�Եظ߶���8~1200cm֮��
												if(Height.ALT > 8 && Height.ALT < 1200)
												{
														vector3<double> position;
														position.z = Height.ALT;
														//��ȡ���
														Quaternion quat;
														get_Airframe_quat( &quat );
														double lean_cosin = quat.get_lean_angle_cosin();
														//����
														position.z *= lean_cosin;
														PositionSensorUpdatePosition( default_ano_laser_index, position, true );
												}
												else
													PositionSensorSetInavailable( default_ano_laser_index );
										}
										else if( Frame.FUNC == InertiaFrame )
										{
												//��������������
												Inertia.MODE = datatemp[0];
												Inertia.GYR_X = (datatemp[1] << 8) + datatemp[2];
												Inertia.GYR_Y = (datatemp[3] << 8) + datatemp[4];
												Inertia.GYR_Z = (datatemp[5] << 8) + datatemp[6];
												Inertia.ACC_X = (datatemp[7] << 8) + datatemp[8];
												Inertia.ACC_Y = (datatemp[9] << 8) + datatemp[10];
												Inertia.ACC_Z = (datatemp[11] << 8) + datatemp[12];
										}
										else if( Frame.FUNC == QuatFrame )
										{
												//��������������
												Quat.MODE = datatemp[0];
												Quat.S1 = (datatemp[1] << 8) + datatemp[2];
												Quat.S2 = (datatemp[3] << 8) + datatemp[4];
												Quat.S3 = (datatemp[5] << 8) + datatemp[6];
												Quat.S4 = (datatemp[7] << 8) + datatemp[8];
										}
								}
								rc_counter = 0;
						}
				}
		}
}

static bool AnoOpticalFlow_DriverInit( Port port, uint32_t param )
{
		//������500000
		port.SetBaudRate( 500000, 2, 2 );
		
		//ע�ᴫ����
		bool res1 = PositionSensorRegister( default_ano_optical_flow_index , \
																				Position_Sensor_Type_RelativePositioning , \
																				Position_Sensor_DataType_v_xy , \
																				Position_Sensor_frame_BodyHeading , \
																				0.1 , \
																				100);
		//ע�ᴫ����
		bool res2 = PositionSensorRegister( default_ano_laser_index , \
																				Position_Sensor_Type_RangePositioning , \
																				Position_Sensor_DataType_s_z , \
																				Position_Sensor_frame_ENU , \
																				0.05);
		if(!(res1 && res2))
			return false;
		DriverInfo* driver_info = new DriverInfo;
		driver_info->param = param;
		driver_info->port = port;
		xTaskCreate( AnoOpticalFlow_Server, "AnoOpticalFlow", 1024, (void*)driver_info, SysPriority_ExtSensor, NULL);
		return true;
}

void init_drv_AnoOpticalFlow()
{
		PortFunc_Register( 88, AnoOpticalFlow_DriverInit );
}