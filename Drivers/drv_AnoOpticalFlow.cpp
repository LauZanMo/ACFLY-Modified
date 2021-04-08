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
								//接收包头
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
								//接收帧
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
								//校验
								if( sum == rdata )
								{
										//校验成功
										if( Frame.FUNC == FlowFrame )
										{
												//解析传感器数据
												Flow.MODE = datatemp[0];
												Flow.STATE = datatemp[1];
												Flow.DX = (datatemp[2] << 8) + datatemp[3];
												Flow.DY = (datatemp[4] << 8) + datatemp[5];
												Flow.DX_FIX = (datatemp[6] << 8) + datatemp[7];
												Flow.DY_FIX = (datatemp[8] << 8) + datatemp[9];
												Flow.DIS_X = (datatemp[10] << 8) + datatemp[11];
												Flow.DIS_Y = (datatemp[12] << 8) + datatemp[13];
												Flow.QUALITY = datatemp[14];
											
												//发布传感器数据
												//光流质量大于50
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
												//解析传感器数据
												Height.MODE = datatemp[0];
												Height.ALT = (datatemp[1] << 8) + datatemp[2];
												
												//发布传感器数据
												//对地高度在8~1200cm之间
												if(Height.ALT > 8 && Height.ALT < 1200)
												{
														vector3<double> position;
														position.z = Height.ALT;
														//获取倾角
														Quaternion quat;
														get_Airframe_quat( &quat );
														double lean_cosin = quat.get_lean_angle_cosin();
														//更新
														position.z *= lean_cosin;
														PositionSensorUpdatePosition( default_ano_laser_index, position, true );
												}
												else
													PositionSensorSetInavailable( default_ano_laser_index );
										}
										else if( Frame.FUNC == InertiaFrame )
										{
												//解析传感器数据
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
												//解析传感器数据
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
		//波特率500000
		port.SetBaudRate( 500000, 2, 2 );
		
		//注册传感器
		bool res1 = PositionSensorRegister( default_ano_optical_flow_index , \
																				Position_Sensor_Type_RelativePositioning , \
																				Position_Sensor_DataType_v_xy , \
																				Position_Sensor_frame_BodyHeading , \
																				0.1 , \
																				100);
		//注册传感器
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