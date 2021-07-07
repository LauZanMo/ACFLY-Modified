#pragma once

#include "Basic.hpp"
#include "Receiver.hpp"

/*AuxFuncs
	0:无
	1-16:映射遥控器对应通道（raw data）
	25-40:用遥控器对应通道进行相机快门触发（raw_data）
	49-64:用遥控器对应通道进行云台控制（raw_data）
*/
struct AuxFuncsConfig
{
	uint8_t Aux1Func[8];
	uint8_t Aux2Func[8];
	uint8_t Aux3Func[8];
	uint8_t Aux4Func[8];
	uint8_t Aux5Func[8];
	uint8_t Aux6Func[8];
	uint8_t Aux7Func[8];
	uint8_t Aux8Func[8];
	float Aux1Param1[2];
	float Aux2Param1[2];
	float Aux3Param1[2];
	float Aux4Param1[2];
	float Aux5Param1[2];
	float Aux6Param1[2];
	float Aux7Param1[2];
	float Aux8Param1[2];
	float Aux1Param2[2];
	float Aux2Param2[2];
	float Aux3Param2[2];
	float Aux4Param2[2];
	float Aux5Param2[2];
	float Aux6Param2[2];
	float Aux7Param2[2];
	float Aux8Param2[2];
	uint16_t Aux_CamOnPwm[4];
	uint16_t Aux_CamOffPwm[4];
	float Aux_CamShTime[2];
	uint16_t Aux_BsYTPit0[4];
	uint16_t Aux_BsYTPit90[4];
	uint16_t Aux_StYTPit0[4];
	uint16_t Aux_StYTPit90[4];
	uint16_t Aux_StYTRolN45[4];
	uint16_t Aux_StYTRolP45[4];
	float Aux_YTPitMin[2];
	float Aux_YTPitMax[2];
	float Aux_YTRollMax[2];
};

//初始化Aux处理
void init_process_AuxFuncs();
//进行Aux处理
void process_AuxFuncs(const Receiver* rc);

//拍照
bool AuxCamTakePhoto();
//自动控制云台角度
bool AuxGimbalSetAngle( double angle );

void init_AuxFuncs();