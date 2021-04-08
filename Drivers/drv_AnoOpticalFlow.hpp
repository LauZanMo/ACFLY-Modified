#pragma once

#include "Basic.hpp"

typedef struct
{
		uint8_t ADDR;
		uint8_t FUNC;
		uint8_t LENGTH;
}__PACKED _AnoFrame;

enum
{
	FlowFrame = 0x51,
	HeightFrame = 0x52,
	InertiaFrame = 0x53,
	QuatFrame = 0x54,
};

typedef struct
{
		uint8_t MODE;
		uint8_t STATE;
		int16_t DX;
		int16_t DY;
		int16_t DX_FIX;
		int16_t DY_FIX;
		int16_t DIS_X;
		int16_t DIS_Y;
		uint8_t QUALITY;
}__PACKED _AnoFlow;

typedef struct
{
		uint8_t MODE;
		int16_t ALT;
}__PACKED _AnoHeight;

typedef struct
{
		uint8_t MODE;
		int16_t GYR_X;
		int16_t GYR_Y;
		int16_t GYR_Z;
		int16_t ACC_X;
		int16_t ACC_Y;
		int16_t ACC_Z;
}__PACKED _AnoInertia;

typedef struct
{
		uint8_t MODE;
		int16_t S1;
		int16_t S2;
		int16_t S3;
		int16_t S4;
}__PACKED _AnoQuat;

void init_drv_AnoOpticalFlow();