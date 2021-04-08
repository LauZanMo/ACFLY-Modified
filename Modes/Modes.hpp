#pragma once

#include "Basic.hpp"
#include "mavlink.h"
#include "Receiver.hpp"

#define CMD_TYPE_MAVLINK (1<<4)
#define CMD_TYPE_MASK 0xf0
#define CMD_TYPE_PORT_MASK 0x0f
struct ModeMsg
{
	//��4λ��1-mavlink��Ϣ
	//��4λ������port
	uint8_t cmd_type;
	uint8_t sd_sysid;
	uint8_t sd_compid;
	uint8_t frame;
	uint32_t cmd;
	double params[8];
};
bool ModeReceiveMsg( ModeMsg* msg, double TIMEOUT );
bool SendMsgToMode( ModeMsg msg, double TIMEOUT );

//ģʽ����ѡ��
struct ModeFuncCfg
{
	//PA-��ť1����ǰ���ܣ�ģʽ��ţ�
	uint8_t Bt1PAFunc1[8];
	uint8_t Bt1PAFunc2[8];
	uint8_t Bt1PAFunc3[8];
	uint8_t Bt1PAFunc4[8];
	uint8_t Bt1PAFunc5[8];
	uint8_t Bt1PAFunc6[8];
	
	//A-��ť1�������ܣ�1-����ģʽ 2-λ��ģʽ 3-�˶�ģʽ 22-����ģʽ 23-����ģʽ��
	uint8_t Bt1AFunc1[8];
	uint8_t Bt1AFunc2[8];
	uint8_t Bt1AFunc3[8];
	uint8_t Bt1AFunc4[8];
	uint8_t Bt1AFunc5[8];
	uint8_t Bt1AFunc6[8];
	
	//����ִ�а�ť
	//2-4����Ӧ��ť����(100%)ִ������
	//12-14����Ӧ��ť�仯ִ������
	uint8_t MissionBt[8];
	
	//������ť
	//2-4����Ӧ��ť����(100%)����
	//12-14����Ӧ��ť�仯����
	uint8_t RTLBt[8];
	
	//��ȫ��ť
	//0����
	//2-4����Ӧ��ť����(100%)ǿ���������
	//10����������ƫ������ǿ���������
	uint8_t SafeBt[8];
	
	//��λ����
	float NeutralZone[2];
	//λ���ٶ���Ӧ����ϵ��
	float PosVelAlpha[2];
	//��̬��Ӧ����ϵ��
	float AttAlpha[2];
	
	//����ʹ��0�ź���
	uint8_t RstWp0[8];
}__PACKED;
#define is_AFunc_auto(x) (x>=20)
enum AFunc
{
	AFunc_Stabilize = 0,
	AFunc_AltHold = 1,
	AFunc_PosHold = 2,
	
	AFunc_TakeOff = 20,
	AFunc_Mission = 22,
	AFunc_RTL = 23,
};
	
//��ȡң�ذ�ť��Ӧ������0-5��
inline int8_t get_RcButtonZone( double rc, uint8_t current_zone )
{
	double st = 1.0/6*100;
	if( current_zone<=5 && rc>=current_zone*st-1 && rc<(current_zone+1)*st+1 )
		return current_zone;
	
	if( rc < 1*st )
		return 0;
	else if( rc < 2*st )
		return 1;
	else if( rc < 3*st )
		return 2;
	else if( rc < 4*st )
		return 3;
	else if( rc < 5*st )
		return 4;
	else
		return 5;
}

class Mode_Base;
void ModeRegister( Mode_Base* mode, uint8_t id );

enum ModeResult
{
	MR_OK = 0 ,
	MR_Err ,
};

class Mode_Base
{
	private:
		
	public:
		SName name;
		Mode_Base( SName name, uint8_t mode_id )
		{
			this->name = name;
			ModeRegister( this, mode_id );
		}		
		//ģʽ������
		virtual ModeResult main_func( void* param1, uint32_t param2 ) = 0;
		//��ȡģʽMavlinkģʽ���
		virtual void get_MavlinkMode( ModeFuncCfg cfg, Receiver rc, 
																	uint8_t btn_zones[4], AFunc* mode )
		{
			*mode = AFunc_Stabilize;
//			*mav_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
//			*mav_main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
//			*mav_sub_mode = 0;
		}
};

//��ǰ�����¼�����ڶϵ����ɣ�
struct CurrentWpInf
{
	//��ǰ�������
	uint32_t CurrentWp[2];
	//��������AB A-Ŀ�� B���
	double line_x;
	double line_y;
	double line_z;
	//������ɾ���
	double line_fs;
	//�������վ���
	double CamTrigDist;
}__PACKED;

void init_Modes();