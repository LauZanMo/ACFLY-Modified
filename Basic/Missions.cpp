#include "Missions.hpp"

#include "Parameters.hpp"
#include "semphr.h"
#include "Modes.hpp"

#define MissionParamVersion 1
#define MaxMissions 512
#define MissionsInParamGroupBit 5
#define MissionsInParamGroup (1<<MissionsInParamGroupBit)

static SemaphoreHandle_t MissionsSemphr = xSemaphoreCreateMutex();

static inline bool Lock_Missions( double TIMEOUT = -1 )
{
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake( MissionsSemphr , TIMEOUT_Ticks ) )
		return true;
	return false;
}
static inline void UnLock_Missions()
{
	xSemaphoreGive(MissionsSemphr);
}

//�������
static uint16_t MissionsCount = 0;
static uint16_t UploadingMissionsCount = 0;

//��ǰ����
static uint16_t CurrentMission = 0;
/*
	���õ�ǰ����
	wpInd����ǰ�������
*/
bool setCurrentMission( uint16_t wpInd )
{
	if( MissionsCount==0 && wpInd==0 )
	{
		CurrentMission = 0;
		return false;
	}
	if( wpInd >= MissionsCount )
		return false;
	CurrentMission = wpInd;
	return true;
}
/*
	��ȡ��ǰ�������
*/
uint16_t getCurrentMissionInd() { return CurrentMission; }

/*
	��ȡ�������
*/
uint16_t getMissionsCount()
{
	return MissionsCount;
}
/*
	��ȡ�����ϴ��������
*/
uint16_t getUploadingMissionsCount()
{
	return UploadingMissionsCount;
}

/*
	������к�������
	TIMEOUT: �߳�ͬ����ʱʱ��

	����ֵ��
	true:��ӳɹ�
	false:���ʧ�ܣ��������򺽵���Ϣ�������������
*/
bool clearMissions( double TIMEOUT )
{
	if( Lock_Missions(TIMEOUT) )
	{
		TruncateVolatileParamGroup( "Missions", 0 );
		
		CurrentMission = MissionsCount = UploadingMissionsCount = 0;
		
		UnLock_Missions();
		
	}
	else
		return false;
	
	CurrentWpInf initial_CurrentWpInf;
	initial_CurrentWpInf.CurrentWp[0] = 0;
	initial_CurrentWpInf.line_x = 0;
	initial_CurrentWpInf.line_y = 0;
	initial_CurrentWpInf.line_z = 0;
	initial_CurrentWpInf.line_fs = -1;
	UpdateParamGroup( "CurrentWp", (uint64_t*)&initial_CurrentWpInf, 0, sizeof(CurrentWpInf)/8 );
	return true;
}

/*
	��Ӻ�������
	wp_inf��������Ϣ
	st���Ƿ�д��洢����ֻ�е�ǰʵ�ʺ�������Ϊ0�ſ��Ի��治д��洢����
	TIMEOUT: �߳�ͬ����ʱʱ��

	����ֵ��
	true:��ӳɹ�
	false:���ʧ�ܣ��������򺽵���Ϣ�������������
*/
bool addMission( MissionInf wp_inf, bool st, double TIMEOUT )
{
	if( wp_inf.cmd == 0 )
		return false;
	if( MissionsCount > MaxMissions )
		return false;
	if( st==false && MissionsCount>0 )
		return false;
	if( st && MissionsCount!=UploadingMissionsCount )
		return false;
	
	if( Lock_Missions(TIMEOUT) )
	{
		WriteVolatileParamGroup( "Missions", &wp_inf, UploadingMissionsCount, 1, st );
		if(st)
		{
			++MissionsCount;
			UploadingMissionsCount = MissionsCount;
		}
		else
			++UploadingMissionsCount;
		
		UnLock_Missions();
		return true;
	}
	return false;
}

/*
	���溽������
	TIMEOUT: �߳�ͬ����ʱʱ��

	����ֵ��
	true:����ɹ�
	false:����ʧ�ܣ���ʱ��
*/
bool saveMissions( double TIMEOUT )
{
	if( UploadingMissionsCount == 0 )
		return true;
	
	if( Lock_Missions(TIMEOUT) )
	{
		SaveVolatileParamGroup( "Missions" );
		MissionsCount = UploadingMissionsCount;
		
		UnLock_Missions();
		return true;
	}
	return false;
}

/*
	��ȡ��������
	wp_ind���������
	wp_inf����ȡ�ĺ�����Ϣ
	TIMEOUT: �߳�ͬ����ʱʱ��

	����ֵ��
	true:��ȡ�ɹ�
	false:��ȡʧ�ܣ��޺������������
*/
bool ReadMission( uint16_t wp_ind, MissionInf* wp_inf, double TIMEOUT )
{
	if( wp_ind >= MissionsCount )
		return false;
	
	PR_RESULT res = ReadVolatileParamGroup( "Missions", wp_inf, wp_ind, 1, TIMEOUT );
	
	if( res == PR_ERR )
		return false;
	return true;
}

/*
	��ȡ��ǰ��������
	wp_inf����ȡ�ĺ�����Ϣ
	ind����ǰ�������
	TIMEOUT: �߳�ͬ����ʱʱ��

	����ֵ��
	true:��ȡ�ɹ�
	false:��ȡʧ�ܣ��޺������������
*/
bool ReadCurrentMission( MissionInf* wp_inf, uint16_t* ind, double TIMEOUT )
{
	if( MissionsCount == 0 )
		return false;
	if( CurrentMission >= MissionsCount )
		return false;
	
	PR_RESULT res = ReadVolatileParamGroup( "Missions", wp_inf, CurrentMission, 1, TIMEOUT );

	if( res == PR_ERR )
		return false;
	if(ind!=0)
		*ind = CurrentMission;
	return true;
}

void init_Missions()
{	
	char WPGroupName[17];	
	MissionsCount = UploadingMissionsCount = 0;
	
	VolatileParamGroupRegister( "Missions", 1, sizeof(MissionInf), 50 );
	uint16_t missions_count;
	GetVolatileParamGroupParamCount( "Missions", &missions_count );
	MissionsCount = UploadingMissionsCount = missions_count;
	
	//��ȡģʽ����
	ModeFuncCfg MFunc_cfg;
	ReadParamGroup( "MFunc", (uint64_t*)&MFunc_cfg, 0 );
	if( MFunc_cfg.RstWp0[0] == 0 )
	{
		//��ʼ��CurrentMissionInd
		CurrentWpInf currentWpInf;
		ReadParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0 );
		
		if( currentWpInf.CurrentWp[0] < missions_count )
			setCurrentMission(currentWpInf.CurrentWp[0]);
	}
	else
		setCurrentMission(0);
}