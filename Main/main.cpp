#include "Basic.hpp"
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "drv_Main.hpp"
#include "MS_Main.hpp"
#include "Parameters.hpp"
#include "FlashIO.h"
#include "Modes.hpp"
#include "MSafe.hpp"
#include "ctrl_Main.hpp"
#include "drv_PWMOut.hpp"
#include "AC_Math.hpp"
#include "Missions.hpp"
#include "ControlSystem.hpp"

#include "debug.hpp"

#if 1 //���û����Σ�����Ҫ��targetѡ����ѡ��ʹ��USE microLIB

	__asm(".global __use_no_semihosting\n\t") ;//ע�ͱ���, ����1
	extern "C"
	{
//		struct __FILE {
//		int handle;
//		};
//		std::FILE __stdout;

		void _sys_exit(int x)
		{
			x = x;
		}

		//__use_no_semihosting was requested, but _ttywrch was referenced, �������º���, ����2
		void _ttywrch(int ch)
		{
			ch = ch;
		}
		
		char *_sys_command_string(char *cmd, int len)
		{
				return 0;
		}
 
	}
#endif
	
//�̼��汾
#define Firmware_Version 16.5
	
void DriverInit_task(void* pvParameters)
{
	//��ʼ���豸����
	init_drv_Main();
	//��ʼ������ϵͳ
	init_MS_Main();
	init_Debug();
	init_Modes();
	init_MSafe();
	init_ControlSystem();	
	
	/*ע���ʼ������*/
		struct
		{
			uint32_t calib_ESC[2];	//У׼���
			uint32_t calib_ESC_T[2];	//���У׼ʱ��
			uint32_t boot_count[2];	//ϵͳ���д���
			float Firmvare_Version[2];	//�̼��汾
		}init_cfg;
		init_cfg.calib_ESC[0] = 0;
		init_cfg.calib_ESC_T[0] = 3;
		init_cfg.boot_count[0] = 0;
		init_cfg.Firmvare_Version[0] = 16.3;
		MAV_PARAM_TYPE param_types[] = {
			MAV_PARAM_TYPE_UINT32 ,	//У׼���
			MAV_PARAM_TYPE_UINT32 ,	//���У׼ʱ��
			MAV_PARAM_TYPE_UINT32 , //ϵͳ���д���
			MAV_PARAM_TYPE_REAL32   //�̼��汾
			
		};
		SName param_names[] = {
			"Init_CalibESC",	//У׼���
			"Init_CalibESC_T",//���У׼ʱ��
			"Init_Boot_Count",//ϵͳ���д���
			"Init_Firmware_V" //�̼��汾
		};
		ParamGroupRegister( "Init", 1, 4, param_types, param_names, (uint64_t*)&init_cfg );	
		/*ע���ʼ������*/
	
	//��ɳ�ʼ��
	//��ɺ����ٽ��г�ʼ������
	while( getInitializationCompleted() == false )
	{
		setInitializationCompleted();
		os_delay(0.1);
	}
	
	uint8_t uat_type[8];
	if( ReadParam( "AC_UAVType", 0, 0, (uint64_t*)uat_type, 0 ) == PR_OK )
	{	//��ȡ���������
		set_MainMotorCount(UAV_MainMotorCount(uat_type[0]));
	}
	
	/*��ȡ��ʼ���������г�ʼ������*/
		//����ʼ������
		ReadParamGroup( "Init", (uint64_t*)&init_cfg, 0 );
	
		//У׼���
		if( init_cfg.calib_ESC[0] == 21586 )
		{
			MainMotor_PullUpAll();
			os_delay(init_cfg.calib_ESC_T[0]);
			MainMotor_PullDownAll();
			
			init_cfg.calib_ESC[0] = 0;
		}
		
		//ϵͳ����������1
		init_cfg.boot_count[0]++;
	  	init_cfg.Firmvare_Version[0] = Firmware_Version;
		//���ó�ʼ������
		UpdateParamGroup( "Init", (uint64_t*)&init_cfg, 0, sizeof(init_cfg)/8 );
		
		//���͵�����
		MainMotor_PullDownAll();
	/*��ȡ��ʼ���������г�ʼ������*/
	
	//���������ʼ��
	init_Missions();
	
	//ɾ��������
	vTaskDelete(0);
}
	
int main(void)
{	
	//��ʼ��оƬʱ��
	//ʱ���׼�Ȼ�������
  init_Basic();
			
	//������ʼ�����񲢽������������
	xTaskCreate( DriverInit_task , "Init" ,8192,NULL,3,NULL);
	vTaskStartScheduler();
	while(1);
}

extern "C" void HardFault_Handler()
{
	//�����ж������������
	PWM_PullDownAll();
}

extern "C" void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
	static char StackOvTaskName[20];
	strcpy( StackOvTaskName, (char*)pcTaskName );
}



