#include "M30_AttCtrl.hpp"
#include "vector3.hpp"
#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "AC_Math.hpp"
#include "Receiver.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"

M30_AttCtrl::M30_AttCtrl():Mode_Base( "AttCtrl", 30 )
{
	
}

ModeResult M30_AttCtrl::main_func( void* param1, uint32_t param2 )
{
	setLedMode(LEDMode_Flying1);
	Attitude_Control_Enable();
	uint16_t exit_mode_counter_rs = 0;
	uint16_t exit_mode_counter = 0;
	uint16_t exit_mode_Gcounter = 0;
	while(1)
	{
		os_delay(0.02);
			
		Receiver rc;
		getReceiver( &rc, 0, 0.02 );
		
		if( rc.available )
		{
			set_mav_mode( 
				MAV_MODE_FLAG_SAFETY_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
				PX4_CUSTOM_MAIN_MODE_STABILIZED,
				0 );

			/*�ж��˳�ģʽ*/
				//��ȡ����״̬
				bool inFlight;
				get_is_inFlight(&inFlight);
				if( inFlight )
				{
					exit_mode_counter_rs = 400;
					if( exit_mode_counter < exit_mode_counter_rs )
						exit_mode_counter = exit_mode_counter_rs;
				}
				//������Զ�����
				if( inFlight==false && rc.data[0]<30 )
				{
					if( ++exit_mode_counter >= 500 )
					{
						Attitude_Control_Disable();
						return MR_OK;
					}
				}
				else
					exit_mode_counter = exit_mode_counter_rs;
				//���Ƽ���
				if( inFlight==false && (rc.data[0] < 5 && rc.data[1] < 5 && rc.data[2] < 5 && rc.data[3] > 95) )
				{
					if( ++exit_mode_Gcounter >= 50 )
					{
						Attitude_Control_Disable();
						return MR_OK;
					}
				}
				else
					exit_mode_Gcounter = 0;
			/*�ж��˳�ģʽ*/
			
			//���Ÿ�ֱ�ӿ�����
			Attitude_Control_set_Throttle(rc.data[0]);
			//��������˿ظ������
			double RPCtrlScale = degree2rad( get_maxLean() / 50.0 );
			Attitude_Control_set_Target_RollPitch( ( rc.data[3] - 50 )*RPCtrlScale , ( rc.data[2] - 50 )*RPCtrlScale );
			//ƫ�������м���ƫ��
			//�����м����ƫ���ٶ�
			double YCtrlScale = degree2rad( get_maxYawSpeed() / 50.0 );
			if( in_symmetry_range_mid( rc.data[1] , 50 , 5 ) )
				Attitude_Control_set_YawLock();
			else
				Attitude_Control_set_Target_YawRate( ( 50 - rc.data[1] )*YCtrlScale );
		}
		else
		{
			//��ң���źŽ��밲ȫģʽ
			enter_MSafe();		
			/*�ж��˳�ģʽ*/
				bool inFlight;
				get_is_inFlight(&inFlight);
				if( inFlight==false )
				{
					Attitude_Control_Disable();
					return MR_OK;
				}
			/*�ж��˳�ģʽ*/
		}
	}
	return MR_OK;
}
