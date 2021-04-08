#include "drv_Main.hpp"

#include "SensorsBackend.hpp"
#include "drv_LED.hpp"
#include "drv_Oled.hpp"
#include "GUI.hpp"
#include "drv_USB.hpp"
#include "drv_Uart1.hpp"
#include "drv_Uart3.hpp"
#include "drv_Uart5.hpp"
#include "drv_Uart7.hpp"
#include "drv_Uart8.hpp"
#include "Commulink.hpp"
#include "drv_SDMMC.hpp"
#include "drv_Sensors.hpp"
#include "drv_Ultrasonic.hpp"
#include "drv_Flash.hpp"
#include "drv_RCSbus.hpp"
#include "drv_RCPPM.hpp"
#include "drv_PWMOut.hpp"
#include "drv_ADC.hpp"

#include "StorageSystem.hpp"
#include "Parameters.hpp"
#include "drv_BootLoader.hpp"
#include "drv_ExtIIC.hpp"
#include "drv_GPS.hpp"
#include "drv_RTK.hpp"
#include "drv_InternalMag.hpp"
#include "drv_ExtMag.hpp"
#include "drv_ExtSPL06.hpp"
#include "drv_ExtLed.hpp"

#include "drv_OpticalFlow_LC302.hpp"
#include "drv_OpticalFlow_LC306.hpp"
#include "drv_OpticalFlow_JL32xx.hpp"
#include "drv_OpticalFlow_GL9306.hpp"
#include "drv_AnoOpticalFlow.hpp"
#include "drv_TFMini.hpp"

#include "drv_SDI.hpp"

void init_drv_Main()
{	
	//LED��ʼ��
	init_drv_LED();
	//ADC��ʼ��
	init_drv_ADC();
	//�ȴ���ѹ�ȶ�
	while( Get_VDDA_Voltage() < 3.0 )
	{	//�ȴ���ѹ�ȶ�
		os_delay(0.1);
	}
	
	//�ڲ��洢��ʼ��
	init_drv_Flash();
	init_InternalStorage();	
	//������ʼ��
	init_Parameters();
	//�洢��������	
	init_drv_SDMMC();		
	//BL���¼��
	init_drv_BootLoader();
	
	//�ڲ�����������
	init_Sensors();
	init_Commulink();
	init_drv_Oled();
	init_GUI();
	os_delay(0.1);
	
	//PWWM����
	init_drv_PWMOut();	
	
	//SD������
	init_SDStorage();
	
	//�˿�����
	init_drv_USB();	
	init_drv_Uart1();
	init_drv_Uart3();
	init_drv_Uart5();
	init_drv_Uart7();
	init_drv_Uart8();
	
	//�ڲ�����������
	init_drv_Sensors();
	//���ջ�����
	init_drv_RCSbus();
	init_drv_RCPPM();
	
	//����IIC����
	init_drv_ExtIIC();
	init_drv_ExtSPL06();
	init_drv_InternalMag();
	init_drv_ExtMag();
	
	//GPS����
	init_drv_GPS();
	init_drv_RTK();
	
	//��������
	init_drv_OpticalFlow_LC302();
	init_drv_OpticalFlow_LC306();
	init_drv_OpticalFlow_JL32xx();
	init_drv_OpticalFlow_GL9306();

	//������������
	init_drv_AnoOpticalFlow();
	
	//��ഫ��������
	init_drv_ultrasonic();
	init_drv_TFMini();
	
	//�ⲿLED����
	init_drv_ExtLed();
	
	//init_drv_SDI();
}