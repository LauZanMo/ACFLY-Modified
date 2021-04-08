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
	//LED初始化
	init_drv_LED();
	//ADC初始化
	init_drv_ADC();
	//等待电压稳定
	while( Get_VDDA_Voltage() < 3.0 )
	{	//等待电压稳定
		os_delay(0.1);
	}
	
	//内部存储初始化
	init_drv_Flash();
	init_InternalStorage();	
	//参数初始化
	init_Parameters();
	//存储外设驱动	
	init_drv_SDMMC();		
	//BL更新检测
	init_drv_BootLoader();
	
	//内部传感器驱动
	init_Sensors();
	init_Commulink();
	init_drv_Oled();
	init_GUI();
	os_delay(0.1);
	
	//PWWM驱动
	init_drv_PWMOut();	
	
	//SD卡驱动
	init_SDStorage();
	
	//端口驱动
	init_drv_USB();	
	init_drv_Uart1();
	init_drv_Uart3();
	init_drv_Uart5();
	init_drv_Uart7();
	init_drv_Uart8();
	
	//内部传感器驱动
	init_drv_Sensors();
	//接收机驱动
	init_drv_RCSbus();
	init_drv_RCPPM();
	
	//外置IIC驱动
	init_drv_ExtIIC();
	init_drv_ExtSPL06();
	init_drv_InternalMag();
	init_drv_ExtMag();
	
	//GPS驱动
	init_drv_GPS();
	init_drv_RTK();
	
	//光流驱动
	init_drv_OpticalFlow_LC302();
	init_drv_OpticalFlow_LC306();
	init_drv_OpticalFlow_JL32xx();
	init_drv_OpticalFlow_GL9306();

	//匿名光流驱动
	init_drv_AnoOpticalFlow();
	
	//测距传感器驱动
	init_drv_ultrasonic();
	init_drv_TFMini();
	
	//外部LED驱动
	init_drv_ExtLed();
	
	//init_drv_SDI();
}