#include "TimeBase.hpp"

#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rtc.h"
#include "stdlib.h"

#include "FreeRTOS.h"
#include "semphr.h"

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 480M (CPU Clock)
  *            HCLK(Hz)                       = 240M (AXI and AHBs Clock)
  *            AHB Prescaler                  = 2
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  120MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  120MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  120MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  120MHz)
  *            HSE Frequency(Hz)              = 16M
  *            PLL_M                          = 1
  *            PLL_N                          = 60
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
    69. 因为 APB1 prescaler != 1, 所以 APB1 上的 TIMxCLK = APB1 x 2 = 240MHz;
    70. 因为 APB2 prescaler != 1, 所以 APB2 上的 TIMxCLK = APB2 x 2 = 240MHz;
    71. APB4 上面的 TIMxCLK 没有分频，所以就是 120MHz;
    72.
    73. APB1 定时器有 TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM7, TIM12, TIM13, TIM14，LPTIM1
    74. APB2 定时器有 TIM1, TIM8 , TIM15, TIM16，TIM17
    75.
    76. APB4 定时器有 LPTIM2，LPTIM3，LPTIM4，LPTIM5
  * @param  None
  * @retval None
  */
static inline void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
	__HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMLOW);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
	#ifdef SYSFREQ480
		RCC_OscInitStruct.PLL.PLLN = 60;
	#else
		RCC_OscInitStruct.PLL.PLLN = 50;
	#endif
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1);
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    while(1);
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1|RCC_PERIPHCLK_SDMMC
															|RCC_PERIPHCLK_USART16|RCC_PERIPHCLK_USART234578
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_QSPI|RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_SPI4|RCC_PERIPHCLK_I2C123
															|RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 50;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 4;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
	PeriphClkInitStruct.PLL3.PLL3M = 1;
  PeriphClkInitStruct.PLL3.PLL3N = 50;
  PeriphClkInitStruct.PLL3.PLL3P = 4;
  PeriphClkInitStruct.PLL3.PLL3Q = 8;
  PeriphClkInitStruct.PLL3.PLL3R = 8;
	PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
	PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
	PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_PLL3;
	PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL3;
  PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_D1HCLK;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
	PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PLL2;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
	PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    while(1);
  }

  /** Enable USB Voltage detector 
  */
  HAL_PWREx_EnableUSBVoltageDetector();
	
	/*RTC初始化*/
		//打开RTC
		RCC->BDCR |= (1<<15);
		for( int i = 0 ; i < 1000 ; ++i );
		/* Wait the registers to be synchronised */
		while((RTC->ISR&(1<<5))!=(1<<5));
		
		//如果日历尚未初始化过,则进行初始化
		RTC_TimeStruct RTC_Time;
		RTC_Time.SubSeconds=(uint32_t)RTC->SSR;
		uint32_t RTC_TR = RTC->TR;
		uint32_t RTC_DR = RTC->DR;
		RTC_Time.Hours = ((RTC_TR>>20) & 0x3)*10 + ((RTC_TR>>16) & 0xf);
		RTC_Time.Minutes = ((RTC_TR>>12) & 0x7)*10 + ((RTC_TR>>8) & 0xf);
		RTC_Time.Seconds = ((RTC_TR>>4) & 0x7)*10 + ((RTC_TR>>0) & 0xf);
		uint32_t div_s = ( RTC->PRER & 0x7fff);
		RTC_Time.Seconds_f = RTC_Time.Seconds + (div_s-RTC_Time.SubSeconds)/(double)(div_s+1);
		
		RTC_Time.Year = 2000 + ((RTC_DR>>20) & 0xf)*10 + ((RTC_DR>>16) & 0xf);
		RTC_Time.Month = ((RTC_DR>>12) & 0x1)*10 + ((RTC_DR>>8) & 0xf);
		RTC_Time.Date = ((RTC_DR>>4) & 0x3)*10 + ((RTC_DR>>0) & 0xf);
		RTC_Time.WeekDay = ((RTC_DR>>13) & 0x7);
		if( (RTC->ISR&(1<<4))!=(1<<4) || RTC_Time.Month==0 || RTC_Time.Date==0 )
		{
			//解锁所有 RTC 寄存器
			RTC->WPR=0xCA;
			RTC->WPR=0x53;
			
			//进入初始化模式
			RTC->ISR|=(1<<7);

			//等待进入初始化模式
			while((RTC->ISR&(1<<6))!=(1<<6));

			//设置异步预分频系数128和同步预分频系数256,日历计数器生成 1 Hz 时钟
			RTC->PRER|=((128-1)<<16);
			RTC->PRER|=((256-1)<<0);

			//24小时制
			RTC->TR = (0<<22) | 
			//小时:12
								(1<<20)|(2<<16) |
			//分钟:00
								(0<<12)|(0<<8) |
			//秒:00
								(0<<4)|(0<<0);
			
			//日：7
			RTC->DR = (0<<4)|(7<<0) | 
			//月份:10
								(1<<12)|(0<<8) |
			//年份:21
								(1<<20)|(21<<16);
			
			//24小时/天格式
			RTC->CR&=~(1<<6);

			//退出初始化模式
			RTC->ISR&=~(1<<7);
			
			//Enable the write protection for RTC registers 
			RTC->WPR=0xFF;
		}
	/*RTC初始化*/
}

namespace __global_time
{
	const uint32_t timer_load = 0xFFFFFFF0;
	
	volatile uint32_t current_time_part_1 = 0;
	const double TIM2sec_scale = 1.0 / (double)TIMEBASECLK;
	const double TIM2sec_part1_scale = (double)4294967296 / (double)TIMEBASECLK;
}

/*HAL库时间基准重写*/
	extern "C" uint32_t HAL_GetTick(void)
	{
		return TIME::get_System_Run_Time() * 1000;
	}
	extern "C" void HAL_Delay(uint32_t Delay)
	{
		delay(Delay*1e-3f);
	}
	extern "C" void HAL_SuspendTick(void)
	{
		NVIC_DisableIRQ( TIM6_DAC_IRQn );
	}
	extern "C" void HAL_ResumeTick(void)
	{
		NVIC_EnableIRQ( TIM6_DAC_IRQn );
	}
	extern "C" HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
	{
		return HAL_OK;
	}
/*HAL库时间基准重写*/

/*RTC时间获取*/
	static RTC_TimeStruct RTC_Time;
	static SemaphoreHandle_t RTC_Semaphore;
	
	//获取RTC时间
	extern "C" RTC_TimeStruct Get_RTC_Time()
	{
		if((RTC->ISR&(1<<5))!=0)
		{
			if( xSemaphoreTake( RTC_Semaphore, portMAX_DELAY ) == pdTRUE )
			{
				RTC_Time.SubSeconds=(uint32_t)RTC->SSR;
				uint32_t RTC_TR = RTC->TR;
				uint32_t RTC_DR = RTC->DR;
				RTC_Time.Hours = ((RTC_TR>>20) & 0x3)*10 + ((RTC_TR>>16) & 0xf);
				RTC_Time.Minutes = ((RTC_TR>>12) & 0x7)*10 + ((RTC_TR>>8) & 0xf);
				RTC_Time.Seconds = ((RTC_TR>>4) & 0x7)*10 + ((RTC_TR>>0) & 0xf);
				uint32_t div_s = ( RTC->PRER & 0x7fff);
				RTC_Time.Seconds_f = RTC_Time.Seconds + (div_s-RTC_Time.SubSeconds)/(double)(div_s+1);
								
				RTC_Time.Year = 2000 + ((RTC_DR>>20) & 0xf)*10 + ((RTC_DR>>16) & 0xf);
				RTC_Time.Month = ((RTC_DR>>12) & 0x1)*10 + ((RTC_DR>>8) & 0xf);
				RTC_Time.Date = ((RTC_DR>>4) & 0x3)*10 + ((RTC_DR>>0) & 0xf);
				RTC_Time.WeekDay = ((RTC_DR>>13) & 0x7);
				
				RTC->ISR&=~(1<<5);
				xSemaphoreGive(RTC_Semaphore);
			}
		}
		return RTC_Time;
	}
	
	uint8_t RTC_ByteToBcd2(uint8_t Value)
	{
		uint32_t bcdhigh = 0u;
		uint8_t Param = Value;

		while(Param >= 10u)
		{
			bcdhigh++;
			Param -= 10u;
		}

		return  ((uint8_t)(bcdhigh << 4u) | Param);
	}
	
	
		
	//设置RTC日期和时间
	extern "C" void Set_RTC_Time(const RTC_TimeStruct* T)
	{
		if( xSemaphoreTake( RTC_Semaphore, 0 ) == pdTRUE )
		{
			//解锁所有 RTC 寄存器
			RTC->WPR=0xCA;
			RTC->WPR=0x53;
			//进入初始化模式
			RTC->ISR|=(1<<7);
			//等待进入初始化模式
			while((RTC->ISR&(1<<6))!=(1<<6));

			uint32_t tmpreg = 0;
			uint32_t datetmpreg = 0;
			tmpreg = (uint32_t) (( (uint32_t)RTC_ByteToBcd2(T->Hours)   << RTC_TR_HU_Pos)| \
													((uint32_t)RTC_ByteToBcd2(T->Minutes) << RTC_TR_MNU_Pos) | \
													((uint32_t)RTC_ByteToBcd2(T->Seconds)));
			
			datetmpreg = (uint32_t) (((uint32_t)RTC_ByteToBcd2(T->Year-2000)  << RTC_DR_YU_Pos) | \
															 ((uint32_t)RTC_ByteToBcd2(T->Month) << RTC_DR_MU_Pos) | \
															 ((uint32_t)RTC_ByteToBcd2(T->Date))                   | \
															 ((uint32_t)T->WeekDay << RTC_DR_WDU_Pos));									 
			
			RTC->DR = (uint32_t)(datetmpreg & RTC_DR_RESERVED_MASK);
			RTC->TR = (uint32_t)(tmpreg & RTC_TR_RESERVED_MASK);
			
			//退出初始化模式
			RTC->ISR&=~(1<<7);
			//Enable the write protection for RTC registers 
			RTC->WPR=0xFF;
			xSemaphoreGive(RTC_Semaphore);
		}
	}

  //根据经度来计算计算时区
	extern "C" int GetTimeZone(double lat,double lon)
	{
		int a,b,c,timezone;
		a = (int)(fabs(lon)+0.5);
		b = a/15; 
		c = a%15; 
		if((lat >=17.9 && lat <=53 && lon>=75 && lon<=125) || (lat>=40 && lat <=53 && lon>=125 && lon<= 135))
		{ 
			/*北京时区为东八区*/
			timezone = 8;
		}
		else
		{
			if(c > 7.5)
					timezone = b+1;
			else
					timezone = b;
			if(lon > 0.0f)
					timezone = timezone;
			else
					timezone = (-1)*timezone;
		}
		return timezone;
	}	
	
  //UTC转本地时间
	extern "C" void UTC2LocalTime(RTC_TimeStruct* rtc_local, uint16_t utc_year,uint8_t utc_month,uint8_t utc_day,
		                        uint8_t utc_hour,uint8_t utc_minute,uint8_t utc_second,int8_t TimeZone,int8_t leapS)
	{
		uint16_t year=0;
		uint8_t month=0,day=0,hour=0,minute=0,second=0;
		uint8_t lastday = 0;     //月的最后一天日期
		uint8_t lastlastday = 0; //上月的最后一天日期
	 
		year = utc_year;
		month = utc_month;
		day = utc_day;
		hour = utc_hour + TimeZone;
		minute = utc_minute;
	  second = utc_second + leapS;
		
		if(month==1 || month==3 || month==5 || month==7 || month==8 || month==10 || month==12)
		{
			lastday = 31;
			if( month == 3 ){
				if((year%400 == 0)||(year%4 == 0 && year%100 != 0))//判断是否为闰年
					lastlastday = 29;//闰年的2月为29天，平年为28天
				else
					lastlastday = 28;
			}else if(month == 8 || month == 1)
				lastlastday = 31;
			else 
				lastlastday = 30;
		}
		else if(month == 4 || month == 6 || month == 9 || month == 11)
		{
			lastday = 30;
			lastlastday = 31;
		}
		else
		{
			lastlastday = 31;
			if((year%400 == 0)||(year%4 == 0 && year%100 != 0))//闰年的2月为29天，平年为28天
					lastday = 29;
			else
					lastday = 28;
		}
		
		if( second > 59 )
		{
			second -= 60;
			minute += 1;
			if( minute > 59 ){
				minute -= 60;
				hour += 1;
			}			
		}else if(second < 0)
	  {
			second += 60;
			minute -= 1;
			if( minute < 0 ){
				minute += 60;
				hour -= 1;
			}			
		}
		if(hour >23)
		{//当算出的时大于或等于24：00时，应减去24：00，日期加一天
			hour -= 24;
			day += 1; 
			if(day > lastday)
			{//当算出的日期大于该月最后一天时，应减去该月最后一天的日期，月份加上一个月 
				day -= lastday;
				month += 1;		 
				if(month > 12)
				{//当算出的月份大于12，应减去12，年份加上1年
					month -= 12;
					year += 1;
				}
			}
		}else if( hour < 0 )
		{//当算出的时小于0时,应加上24：00，日期减一天
		  hour += 24;
		  day -= 1;
			if( day < 1 )
			{//当算出的日期小于1，日期等于上月的最后一天，月份减去一个月 
				day = lastlastday;
				month -= 1;	
				if( month < 1 )
				{//当算出的月份小于1，月份应等于12，年份减去1年
					month = 12;
				  year -= 1;
				}					
			}	      			
		}		
		rtc_local->Year = year;
		rtc_local->Month = month;
		rtc_local->Date = day;
		rtc_local->Hours = hour;
		rtc_local->Minutes = minute;
		rtc_local->Seconds = second;
		if( month == 1){
			month = 13;
			year--;
		}else if(month == 2){
			month = 14;
			year--;
		}
		rtc_local->WeekDay = ((day+2*month+3*(month+1)/5+year+year/4-year/100+year/400)%7)+1;	
		if( rtc_local->WeekDay > 6 || rtc_local->WeekDay < 0)
			rtc_local->WeekDay = 1;
	}	
	/*RTC时间获取*/
	
extern "C" void TIM5_IRQHandler()
{
	if( TIM5->SR & (1<<0) )
	{		
		++__global_time::current_time_part_1;
	}
	TIM5->SR = 0;
	__DSB();
}

void init_TimeBase()
{		
	//配置单片机时钟
	SystemClock_Config();
	//初始化HAL库
	HAL_Init();
	HAL_EnableCompensationCell();

	//打开TIM5用作全局计时器
	RCC->APB1LENR |= (1<<3);
	for( volatile int i = 0 ; i < 1000 ; ++i );
	//分频至10Mhz
	TIM5->PSC = ( APB1TIMERCLK / 10000000 ) - 1;
	TIM5->ARR = __global_time::timer_load;	
	TIM5->EGR = (1<<0);
	TIM5->SR = 0;
	//打开定时器中断（必须最高优先级）
	TIM5->DIER = 1<<0;
	NVIC_SetPriority( TIM5_IRQn , 0 );
	NVIC_EnableIRQ( TIM5_IRQn );
	//打开定时器
	TIM5->CR1 = (1<<7) | (1<<0);
	
		
	RTC_Semaphore = xSemaphoreCreateMutex();
}
