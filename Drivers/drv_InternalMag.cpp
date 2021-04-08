#include "Basic.hpp"
#include "drv_ExtIIC.hpp"
#include "SensorsBackend.hpp"

/*�������̶���*/
	struct Internal_MagnetoMeter
	{
		//����������
		SName name;
		
		//iic��ַ
		unsigned char device_address;
		//���ݼĴ�����ַ
		unsigned char data_address;
		
		//�Ƿ��λ��ǰ
		bool MSB;
		//�Ƿ���Ҫ�ֶ���ʼ����
		bool need_set_sample;
		//�����Ĵ�����ַ
		unsigned char sample_address;
		//�����Ĵ���ֵ
		unsigned char sample_cfg;
		
		//ID�Ĵ�����ַ
		unsigned char ID_address;
		//ID
		unsigned char ID;
		
		//���üĴ�����ַ
		unsigned char configurations_addresses[10];
		//������Ŀ
		unsigned char configuration_count;
		//����
		unsigned char configurations[10];
		//�����ȣ�����->Gauss��
		double sensitivity;
		
		//�����ʣ�����msһ�����ݣ�
		unsigned char sample_mseconds;
		
		//����˳��
		//��1��ʼ������Ϊ����
		//�磺1,-3,2��������Ϊ x , -z , y
		signed char axis_index[3];
	};
		
	static const Internal_MagnetoMeter Internal_MagnetoMeters[] = 
	{
	/*     ����     , iic��ַ, ���ݼĴ�����ַ , ��λ��ǰ, �ֶ����� , �����Ĵ�����ַ, ��������, ID ��ַ , ID   ,      ���üĴ�����ַ      , ������Ŀ,  ����                                                     ,  ������      , �����ʺ���  , ����                 ,  */
		//{ "IST8310"   , 0x0c   ,      3         ,   false ,    true  ,          0xa  ,      1  ,      0  , 0x10 ,      { 0x41, 0x42 }      , 2       , { 0x24 , 0xc0 }                                           , 0.003        , 30          , { -2 , -1 , 3 }    } , //IST8310
		{ "FXOS8700"  , 0x1c   ,     0x33       ,   true ,    false  ,            0  ,      0  ,   0x0d  , 0xc7 ,      { 0x2a, 0x5b }      , 2       , { (3<<3)|(1<<0) , (7<<2)|(1<<0) }                         , 0.001        , 30          , { 2 , -1 , 3 }    } , //FXOS8700
		//{ "QMC6983"   , 0x0c   ,      3         ,   false ,    true  ,          0xa  ,      1  ,      0  , 0x10 ,      { 0x41, 0x42 }      , 2       , { 0x24 , 0xc0 }                                           , 0.003        , 30          , { -2 , -1 , 3 }    } , //QMC6983
	};
	static const uint8_t Supported_Internal_MagnetoMeter_Count = sizeof( Internal_MagnetoMeters ) / sizeof( Internal_MagnetoMeter );
/*�������̶���*/


static void InternalMag_Server(void* pvParameters)
{
ScanExtMag:
	//��ǰʹ�õ����������ͺ�
	int8_t current_ExtMag = -1;
	//������
	Static_AXIDMABuf uint8_t tx_buf[12];
	__attribute__ ((aligned (4))) Static_AXIDMABuf uint8_t rx_buf[32];
	
	uint8_t magnetometer_ind;
	
	//�������
	bool res;
	//׼ȷ������ʱ
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		vTaskDelayUntil( &xLastWakeTime, 0.1*configTICK_RATE_HZ );
		for( int8_t i = 0; i < Supported_Internal_MagnetoMeter_Count; ++i )
		{
			//�жϴ�����ID�Ƿ���ȷ
			tx_buf[0] = Internal_MagnetoMeters[i].ID_address;
			res = ExtIIC_SendReceiveAddr7( Internal_MagnetoMeters[i].device_address, tx_buf, 1, rx_buf, 1 );
			if(!res)
				continue;
			if( rx_buf[0] != Internal_MagnetoMeters[i].ID )
				continue;
			
			//���ʹ���������			
			for( uint8_t k = 0; k < Internal_MagnetoMeters[i].configuration_count; ++k )
			{
				tx_buf[0] = Internal_MagnetoMeters[i].configurations_addresses[k];
				tx_buf[1] = Internal_MagnetoMeters[i].configurations[k];
				res = ExtIIC_SendAddr7( Internal_MagnetoMeters[i].device_address, tx_buf, 2 );
				if(!res)
					break;
			}			
			if(!res)
				continue;
			
			//��鴫���������Ƿ�ɹ�
			for( uint8_t k = 0; k < Internal_MagnetoMeters[i].configuration_count; ++k )
			{
				tx_buf[0] = Internal_MagnetoMeters[i].configurations_addresses[k];
				res = ExtIIC_SendReceiveAddr7( Internal_MagnetoMeters[i].device_address, tx_buf, 1, rx_buf, 1 );
				if(!res)
					break;
				if( rx_buf[0] != Internal_MagnetoMeters[i].configurations[k] )
				{	//����У�����
					res = false;
					break;
				}
			}
			if(!res)
				continue;
			
			//�ѳɹ�ʶ�𵽴�����
			if( IMUMagnetometerRegister( Internal_Magnetometer_Index, Internal_MagnetoMeters[i].name, Internal_MagnetoMeters[i].sensitivity ) )
			{
				magnetometer_ind = Internal_Magnetometer_Index;
				current_ExtMag = i;
				goto ExtMagDetected;
			}
			else if( IMUMagnetometerRegister( 1, Internal_MagnetoMeters[i].name, Internal_MagnetoMeters[i].sensitivity ) )
			{
				magnetometer_ind = 1;
				current_ExtMag = i;
				goto ExtMagDetected;
			}
			else
				os_delay(5.0);
		}
	}
	
ExtMagDetected:
	const Internal_MagnetoMeter* sensor = &Internal_MagnetoMeters[current_ExtMag];
	while(1)
	{
		//���ڶԴ���������
		vTaskDelay( sensor->sample_mseconds*1e-3*configTICK_RATE_HZ+1 );
		
		//����
		uint8_t rt = 0;
		do
		{
			//ʧ�ܴ�����������ɨ��
			if( ++rt > 3 )
			{
				IMUMagnetometerUnRegister(Internal_Magnetometer_Index);
				goto ScanExtMag;
			}
			//��ȡ����
			tx_buf[0] = sensor->data_address;
			res = ExtIIC_SendReceiveAddr7( sensor->device_address, tx_buf, 1, rx_buf, 6 );
			if( sensor->need_set_sample )
			{	//��Ҫ���Ͳ���ָ��
				tx_buf[0] = sensor->sample_address;
				tx_buf[1] = sensor->sample_cfg;
				res &= ExtIIC_SendAddr7( sensor->device_address, tx_buf, 2 );
			}
		}while(res==false);
		
		/*���´���������*/						
			//���תС��
			if( sensor->MSB )
			{
				((uint16_t*)&rx_buf)[0] = __REV16( ((uint16_t*)&rx_buf)[0] );
				((uint16_t*)&rx_buf)[1] = __REV16( ((uint16_t*)&rx_buf)[1] );
				((uint16_t*)&rx_buf)[2] = __REV16( ((uint16_t*)&rx_buf)[2] );
			}			
			
			//����ת��
			vector3<int32_t> data;
			if( sensor->axis_index[0] > 0 )
				data.x = (signed short)((uint16_t*)&rx_buf)[ sensor->axis_index[0] - 1 ];
			else
				data.x = -(signed short)((uint16_t*)&rx_buf)[ (-sensor->axis_index[0]) - 1 ];
			if( sensor->axis_index[1] > 0 )
				data.y = (signed short)((uint16_t*)&rx_buf)[ sensor->axis_index[1] - 1 ];
			else
				data.y = -(signed short)((uint16_t*)&rx_buf)[ (-sensor->axis_index[1]) - 1 ];
			if( sensor->axis_index[2] > 0 )
				data.z = (signed short)((uint16_t*)&rx_buf)[ sensor->axis_index[2] - 1 ];
			else
				data.z = -(signed short)((uint16_t*)&rx_buf)[ (-sensor->axis_index[2]) - 1 ];
				
			//��������
			IMUMagnetometerUpdate( magnetometer_ind, data, false);
		/*���´���������*/
	}
}

void init_drv_InternalMag()
{
	xTaskCreate( InternalMag_Server, "InternalMag", 1024, NULL, SysPriority_ExtSensor, NULL);
}
