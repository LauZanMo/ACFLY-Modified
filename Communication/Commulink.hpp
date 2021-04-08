#pragma once

#include "Basic.hpp"

struct CommulinkConfig
{
	//ͨ�Ŷ˿�id
	uint8_t sys_id[8];
	uint8_t comp_id[8];
	//Uart1����
	uint8_t Uart1_Func[8];
	uint32_t Uart1_Param[2];
	//Uart3����
	uint8_t Uart3_Func[8];
	uint32_t Uart3_Param[2];
	//Uart5����
	uint8_t Uart5_Func[8];
	uint32_t Uart5_Param[2];
	//Uart7����
	uint8_t Uart7_Func[8];
	uint32_t Uart7_Param[2];
	//Uart8����
	uint8_t Uart8_Func[8];
	uint32_t Uart8_Param[2];
};

/*������ʾ*/
	enum LEDSignal
	{
		LEDSignal_Start1 ,
		LEDSignal_Start2 ,
		
		LEDSignal_Continue1 ,
		LEDSignal_Success1 ,
		
		LEDSignal_Err1 ,
	};
	enum LEDMode
	{
		//�ر��Զ�ģʽ
		LEDMode_Manual ,
		
		//����ģʽ
		LEDMode_Normal1 ,
		LEDMode_Normal2 ,
		
		//����ģʽ
		LEDMode_Flying1 ,
		LEDMode_Flying2 ,
		
		//���ڴ���
		LEDMode_Processing1 ,
		LEDMode_Processing2 ,
	};
	void sendLedSignal( LEDSignal signal );
	void setLedMode( LEDMode mode );
	void setLedManualCtrl( float R, float G, float B, bool BuzzerOn, uint16_t BuzzerFreq );
/*������ʾ*/
	
/*�˿�*/
	//�˿ڶ���
	typedef struct
	{
		//д�˿ں���
		uint16_t (*write)( const uint8_t* data, uint16_t length, double Write_waitTime, double Sync_waitTime );
		//�ȴ��������
		bool (*wait_sent)( double waitTime );
		//������������
		bool (*lock)( double Sync_waitTime );
		void (*unlock)();
		//���˿ں���
		uint16_t (*read)( uint8_t* data , uint16_t length, double Rc_waitTime, double Sync_waitTime );
		//��ս��ջ�����
		bool (*reset_rx)( double Sync_waitTime );
		//���Ĳ����ʺ���
		bool (*SetBaudRate)( uint32_t baud_rate, double Send_waitTime, double Sync_waitTime );
	}Port;
	#define Port_isBasicFunc(port) ( port.read && port.write && port.lock && port.unlock )
	#define Port_isFullFunc(port) ( port.read && port.write && port.lock && port.unlock && port.reset_rx && port.wait_sent )
	
	#define MAXPorts 10
	//ע��˿�
	bool PortRegister( uint8_t ind, Port port );
/*�˿�*/
	
/*ͨ�Ŷ˿�*/
	//��ָ���˿�������Ϣ����
	bool SetMsgRate( uint8_t port_index, uint16_t Msg, float RateHz, double TIMEOUT = -1 );
	//��ָ���˿ڷ�����Ϣ�б�
	void sendParamList();
	//��ȡ�˿�
	const Port* get_CommuPort( uint8_t port );
	
	//�趨mavlinkģʽ
	bool set_mav_mode_arm();
	bool set_mav_mode_disarm();
	bool set_mav_mode( uint16_t req_mav_mode, uint16_t req_mav_main_mode, uint16_t req_mav_sub_mode );
	bool get_mav_mode( uint16_t* req_mav_mode, uint16_t* req_mav_main_mode, uint16_t* req_mav_sub_mode );
	
	//��ȡ����id
	uint8_t get_CommulinkSysId();
	uint8_t get_CommulinkCompId();
/*ͨ�Ŷ˿�*/
	
/*RTK�˿�*/
	//�˿ڶ���
	typedef struct
	{
		bool ena;
		//д�˿ں���
		uint16_t (*write)( const uint8_t* data, uint16_t length, double Write_waitTime, double Sync_waitTime );
		//������������
		bool (*lock)( double Sync_waitTime );
		void (*unlock)();
	}RtkPort;
	
	//ע��Rtk�˿�
	int8_t RtkPortRegister( RtkPort port );
	//ʹ��ʧ��Rtk�˿�
	bool RtkPort_setEna( uint8_t port, bool ena );
	//��ȡ�˿�
	const RtkPort* get_RtkPort( uint8_t port );
	//��rtk�˿ڷ���ע������
	void inject_RtkPorts( const uint8_t data[], uint16_t length );
/*RTK�˿�*/
	
/*���ܽӿ�*/
	bool PortFunc_Register( uint8_t FuncInd, bool (*init)( Port port, uint32_t param ) );
/*���ܽӿ�*/
	
void init_Commulink();