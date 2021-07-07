#include "drv_BootLoader.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "message_buffer.h"
#include <string.h>
#include "fatfs.h"
#include "ff.h"
#include "Basic.h"
#include "Basic.hpp"
#include "TimeBase.h"
#include "TimeBase.hpp"
#include "Commulink.hpp"
#include "drv_SDMMC.hpp"
#include "drv_LED.hpp"

//ASCALL��תchar
char ConvertHexChar(char ch)
{
	if((ch >= '0') && (ch <= '9'))
		return (ch-'0');
	else if((ch >= 'A') && (ch <= 'F'))
		return ((ch-'A')+10);
	else if((ch >= 'a') && (ch <= 'f'))
		return ((ch-'a')+10);
	else return (-1);
}

//Hex��ʽ�ṹ��
typedef struct
{
	uint8_t  data[32];
	uint8_t  type;
	uint8_t  count;
	uint32_t address;
	uint8_t  checksum;
	uint8_t  datalen;
}_HexData;

//Hex�ļ���������
uint8_t ParseHex(char data, _HexData* hex_data)
{
	static uint32_t OffsetAddress = 0;  //��չ���Ե�ַ 			
	static uint8_t states_machine = 0;  //״̬��
	static char data_len[2]; //ÿ�е���Ч���ݳ���
	static char addr[4];     //ÿ�еĵ�ַ
	static char data_type[2];//ÿ�е���������
	static char data_raw[32];//ÿ�е���Ч����
	static char checksum[2]; //ÿ�е�У��
		
	if( states_machine == 0 )
	{//��ȡð��,1���ַ�
		if(data == ':'){
			states_machine++;
			hex_data->checksum = 0;
		}
	}
	else if( states_machine < 3 )
	{//��ȡ��ǰ�е����ݳ���,1���ֽ�,2���ַ�
		data_len[states_machine-1] = data;
		states_machine++;	
		if(states_machine == 3){
			hex_data->datalen = (ConvertHexChar(data_len[0]) << 4) | ConvertHexChar(data_len[1]);
			hex_data->checksum += hex_data->datalen;			
		}		
	}
	else if( states_machine < 7 )
	{//��ȡ��ǰ�еĵ�ַ,2���ֽ�,4���ַ�
		addr[states_machine-3] = data;
		states_machine++;	
		if(states_machine == 7){
			hex_data->address = (ConvertHexChar(addr[0]) << 12) | (ConvertHexChar(addr[1]) << 8) |
			                    (ConvertHexChar(addr[2]) << 4) | (ConvertHexChar(addr[3]));						
			hex_data->checksum += (hex_data->address>>8) & 0xFF;
			hex_data->checksum += (hex_data->address) & 0xFF;		
		}
	}
	else if(states_machine < 9)
	{//ÿ����������,1���ֽ�,2���ַ�
	  data_type[states_machine - 7] = data;
	  states_machine++;	
		if(states_machine == 9){
			hex_data->type = (ConvertHexChar(data_type[0]) << 4) | (ConvertHexChar(data_type[1]));	
			hex_data->checksum += hex_data->type;	
		}
	}
	else if( states_machine < 9 + 2*hex_data->datalen )
	{//ÿ����Ч����
		data_raw[states_machine - 9] = data;
    states_machine++;
		if( states_machine == 9 + 2*hex_data->datalen ){
			for(uint8_t i = 0; i < hex_data->datalen; i++)
			{
				hex_data->data[i] = (ConvertHexChar(*(data_raw + 2*i)) << 4) | ConvertHexChar(*(data_raw + 2*i + 1));
				hex_data->checksum += hex_data->data[i];
			}
			if(hex_data->type == 4){
				//Extended Linear Address Record
				OffsetAddress = ((ConvertHexChar(data_raw[0]) << 12) | (ConvertHexChar(data_raw[1]) << 8) |
						             (ConvertHexChar(data_raw[2]) << 4) | (ConvertHexChar(data_raw[3]))) << 16;
			}				
		}
	}
	else if( states_machine < 9 + 2 * hex_data->datalen + 2)
	{//ÿ��У��,1���ֽ�,2���ַ�
	  checksum[states_machine - 9 - 2 * hex_data->datalen] = data;
		states_machine++;
		if( states_machine == 9 + 2 * hex_data->datalen + 2 ){			
			hex_data->checksum = 0x100 - ((hex_data->checksum));
			if( hex_data->checksum == ((ConvertHexChar(checksum[0]) << 4)|(ConvertHexChar(checksum[1]))) )
			{//У��ͨ��
				states_machine = 0;				
				if(OffsetAddress!=0)
					hex_data->address |= OffsetAddress;
				
					if(hex_data->type == 0)//Data Rrecord
						return 0;
					else if(hex_data->type == 1)//End of File Record
						return 1;
					else if(hex_data->type == 2)//Extended Segment Address Record
						return 2;
					else if(hex_data->type == 3)//Start Segment Address Record
						return 3;
					else if(hex_data->type == 4)//Extended Linear Address Record
						return 4;
					else if(hex_data->type == 4)//Start Linear Address Record
						return 5;								
			}
			else
				return 100;
		}	
	}
	return 6;
}	

//FLASH �����Ż�ȡ����
static uint32_t GetSector(uint32_t Address)
{
  if ( (Address < ADDR_FLASH_SECTOR_1_BANK1)&&(Address >= ADDR_FLASH_SECTOR_0_BANK1) )
  {
    return 0;
  }
  else if( (Address < ADDR_FLASH_SECTOR_2_BANK1)&&(Address >= ADDR_FLASH_SECTOR_1_BANK1) )
	{
		return 1;
	}
  else if( (Address < ADDR_FLASH_SECTOR_3_BANK1)&&(Address >= ADDR_FLASH_SECTOR_2_BANK1) )
	{
		return 2;
	}
  else if( (Address < ADDR_FLASH_SECTOR_4_BANK1)&&(Address >= ADDR_FLASH_SECTOR_3_BANK1) )
	{
		return 3;
	}
  else if( (Address < ADDR_FLASH_SECTOR_5_BANK1)&&(Address >= ADDR_FLASH_SECTOR_4_BANK1) )
	{
		return 4;
	}	
  else if( (Address < ADDR_FLASH_SECTOR_6_BANK1)&&(Address >= ADDR_FLASH_SECTOR_5_BANK1) )
	{
		return 5;
	}	
  else if( (Address < ADDR_FLASH_SECTOR_7_BANK1)&&(Address >= ADDR_FLASH_SECTOR_6_BANK1) )
	{
		return 6;
	}	
  else if( (Address < ADDR_FLASH_SECTOR_0_BANK2)&&(Address >= ADDR_FLASH_SECTOR_7_BANK1) )
	{
		return 7;
	}	
  if ( (Address < ADDR_FLASH_SECTOR_1_BANK2)&&(Address >= ADDR_FLASH_SECTOR_0_BANK2) )
  {
    return 8;
  }
  else if( (Address < ADDR_FLASH_SECTOR_2_BANK2)&&(Address >= ADDR_FLASH_SECTOR_1_BANK2) )
	{
		return 9;
	}
  else if( (Address < ADDR_FLASH_SECTOR_3_BANK2)&&(Address >= ADDR_FLASH_SECTOR_2_BANK2) )
	{
		return 10;
	}
  else if( (Address < ADDR_FLASH_SECTOR_4_BANK2)&&(Address >= ADDR_FLASH_SECTOR_3_BANK2) )
	{
		return 11;
	}
  else if( (Address < ADDR_FLASH_SECTOR_5_BANK2)&&(Address >= ADDR_FLASH_SECTOR_4_BANK2) )
	{
		return 12;
	}	
  else if( (Address < ADDR_FLASH_SECTOR_6_BANK2)&&(Address >= ADDR_FLASH_SECTOR_5_BANK2) )
	{
		return 13;
	}	
  else if( (Address < ADDR_FLASH_SECTOR_7_BANK2)&&(Address >= ADDR_FLASH_SECTOR_6_BANK2) )
	{
		return 14;
	}	
  else if( (Address < 0x08200000)&&(Address >= ADDR_FLASH_SECTOR_7_BANK2) )
	{
		return 15;
	}
	else
		return 100;
}	
	



//����Sectors��������
static uint16_t Flash_If_Erase(uint32_t Add)
{
	static uint8_t error = 0;
  uint32_t startsector = 0, sectorerror = 0;
  /* Variable contains Flash operation status */
  HAL_StatusTypeDef status;
  FLASH_EraseInitTypeDef eraseinitstruct;
  
	if ( Add > ADDR_FLASH_SECTOR_1_BANK1 )
  {
		error = 0xff;
    return 1;
  }
  /* Get the number of sector */
  startsector = GetSector(Add);
  eraseinitstruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  eraseinitstruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  eraseinitstruct.Banks = LOCATE_BANK(Add);
  eraseinitstruct.Sector = startsector;
  eraseinitstruct.NbSectors = 1;

  status = HAL_FLASHEx_Erase(&eraseinitstruct, &sectorerror);

  if (status != HAL_OK)
  {
    return 1;
  }
  return 0;
}


//Flashд�뺯��
static uint16_t Flash_If_Write(uint8_t * src, uint8_t * dest, uint32_t Len)
{
  uint32_t i = 0;
  for (i = 0; i < Len; i += 32)
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
     * be done by byte */
    if (HAL_FLASH_Program
        (FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t) (dest + i),
         (uint32_t) (src + i)) == HAL_OK)
    {
      /* Check the written value */
      if (*(uint64_t *) (src + i) != *(uint64_t *) (dest + i))
      {
        /* Flash content doesn't match SRAM content */
        return 2;
      }
    }
    else
    {
      /* Error occurred while writing data in Flash memory */
      return 1;
    }
  }
  return 0;
}

//Flash��ȡ����
static uint8_t *Flash_If_Read(uint8_t * src, uint8_t * dest, uint32_t Len)
{
  uint32_t i = 0;
  uint8_t *psrc = src;

  for (i = 0; i < Len; i++)
  {
    dest[i] = *psrc++;
  }
  /* Return a valid address to avoid HardFault */
  return (uint8_t *) (dest);
}

//SD������BL
static bool SD_BL_Update()
{
	while(1)
	{	
		set_BuzzerOnOff(0);
reload_SD:
		//�ȴ�SD�����뿨��
		if( BSP_SD_IsDetected() == false ){
			if( BSP_SD_IsDetected() == false ){
				//ʶ�𲻵�SD��
				Clear_SD_Init_Complete();
				return false;
			}
		}
		if(Lock_SD(-1))
		{	
			if(!Get_SD_Init_Complete())
			{			
				if( BSP_SD_Init() != MSD_OK )
				{	
					UnLock_SD();
					goto reload_SD;
				}
				SD_Driver.disk_initialize(0);	
				Set_SD_Init_Complete();
			}
			UnLock_SD();
		}

		if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 1) == FR_OK)
		{//SD�����سɹ�		
			//�ļ�ָ��
			static DIR Directory;
			static FIL SDFile;
			//�ļ�����״̬
			FRESULT fres;
			//�ļ����У�״̬
			FILINFO finfo;
			//�ļ����У���
			TCHAR filename[50];

			//�Ƿ���BootLoaderUpdateĿ¼
			filename[0] = 0;
			strcat( filename, SDPath );
			strcat( filename,  "ACFly/BootLoaderUpdate" );
			fres = f_stat( filename, &finfo );
			if( fres == FR_OK )
			{
				//��BootLoader_UpdateĿ¼
				fres = f_opendir(&Directory,filename);
				if(fres != FR_OK){
					f_unmount(SDPath);				
					return false;
				}
				//����hex��׺�ļ�			
				fres = f_findfirst(&Directory, &finfo, filename, "*.hex"); 				
				if(fres == FR_OK && finfo.fname[0])
				{//hex��׺�ļ�����					
					strcat( filename, "/" );
					strcat( filename, finfo.fname );
ReTry:
					if(!f_open( &SDFile, filename, FA_OPEN_EXISTING | FA_READ ))
					{//�̼��ļ��򿪳ɹ�				
						uint8_t res;								
						bool erasing_flag[16];
						uint8_t sector = 100;				
						uint64_t LED_Counter1 = 0;
						static int8_t byte_to_write = 0;	
						static bool Extended_Linear_Address_Record = false;						
						__attribute__ ((aligned (32))) _HexData hexdata;
						__attribute__ ((aligned (32))) uint32_t byte_read=0;	
						__attribute__ ((aligned (32))) uint8_t data_write[32];						
						__attribute__ ((aligned (32))) uint32_t last_aligned_addr=0;																	
						Static_AXIDMABuf __attribute__ ((aligned (32))) char data_read[1024];					
						memset(erasing_flag,0,sizeof(erasing_flag));		
						memset(hexdata.data,0,sizeof(hexdata.data));		
						HAL_FLASH_Lock();						
						while(1)
						{		              					
							res = f_read( &SDFile, data_read, sizeof(data_read), &byte_read );
							if( byte_read > 0 && !res )
							{
								for( uint32_t i=0; i < byte_read; i++ )
								{//�����̼��ļ�
									setLedMode(LEDMode_Manual);	
									set_BuzzerOnOff(0);											
									res = ParseHex( data_read[i], &hexdata );	
									if( res==0 && hexdata.datalen != 0 )
									{//Data Rrecord							
										sector = GetSector( hexdata.address );
										if( erasing_flag[sector] == false && sector < 2 && sector >= 0 )
										{//��ʼ������Ӧsector
											while(HAL_FLASH_Unlock()!=HAL_OK);	
											if( Flash_If_Erase(hexdata.address)!=0 )
											{//����sectorʧ��																	
												HAL_FLASH_Lock();							
												f_close(&SDFile);
												f_closedir(&Directory);
												f_unmount(SDPath);									
												return false;
											}
											else
											{	//����sector�ɹ�								
												HAL_FLASH_Lock();
												erasing_flag[sector] = true;	
											}																									
										}
										else if( sector > 2  )
										{//����sector�������,ɾ���˹̼�
											HAL_FLASH_Lock();
											f_close(&SDFile);
											f_unlink(filename);
											f_sync(&SDFile);
											f_closedir(&Directory);
											f_unmount(SDPath);	
											return false;
										}
																		
										if( byte_to_write>=32 && (hexdata.address%32==0) )
										{//ÿ������32�ֽڶ���ĵ�ַ,�ͽ�����һ��д���ϴ�32�ֽڶ���ĵ�ַ
											while(HAL_FLASH_Unlock()!=HAL_OK)vTaskDelay(1);
											uint32_t i=0;
											while(byte_to_write>0){															
												if(Flash_If_Write( &data_write[i], (uint8_t*)last_aligned_addr, 1)!=0)
												{//д��ʧ��,�����û�BL���븴λ�����洢��ַ����sector	
													HAL_FLASH_Unlock();	
													hexdata.address = 0x080000000;
													Flash_If_Erase(hexdata.address);
													HAL_FLASH_Lock();
													f_close(&SDFile);
													f_unlink(filename);
													f_sync(&SDFile);
													f_closedir(&Directory);
													f_unmount(SDPath);													
													return false;							
												}
												else
												{//д��ɹ�
													i+=32;
												}											
												byte_to_write -= 32;
											}
											if( byte_to_write < 0 )
												byte_to_write = 0;
											HAL_FLASH_Lock();
											last_aligned_addr = hexdata.address;
										}
										else if( Extended_Linear_Address_Record == true && (hexdata.address%32 == 0) )
										{//����ַ
											last_aligned_addr = hexdata.address;
										}
										Extended_Linear_Address_Record = false;	
										memcpy(&data_write[byte_to_write],hexdata.data,hexdata.datalen);
										byte_to_write += hexdata.datalen;																						
									}
									else if(res==1)
									{//End of File Record	
										if( byte_to_write > 0 )
										{
											uint32_t i=0;
											HAL_FLASH_Unlock();										
											while( byte_to_write > 0 ){								
												if( Flash_If_Write( &data_write[i], (uint8_t*)last_aligned_addr, 1)!=0 )
												{//д��ʧ��
													HAL_FLASH_Unlock();	
													hexdata.address = 0x080000000;
													Flash_If_Erase(hexdata.address);
													HAL_FLASH_Lock();
													f_close(&SDFile);
													f_unlink(filename);
													f_sync(&SDFile);
													f_closedir(&Directory);
													f_unmount(SDPath);	
													return false;										
												}
												else
												{//д��ɹ�
													i+=32;
													byte_to_write -= 32;
												}																						
											}
											if( byte_to_write < 0 )
												byte_to_write = 0;
											HAL_FLASH_Lock();						  
										}	
										//��¼���									
										f_close(&SDFile);												
										f_unlink(filename);
										f_sync(&SDFile);
										f_closedir(&Directory);
										f_unmount(SDPath);	
										FATFS_UnLinkDriver(SDPath);		
										//ϵͳ��λ									
										NVIC_SystemReset();																										
									}
									else if(res==4)
									{//Extended Linear Address Record
										if( byte_to_write > 0)
										{
											uint32_t i=0;
											HAL_FLASH_Unlock();										
											while(byte_to_write>0){								
												if( Flash_If_Write( &data_write[i], (uint8_t*)last_aligned_addr, 1) != 0 )
												{//д��ʧ��
													HAL_FLASH_Unlock();	
													hexdata.address = 0x080000000;
													Flash_If_Erase(hexdata.address);
													HAL_FLASH_Lock();
													f_close(&SDFile);
													f_unlink(filename);
													f_sync(&SDFile);
													f_closedir(&Directory);
													f_unmount(SDPath);													
													return false;									
												}
												else
												{//д��ɹ�
													i+=32;
													byte_to_write -= 32;
												}																						
											}
											if( byte_to_write < 0 )
												byte_to_write = 0;
											HAL_FLASH_Lock();						  
										}
										Extended_Linear_Address_Record = true;
									}
									else if(res==100)
									{//��������
										if( !erasing_flag[0] ){
											HAL_FLASH_Lock();									
											f_close(&SDFile);
											f_unlink(filename);
											f_sync(&SDFile);
											f_closedir(&Directory);
											f_unmount(SDPath);								
											return false;		
										}else{
											memset(erasing_flag,0,sizeof(erasing_flag));	
											HAL_FLASH_Lock();												
											f_close(&SDFile);
											f_sync(&SDFile);
											goto ReTry;										
										}											
									}																			
									//���ƿ���
									if( LED_Counter1 < 25000 ){
										LED_Counter1++;
										set_LedBrightness(0,0,100);
									}else{
										LED_Counter1++;
										set_LedBrightness(0,0,0);
										if(LED_Counter1 >50000){
											LED_Counter1 = 0;											
										}
									}								
								}         							
							}
							else
							{//�ļ���ȡ���ݴ�СΪ0���߶�ȡ����,��Ӧ�õ���		
								f_close(&SDFile);	
								f_unlink(filename);		
								f_sync(&SDFile);
								f_closedir(&Directory);
								f_unmount(SDPath);									
								return false;							
							}												
						}													
					}
					else
					{//�޷��򿪹̼��ļ�
						f_unlink(filename);		
						f_sync(&SDFile);
						f_closedir(&Directory);
						f_unmount(SDPath);
						return false;				
					}	
				}	
				else
				{//�޹̼�����
					f_closedir(&Directory);
					f_unmount(SDPath);				
					return false;	
				}				 
			}
			else
			{//������BootLoaderUpdateĿ¼
				f_unmount(SDPath);
				return false;
			}				
		}
		else
			return false;
		
	}
}


//bootloader��ʼ��
void init_drv_BootLoader()
{
	SD_BL_Update(); 
}

