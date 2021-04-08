#include "StorageSystem.hpp"

#include "flash_diskio.h"
#include "FlashIO.h"
#include "drv_ADC.hpp"
#include "lfs.h"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

//�Ƿ��ѳ�ʼ��
static bool InternalStorage_Initialized = false;
//�ڲ��洢�Ƿ��һ�����У�����ʱ��ʼ����
static bool InternalStorage_FirstTime = false;

//�豸���ʻ�����
static SemaphoreHandle_t InternalStorage_Semphr;
//�ļ�
static char UserFile_filename[256];
static lfs_t FlashFs;
static lfs_file_t UserFile;

/*�ļ���д*/
	/*
		�ضϴ洢�ļ�
		group_name���ļ��������
		name���ļ�����
		length������
		TIMEOUT���߳�ͬ����ʱʱ��
	*/
	SS_RESULT InternalStorage_TruncateFile( const char* group_name, const char* name, uint32_t length, double TIMEOUT )
	{
		if( InternalStorage_Initialized == false )
			return SS_ERR;
		
		//��ȡ������
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( InternalStorage_Semphr , TIMEOUT_Ticks ) )
		{
			//��ȡ�ļ���
			UserFile_filename[0] = 0;
			strcat( UserFile_filename , group_name );
			//�������
			if( lfs_stat( &FlashFs, UserFile_filename, 0 ) < 0 )
				lfs_mkdir(&FlashFs, UserFile_filename);
			strcat( UserFile_filename , "/" );
			strcat( UserFile_filename , name );
			strcat( UserFile_filename , ".cfg" );
			
			int res = lfs_file_open( &FlashFs, &UserFile, UserFile_filename, LFS_O_CREAT | LFS_O_WRONLY );
			//���ļ�
			if(res == LFS_ERR_OK) 
			{	//д���ļ�
				res = lfs_file_truncate( &FlashFs, &UserFile, length );
				lfs_file_close(&FlashFs, &UserFile);
			}
			
			//�ͷŻ�����
			xSemaphoreGive(InternalStorage_Semphr);
			if( res >= 0 )
				return SS_OK;
			else
				return SS_ERR;
		}
		else
			return SS_TimeOut;
	}

	/*
		д�洢�ļ�
		group_name���ļ��������
		name���ļ�����
		content������
		length�����ݳ���
		TIMEOUT���߳�ͬ����ʱʱ��
	*/
	SS_RESULT InternalStorage_WriteFile( const char* group_name, const char* name, const void* content, uint32_t offset, uint32_t length, double TIMEOUT )
	{
		if( InternalStorage_Initialized == false )
			return SS_ERR;
		
		//��ȡ������
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( InternalStorage_Semphr , TIMEOUT_Ticks ) )
		{
			//��ȡ�ļ���
			UserFile_filename[0] = 0;
			strcat( UserFile_filename , group_name );
			//�������
			if( lfs_stat( &FlashFs, UserFile_filename, 0 ) < 0 )
				lfs_mkdir(&FlashFs, UserFile_filename);
			strcat( UserFile_filename , "/" );
			strcat( UserFile_filename , name );
			strcat( UserFile_filename , ".cfg" );
			
			int res = lfs_file_open( &FlashFs, &UserFile, UserFile_filename, LFS_O_CREAT | LFS_O_WRONLY );
			//���ļ�
			if(res == LFS_ERR_OK) 
			{	//д���ļ�
				lfs_file_seek( &FlashFs, &UserFile, offset, LFS_SEEK_SET );
				lfs_ssize_t byteswritten = lfs_file_write( &FlashFs, &UserFile, content, length );
				if( byteswritten < length )
					res = -1;
				lfs_file_close(&FlashFs, &UserFile);
			}
			
			//�ͷŻ�����
			xSemaphoreGive(InternalStorage_Semphr);
			if( res >= 0 )
				return SS_OK;
			else
				return SS_ERR;
		}
		else
			return SS_TimeOut;
	}

	/*
		����洢�ļ�
		group_name���ļ��������
		name���ļ�����
		content������
		length�����ݳ���
		TIMEOUT���߳�ͬ����ʱʱ��
	*/
	SS_RESULT InternalStorage_SaveFile( const char* group_name, const char* name, const void* content, uint32_t length, double TIMEOUT )
	{
		if( InternalStorage_Initialized == false )
			return SS_ERR;
		
		//��ȡ������
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( InternalStorage_Semphr , TIMEOUT_Ticks ) )
		{
			//��ȡ�ļ���
			UserFile_filename[0] = 0;
			strcat( UserFile_filename , group_name );
			//�������
			if( lfs_stat( &FlashFs, UserFile_filename, 0 ) < 0 )
				lfs_mkdir(&FlashFs, UserFile_filename);
			strcat( UserFile_filename , "/" );
			strcat( UserFile_filename , name );
			strcat( UserFile_filename , ".cfg" );
			
			int res = lfs_file_open( &FlashFs, &UserFile, UserFile_filename, LFS_O_CREAT | LFS_O_TRUNC | LFS_O_WRONLY );
			//���ļ�
			if(res == LFS_ERR_OK) 
			{	//д���ļ�
				lfs_ssize_t byteswritten = lfs_file_write( &FlashFs, &UserFile, content, length );
				if( byteswritten < length )
					res = -1;
				lfs_file_close(&FlashFs, &UserFile);
			}
			
			//�ͷŻ�����
			xSemaphoreGive(InternalStorage_Semphr);
			if( res >= 0 )
				return SS_OK;
			else
				return SS_ERR;
		}
		else
			return SS_TimeOut;
	}
	
	/*
		��ȡ�洢�ļ���С
		group_name���ļ��������
		name���ļ�����
		size����ȡ���ļ���С
		TIMEOUT���߳�ͬ����ʱʱ��
	*/
	SS_RESULT InternalStorage_GetFileSize( const char* group_name, const char* name , uint32_t* size , double TIMEOUT )
	{
		if( InternalStorage_Initialized == false )
			return SS_ERR;
		
		//��ȡ������
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( InternalStorage_Semphr , TIMEOUT_Ticks ) )
		{
			//��ȡ�ļ���
			UserFile_filename[0] = 0;
			strcat( UserFile_filename , group_name );
			strcat( UserFile_filename , "/" );
			strcat( UserFile_filename , name );
			strcat( UserFile_filename , ".cfg" );
			
			lfs_info info;
			int res = lfs_stat( &FlashFs, UserFile_filename, &info );
			if( res == LFS_ERR_OK )
				*size = info.size;
			
			//�ͷŻ�����
			xSemaphoreGive(InternalStorage_Semphr);
			if( res == LFS_ERR_OK )
				return SS_OK;
			else
				return SS_ERR;
		}
		else
			return SS_TimeOut;
	}
	/*
		��ȡ�洢�ļ�
		group_name���ļ��������
		name���ļ�����
		content����ȡ������
		length����ȡ���ļ�����
		TIMEOUT���߳�ͬ����ʱʱ��
	*/
	SS_RESULT InternalStorage_ReadFile( const char* group_name, const char* name, void* content, uint32_t* length, 
		uint32_t read_offset, int32_t read_length, double TIMEOUT )
	{
		if( InternalStorage_Initialized == false )
			return SS_ERR;
		
		//��ȡ������
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( InternalStorage_Semphr , TIMEOUT_Ticks ) )
		{
			//��ȡ�ļ���
			UserFile_filename[0] = 0;
			strcat( UserFile_filename , group_name );
			strcat( UserFile_filename , "/" );
			strcat( UserFile_filename , name );
			strcat( UserFile_filename , ".cfg" );
			
			int res = lfs_file_open( &FlashFs, &UserFile, UserFile_filename, LFS_O_RDONLY );
			//���ļ�
			if(res == LFS_ERR_OK) 
			{	//��ȡ�ļ�
				uint32_t file_size = lfs_file_size(&FlashFs, &UserFile);
				if( read_length < 0 )
					read_length = file_size - read_offset;
				else if( read_length + read_offset > file_size )
					res = -1;
				
				if( res >= 0 )
				{
					lfs_file_seek( &FlashFs, &UserFile, read_offset, LFS_SEEK_SET );
					lfs_ssize_t bytesread = lfs_file_read( &FlashFs, &UserFile, content, read_length );
					if( bytesread < read_length )
						res = -1;
					*length = bytesread;
				}
				lfs_file_close(&FlashFs, &UserFile);
			}
			
			//�ͷŻ�����
			xSemaphoreGive(InternalStorage_Semphr);
			if( res >= 0 )
				return SS_OK;
			else
				return SS_ERR;
		}
		else
			return SS_TimeOut;
	}
/*�ļ���д*/

static lfs_config cfg;
void init_InternalStorage()
{
	while( Get_VDDA_Voltage() < 3.0 )
	{	//��ѹ���Ͳ���ʼ��flash��ֹ�����
		os_delay(0.1);
	}
	
	int res;
	//�����ļ�ϵͳ	
	cfg = {
			// block device operations
			.read  = Flash_read,
			.prog  = Flash_write,
			.erase = Flash_erase,
			.sync  = Flash_sync,

			// block device configuration
			.read_size = getFlashPageSize(),
			.prog_size = getFlashPageSize(),
			.block_size = getFlashSectorSize(),
			.block_count = getFlashSectorCount(),
			.block_cycles = 1000,
			.cache_size = getFlashSectorSize(),
			.lookahead_size = 32,
	};
	res = lfs_mount(&FlashFs, &cfg);
	if( res )
	{	//���ļ�ϵͳ�ȸ�ʽ��
		res = lfs_format(&FlashFs, &cfg);
		res = lfs_mount(&FlashFs, &cfg);
	}
	
	//�½�ConfigĿ¼
	char name[50] = {0};
	strcat( name, "Config" );
	if( lfs_stat( &FlashFs, name, 0 ) < 0 )
		lfs_mkdir(&FlashFs, name);
	
	//�½�LogĿ¼
	name[0] = 0;
	strcat( name, "Log" );
	if( lfs_stat( &FlashFs, name, 0 ) < 0 )
		lfs_mkdir(&FlashFs, name);
	
	//�����Ƿ��ѳ�ʼ��״̬
	if( res == LFS_ERR_OK )
	{
		InternalStorage_Semphr = xSemaphoreCreateMutex();
		InternalStorage_Initialized = true;
	}
}