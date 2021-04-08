#include "StorageSystem.hpp"

#include "flash_diskio.h"
#include "FlashIO.h"
#include "drv_ADC.hpp"
#include "lfs.h"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

//是否已初始化
static bool InternalStorage_Initialized = false;
//内部存储是否第一次运行（开机时初始化）
static bool InternalStorage_FirstTime = false;

//设备访问互斥锁
static SemaphoreHandle_t InternalStorage_Semphr;
//文件
static char UserFile_filename[256];
static lfs_t FlashFs;
static lfs_file_t UserFile;

/*文件读写*/
	/*
		截断存储文件
		group_name：文件组别名称
		name：文件名称
		length：长度
		TIMEOUT：线程同步超时时间
	*/
	SS_RESULT InternalStorage_TruncateFile( const char* group_name, const char* name, uint32_t length, double TIMEOUT )
	{
		if( InternalStorage_Initialized == false )
			return SS_ERR;
		
		//获取互斥锁
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( InternalStorage_Semphr , TIMEOUT_Ticks ) )
		{
			//获取文件名
			UserFile_filename[0] = 0;
			strcat( UserFile_filename , group_name );
			//创建组别
			if( lfs_stat( &FlashFs, UserFile_filename, 0 ) < 0 )
				lfs_mkdir(&FlashFs, UserFile_filename);
			strcat( UserFile_filename , "/" );
			strcat( UserFile_filename , name );
			strcat( UserFile_filename , ".cfg" );
			
			int res = lfs_file_open( &FlashFs, &UserFile, UserFile_filename, LFS_O_CREAT | LFS_O_WRONLY );
			//打开文件
			if(res == LFS_ERR_OK) 
			{	//写入文件
				res = lfs_file_truncate( &FlashFs, &UserFile, length );
				lfs_file_close(&FlashFs, &UserFile);
			}
			
			//释放互斥锁
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
		写存储文件
		group_name：文件组别名称
		name：文件名称
		content：内容
		length：内容长度
		TIMEOUT：线程同步超时时间
	*/
	SS_RESULT InternalStorage_WriteFile( const char* group_name, const char* name, const void* content, uint32_t offset, uint32_t length, double TIMEOUT )
	{
		if( InternalStorage_Initialized == false )
			return SS_ERR;
		
		//获取互斥锁
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( InternalStorage_Semphr , TIMEOUT_Ticks ) )
		{
			//获取文件名
			UserFile_filename[0] = 0;
			strcat( UserFile_filename , group_name );
			//创建组别
			if( lfs_stat( &FlashFs, UserFile_filename, 0 ) < 0 )
				lfs_mkdir(&FlashFs, UserFile_filename);
			strcat( UserFile_filename , "/" );
			strcat( UserFile_filename , name );
			strcat( UserFile_filename , ".cfg" );
			
			int res = lfs_file_open( &FlashFs, &UserFile, UserFile_filename, LFS_O_CREAT | LFS_O_WRONLY );
			//打开文件
			if(res == LFS_ERR_OK) 
			{	//写入文件
				lfs_file_seek( &FlashFs, &UserFile, offset, LFS_SEEK_SET );
				lfs_ssize_t byteswritten = lfs_file_write( &FlashFs, &UserFile, content, length );
				if( byteswritten < length )
					res = -1;
				lfs_file_close(&FlashFs, &UserFile);
			}
			
			//释放互斥锁
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
		保存存储文件
		group_name：文件组别名称
		name：文件名称
		content：内容
		length：内容长度
		TIMEOUT：线程同步超时时间
	*/
	SS_RESULT InternalStorage_SaveFile( const char* group_name, const char* name, const void* content, uint32_t length, double TIMEOUT )
	{
		if( InternalStorage_Initialized == false )
			return SS_ERR;
		
		//获取互斥锁
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( InternalStorage_Semphr , TIMEOUT_Ticks ) )
		{
			//获取文件名
			UserFile_filename[0] = 0;
			strcat( UserFile_filename , group_name );
			//创建组别
			if( lfs_stat( &FlashFs, UserFile_filename, 0 ) < 0 )
				lfs_mkdir(&FlashFs, UserFile_filename);
			strcat( UserFile_filename , "/" );
			strcat( UserFile_filename , name );
			strcat( UserFile_filename , ".cfg" );
			
			int res = lfs_file_open( &FlashFs, &UserFile, UserFile_filename, LFS_O_CREAT | LFS_O_TRUNC | LFS_O_WRONLY );
			//打开文件
			if(res == LFS_ERR_OK) 
			{	//写入文件
				lfs_ssize_t byteswritten = lfs_file_write( &FlashFs, &UserFile, content, length );
				if( byteswritten < length )
					res = -1;
				lfs_file_close(&FlashFs, &UserFile);
			}
			
			//释放互斥锁
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
		读取存储文件大小
		group_name：文件组别名称
		name：文件名称
		size：读取的文件大小
		TIMEOUT：线程同步超时时间
	*/
	SS_RESULT InternalStorage_GetFileSize( const char* group_name, const char* name , uint32_t* size , double TIMEOUT )
	{
		if( InternalStorage_Initialized == false )
			return SS_ERR;
		
		//获取互斥锁
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( InternalStorage_Semphr , TIMEOUT_Ticks ) )
		{
			//获取文件名
			UserFile_filename[0] = 0;
			strcat( UserFile_filename , group_name );
			strcat( UserFile_filename , "/" );
			strcat( UserFile_filename , name );
			strcat( UserFile_filename , ".cfg" );
			
			lfs_info info;
			int res = lfs_stat( &FlashFs, UserFile_filename, &info );
			if( res == LFS_ERR_OK )
				*size = info.size;
			
			//释放互斥锁
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
		读取存储文件
		group_name：文件组别名称
		name：文件名称
		content：读取的内容
		length：读取的文件长度
		TIMEOUT：线程同步超时时间
	*/
	SS_RESULT InternalStorage_ReadFile( const char* group_name, const char* name, void* content, uint32_t* length, 
		uint32_t read_offset, int32_t read_length, double TIMEOUT )
	{
		if( InternalStorage_Initialized == false )
			return SS_ERR;
		
		//获取互斥锁
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( InternalStorage_Semphr , TIMEOUT_Ticks ) )
		{
			//获取文件名
			UserFile_filename[0] = 0;
			strcat( UserFile_filename , group_name );
			strcat( UserFile_filename , "/" );
			strcat( UserFile_filename , name );
			strcat( UserFile_filename , ".cfg" );
			
			int res = lfs_file_open( &FlashFs, &UserFile, UserFile_filename, LFS_O_RDONLY );
			//打开文件
			if(res == LFS_ERR_OK) 
			{	//读取文件
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
			
			//释放互斥锁
			xSemaphoreGive(InternalStorage_Semphr);
			if( res >= 0 )
				return SS_OK;
			else
				return SS_ERR;
		}
		else
			return SS_TimeOut;
	}
/*文件读写*/

static lfs_config cfg;
void init_InternalStorage()
{
	while( Get_VDDA_Voltage() < 3.0 )
	{	//电压过低不初始化flash防止误擦除
		os_delay(0.1);
	}
	
	int res;
	//挂载文件系统	
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
	{	//无文件系统先格式化
		res = lfs_format(&FlashFs, &cfg);
		res = lfs_mount(&FlashFs, &cfg);
	}
	
	//新建Config目录
	char name[50] = {0};
	strcat( name, "Config" );
	if( lfs_stat( &FlashFs, name, 0 ) < 0 )
		lfs_mkdir(&FlashFs, name);
	
	//新建Log目录
	name[0] = 0;
	strcat( name, "Log" );
	if( lfs_stat( &FlashFs, name, 0 ) < 0 )
		lfs_mkdir(&FlashFs, name);
	
	//更新是否已初始化状态
	if( res == LFS_ERR_OK )
	{
		InternalStorage_Semphr = xSemaphoreCreateMutex();
		InternalStorage_Initialized = true;
	}
}