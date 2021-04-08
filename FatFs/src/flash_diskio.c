
/* Includes ------------------------------------------------------------------*/
#include "lfs.h"
#include "flash_diskio.h"
#include "FlashIO.h"
#include <string.h>
#include "Basic.h"
#include "stm32h743xx.h"

#define Flash_TIMEOUT 1.0
#ifdef DCACHE_SIZE
 Static_AXIDMABuf uint8_t scratch[4096];
#endif

// Erase a block. A block must be erased before being programmed.
// The state of an erased block is undefined. Negative error codes
// are propogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int Flash_erase(const struct lfs_config *c, lfs_block_t block)
{
	if( FlashEraseSectors( block, 1, Flash_TIMEOUT ) )
		return 0;
	else
		return LFS_ERR_CORRUPT;
}

// Read a region in a block. Negative error codes are propogated
// to the user.
int Flash_read(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, void *buff, lfs_size_t size)
{
	uint16_t BLOCKSIZE = getFlashSectorSize();
	if( BLOCKSIZE==0 )
		return LFS_ERR_INVAL;
		
	uint16_t PAGESIZE = getFlashPageSize();
	if( off + size > BLOCKSIZE )
		return LFS_ERR_INVAL;
	uint16_t page_addr = off / PAGESIZE;
	uint16_t pages = size / PAGESIZE;
	
  int res = LFS_ERR_CORRUPT;
  uint32_t timer;
#ifdef DCACHE_SIZE
	if( isDMABuf( (uint32_t)buff ) )
	{	//缓冲区为非cache区域
#endif
		//直接dma送进缓冲区		
		if( FlashReadPages( (uint8_t*)buff, block, page_addr, pages, Flash_TIMEOUT ) == true )
		{
			res = 0;
		}	
#ifdef DCACHE_SIZE
	}
	else if( ((uint32_t)buff & 0x1f) == 0 )
	{	//缓冲区32字节对齐
		//直接dma送进缓冲区
		SCB_InvalidateDCache_by_Addr((uint32_t*)buff, size);
		if( FlashReadPages( (uint8_t*)buff, block, page_addr, pages, Flash_TIMEOUT ) == true )
		{
			res = 0;
		}
	}
	else
	{
		//缓冲区非32字节对齐
		//把数据用dma传送至内置缓冲区在复制过去
		if( FlashReadPages( (uint8_t*)scratch, block, page_addr, pages, Flash_TIMEOUT ) == true )
		{
			//把数据从内置缓冲区复制到buf
			memcpy(buff, scratch, size);
			res = 0;
		}
	}
#endif
  return res;
}

// Program a region in a block. The block must have previously
// been erased. Negative error codes are propogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int Flash_write(const struct lfs_config *c, lfs_block_t block,
				lfs_off_t off, const void *buff, lfs_size_t size)
{
	uint16_t BLOCKSIZE = getFlashSectorSize();
	if( BLOCKSIZE==0 )
		return LFS_ERR_INVAL;
		
	uint16_t PAGESIZE = getFlashPageSize();
	if( off + size > BLOCKSIZE )
		return LFS_ERR_INVAL;
	uint16_t page_addr = off / PAGESIZE;
	uint16_t pages = size / PAGESIZE;
	
  int res = LFS_ERR_CORRUPT;
  uint32_t timer;

#ifdef DCACHE_SIZE
	if( isDMABuf( (uint32_t)buff ) )
	{	//缓冲区为非cache区域
		//直接用dma发送
#endif
		if( FlashProgramPages( (uint8_t*)buff, block, page_addr, pages, Flash_TIMEOUT ) == true )
		{
			res = 0;
		}
	
#ifdef DCACHE_SIZE
	}
	else if( ((uint32_t)buff & 0x1f) == 0 )
	{	//缓冲区32字节对齐
		//直接用dma发送
		
		//把Cache内容写入内存
		SCB_CleanDCache_by_Addr((uint32_t*)buff, size);
		if( FlashProgramPages( (uint8_t*)buff, block, page_addr, pages, Flash_TIMEOUT ) == true )
		{
			res = 0;
		}
	}
	else
	{	//缓冲区非32字节对齐
		//把数据先复制到内置缓冲区再发送	
		int i;
    uint8_t ret;
		
		memcpy((void *)scratch, (void *)buff, size);
		if( FlashProgramPages( (uint8_t*)scratch, block, page_addr, pages, Flash_TIMEOUT ) == true )
		{
			res = 0;
		}
	}
#endif
  return res;
}

// Sync the state of the underlying block device. Negative error codes
// are propogated to the user.
int Flash_sync(const struct lfs_config *c)
{
	return 0;
}