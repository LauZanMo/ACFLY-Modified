
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
	{	//������Ϊ��cache����
#endif
		//ֱ��dma�ͽ�������		
		if( FlashReadPages( (uint8_t*)buff, block, page_addr, pages, Flash_TIMEOUT ) == true )
		{
			res = 0;
		}	
#ifdef DCACHE_SIZE
	}
	else if( ((uint32_t)buff & 0x1f) == 0 )
	{	//������32�ֽڶ���
		//ֱ��dma�ͽ�������
		SCB_InvalidateDCache_by_Addr((uint32_t*)buff, size);
		if( FlashReadPages( (uint8_t*)buff, block, page_addr, pages, Flash_TIMEOUT ) == true )
		{
			res = 0;
		}
	}
	else
	{
		//��������32�ֽڶ���
		//��������dma���������û������ڸ��ƹ�ȥ
		if( FlashReadPages( (uint8_t*)scratch, block, page_addr, pages, Flash_TIMEOUT ) == true )
		{
			//�����ݴ����û��������Ƶ�buf
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
	{	//������Ϊ��cache����
		//ֱ����dma����
#endif
		if( FlashProgramPages( (uint8_t*)buff, block, page_addr, pages, Flash_TIMEOUT ) == true )
		{
			res = 0;
		}
	
#ifdef DCACHE_SIZE
	}
	else if( ((uint32_t)buff & 0x1f) == 0 )
	{	//������32�ֽڶ���
		//ֱ����dma����
		
		//��Cache����д���ڴ�
		SCB_CleanDCache_by_Addr((uint32_t*)buff, size);
		if( FlashProgramPages( (uint8_t*)buff, block, page_addr, pages, Flash_TIMEOUT ) == true )
		{
			res = 0;
		}
	}
	else
	{	//��������32�ֽڶ���
		//�������ȸ��Ƶ����û������ٷ���	
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