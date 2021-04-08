#pragma once

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*Flash读写接口
	buf:读或写缓冲区
	sector_addr:起始扇区地址（1代表第二个扇区）
	sectors:读或写的扇区数量
	waitT:超时时间（秒）

	length:要读的字节数
	addr:读起始地址（1代表从Flash第二个字节开始读）
*/
	//读写扇区
	bool FlashWriteSectors( const uint8_t* buf , uint32_t sector_addr , uint16_t sectors , double waitT );
	bool FlashReadSectors( uint8_t* buf , uint32_t sector_addr , uint16_t sectors , double waitT );

	//随机读
	bool FlashRead( uint8_t* buf , uint16_t length , uint32_t addr , double waitT );
	
	//擦除扇区
	bool FlashEraseSectors( uint32_t sector_addr , uint16_t sectors , double waitT );
	
	/*Flash读写Page
		buf:读或写缓冲区
		sector_addr:起始扇区地址（1代表第二个扇区）
		page_addr:起始page地址（1代表此扇区第二个page）
		pages:读或写的page数量
		waitT:超时时间（秒）

		length:要读的字节数
		addr:读起始地址（1代表从Flash第二个字节开始读）
	*/
	bool FlashProgramPages( const uint8_t* buf , uint32_t sector_addr , uint16_t page_addr , uint16_t pages , double waitT );
	bool FlashReadPages( uint8_t* buf , uint32_t sector_addr , uint32_t page_addr , uint16_t pages , double waitT );
	
	//获取Flash扇区数目
	uint16_t getFlashSectorCount();
	//获取Flash扇区大小
	uint32_t getFlashSectorSize();
	//获取Flash page大小
	uint32_t getFlashPageSize();
/*Flash读写接口*/

#ifdef __cplusplus
	}
#endif