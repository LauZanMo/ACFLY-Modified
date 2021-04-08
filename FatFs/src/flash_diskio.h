#pragma once

#ifdef __cplusplus
	extern "C" {
#endif

#include "lfs.h"

// Erase a block. A block must be erased before being programmed.
// The state of an erased block is undefined. Negative error codes
// are propogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int Flash_erase(const struct lfs_config *c, lfs_block_t block);

// Read a region in a block. Negative error codes are propogated
// to the user.
int Flash_read(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, void *buff, lfs_size_t size);

// Program a region in a block. The block must have previously
// been erased. Negative error codes are propogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int Flash_write(const struct lfs_config *c, lfs_block_t block,
				lfs_off_t off, const void *buff, lfs_size_t size);

// Sync the state of the underlying block device. Negative error codes
// are propogated to the user.
int Flash_sync(const struct lfs_config *c);

#ifdef __cplusplus
	}
#endif