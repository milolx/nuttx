#ifndef __YAFFS_H__
#define __YAFFS_H__


#include "yportenv.h"

void yaffsfs_Lock(void);
void yaffsfs_Unlock(void);

u32 yaffsfs_CurrentTime(void);

void yaffsfs_SetError(int err);

void *yaffsfs_malloc(size_t size);
void yaffsfs_free(void *ptr);

int yaffsfs_CheckMemRegion(const void *addr, size_t size, int write_request);

#endif

