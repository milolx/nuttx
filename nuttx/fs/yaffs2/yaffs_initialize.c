#include <nuttx/config.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/nand_model.h>
#include <nuttx/mtd/nand_raw.h>
#include <errno.h>

#include <pthread.h>

#include "yaffs_guts.h"
#include "yaffsfs.h"
#include "yaffs_trace.h"
#include "yaffs_mtdif.h"

static pthread_mutex_t mutex1;
static pthread_t bc_gc_thread;

static int yaffsfs_lastError;

void yaffsfs_SetError(int err)
{
	//Do whatever to set error
	yaffsfs_lastError = err;
	errno = err;
}

int yaffsfs_GetLastError(void)
{
	return yaffsfs_lastError;
}

int yaffsfs_CheckMemRegion(const void *addr, size_t size, int write_request)
{               
	if(!addr)
		return -1;
	return 0;
}

void yaffsfs_Lock(void)
{
	pthread_mutex_lock( &mutex1 );
}

void yaffsfs_Unlock(void)
{
	pthread_mutex_unlock( &mutex1 );
}

static void *bg_gc_func(void *dummy)
{
	struct yaffs_dev *dev;
	int urgent = 0;
	int result;
	int next_urgent;

	/* Sleep for a bit to allow start up */
	sleep(2);


	while (1) {
		/* Iterate through devices, do bg gc updating ungency */
		yaffs_dev_rewind();
		next_urgent = 0;

		while ((dev = yaffs_next_dev()) != NULL) {
			result = yaffs_do_background_gc_reldev(dev, urgent);
			if (result > 0)
				next_urgent = 1;
		}

		urgent = next_urgent;

		if (next_urgent)
			sleep(1);
		else
			sleep(5);
	}

	/* Don't ever return. */
	return NULL;
}

/*
 * yaffsfs_CurrentTime() retrns a 32-bit timestamp.
 *
 * Can return 0 if your system does not care about time.
 */
u32 yaffsfs_CurrentTime(void)
{
	return time(NULL);
}


/*
 * yaffsfs_malloc()
 * yaffsfs_free()
 *
 * Functions to allocate and free memory.
 */
void *yaffsfs_malloc(size_t size)
{
	return malloc(size);
}

void yaffsfs_free(void *ptr)
{
	free(ptr);
}

/*
 * yaffs_bug_fn()
 * Function to report a bug.
 */
void yaffs_bug_fn(const char *file_name, int line_no)
{
	printf("yaffs bug detected %s:%d\n",
		file_name, line_no);
	assert(0);
}

static char *dev_name = "nand";

int yaffs2_initialize(FAR struct mtd_dev_s *mtd)
{
	struct yaffs_dev *dev;
	char *name_copy = NULL;
	struct yaffs_param *param;

	/* Initialise lock */
	pthread_mutex_init(&mutex1, NULL);

	/* Sneak in starting a background gc thread too */
	pthread_create(&bc_gc_thread, NULL, bg_gc_func, NULL);


	dev = (struct yaffs_dev *)malloc(sizeof(struct yaffs_dev));

	if(!dev)
		goto fail;

	memset(dev, 0, sizeof(*dev));

	param = &dev->param;

	param->name = dev_name;

	param->total_bytes_per_chunk = nandmodel_getpagesize(&mtd->raw->model);
	param->chunks_per_block = nandmodel_pagesperblock(&mtd->raw->model);
	param->n_reserved_blocks = 5;
	param->start_block = 0;		// First block
	param->end_block = nandmodel_getdevblocks(&mtd->raw->model) - 1;
					// Last block
	param->is_yaffs2 = 1;
	param->use_nand_ecc = 1;
	param->n_caches = 10;

	yaffs_mtd_drv_install(dev, mtd);

	/* The yaffs device has been configured, install it into yaffs */
	yaffs_add_device(dev);

	return OK;

fail:
	free(dev);
	free(name_copy);
	return ERROR;
}
