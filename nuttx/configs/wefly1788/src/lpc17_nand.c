#include <nuttx/config.h>
#include <nuttx/mtd/nand_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/nand.h>
#include <nuttx/mtd/nand_raw.h>
#include <nuttx/mtd/nand_model.h>
#include <nuttx/fs/nxffs.h>
#include <sys/mount.h>

#include <nuttx/mtd/mtd.h>

#include <lpc17_gpio.h>

#include "up_arch.h"
#include "up_internal.h"

#include "lpc17_nand.h"
#include "wefly1788.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void     nand_wait_ready(struct nand_raw_s *raw);

/* MTD driver methods */

static int      nand_eraseblock(struct nand_raw_s *raw, off_t block);
static int      nand_rawread(struct nand_raw_s *raw, off_t block,
                  unsigned int page, void *data, void *spare);
static int      nand_rawwrite(struct nand_raw_s *raw, off_t block,
                  unsigned int page, const void *data, const void *spare);

/* Initialization */

static void     nand_reset(struct nand_raw_s *raw);


static void nand_wait_ready(struct nand_raw_s *raw)
{
	while (!lpc17_gpioread(GPIO_NAND_RB));
}

static int nand_rawread(struct nand_raw_s *raw, off_t block,
                        unsigned int page, void *data, void *spare)
{
	uint16_t pagesize;
	uint16_t sparesize;
	off_t rowaddr;
	off_t coladdr;

	fvdbg("block=%d page=%d data=%p spare=%p\n", (int)block, page, data, spare);
	DEBUGASSERT(raw && (data || spare));

	/* Get page and spare sizes */

	pagesize  = nandmodel_getpagesize(&raw->model);
	sparesize = nandmodel_getsparesize(&raw->model);

	/* If data is requested, the start addr is 0.
	 * If there is no data requested, the start addr is pagesize.
	 */
	rowaddr = block * nandmodel_pagesperblock(&raw->model) + page;
	coladdr = data ? 0 : pagesize;

	WRITE_COMMAND8(raw, COMMAND_READ_1);
	WRITE_ADDRESS8(raw, coladdr & 0x00ff);		/* column address low */
	WRITE_ADDRESS8(raw, (coladdr & 0xff00) >> 8);	/* column address high */
	WRITE_ADDRESS8(raw, rowaddr & 0x00ff);		/* row address low */
	WRITE_ADDRESS8(raw, (rowaddr & 0xff00) >> 8);	/* row address high */
	WRITE_COMMAND8(raw, COMMAND_READ_2);

	nand_wait_ready(raw);

	/* Read data area if requested */
	if (data)
	{
		uint8_t *p = (uint8_t *)data;
		while (pagesize--)
			*(p++) = READ_DATA8(raw);
	}

	/* Read the spare area if requested. */
	if (spare)
	{
		uint8_t *p = (uint8_t *)spare;
		while (sparesize--)
			*(p++) = READ_DATA8(raw);
	}

	return OK;
}

static int nand_eraseblock(struct nand_raw_s *raw, off_t block)
{
	uint32_t rowaddr;

	DEBUGASSERT(raw);
	fvdbg("block=%d\n", (int)block);

	rowaddr = block * nandmodel_pagesperblock(&raw->model);

	WRITE_COMMAND8(raw, COMMAND_ERASE_1);
	WRITE_ADDRESS8(raw, rowaddr & 0x00ff);		/* column address low */
	WRITE_ADDRESS8(raw, (rowaddr & 0xff00) >> 8);	/* column address high */
	WRITE_COMMAND8(raw, COMMAND_ERASE_2);

	nand_wait_ready(raw);

	return OK;
}

static int nand_rawwrite(struct nand_raw_s *raw, off_t block,
                         unsigned int page, const void *data,
                         const void *spare)
{
	uint16_t pagesize;
	uint16_t sparesize;
	off_t rowaddr;
	off_t coladdr;

	fvdbg("block=%d page=%d data=%p spare=%p\n", (int)block, page, data, spare);
	DEBUGASSERT(raw && (data || spare));

	/* Get page and spare sizes */

	pagesize  = nandmodel_getpagesize(&raw->model);
	sparesize = nandmodel_getsparesize(&raw->model);

	/* If data is requested, the start addr is 0.
	 * If there is no data requested, the start addr is pagesize.
	 */
	rowaddr = block * nandmodel_pagesperblock(&raw->model) + page;
	coladdr = data ? 0 : pagesize;

	WRITE_COMMAND8(raw, COMMAND_WRITE_1);
	WRITE_ADDRESS8(raw, coladdr & 0x00ff);		/* column address low */
	WRITE_ADDRESS8(raw, (coladdr & 0xff00) >> 8);	/* column address high */
	WRITE_ADDRESS8(raw, rowaddr & 0x00ff);		/* row address low */
	WRITE_ADDRESS8(raw, (rowaddr & 0xff00) >> 8);	/* row address high */

	/* Write data area if requested */
	if (data)
	{
		uint8_t *p = (uint8_t *)data;
		while (pagesize--)
			WRITE_DATA8(raw, *(p++));
	}

	/* Write the spare area if requested. */
	if (spare)
	{
		uint8_t *p = (uint8_t *)spare;
		while (sparesize--)
			WRITE_DATA8(raw, *(p++));
	}

	WRITE_COMMAND8(raw, COMMAND_WRITE_2);

	nand_wait_ready(raw);

	return OK;
}

static void nand_reset(struct nand_raw_s *raw)
{
	fvdbg("Resetting\n");

	WRITE_COMMAND8(raw, COMMAND_RESET);
	up_mdelay(2);
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct nand_raw_s nand_raw;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct mtd_dev_s *lpc17_nand_initialize(void)
{
	struct mtd_dev_s *mtd;

	/* Initialize the device structure */

	memset(&nand_raw, 0, sizeof nand_raw);
	nand_raw.cmdaddr    = NAND_CMDADDR;
	nand_raw.addraddr   = NAND_ADDRADDR;
	nand_raw.dataaddr   = NAND_DATAADDR;
	nand_raw.ecctype    = NAND_ECCMODE;
	nand_raw.eraseblock = nand_eraseblock;
	nand_raw.rawread    = nand_rawread;
	nand_raw.rawwrite   = nand_rawwrite;

	/* Reset the NAND FLASH part */
	nand_reset(&nand_raw);

	/* Probe the NAND part.  On success, an MTD interface that wraps
	 * our nand_raw NAND interface is returned.
	 */

	mtd = nand_initialize(&nand_raw);
	if (!mtd)
	{
		fdbg("ERROR: nand_initialize failed\n");
		return NULL;
	}

	return mtd;
}

int lpc17_nand_automount(void)
{
	FAR struct mtd_dev_s *mtd;
	static bool initialized = false;
	int ret;

	/* Have we already initialized? */
	if (!initialized)
	{
		/* Create and initialize an NAND MATD device */
		mtd = lpc17_nand_initialize();
		if (!mtd)
		{
			fdbg("ERROR: Failed to create the NAND driver\n");
			return -ENODEV;
		}

		/* Use the FTL layer to wrap the MTD driver as a block driver */
		ret = ftl_initialize(0, mtd);
		if (ret < 0)
		{
			fdbg("ERROR: Failed to initialize the FTL layer: %d\n", ret);
			return ret;
		}

		/* Initialize to provide NXFFS on the MTD interface */
		ret = nxffs_initialize(mtd);
		if (ret < 0)
		{
			fdbg("ERROR: NXFFS initialization failed: %d\n", ret);
			return ret;
		}

		/* Mount the file system at /mnt/nand */
		ret = mount(NULL, "/mnt/nand", "nxffs", 0, NULL);
		if (ret < 0)
		{
			fdbg("ERROR: Failed to mount the NXFFS volume: %d\n", errno);
			return ret;
		}

		/* Now we are initialized */
		initialized = true;
	}

	return OK;
}

