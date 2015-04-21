/*
 * YAFFS: Yet Another Flash File System. A NAND-flash specific file system.
 *
 * Copyright (C) 2002-2011 Aleph One Ltd.
 *   for Toby Churchill Ltd and Brightstar Engineering
 *
 * Created by Charles Manning <charles@aleph1.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <nuttx/config.h>

#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/nand.h>
#include <nuttx/mtd/nand_model.h>
#include <nuttx/mtd/nand_raw.h>
#include <nuttx/mtd/nand_scheme.h>
#include <nuttx/mtd/nand_ecc.h>
#include <semaphore.h>

#include "yportenv.h"

#include "yaffs_mtdif.h"

#include "yaffs_trace.h"
#include "yaffs_guts.h"

#define nand_lock(n) sem_wait(&(n)->exclsem)
#define nand_unlock(n) sem_post(&(n)->exclsem)

static 	int yaffs_mtd_write(struct yaffs_dev *dev, int nand_chunk,
				   const u8 *data, int data_len,
				   const u8 *oob, int oob_len)
{
	struct mtd_dev_s *mtd = yaffs_dev_to_mtd(dev);
	struct nand_dev_s *nand = (struct nand_dev_s *)mtd;
	struct nand_raw_s *raw = nand->raw;
	struct nand_model_s *model = &raw->model;

	static u8 pagedata[CONFIG_MTD_NAND_MAXPAGEDATASIZE];
	static u8 extra[CONFIG_MTD_NAND_MAXSPAREEXTRABYTES];
	unsigned int block;
	unsigned int pagesperblock;
	unsigned int page;
	int ret;

	yaffs_trace(YAFFS_TRACE_MTD,
			"yaffs_mtd_write(%p, %d, %p, %d, %p, %d)\n",
			dev, nand_chunk, data, data_len, oob, oob_len);

	if (!data || !data_len)
		data = NULL;
	else {
		memset(pagedata, 0xff, CONFIG_MTD_NAND_MAXPAGEDATASIZE);
		memcpy(pagedata, data, data_len);
		data = pagedata;
	}
	if (oob_len > model->scheme->nxbytes)
		return YAFFS_FAIL;
	else {
		memset(extra, 0xff, CONFIG_MTD_NAND_MAXSPAREEXTRABYTES);
		memcpy(extra, oob, oob_len);
		oob = extra;
	}

	/* Get the number of pages in one block, the size of one page, and
	 * the number of blocks on the device.
	 */
	pagesperblock = nandmodel_pagesperblock(model);

	/* Get the block and page offset associated with the startpage */
	block = nand_chunk / pagesperblock;
	page  = nand_chunk % pagesperblock;
	nand_lock(nand);
	ret = nandecc_writepage(nand, block, page, data, (void *)oob);
	nand_unlock(nand);
	if (ret < 0) {
		yaffs_trace(YAFFS_TRACE_MTD,
				"nandecc_writepage failed, chunk %d, mtd error %d",
				nand_chunk, ret);
		return YAFFS_FAIL;
	}

	return YAFFS_OK;
}

static int yaffs_mtd_read(struct yaffs_dev *dev, int nand_chunk,
				   u8 *data, int data_len,
				   u8 *oob, int oob_len,
				   enum yaffs_ecc_result *ecc_result)
{
	struct mtd_dev_s *mtd = yaffs_dev_to_mtd(dev);
	struct nand_dev_s *nand = (struct nand_dev_s *)mtd;
	struct nand_raw_s *raw = nand->raw;
	struct nand_model_s *model = &raw->model;

	static u8 pagedata[CONFIG_MTD_NAND_MAXPAGEDATASIZE];
	static u8 extra[CONFIG_MTD_NAND_MAXSPAREEXTRABYTES];
	u8 *datap;
	u8 *extrap;
	unsigned int block;
	unsigned int pagesperblock;
	unsigned int page;
	int ret;
	int result;

	yaffs_trace(YAFFS_TRACE_MTD,
			"yaffs_mtd_read(%p, %d, %p, %d, %p, %d)\n",
			dev, nand_chunk, data, data_len, oob, oob_len);

	if (!data || !data_len)
		datap = data = NULL;
	else
		datap = pagedata;
	if (!oob || !oob_len)
		extrap = oob = NULL;
	else if (oob_len > CONFIG_MTD_NAND_MAXSPAREEXTRABYTES)
		return YAFFS_FAIL;
	else
		extrap = extra;

	/* Get the number of pages in one block, the size of one page, and
	 * the number of blocks on the device.
	 */
	pagesperblock = nandmodel_pagesperblock(model);

	/* Get the block and page offset associated with the startpage */
	block = nand_chunk / pagesperblock;
	page  = nand_chunk % pagesperblock;
	nand_lock(nand);
	ret = nandecc_readpage(nand, block, page, datap, extrap);
	nand_unlock(nand);

	result = YAFFS_OK;
	if (!ret && ecc_result)
		*ecc_result = YAFFS_ECC_RESULT_NO_ERROR;
	else if (ret < 0) {
		dev->n_ecc_unfixed++;
		if(ecc_result)
			*ecc_result = YAFFS_ECC_RESULT_UNFIXED;
		yaffs_trace(YAFFS_TRACE_MTD,
				"nandecc_readpage failed, chunk %d, mtd error %d",
				nand_chunk, ret);
		result = YAFFS_FAIL;
	}
	else if (ret == 1) {
		if(ecc_result)
			*ecc_result = YAFFS_ECC_RESULT_FIXED;
		dev->n_ecc_fixed++;
	}

	if (result == YAFFS_OK) {
		if (data)
			memcpy(data, datap, data_len);
		if (oob)
			memcpy(oob, extrap, oob_len);
	}

	return result;
}

static 	int yaffs_mtd_erase(struct yaffs_dev *dev, int block_no)
{
	struct mtd_dev_s *mtd = yaffs_dev_to_mtd(dev);
	struct nand_dev_s *nand = (struct nand_dev_s *)mtd;
	struct nand_raw_s *raw = nand->raw;
	int ret;

	ret = NAND_ERASEBLOCK(raw, block_no);
	if (ret < 0) {
		fdbg("ERROR: Erase block %d failed: %d\n", block_no, ret);
		return YAFFS_FAIL;
	}

	return YAFFS_OK;
}

static int yaffs_mtd_mark_bad(struct yaffs_dev *dev, int block_no)
{
	struct mtd_dev_s *mtd = yaffs_dev_to_mtd(dev);
	struct nand_dev_s *nand = (struct nand_dev_s *)mtd;
	struct nand_raw_s *raw = nand->raw;
	struct nand_scheme_s *scheme = nandmodel_getscheme(&raw->model);

	static u8 spare[CONFIG_MTD_NAND_MAXPAGESPARESIZE];
	int ret;

	memset(spare, 0xff, CONFIG_MTD_NAND_MAXPAGESPARESIZE);
	nandscheme_writebadblockmarker(scheme, spare, 0xba);
	ret = NAND_WRITEPAGE(raw, block_no, 0, 0, spare); 
	if (ret < 0) 
	{
		dbg("ERROR: Failed to mark block %ld as BAD\n", (long)block_no);
		return YAFFS_FAIL;
	}
	return YAFFS_OK;
}

static int yaffs_mtd_check_bad(struct yaffs_dev *dev, int block_no)
{
	struct mtd_dev_s *mtd = yaffs_dev_to_mtd(dev);
	struct nand_dev_s *nand = (struct nand_dev_s *)mtd;
	struct nand_raw_s *raw = nand->raw;

	u8 spare[CONFIG_MTD_NAND_MAXPAGESPARESIZE];
	int ret;

	memset(spare, 0xff, CONFIG_MTD_NAND_MAXPAGESPARESIZE);
	ret = NAND_READPAGE(raw, block_no, 0, 0, spare); 
	if (ret < 0) 
	{
		dbg("ERROR: Failed to check block %ld\n", (long)block_no);
		return YAFFS_FAIL;
	}
	else if (yaffs_hweight8(spare[0]) + yaffs_hweight8(spare[1]) < 14)
		return YAFFS_FAIL;

	return YAFFS_OK;
}

static int yaffs_mtd_initialise(struct yaffs_dev *dev)
{
	return YAFFS_OK;
}

static int yaffs_mtd_deinitialise(struct yaffs_dev *dev)
{
	return YAFFS_OK;
}


void yaffs_mtd_drv_install(struct yaffs_dev *dev, struct mtd_dev_s *mtd)
{
	struct yaffs_driver *drv = &dev->drv;

	drv->drv_write_chunk_fn = yaffs_mtd_write;
	drv->drv_read_chunk_fn = yaffs_mtd_read;
	drv->drv_erase_fn = yaffs_mtd_erase;
	drv->drv_mark_bad_fn = yaffs_mtd_mark_bad;
	drv->drv_check_bad_fn = yaffs_mtd_check_bad;
	drv->drv_initialise_fn = yaffs_mtd_initialise;
	drv->drv_deinitialise_fn = yaffs_mtd_deinitialise;

	dev->driver_context = mtd;
}
