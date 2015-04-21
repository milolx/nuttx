/*
 * YAFFS: Yet another Flash File System . A NAND-flash specific file system.
 *
 * Copyright (C) 2002-2011 Aleph One Ltd.
 *   for Toby Churchill Ltd and Brightstar Engineering
 *
 * Created by Charles Manning <charles@aleph1.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License version 2.1 as
 * published by the Free Software Foundation.
 *
 * Note: Only YAFFS headers are LGPL, YAFFS C code is covered by GPL.
 */

#ifndef __YAFFS_MTDIF_H__
#define __YAFFS_MTDIF_H__

#include <nuttx/mtd/mtd.h>
#include "yaffs_guts.h"

#define yaffs_dev_to_mtd(dev) ((struct mtd_dev_s *)((dev)->driver_context))

void yaffs_mtd_drv_install(struct yaffs_dev *dev, struct mtd_dev_s *mtd);
#endif
