#ifndef __ARCH_ARM_SRC_SAMA5_SAM_NAND_H
#define __ARCH_ARM_SRC_SAMA5_SAM_NAND_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mtd/nand_config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <debug.h>

#include <nuttx/mtd/nand_raw.h>

#include "up_arch.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define NAND_CMDADDR	0x92000000	// A25
#define NAND_ADDRADDR	0x91000000	// A24
#define NAND_DATAADDR	0x90000000	// both A24 & A25 eq 0
#define NAND_ECCMODE	NANDECC_SWECC

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct mtd_dev_s *lpc17xx_nand_initialize(void);
int lpc17_nand_automount(void);

#endif /* __ARCH_ARM_SRC_SAMA5_SAM_NAND_H */
