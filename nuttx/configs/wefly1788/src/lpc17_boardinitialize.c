/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "lpc17_emc.h"

#include "../include/board.h"
#include "wefly1788.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

int wefly1788_nand_initialize(void);

/************************************************************************************
 * Name: lpc17_boardinitialize
 *
 * Description:
 *   All LPC17xx architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void lpc17_boardinitialize(void)
{
  /* Initialize the EMC, SDRAM, NOR FLASH, and NAND FLASH */

#ifdef CONFIG_LPC17_EMC
  lpc17_emcinitialize();
#ifdef CONFIG_LPC17_EXTDRAM
  wefly1788_sdram_initialize();
#endif
#ifdef CONFIG_LPC17_EXTNAND
  wefly1788_nand_initialize();
#endif
#endif

  /* Configure the LCD GPIOs if LCD support has been selected. */

#ifdef CONFIG_LPC17_LCD
  wefly1788_lcd_initialize();
#endif
}

/****************************************************************************
 * Name: board_initialize
 *
 * Description:
 *   If CONFIG_BOARD_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_initialize().  board_initialize() will be
 *   called immediately after up_intiialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_INITIALIZE
void board_initialize(void)
{
  /* Perform NSH initialization here instead of from the NSH.  This
   * alternative NSH initialization is necessary when NSH is ran in user-space
   * but the initialization function must run in kernel space.
   */

#if defined(CONFIG_NSH_LIBRARY) && !defined(CONFIG_NSH_ARCHINIT)
  (void)nsh_archinitialize();
#endif
}
#endif
