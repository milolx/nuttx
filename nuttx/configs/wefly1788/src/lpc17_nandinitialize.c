#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <lpc17_gpio.h>

#include "up_arch.h"
#include "up_internal.h"
#include "chip/lpc17_syscon.h"
#include "lpc17_emc.h"

#include "wefly1788.h"

#if defined(CONFIG_LPC17_EMC) && defined(CONFIG_LPC17_EXTNAND)

/************************************************************************************
 * Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: wefly1788_nand_initialize
 *
 * Description:
 *   Initialize NAND FLASH
 *
 ************************************************************************************/

void wefly1788_nand_initialize(void)
{
  uint32_t regval;

  /* Set the memory width and byte lanes */

  regval = getreg32(LPC17_EMC_STATICCONFIG1);
  regval &= ~EMC_STATICCONFIG_MW_MASK;
  regval |= (EMC_STATICCONFIG_MW_8BIT | EMC_STATICCONFIG_PB);
  putreg32(regval, LPC17_EMC_STATICCONFIG1);

  /* Configure timing */

  putreg32(2, LPC17_EMC_STATICWAITWEN1);
  putreg32(2, LPC17_EMC_STATICWAITOEN1);
  putreg32(31, LPC17_EMC_STATICWAITRD1);
  putreg32(31, LPC17_EMC_STATICWAITPAGE1);
  putreg32(31, LPC17_EMC_STATICWAITWR1);
  putreg32(31, LPC17_EMC_STATICWAITTURN1);

  /* GPIO P2[21] connects to the Ready/Busy pin of the NAND part.  We need to
   * reconfigure this pin as normal GPIO input.
   */

  lpc17_configgpio(GPIO_NAND_RB);
}

#endif /* CONFIG_LPC17_EMC && CONFIG_LPC17_EXTNAND */
