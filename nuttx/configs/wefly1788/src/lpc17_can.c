#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/can.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"

#include "lpc17_can.h"

#if defined(CONFIG_CAN) && (defined(CONFIG_LPC17_CAN1) || defined(CONFIG_LPC17_CAN2))

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

/* Debug ***************************************************************************/
/* Non-standard debug that may be enabled just for testing CAN */

#ifdef CONFIG_DEBUG_CAN
#  define candbg    dbg
#  define canvdbg   vdbg
#  define canlldbg  lldbg
#  define canllvdbg llvdbg
#else
#  define candbg(x...)
#  define canvdbg(x...)
#  define canlldbg(x...)
#  define canllvdbg(x...)
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: can_devinit
 *
 * Description:
 *   All LPC17 architectures must provide the following interface to work with
 *   examples/can.
 *
 ************************************************************************************/

int can_devinit(void)
{
	static bool initialized = false;
	struct can_dev_s *can;
	int ret;

	/* Check if we have already initialized */

	if (!initialized)
	{
		/* Call lpc17_caninitialize() to get an instance of the CAN interface */

#ifdef CONFIG_LPC17_CAN1
		can = lpc17_caninitialize(1);
		if (can == NULL)
		{
			candbg("ERROR:  Failed to get CAN1 interface\n");
			return -ENODEV;
		}
#endif

		/* Register the CAN driver at "/dev/can0" */

		ret = can_register("/dev/can0", can);
		if (ret < 0)
		{
			candbg("ERROR: can_register failed(can0): %d\n", ret);
			return ret;
		}

#ifdef CONFIG_LPC17_CAN2
		can = lpc17_caninitialize(2);
		if (can == NULL)
		{
			candbg("ERROR:  Failed to get CAN2 interface\n");
			return -ENODEV;
		}
#endif

		/* Register the CAN driver at "/dev/can1" */

		ret = can_register("/dev/can1", can);
		if (ret < 0)
		{
			candbg("ERROR: can_register failed(can1): %d\n", ret);
			return ret;
		}

		/* Now we are initialized */

		initialized = true;
	}

	return OK;
}

#endif /* CONFIG_CAN && (CONFIG_LPC17_CAN1 || CONFIG_LPC17_CAN2) */
