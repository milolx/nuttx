#include <nuttx/config.h>

#include <stdbool.h>
#include <debug.h>

#include "lpc17_lcd.h"
#include "lpc17_gpio.h"

#include "wefly1788.h"

#ifdef CONFIG_LPC17_LCD

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
 * Name: wefly1788_lcd_initialize
 *
 * Description:
 *   Initialize the LCD.  Setup backlight (initially off)
 *
 ************************************************************************************/

void wefly1788_lcd_initialize(void)
{
  /* Configure the LCD backlight (and turn the backlight off) */

  lpc17_configgpio(GPIO_LCD_BL);
}

/************************************************************************************
 * Name: lpc17_backlight
 *
 * Description:
 *   If CONFIG_LPC17_LCD_BACKLIGHT is defined, then the board-specific logic must
 *   provide this interface to turn the backlight on and off.
 *
 ************************************************************************************/

#ifdef CONFIG_LPC17_LCD_BACKLIGHT
void lpc17_backlight(bool blon)
{
  lpc17_gpiowrite(GPIO_LCD_BL, blon);
}
#endif

#endif /* CONFIG_LPC17_LCD */
