#ifndef _CONFIGS_WEFLY1788_H
#define _CONFIGS_WEFLY1788_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <lpc17_gpio.h>
#include <arch/irq.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* wefly1788 GPIO Pin Definitions ****************************************************/
/* GPIO P2[21] connects to the Ready/Busy pin of the NAND part.  We need to
 * reconfigure this pin as normal GPIO input if NAND is used.
 */

#define GPIO_NAND_RB     (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN21)

/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 *
 * LED1 -- Connected to P1[14]
 * LED2 -- Connected to P0[16]
 * LED3 -- Connected to P1[13]
 * LED4 -- Connected to P4[27]
 *
 * These LEDs are connecte to ground so a high output value will illuminate them.
 */

//#define GPIO_LED1        (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN14)
//#define GPIO_LED2        (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN16)
//#define GPIO_LED3        (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN13)
//#define GPIO_LED4        (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT4 | GPIO_PIN27)

/* Button definitions ***************************************************************/
/* The wefly1788 supports several buttons.  All must be pulled up by the wefly1788.
 * When closed, the pins will be pulled to ground.  So the buttons will read "1"
 * when open and "0" when closed.  All except USER1 are capable of generating
 * interrupts.
 *
 * USER1           -- Connected to P4[26]
 * USER2           -- Connected to P2[22]
 * USER3           -- Connected to P0[10]
 *
 * And a Joystick
 *
 * JOY_A           -- Connected to P2[25]
 * JOY_B           -- Connected to P2[26]
 * JOY_C           -- Connected to P2[23]
 * JOY_D           -- Connected to P2[19]
 * JOY_CTR         -- Connected to P0[14] (shared with SSP1 SSEL)
 *
 * For the interrupting buttons, interrupts are generated on both edges (press and
 * release).
 */

//#define GPIO_USER1       (GPIO_INPUT   | GPIO_PULLUP | GPIO_PORT4 | GPIO_PIN26)
//#define GPIO_USER2       (GPIO_INTBOTH | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN22)
//#define GPIO_USER3       (GPIO_INTBOTH | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN10)

//#define GPIO_JOY_A       (GPIO_INTBOTH | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN25)
//#define GPIO_JOY_B       (GPIO_INTBOTH | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN26)
//#define GPIO_JOY_C       (GPIO_INTBOTH | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN23)
//#define GPIO_JOY_D       (GPIO_INTBOTH | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN19)
//#define GPIO_JOY_CTR     (GPIO_INTBOTH | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN14)

/* IRQ numbers for the buttons that do support interrrupts */

//#define GPIO_USER2_IRQ   LPC17_IRQ_P2p22
//#define GPIO_USER3_IRQ   LPC17_IRQ_P0p10
//#define GPIO_JOY_A_IRQ   LPC17_IRQ_P2p25
//#define GPIO_JOY_B_IRQ   LPC17_IRQ_P2p26
//#define GPIO_JOY_C_IRQ   LPC17_IRQ_P2p23
//#define GPIO_JOY_D_IRQ   LPC17_IRQ_P2p19
//#define GPIO_JOY_CTR_IRQ LPC17_IRQ_P0p14

/* LCD ******************************************************************************/
/* Backlight enable, P2[1].  Initial state is OFF (zero) */

#define GPIO_LCD_BL      (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT3 | GPIO_PIN24)

/* ABME2000 */
#define GPIO_AMBE2K_CLK    GPIO_SSP0_SCK_3
#define GPIO_AMBE2K_STRB   GPIO_SSP0_SSEL_4
#define GPIO_AMBE2K_RXDATA GPIO_SSP0_MISO_2
#define GPIO_AMBE2K_TXDATA GPIO_SSP0_MOSI_4
#define GPIO_AMBE2K_EPR    GPIO_EINT1_2      // use ext-interrupt1
//#define GPIO_AMBE2K_EPR    (GPIO_INTFE | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN11)
#define GPIO_AMBE2K_RST    (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT3 | GPIO_PIN27)
#define INTR_AMBE2K_EPR    LPC17_IRQ_EINT1
//#define INTR_AMBE2K_EPR    LPC17_IRQ_P2p11

/* XPT2046 Touchscreen **************************************************************/
/* -------------- -------------------- ------------ --------------------------------
 * XTPT2046       Module               Module       wefly1788 LED
 *                Signal               Connector    Connector
 * -------------- -------------------- ------------ ---------------------------------
 * Pin 11 PENIRQ\ PENIRQ (pulled high) PORT3 Pin 1  P2.15 PENIRQ
 * Pin 12 DOUT    MISO                 PORT3 Pin 4  P1.18 MISO1  (Also USB HOST UP LED)
 * Pin 13 BUSY    BUSY (pulled high)   PORT3 Pin 9  P2.14 BUSY
 * Pin 14 DIN     MOSI                 PORT3 Pin 3  P0.13 MOSI1  (Also USB Device up LED and SD CD pin)
 * Pin 15 CS\     SSEL (pulled high)   PORT3 Pin 6  P1.8  GPIO   (Also RMII_CRS_DV)
 * Pin 16 DCLK    SCK                  PORT3 Pin 5  P1.19 SCK1
 * -------------- -------------------- ------------ ---------------------------------
 *
 * Pins should not need to be configured as pull-ups because, according to the LCD
 * schematic, the are pulled-up on board the LCD module.
 */

//#define GPIO_TC_PENIRQ   (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN15)
//#define GPIO_TC_BUSY     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN14)
//#define GPIO_TC_CS       (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN8)

//#define LPC17_IRQ_PENIRQ LPC17_IRQ_P2p15

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: wefly1788_sspinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the WaveShare wefly1788 board.
 *
 ************************************************************************************/

void weak_function wefly1788_sspinitialize(void);

/************************************************************************************
 * Name: wefly1788_sdram_initialize
 *
 * Description:
 *   Initialize SDRAM
 *
 ************************************************************************************/

#ifdef CONFIG_LPC17_EMC
#ifdef CONFIG_LPC17_EXTDRAM
void wefly1788_sdram_initialize(void);
#endif

/************************************************************************************
 * Name: wefly1788_nor_initialize
 *
 * Description:
 *   Initialize NOR FLASH
 *
 ************************************************************************************/

#ifdef CONFIG_LPC17_EXTNOR
void wefly1788_nor_initialize(void);
#endif

/************************************************************************************
 * Name: wefly1788_nand_initialize
 *
 * Description:
 *   Initialize NAND FLASH
 *
 ************************************************************************************/

#ifdef CONFIG_LPC17_EXTNAND
void wefly1788_nand_initialize(void);
#endif
#endif /* CONFIG_LPC17_EMC */

/************************************************************************************
 * Name: wefly1788_lcd_initialize
 *
 * Description:
 *   Initialize the LCD.  Setup backlight (initially off)
 *
 ************************************************************************************/

#ifdef CONFIG_LPC17_LCD
void wefly1788_lcd_initialize(void);
#endif

/************************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization for NSH.
 *
 *   CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_NSH_ARCHINIT=n:
 *     Called from board_initialize().
 *
 ************************************************************************************/

#ifdef CONFIG_NSH_LIBRARY
int nsh_archinitialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* _CONFIGS_WEFLY1788_H */
