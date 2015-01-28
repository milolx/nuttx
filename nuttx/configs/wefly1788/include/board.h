#ifndef __CONFIG_WEFLY1788_INCLUDE_BOARD_H
#define __CONFIG_WEFLY1788_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_GPIO_IRQ)
#  include <nuttx/irq.h>
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* Clocking *************************************************************************/
/* NOTE:  The following definitions require lpc17_syscon.h.  It is not included here
 * because the including C file may not have that file in its include path.
 */

#define BOARD_XTAL_FREQUENCY       (12000000)            /* XTAL oscillator frequency */
#define BOARD_OSCCLK_FREQUENCY     BOARD_XTAL_FREQUENCY  /* Main oscillator frequency */
#define BOARD_RTCCLK_FREQUENCY     (32768)               /* RTC oscillator frequency */
#define BOARD_INTRCOSC_FREQUENCY   (4000000)             /* Internal RC oscillator frequency */
#define BOARD_WDTOSC_FREQUENCY     (500000)              /* WDT oscillator frequency */

/* This is the clock setup we configure for:
 *
 *   SYSCLK = BOARD_OSCCLK_FREQUENCY = 12MHz  -> Select Main oscillator for source
 *   PLL0CLK = (10 * SYSCLK) / 1 = 120MHz -> PLL0 multipler=10, pre-divider=1
 *   CCLCK = 120MHz  -> CCLK divider = 1
 */

#define LPC17_CCLK                 120000000 /* 120Mhz */
#define BOARD_PCLKDIV              2         /* Peripheral clock = LPC17_CCLK/2 */
#define BOARD_PCLK_FREQUENCY       (LPC17_CCLK / BOARD_PCLKDIV)

/* Select the main oscillator as the frequency source.  SYSCLK is then the frequency
 * of the main oscillator.
 *
 * If BOARD_XTAL_FREQUENCY > 15000000, then the SCS OSCRS bit (bit 4) should also
 * be set in the BOARD_SCS_VALUE.
 */

#undef CONFIG_LPC17_MAINOSC
#define CONFIG_LPC17_MAINOSC       1
#define BOARD_SCS_VALUE            SYSCON_SCS_OSCEN

/* Select the main oscillator and CCLK divider. The output of the divider is CCLK.
 * The input to the divider (PLLCLK) will be determined by the PLL output.
 */

#define BOARD_CCLKSEL_DIVIDER      1
#define BOARD_CCLKSEL_VALUE        (BOARD_CCLKSEL_DIVIDER | SYSCON_CCLKSEL_CCLKSEL)

/* PLL0.  PLL0 is used to generate the CPU clock (PLLCLK).
 *
 *  Source clock:               Main oscillator
 *  PLL0 Multiplier value (M):  10
 *  PLL0 Pre-divider value (P): 1
 *
 *  PLL0CLK = (M * SYSCLK) = 120MHz
 */

#undef CONFIG_LPC17_PLL0
#define CONFIG_LPC17_PLL0          1
#define BOARD_CLKSRCSEL_VALUE      SYSCON_CLKSRCSEL_MAIN

#define BOARD_PLL0CFG_MSEL         10
#define BOARD_PLL0CFG_PSEL         1
#define BOARD_PLL0CFG_VALUE \
  (((BOARD_PLL0CFG_MSEL-1) << SYSCON_PLLCFG_MSEL_SHIFT) | \
   ((BOARD_PLL0CFG_PSEL-1) << SYSCON_PLLCFG_PSEL_SHIFT))

/* PLL1 : PLL1 is used to generate clock for the USB */

#undef  CONFIG_LPC17_PLL1
#define BOARD_PLL1CFG_MSEL        4
#define BOARD_PLL1CFG_PSEL        2
#define BOARD_PLL1CFG_VALUE \
  (((BOARD_PLL1CFG_MSEL-1) << SYSCON_PLLCFG_MSEL_SHIFT) | \
   ((BOARD_PLL1CFG_PSEL-1) << SYSCON_PLLCFG_PSEL_SHIFT))

#ifdef CONFIG_LPC17_EMC
/* EMC clock selection.
 *
 * The EMC clock should not be driven above 80MHz.  As a result the EMC
 * uses the CPU clock divided by two.
 */

#  define BOARD_EMCCLKSEL_DIVIDER  2
#  define BOARD_EMCCLKSEL_VALUE    SYSCON_EMCCLKSEL_CCLK_DIV2
#  define LPC17_EMCCLK             (LPC17_CCLK / BOARD_EMCCLKSEL_DIVIDER)
#endif

#if defined(CONFIG_LPC17_USBHOST) || (CONFIG_LPC17_USBDEV)
/* USB divider.  The output of the PLL is used as the USB clock
 *
 *  USBCLK = PLL1CLK = (SYSCLK * 4)  = 48MHz
 */

#  define BOARD_USBCLKSEL_DIVIDER  1
#  define BOARD_USBCLKSEL_VALUE    (SYSCON_USBCLKSEL_USBDIV_DIV1 | \
                                    SYSCON_USBCLKSEL_USBSEL_PLL1)
#endif

/* FLASH Configuration */

#undef  CONFIG_LPC17_FLASH
#define CONFIG_LPC17_FLASH         1

/* Flash access use 6 CPU clocks - Safe for any allowed conditions */

#define BOARD_FLASHCFG_VALUE       (SYSCON_FLASHCFG_TIM_5 | 0x03a)

/* Ethernet configuration */

#define ETH_MCFG_CLKSEL_DIV        ETH_MCFG_CLKSEL_DIV20

/* Set EMC delay values:
 *
 * CMDDLY: Programmable delay value for EMC outputs in command delayed
 *   mode.  The delay amount is roughly CMDDLY * 250 picoseconds.
 * FBCLKDLY: Programmable delay value for the feedback clock that controls
 *   input data sampling.  The delay amount is roughly (FBCLKDLY+1) * 250
 *   picoseconds.
 * CLKOUT0DLY: Programmable delay value for the CLKOUT0 output. This would
 *   typically be used in clock delayed mode.  The delay amount is roughly
 *  (CLKOUT0DLY+1) * 250 picoseconds.
 * CLKOUT1DLY: Programmable delay value for the CLKOUT1 output. This would
 *  typically be used in clock delayed mode.  The delay amount is roughly
 *  (CLKOUT1DLY+1) * 250 picoseconds.
 *
 * Optimal for NOR: {1,1,1,1}
 * Needed for NAND and SDRAM: {17,1,2,1}
 */

#ifdef CONFIG_LPC17_EMC
#if defined(CONFIG_LPC17_EXTNAND) || defined(CONFIG_LPC17_EXTDRAM)
#  define BOARD_CMDDLY             17
#  define BOARD_FBCLKDLY           17
#  define BOARD_CLKOUT0DLY         1
#  define BOARD_CLKOUT1DLY         1
#else
#  define BOARD_CMDDLY             1
#  define BOARD_FBCLKDLY           1
#  define BOARD_CLKOUT0DLY         1
#  define BOARD_CLKOUT1DLY         1
#endif
#endif

/* Alternate pin selections *********************************************************/

/* UART0:
 *
 * TX    --- Connected to P0[2]
 * RX    --- Connected to P0[3]
 */

#define GPIO_UART0_TXD             GPIO_UART0_TXD_2
#define GPIO_UART0_RXD             GPIO_UART0_RXD_2

/* UART1:
 *
 * TXD   --- Connected to P0[15]
 * RXD   --- Connected to P0[16]
 */

#define GPIO_UART1_TXD             GPIO_UART1_TXD_1
#define GPIO_UART1_RXD             GPIO_UART1_RXD_1

/* LCD: R,G,B - 5,6,5
 *
 * pin0 : red   VD3  -- Connected to P2[9]
 * pin1 : red   VD4  -- Connected to P2[6]
 * pin2 : red   VD5  -- Connected to P2[7]
 * pin3 : red   VD6  -- Connected to P4[28]
 * pin4 : red   VD7  -- Connected to P4[29]
 * pin5 : green VD10 -- Connected to P1[20]
 * pin6 : green VD11 -- Connected to P1[21]
 * pin7 : green VD12 -- Connected to P1[22]
 * pin8 : green VD13 -- Connected to P1[23]
 * pin9 : green VD14 -- Connected to P1[24]
 * pin10: green VD15 -- Connected to P1[25]
 * pin11: blue  VD19 -- Connected to P2[13]
 * pin12: blue  VD20 -- Connected to P1[26]
 * pin13: blue  VD21 -- Connected to P1[27]
 * pin14: blue  VD22 -- Connected to P1[28]
 * pin15: blue  VD23 -- Connected to P1[29]
 */

#define GPIO_LCD_VD0                GPIO_LCD_VD3_1
#define GPIO_LCD_VD1                GPIO_LCD_VD4_1
#define GPIO_LCD_VD2                GPIO_LCD_VD5_1
#define GPIO_LCD_VD3                GPIO_LCD_VD6_3
#define GPIO_LCD_VD4                GPIO_LCD_VD7_3
#define GPIO_LCD_VD5                GPIO_LCD_VD10_1
#define GPIO_LCD_VD6                GPIO_LCD_VD11_1
#define GPIO_LCD_VD7                GPIO_LCD_VD12_1
#define GPIO_LCD_VD8                GPIO_LCD_VD13_1
#define GPIO_LCD_VD9                GPIO_LCD_VD14_1
#define GPIO_LCD_VD10               GPIO_LCD_VD15_1
#define GPIO_LCD_VD11               GPIO_LCD_VD19
#define GPIO_LCD_VD12               GPIO_LCD_VD20
#define GPIO_LCD_VD13               GPIO_LCD_VD21
#define GPIO_LCD_VD14               GPIO_LCD_VD22
#define GPIO_LCD_VD15               GPIO_LCD_VD23

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: lpc17_boardinitialize
 *
 * Description:
 *   All LPC17xx architectures must provide the following entry point.  This entry
 *   point is called early in the intitialization -- after all memory has been
 *   configured and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

EXTERN void lpc17_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIG_WEFLY1788_INCLUDE_BOARD_H */
