/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <nuttx/wqueue.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include "lpc17_ssp.h"
#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "chip/lpc17_syscon.h"
#include "chip/lpc17_pwm.h"
#include "lpc17_gpio.h"
#include "wefly1788.h"

#define GPIO_xx    (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT3 | GPIO_PIN24)

/* SSP Clocking *************************************************************/

/* All peripherals are clocked by the same peripheral clock in the LPC178x
 * family.
 */
#define SSP_CLOCK		BOARD_PCLK_FREQUENCY
#define AMBE2K_FRAME_PERIOD	(8*1000)		// in us

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure descibes the state of the SSP driver */

struct lpc17_sspdev_s
{
	uint32_t         sspbase;    /* SPIn base address */
	uint8_t          irq;     /* SPI IRQ number */
};

#define AMBE2K_FRAME_SIZE	24
#define AMBE2K_HDR		0x13ec

// pingpong mode
typedef struct {
	uint16_t data[AMBE2K_FRAME_SIZE];
	uint8_t valid;		// if a valid frame exist
	uint8_t blocking;	// is user waiting for valid data
}ambe2k_buf_t;

typedef struct {
	struct lpc17_sspdev_s sspdev;
	ambe2k_buf_t buf_r;
	ambe2k_buf_t buf_s;

	uint16_t data_r[AMBE2K_FRAME_SIZE];
	int pos_r;		// pos in working frame
	uint16_t data_s[AMBE2K_FRAME_SIZE];
	int pos_s;		// pos in working frame

	sem_t sem_frm_r;
	sem_t sem_frm_s;

	struct work_s work_epr;
	struct work_s work_ssp;
	struct work_s work_tmr;

	int send_data;
	int recv_done;
}ambe2k_dev_t;

enum {
	INPROGRES,	// should continue waiting
	CACHUP,		// rx fifo catch up tx fifo
	DONE		// frame tx/rx ok
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline uint32_t ssp_getreg(FAR struct lpc17_sspdev_s *priv, uint8_t offset);
static inline void ssp_putreg(FAR struct lpc17_sspdev_s *priv, uint8_t offset,
                                 uint32_t value);
static uint32_t ssp_setfrequency(FAR struct lpc17_sspdev_s *priv, uint32_t frequency);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static ambe2k_dev_t g_dev;
static sem_t g_sem_sync_r = SEM_INITIALIZER(0);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ssp_getreg
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline uint32_t ssp_getreg(FAR struct lpc17_sspdev_s *priv, uint8_t offset)
{
	return getreg32(priv->sspbase + (uint32_t)offset);
}

/****************************************************************************
 * Name: ssp_putreg
 *
 * Description:
 *   Write a 32-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 16-bit value to be written
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static inline void ssp_putreg(FAR struct lpc17_sspdev_s *priv, uint8_t offset, uint32_t value)
{
	putreg32(value, priv->sspbase + (uint32_t)offset);
}

/*
 * Name: ssp_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 */
static uint32_t ssp_setfrequency(FAR struct lpc17_sspdev_s *priv, uint32_t frequency)
{
	uint32_t cpsdvsr;
	uint32_t scr;
	uint32_t regval;
	uint32_t actual;

	/* Check if the requested frequency is the same as the frequency selection */
	DEBUGASSERT(priv && frequency <= SSP_CLOCK / 2);

	/* The SSP bit frequency is given by:
	 *
	 *   frequency = SSP_CLOCK / (CPSDVSR * (SCR+1)).
	 *
	 * Let's try for a solution with the smallest value of SCR.  NOTES:
	 * (1) In the calculations below, the value of the variable 'scr' is
	 * (SCR+1) in the above equation. (2) On slower LPC17xx parts, SCR
	 * will probably always be zero.
	 */
	for (scr = 1; scr <= 256; scr++)
	{
		/* CPSDVSR = SSP_CLOCK / (SCR + 1) / frequency */

		cpsdvsr = SSP_CLOCK / (scr * frequency);

		/* Break out on the first solution we find with the smallest value
		 * of SCR and with CPSDVSR within the maximum range or 254.
		 */

		if (cpsdvsr < 255)
		{
			break;
		}
	}

	DEBUGASSERT(scr <= 256 && cpsdvsr <= 255);

	/* "In master mode, CPSDVSRmin = 2 or larger (even numbers only)" */

	if (cpsdvsr < 2)
	{
		/* Clip to the minimum value. */

		cpsdvsr = 2;
	}
	else if (cpsdvsr > 254)
	{
		/* This should never happen */

		cpsdvsr = 254;
	}

	/* Force even */

	cpsdvsr = (cpsdvsr + 1) & ~1;

	/* Save the new CPSDVSR and SCR values */

	dbg("cpsdvsr=%d\n", cpsdvsr);
	dbg("scr=%d\n", scr);
	dbg("sspclk=%d\n", SSP_CLOCK);
	ssp_putreg(priv, LPC17_SSP_CPSR_OFFSET, cpsdvsr);

	regval  = ssp_getreg(priv, LPC17_SSP_CR0_OFFSET);
	regval &= ~SSP_CR0_SCR_MASK;
	regval |= ((scr - 1) << SSP_CR0_SCR_SHIFT);
	ssp_putreg(priv, LPC17_SSP_CR0_OFFSET, regval);

	/* Calculate the new actual */

	actual = SSP_CLOCK / (cpsdvsr * scr);

	/* Save the frequency setting */

	dbg("Frequency %d->%d\n", frequency, actual);
	return actual;
}

static void ssp_recv_rx_fifo(ambe2k_dev_t *dev)
{
	uint16_t val;

	while (ssp_getreg(&dev->sspdev, LPC17_SSP_SR_OFFSET) & SSP_SR_RNE) {
		val = ssp_getreg(&dev->sspdev, LPC17_SSP_DR_OFFSET);

		if (dev->recv_done)
			continue;

		// pos_r < 0	-> waiting for a new frame
		if (dev->pos_r < 0) {
			// a frame should start with AMBE2K_HDR
			if (val == AMBE2K_HDR) {
				static int x=1;
				x=!x;
	lpc17_gpiowrite(GPIO_xx, x);
				dev->pos_r = 0;
				// we can send actual data *now*
				dev->send_data = 1;
			}
			else
				continue;
		}

		dev->data_r[dev->pos_r ++] = val;

		if (dev->pos_r == AMBE2K_FRAME_SIZE) {
			// save the frame for user
			sem_wait(&dev->sem_frm_r);
			memcpy(dev->buf_r.data, dev->data_r, AMBE2K_FRAME_SIZE);
			dev->buf_r.valid = 1;
			if (dev->buf_r.blocking) {
				sem_post(&g_sem_sync_r);
				dev->buf_r.blocking = 0;
			}
			sem_post(&dev->sem_frm_r);

			dev->recv_done = 1;
			// waiting for the next recv-frame
			dev->pos_r = -1;
		}
	}
}

static void ssp_fill_tx_fifo(ambe2k_dev_t *dev)
{
	uint16_t val;

	if (dev->recv_done && !dev->send_data)
		return;

	// get data to send
	// send_data == 1 -> we should send the actural data *now*
	//           == 0 -> just give tx/rx clk (send 0) to locate AMBE2K_HDR
	// pos_s < 0      -> frame to send is not ready
	if (dev->send_data && dev->pos_s < 0) {
		if (dev->buf_s.valid) {
			sem_wait(&dev->sem_frm_s);
			memcpy(dev->data_s, dev->buf_s.data, AMBE2K_FRAME_SIZE);
			dev->buf_s.valid = 0;
			sem_post(&dev->sem_frm_s);
		}
		else {
			// construct empty frame

			bzero(dev->data_s, AMBE2K_FRAME_SIZE);
			dev->data_s[0] = AMBE2K_HDR;
			dev->data_s[1] = 1<<7;	// lost frame indicator
			dev->data_s[2] = 0xbfc0;	// Rate Info0
			dev->data_s[3] = 0;	// Rate Info1
			dev->data_s[4] = 0;	// Rate Info2
			dev->data_s[5] = 0;	// Rate Info3
			dev->data_s[6] = 0x72c0;	// Rate Info4
			dev->data_s[10] = 0x00ff;	// DTMF
			dev->data_s[11] = 0x8000;	// DTMF
		}
		dev->pos_s = 0;
	}

	// fill ssp xmit fifo
	while (ssp_getreg(&dev->sspdev, LPC17_SSP_SR_OFFSET) & SSP_SR_TNF) {
		// default send value, no data, just tx/rx clk
		val = 0;

		if (dev->send_data) {
			if (dev->pos_s < AMBE2K_FRAME_SIZE) {
				val = dev->buf_s.data[dev->pos_s];
				++ dev->pos_s;
			}
			else {	// this frame has been sent, start timer for
				// the next turn(frame)

				// send data not prepared
				dev->pos_s = -1;
				// we're not sending data now
				dev->send_data = 0;

				/*
				 * Timer3 start counting, waiting for next turn
				 */
				putreg32(PWM_TCR_CNTREN,
						LPC17_TMR3_BASE
						+ LPC17_PWM_TCR_OFFSET);
			}
		}

		ssp_putreg(&dev->sspdev, LPC17_SSP_DR_OFFSET, (uint32_t)val);
	}
}

static void tmr_intr_work(FAR void *arg)
{
	ambe2k_dev_t *dev = (ambe2k_dev_t *)arg;

	// enable ssp intr
	up_enable_irq(dev->sspdev.irq);

	// start a new turn
	dev->recv_done = 0;
	ssp_fill_tx_fifo(dev);
	// waiting for ssp intr...
}

static int ambe2k_tmr3_intr(int irq, void *context)
{
	int ret;

	// clear interrupt
	putreg32(PWM_IR_MR0, LPC17_TMR3_BASE + LPC17_PWM_IR_OFFSET);
	// clear Timer Counter
	putreg32(PWM_TCR_CNTRRST, LPC17_TMR3_BASE + LPC17_PWM_TCR_OFFSET);

	ret = work_queue(HPWORK, &g_dev.work_tmr, tmr_intr_work, &g_dev, 0);
	if (ret < 0)
	{
		dbg("ERROR: Failed to schedule epr work\n");
	}

	return OK;
}

static void ssp_intr_work(FAR void *arg)
{
	ambe2k_dev_t *dev = (ambe2k_dev_t *)arg;

	ssp_fill_tx_fifo(dev);
	ssp_recv_rx_fifo(dev);

	up_enable_irq(dev->sspdev.irq);
}

static int ambe2k_ssp_intr(int irq, void *context)
{
	int ret;

	up_disable_irq(g_dev.sspdev.irq);

	ret = work_queue(HPWORK, &g_dev.work_ssp, ssp_intr_work, &g_dev, 0);
	if (ret < 0)
	{
		dbg("ERROR: Failed to schedule ssp work\n");
	}

	return OK;
}

static void epr_intr_work(FAR void *arg)
{
	ambe2k_dev_t *dev = (ambe2k_dev_t *)arg;

	// enable ssp intr
	up_enable_irq(dev->sspdev.irq);

	ssp_fill_tx_fifo(dev);
	// waiting for ssp intr...
}

static int ambe2k_epr_intr(int irq, void *context)
{
	int ret;

	// disable EPR intr permanent
	up_disable_irq(INTR_AMBE2K_EPR);

	ret = work_queue(HPWORK, &g_dev.work_epr, epr_intr_work, &g_dev, 0);
	if (ret < 0)
	{
		dbg("ERROR: Failed to schedule epr work\n");
	}

	return OK;
}

static int lpc17_timer_init(ambe2k_dev_t *dev)
{
	uint32_t regval;
	irqstate_t flags;
	int ret;

	/*
	 * Timer initialize
	 */

	flags = irqsave();

	/* Enable peripheral clocking to Timer3 */
	regval  = getreg32(LPC17_SYSCON_PCONP);
	regval |= SYSCON_PCONP_PCTIM3;
	putreg32(regval, LPC17_SYSCON_PCONP);

	/* Set prescale register to make 1us per Timer Counter  */
	putreg32(BOARD_PCLK_FREQUENCY/1000/1000 - 1,
			LPC17_TMR3_BASE + LPC17_PWM_PR_OFFSET);
	/* Set prescale register to make 1us per Timer Counter  */
	putreg32(AMBE2K_FRAME_PERIOD, LPC17_TMR3_BASE + LPC17_PWM_MR0_OFFSET);
	/* Enable MR0 interrupt, stop after interrupted */
	putreg32(PWM_MCR_MR0I | PWM_MCR_MR0S,
			LPC17_TMR3_BASE + LPC17_PWM_MCR_OFFSET);

	irqrestore(flags);

	/* Attach and enable the Timer3 IRQ */
	ret = irq_attach(LPC17_IRQ_TMR3, ambe2k_tmr3_intr);
	if (ret != OK) {
		dbg("attach Timer3 interrupt error");
	}
	up_enable_irq(LPC17_IRQ_TMR3);

	return OK;
}

static int lpc17_ssp_init(ambe2k_dev_t *dev)
{
	irqstate_t flags;
	uint32_t regval;
	int ret;
	int i;

	/*
	 * SSP initialize
	 */

	// ssp pins
	lpc17_configgpio(GPIO_AMBE2K_CLK);
	lpc17_configgpio(GPIO_AMBE2K_STRB);
	lpc17_configgpio(GPIO_AMBE2K_RXDATA);
	lpc17_configgpio(GPIO_AMBE2K_TXDATA);

	flags = irqsave();

	/* Enable peripheral clocking to SSP0 */
	regval  = getreg32(LPC17_SYSCON_PCONP);
	regval |= SYSCON_PCONP_PCSSP0;
	putreg32(regval, LPC17_SYSCON_PCONP);

	irqrestore(flags);

	/* Configure 16-bit TI mode */
	ssp_putreg(&dev->sspdev, LPC17_SSP_CR0_OFFSET, SSP_CR0_DSS_16BIT|SSP_CR0_FRF_TI);

	/* Disable the SSP and enable RX timout and RX half-full interrupt */
	ssp_putreg(&dev->sspdev, LPC17_SSP_CR1_OFFSET, 0);
	ssp_putreg(&dev->sspdev, LPC17_SSP_IMSC_OFFSET, SSP_INT_RT|SSP_INT_RX);

	/* Select frequency of approx. 1800KHz (MAX 2.048MHz) */
	ssp_setfrequency(&dev->sspdev, 100*1000);

	/* Enable port */
	regval = ssp_getreg(&dev->sspdev, LPC17_SSP_CR1_OFFSET);
	ssp_putreg(&dev->sspdev, LPC17_SSP_CR1_OFFSET, regval | SSP_CR1_SSE);
	for (i = 0; i < LPC17_SSP_FIFOSZ; i++)
	{
		(void)ssp_getreg(&dev->sspdev, LPC17_SSP_DR_OFFSET);
	}

	/* Attach the SSP IRQ */
	ret = irq_attach(dev->sspdev.irq, ambe2k_ssp_intr);
	if (ret != OK) {
		dbg("attach SSP interrupt error");
	}

	return OK;
}

static int lpc17_epr_init(ambe2k_dev_t *dev)
{
	irqstate_t flags;
	uint32_t regval;
	int ret;

	/*
	 * EPR interrupt
	 */

	flags = irqsave();

	/* External interrupt configuration: edge sesitive */
	regval  = getreg32(LPC17_SYSCON_EXTMODE);
	//regval |= SYSCON_EXTMODE_EINT1;
	regval &= ~SYSCON_EXTMODE_EINT1;
	putreg32(regval, LPC17_SYSCON_EXTMODE);

	lpc17_configgpio(GPIO_AMBE2K_EPR);

	irqrestore(flags);

	/* Attach and enable the EPR IRQ */
	ret = irq_attach(INTR_AMBE2K_EPR, ambe2k_epr_intr);
	if (ret != OK) {
		dbg("attach EPR interrupt error");
	}

	return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
void ambe2k_send_frame(uint8_t *frm)
{
	sem_wait(&g_dev.sem_frm_s);

	memcpy(g_dev.buf_s.data, frm, sizeof(uint16_t)*AMBE2K_FRAME_SIZE);
	g_dev.buf_r.valid = 1;

	sem_post(&g_dev.sem_frm_s);
}

void ambe2k_recv_frame(uint8_t *frm)
{
	int should_wait;

	do {
		sem_wait(&g_dev.sem_frm_r);
		if (g_dev.buf_r.valid) {
			memcpy(frm, g_dev.buf_r.data, sizeof(uint16_t)*AMBE2K_FRAME_SIZE);
			g_dev.buf_r.valid = 0;
			should_wait = 0;
		}
		else {
			g_dev.buf_r.blocking = 1;
			should_wait = 1;
		}
		sem_post(&g_dev.sem_frm_r);

		if (should_wait)
			sem_wait(&g_sem_sync_r);
	} while (should_wait);
}

int ambe2k_initialize(void)
{
	/* init ssp rx/tx bufs */
	bzero(&g_dev.buf_r, sizeof g_dev.buf_r);
	bzero(&g_dev.buf_s, sizeof g_dev.buf_s);
	g_dev.sspdev.sspbase = LPC17_SSP0_BASE;
	g_dev.sspdev.irq = LPC17_IRQ_SSP0;
	g_dev.pos_r = g_dev.pos_s = -1;
	g_dev.buf_r.valid = g_dev.buf_s.valid = 0;
	g_dev.buf_r.blocking = g_dev.buf_s.blocking = 0;
	g_dev.send_data = g_dev.recv_done = 0;
	sem_init(&g_dev.sem_frm_s, 0, 1);
	sem_init(&g_dev.sem_frm_r, 0, 1);

	lpc17_configgpio(GPIO_AMBE2K_RST);
	lpc17_gpiowrite(GPIO_AMBE2K_RST, 1);

	lpc17_configgpio(GPIO_xx);
	lpc17_gpiowrite(GPIO_xx, 1);

	lpc17_timer_init(&g_dev);
	lpc17_ssp_init(&g_dev);
	lpc17_epr_init(&g_dev);

	up_enable_irq(INTR_AMBE2K_EPR);

	lpc17_gpiowrite(GPIO_AMBE2K_RST, 0);
	up_mdelay(100);
	lpc17_gpiowrite(GPIO_AMBE2K_RST, 1);

	return OK;
}

