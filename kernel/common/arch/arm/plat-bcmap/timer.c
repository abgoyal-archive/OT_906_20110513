/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/plat-bcmap/timer.c
*
* Unless you and Broadcom execute a separate written software license agreement
* governing use of this software, this software is licensed to you under the
* terms of the GNU General Public License version 2, available at
* http://www.gnu.org/copyleft/gpl.html (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a license
* other than the GPL, without Broadcom's express prior written consent.
*******************************************************************************/

/*
 * Broadcom specific timer implementation
 * Based on realview platforms
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/clockchips.h>
#include <linux/jiffies.h>

#include <asm/mach/time.h>
#include <mach/hardware.h>
#include <plat/timer.h>

/*
 * The GPT device hosts six different counters, with 6 set of
 * registers. These are register names.
 */

#define GPT_ISR		0x00	/* Interrupt Stuff Register */

/* per-timer registers take 0..5 as argument */
#define GPT_CSR(x)	(0x10 * (x) + 0x04)	/* Control & Status */
#define GPT_RELOAD(x)	(0x10 * (x) + 0x08)	/* Reload value */
#define GPT_VALUE(x)	(0x10 * (x) + 0x0c)	/* Current value */

/* bits for the control register */
#define GPT_CSR_EN		(1 << 31)	/* Timer enable */
#define GPT_CSR_CLKSEL		(1 << 30)	/* Timer Clock Select */
#define GPT_CSR_SW_SLPMODE	(1 << 29)	/* Run timer in sleep mode */
#define GPT_CSR_HW_SLPMODE_DIS	(1 << 28)	/* Disable hw sleep signal */
#define GPT_CSR_INT_EN		(1 << 27)	/* Enable Interrupt */
#define GPT_CSR_INT2_ASSIGN	(1 << 26)	/* Assign Intr of this GPT */
#define GPT_CSR_CLKSEL1         (1 << 25)	/* Select Tick Clock */
#define GPT_CSR_TIMER_PWRON	(1 << 24)	/* Power on */
#define GPT_CSR_SW_PEDEMODE	(1 << 23)	/* Pedestal mode */
#define GPT_CSR_HW_PEDEMODE_DIS	(1 << 22)	/* Disable Pedestal mode */
#define GPT_CSR_PEDESTAL_STATE	(1 << 9)	/* Debug purpose */
#define GPT_CSR_SLEEP_STATE	(1 << 8)	/*Debug purpose */
#define GPT_CSR_INT_FLAG	(1 << 0)	/*Terminal Count Status */

#define GPT_CE_CTRL		(GPT_CSR_INT_FLAG | GPT_CSR_INT_EN |\
				 GPT_CSR_TIMER_PWRON | GPT_CSR_HW_PEDEMODE_DIS)
#define GPT_CS_CTRL		(GPT_CSR_SW_SLPMODE | GPT_CSR_HW_SLPMODE_DIS | GPT_CSR_TIMER_PWRON)

#define GPT_MAX_COUNT 		0xffffffff

static struct timer_config config;

#define SLPTMR_SMTDCLR	0x00	/* Sleep Mode Timer Down Counter Load Register */
#define SLPTMR_SMTDCR	0x04	/* Sleep Mode Timer Down Counter Register */
#define SLPTMR_SMTCR	0x08	/* Sleep Mode Timer Control Register */
#define SLPTMR_SMTCMPR	0x0c	/* Sleep Mode Timer Compare Register */
#define SLPTMR_SMTSTR	0x10	/* Sleep Mode Timer System Timer Register */

#define SLPTMR_CLKS		(1 << 1)	/*Clock Select */
#define SLPTMR_TMREN		(1 << 0)	/*Timer Enable */
#define SLPTMR_CE_CTRL		(SLPTMR_TMREN)

#define SLPTMR_VALUE(x)		(0x100 * (x) + SLPTMR_SMTSTR) 
bool clkevt_mode_oneshot;

static uint32_t setup_gpt(uint32_t gpt, uint32_t reload, uint32_t control)
{
	uint32_t reg;

	/* set required controls except enable and power */
	writel(control, config.cs_base + GPT_CSR(gpt));
	writel(reload, config.cs_base + GPT_RELOAD(gpt));
	reg = readl(config.cs_base + GPT_CSR(gpt));
	/* enable, power on */
	reg |= GPT_CSR_EN | GPT_CSR_TIMER_PWRON;
	writel(reg, config.cs_base + GPT_CSR(gpt));

	return readl(config.cs_base + GPT_VALUE(gpt));
}

static void bcm_gpt_clkevt_mode(enum clock_event_mode mode,
				struct clock_event_device *dev)
{
	unsigned long flags;
	uint32_t reg;

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		local_irq_save(flags);
		clkevt_mode_oneshot = false;
		reg = readl(config.ce_base + GPT_CSR(config.ce_index));
		reg |= GPT_CSR_TIMER_PWRON | GPT_CSR_EN;
		writel(reg, config.ce_base + GPT_CSR(config.ce_index));
		local_irq_restore(flags);
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
		local_irq_save(flags);
		reg = readl(config.ce_base + GPT_CSR(config.ce_index));
		reg &= ~(GPT_CSR_TIMER_PWRON | GPT_CSR_EN);
		writel(reg, config.ce_base + GPT_CSR(config.ce_index));
		local_irq_restore(flags);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		local_irq_save(flags);
		clkevt_mode_oneshot = true;
		local_irq_restore(flags);
		break;
	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_UNUSED:
		break;
	}
}

static int bcm_gpt_set_nextevt(unsigned long evt,
			       struct clock_event_device *unused)
{
	setup_gpt(config.ce_index, evt, GPT_CE_CTRL);
	return 0;
}

static uint32_t setup_slptmr(uint32_t reload, uint32_t control)
{
	writel(0, config.ce_base + SLPTMR_SMTCR);	/* Reset the control register */

	writel(0, config.ce_base + SLPTMR_SMTSTR);	/* System Clock Register */
	writel(reload, config.ce_base + SLPTMR_SMTCMPR);	/* Compare Register */

	writel(control, config.ce_base + SLPTMR_SMTCR);	/* Enable the timer via Control Register */
	return 0;
}

static void bcm_slptmr_clkevt_mode(enum clock_event_mode mode,
				   struct clock_event_device *dev)
{
	unsigned long flags;

	switch (mode) {
	case CLOCK_EVT_MODE_SHUTDOWN:
		local_irq_save(flags);
		writel(0, config.ce_base + SLPTMR_SMTCR);
		local_irq_restore(flags);
		break;
	case CLOCK_EVT_MODE_PERIODIC:
		clkevt_mode_oneshot = false;
		writel((readl(config.ce_base + SLPTMR_SMTCR) | SLPTMR_TMREN),
		       config.ce_base + SLPTMR_SMTCR);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		local_irq_save(flags);
		clkevt_mode_oneshot = true;
		local_irq_restore(flags);
		break;
	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_UNUSED:
		break;
	}
}

static int bcm_slptmr_set_nextevt(unsigned long evt,
				  struct clock_event_device *unused)
{
	setup_slptmr(evt, SLPTMR_CE_CTRL);
	return 0;
}

static struct clock_event_device bcm_clkevt = {
	.name = "bcm_clock_event",
	.shift = 32,
	.rating = 200,
};

static void __init bcm_clockevent_init(unsigned int rate)
{
	bcm_clkevt.irq = config.irq;
	bcm_clkevt.mult = div_sc(rate, NSEC_PER_SEC, bcm_clkevt.shift);
	bcm_clkevt.max_delta_ns = clockevent_delta2ns(0xfffffffe, &bcm_clkevt);
	bcm_clkevt.min_delta_ns = clockevent_delta2ns(0, &bcm_clkevt);
	bcm_clkevt.cpumask = cpumask_of(0);
	bcm_clkevt.features = CLOCK_EVT_FEAT_ONESHOT;

	if (config.ce_base == config.cs_base) {
		bcm_clkevt.set_mode = bcm_gpt_clkevt_mode;
		bcm_clkevt.set_next_event = bcm_gpt_set_nextevt;
		bcm_clkevt.features |= CLOCK_EVT_FEAT_PERIODIC;
	} else {
		bcm_clkevt.set_mode = bcm_slptmr_clkevt_mode;
		bcm_clkevt.set_next_event = bcm_slptmr_set_nextevt;
		writel(1, config.ce_base + SLPTMR_SMTDCLR);	/* Setup the Load Register */
	}

	clockevents_register_device(&bcm_clkevt);
	return;
}

/*
 * IRQ handler for the GP timer
 */
static irqreturn_t bcm_gpt_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &bcm_clkevt;

	if (clkevt_mode_oneshot)
		writel((~GPT_CSR_EN &
			readl(config.ce_base + GPT_CSR(config.ce_index))),
		       config.ce_base + GPT_CSR(config.ce_index));

	evt->event_handler(evt);

	writel((readl(config.ce_base + GPT_CSR(config.ce_index)) |
		GPT_CSR_INT_FLAG), config.ce_base + GPT_CSR(config.ce_index));
	return IRQ_HANDLED;
}

/*
 * IRQ handler for the Sleep timer
 */
static irqreturn_t bcm_slptmr_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &bcm_clkevt;
	if (clkevt_mode_oneshot) {
		writel((readl(config.ce_base + SLPTMR_SMTCR) & ~SLPTMR_TMREN),
		       config.ce_base + SLPTMR_SMTCR);
	} else {
		writel((readl(config.ce_base + SLPTMR_SMTSTR) - readl(config.ce_base + SLPTMR_SMTCMPR)), config.ce_base + SLPTMR_SMTSTR);	/* System Clock Register */
		writel((readl(config.ce_base + SLPTMR_SMTCR) | SLPTMR_TMREN),
		       config.ce_base + SLPTMR_SMTCR);
	}
	evt->event_handler(evt);

	return IRQ_HANDLED;
}

/*
 * Set up timer interrupt, and return the current time in seconds.
 */
static struct irqaction bcm_timer_irq = {
	.name = "BCM Timer Tick",
	.flags = IRQF_DISABLED | IRQF_TIMER,
};

/*
 * clocksource: the GPT device is a decrementing counters, so we negate
 * the value being read.
 */
static cycle_t bcm_read_timer(struct clocksource *cs)
{
	return GPT_MAX_COUNT - readl(config.cs_base +
				     GPT_VALUE(config.cs_index));
}

static struct clocksource bcm_clksrc = {
	.name = "bcm_clock_source_gpt5",
	.rating = 200,
	.read = bcm_read_timer,
	.mask = CLOCKSOURCE_MASK(32),
	.shift = 16,
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
};

static __init void bcm_timer_reset(uint32_t rate)
{
	uint32_t reload = (rate + HZ / 2) / HZ;
	setup_gpt(config.cs_index, GPT_MAX_COUNT, GPT_CS_CTRL);
	if (config.ce_base == config.cs_base)
		setup_gpt(config.ce_index, reload, GPT_CE_CTRL);
	else
		setup_slptmr(reload, SLPTMR_CE_CTRL);
	return;
}

static void __init bcm_clocksource_init(unsigned int rate)
{
	bcm_clksrc.mult = clocksource_hz2mult(rate, bcm_clksrc.shift);
	clocksource_register(&bcm_clksrc);
	return;
}

/*
 * Set up the clock source and clock events devices
 */
void __init bcm_timer_init(struct timer_config *bcm_config)
{
	config = *bcm_config;

	/* Init the timer and register clocksource */
	bcm_timer_reset(CLOCK_TICK_RATE);	/* 32kHz */
	bcm_clocksource_init(CLOCK_TICK_RATE);
	bcm_clockevent_init(CLOCK_TICK_RATE);
	/* Register irq for the system timer */
	if (config.ce_base == config.cs_base)
		bcm_timer_irq.handler = bcm_gpt_interrupt;
	else
		bcm_timer_irq.handler = bcm_slptmr_interrupt;
	setup_irq(config.irq, &bcm_timer_irq);

	return;
}

unsigned int timer_get_tick_count(void)
{
	struct clocksource *cs = 0;
	return bcm_read_timer(cs);
}
EXPORT_SYMBOL(timer_get_tick_count);

unsigned long timer_get_cp_tick_count(void)
{
	if (!config.cp_cs_base) {
		pr_info("CP CS colock source is not specified\n");
		return 0;
	}
	return readl(config.cp_cs_base + SLPTMR_VALUE(config.cp_cs_index)); 
}
EXPORT_SYMBOL(timer_get_cp_tick_count);
/*
 * Returns current time from boot in nsecs. It's OK for this to wrap
 * around for now, as it's just a relative time stamp.
 */
unsigned long long sched_clock(void)
{
	if (!config.cs_base)
		return (unsigned long long)(jiffies - INITIAL_JIFFIES)
	                                     * (NSEC_PER_SEC / HZ);
	else
		return clocksource_cyc2ns(bcm_read_timer(NULL),
				bcm_clksrc.mult, bcm_clksrc.shift);
}
