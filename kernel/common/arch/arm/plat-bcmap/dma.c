/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/plat-bcmap/dma.c
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

/**
*
*   @file   dma.c
*   @brief  This file implements DMA Driver
*
*****************************************************************************/

/**
*   @defgroup   DMADRVGroup   DMA Driver API
*   @brief      This group defines the DMA Controller Driver APIs
*
*****************************************************************************/

/**
 * DMA interface functions
 */
#include <linux/module.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>

#include <mach/hardware.h>
#include <plat/dma.h>
#include <plat/cpu.h>
#include <linux/clk.h>

#define DBG_ERROR	0x01
#define DBG_INFO	0x02
#define DBG_TRACE	0x04
#define DBG_TRACE2	0x08
#define DBG_DATA	0x10
#define DBG_DATA2	0x20
#define DBG_DEFAULT_LEVEL	DBG_ERROR
#define DMA_DBG(level, x) { if (level & g_dbg_level) printk x; }

#define INT_STATUS              0x0000
#define INT_TC_STATUS           0x0004
#define INT_TC_CLEAR            0x0008
#define INT_ERROR_STATUS        0x000C
#define INT_ERROR_CLEAR         0x0010
#define RAW_INT_TC_STATUS       0x0014
#define RAW_INT_ERROR_STATUS    0x0018
#define ENABLED_CHANNELS        0x001C
#define SW_BURST_REQUEST        0x0020
#define SW_SINGLE_REQUEST       0x0024
#define SW_LAST_BURST_REQUEST   0x0028
#define SW_LAST_SINGLE_REQUEST  0x002C
#define CONFIG                  0x0030
#define SYNC                    0x0034

/**
 * Macros which allow per channel access based on channel
 */
#define CHAN_SRC_ADDR(chan)    (0x0100 + (0x20 * chan))
#define CHAN_DEST_ADDR(chan)   (0x0104 + (0x20 * chan))
#define CHAN_LINK(chan)        (0x0108 + (0x20 * chan))
#define CHAN_CONTROL(chan)     (0x010C + (0x20 * chan))
#define CHAN_CONFIG(chan)      (0x0110 + (0x20 * chan))

struct dma_chan {
	int lock;		/* lock flag */
	const char name[64];	/* name of peripheral or memory */
	 irqreturn_t(*handler) (void *);	/* handler */
	void *dev_id;		/* argument for handler */
	int called;		/* No. of times interrupt called */
	int dma_error;		/* DMA errors count */
};

static struct dma_chan dma_state[DMA_MAX_CHANNELS];
static void __iomem *dma_base_addr;
static irqreturn_t dma_isr(int irq, void *dev_id);
static int g_dbg_level = DBG_DEFAULT_LEVEL;
/**
 * We need to disable DMAC when all channels go inactive.
 * This is needed in order to enter pedestal mode
 */
static int dma_chan_active[DMA_MAX_CHANNELS];
static int g_dmachan_refcount;
static DEFINE_SPINLOCK(gDmaRefLock);


/** @addtogroup DMADRVGroup
    @{
*/


/**
* @brief 	Disable dma interrupts for a channel
*
* @param chan	channel number
*/
static inline void dma_disable_intr(int chan)
{
	uint32_t val = readl(dma_base_addr + CHAN_CONFIG(chan));
	val &= ~REG_DMA_CHAN_CTL_TC_INT_ENABLE;
	writel(val, dma_base_addr + CHAN_CONFIG(chan));
	writel((1 << (chan)), dma_base_addr + INT_TC_CLEAR);
	writel((1 << (chan)), dma_base_addr + INT_ERROR_CLEAR);
}

/**
* @brief 	Enable DMA interrupts for a channel
*
* @param chan 	channel number
*/
static inline void dma_enable_intr(int chan)
{
	uint32_t val = readl(dma_base_addr + CHAN_CONFIG(chan));
	val |= REG_DMA_CHAN_CTL_TC_INT_ENABLE;
	writel(val, dma_base_addr + CHAN_CONFIG(chan));
}

/**
* @brief 	Diable DMAC core
*/
static inline void dma_disable_core(void)
{
	uint32_t val = readl(dma_base_addr + CONFIG);
	val &= ~REG_DMA_CONFIG_ENABLED;
	writel(val, dma_base_addr + CONFIG);
}

/**
* @brief 	Enable DMAC Core
*/
static inline void dma_enable_core(void)
{
	uint32_t val = readl(dma_base_addr + CONFIG);
	val = (REG_DMA_CONFIG_ENABLED | REG_DMA_CONFIG_LITTLE_ENDIAN);
	writel(val, dma_base_addr + CONFIG);
}

/**
* @brief Mark a channel Active, Enable DMAC if not enabled
*
* @param channel	channel number
*/
static void dma_mark_chan_active(unsigned int channel)
{
	unsigned long flags;

	spin_lock_irqsave(&gDmaRefLock, flags);

	if (dma_chan_active[channel] == 0) {
		dma_chan_active[channel] = 1;
		g_dmachan_refcount++;

		if (g_dmachan_refcount == 1)
			dma_enable_core();
	}
	spin_unlock_irqrestore(&gDmaRefLock, flags);
}

/**
* @brief Mark a channel Inactive, Disable DMAC if no active channels
*
* @param channel
*/
static void dma_mark_chan_inactive(unsigned int channel)
{

	unsigned long flags;

	spin_lock_irqsave(&gDmaRefLock, flags);

	if (dma_chan_active[channel] == 1) {
		dma_chan_active[channel] = 0;
		g_dmachan_refcount--;

		if (g_dmachan_refcount == 0)
			dma_disable_core();
	}

	spin_unlock_irqrestore(&gDmaRefLock, flags);
}


/**
* @brief 	Get the Channel enable status
*
* @param chan 	channel number
*
* @return
*/
static inline int dma_active(int chan)
{
	return readl(dma_base_addr + ENABLED_CHANNELS) & (1 << chan);
}


/**
* @brief 	Initialize the DMA channel
*
* @param chan	channel number
*/
void dma_init_chan(int chan)
{
	DMA_DBG(DBG_TRACE, ("dma_init_chan %d\n", chan));
	writel(0, dma_base_addr + CHAN_CONTROL(chan));
	writel(0, dma_base_addr + CHAN_CONFIG(chan));
	writel((1 << (chan)), dma_base_addr + INT_ERROR_CLEAR);
	writel((1 << (chan)), dma_base_addr + INT_TC_CLEAR);
	dma_mark_chan_inactive(chan);

}


/**
* @brief	Configure the DMA channel
*
* @param chan		channel number
* @param srcaddr	source address
* @param dstaddr	destination address
* @param link		linked list Item (LLI)
* @param ctrl		channel control reg value
* @param cfg		channel configuration reg value
*/
void dma_setup_chan(int chan, int srcaddr, int dstaddr, int link, int ctrl,
		    int cfg)
{
	if ((cfg & REG_DMA_CHAN_CFG_ENABLE))
		dma_mark_chan_active(chan);
	else
		dma_mark_chan_inactive(chan);

	writel(srcaddr, dma_base_addr + CHAN_SRC_ADDR(chan));
	writel(dstaddr, dma_base_addr + CHAN_DEST_ADDR(chan));
	writel(link, dma_base_addr + CHAN_LINK(chan));
	writel(ctrl, dma_base_addr + CHAN_CONTROL(chan));
	writel(cfg, dma_base_addr + CHAN_CONFIG(chan));
}


/**
* @brief  Wait till dma transaction complete
*
* Busy wait if not using interrupt handler. Typically this is
* used for debugging.
*
* @param chan	channel number
*/
void dma_poll_chan(int chan)
{
	while (dma_active(chan))
		DMA_DBG(DBG_TRACE, ("%s busy 0x%x\n", __func__,
				    readl(dma_base_addr + ENABLED_CHANNELS)));
	writel((1 << (chan)), dma_base_addr + INT_TC_CLEAR);
	/* dma_disable_intr(chan); */
}


/**
* @brief  Initialize all channels to zero control/config
*/
static void dma_init_all_chan(void)
{
	int chan;
	writel(0, dma_base_addr + ENABLED_CHANNELS);
	for (chan = 0; chan < DMA_MAX_CHANNELS; chan++)
		dma_init_chan(chan);
}


/**
* @brief  DMA driver module init function
*
* @return 0 on success, else -EIO
*/
static int __init dma_init(void)
{
	int ret, irq;
	struct clk *clk;
	DMA_DBG(DBG_INFO, ("dma_init called\n"));

#if (cpu_is_bcm215xx())
	{
		dma_base_addr = io_p2v(BCM21553_DMAC_BASE);
		irq = IRQ_DMA;
	}
#elif (cpu_is_bcm2153() || cpu_is_bcm2157())
	{
		dma_base_addr = (void __iomem *)HW_DMA_BASE;
		irq = IRQ_DMA;
	}
#else
	{
		DMA_DBG(DBG_ERROR, ("FATAL: DMA not initialized\n"));
		return -EIO;
	}
#endif
	memset(&dma_state, 0, sizeof(dma_state));

	clk = clk_get(NULL, "dmac");
	clk_enable(clk);
	dma_enable_core();
	dma_init_all_chan();

	memset(dma_chan_active, 0, (sizeof(int) * DMA_MAX_CHANNELS));
	g_dmachan_refcount = 0;
	dma_disable_core();

	ret = request_irq(irq, dma_isr, IRQF_DISABLED, "dma", NULL);
	if (ret != 0) {
		DMA_DBG(DBG_ERROR, ("DMA - Failed to register ISR.\n"));
		return ret;
	}

	return 0;
}


/**
* @brief 	Request for specific DMA chaneel
*
* @param chan	channel number
* @param name	channel name
*
* @return	0 on success; else error number
*/
int dma_request_chan(unsigned int chan, const char *name)
{
	DMA_DBG(DBG_TRACE, ("%s %d %s\n", __func__, chan, name));
	if (chan >= DMA_MAX_CHANNELS)
		return -EINVAL;

	if (xchg(&dma_state[chan].lock, 1) != 0)
		return -EBUSY;

	strncpy((char *)dma_state[chan].name, name,
		sizeof(dma_state[chan].name));

	/* old flag was 0, now contains 1 to indicate busy */
	return 0;
}


/**
* @brief 	Request for the available DMA channel
*
* Allocates chaneel starting from the highest number or lowest priority,
* so that higher priority channels are requested using dma_request_chan()
*
* @param chanp  pointer to hold the chaneel number
* @param name
*
* @return 0 on success; -EBUSY if no channels available
*/
int dma_request_avail_chan(unsigned int *chanp, const char *name)
{
	int chan;
	unsigned long flags;

	DMA_DBG(DBG_TRACE, ("%s %s\n", __func__, name));
	/**
	 * Allocate channels starting with highest number (higher number
	 * has lower priority) so that devices that want to pick particular
	 * channel with dma_request_chan will still have highest priority
	 * channels available to choose from.
	 */
	local_irq_save(flags);
	for (chan = DMA_MAX_CHANNELS - 1; chan >= 0; chan--) {
		if (!dma_state[chan].lock) {
			dma_state[chan].lock = 1;
			local_irq_restore(flags);
			strncpy((char *)dma_state[chan].name, name,
				sizeof(dma_state[chan].name));
			*chanp = chan;
			DMA_DBG(DBG_TRACE, ("%s for %s returns chan %d\n",
					    __func__, name, chan));
			return 0;
		}
	}
	/* All channels busy. */
	local_irq_restore(flags);
	return -EBUSY;
}


/**
* @brief 	Free a DMA channel
*
* @param chan	channel number
*/
void dma_free_chan(unsigned int chan)
{
	DMA_DBG(DBG_TRACE, ("dma_free_chan %d\n", chan));
	if (chan >= DMA_MAX_CHANNELS) {
		DMA_DBG(DBG_INFO, ("Trying to free DMA channel %d\n", chan));
		return;
	}

	if (xchg(&dma_state[chan].lock, 0) == 0) {
		DMA_DBG(DBG_INFO,
			("Trying to free already free DMA channel %d\n", chan));
		return;
	}
}


/**
* @brief  Register DMA interrupt handler callback for a DMA channel
*
* @param chan		DMA channel number
* @param handler	pointer to interrupt handler(callback)
* @param dev_id		private data, argument to interrupt handler
*
* @return 	always return 0
*/
int dma_request_irq(int chan, irqreturn_t(*handler) (void *), void *dev_id)
{
	unsigned long flags;

	if (dma_state[chan].handler != NULL) {
		DMA_DBG(DBG_INFO,
			("DMA - ISR %d already registered, overriding ISR.\n",
			 chan));
	}

	local_irq_save(flags);
	/*
	 * Clear any pending interrupts and register interrupt handler.
	 * Do not call dma_reset_chan() here since user likely will have
	 * setup the control/config registers just before enabling interrupts.
	 */
	dma_disable_intr(chan);
	dma_state[chan].handler = handler;	/* Register ISR */
	dma_state[chan].dev_id = dev_id;
	dma_state[chan].called = 0;
	local_irq_restore(flags);

	return 0;
}


/**
* @brief  De-register interrupt handler callback for a DMA channel
*
* @param chan	DMA channel number
*
* @return	0 on success; else  -EINVAL
*/
int dma_free_irq(int chan)
{
	unsigned long flags;
	int handlers;
	int i;

	if (dma_state[chan].handler == NULL) {
		DMA_DBG(DBG_INFO,
			("DMA - ISR %d not registered, cannot free.\n", chan));
		return -EINVAL;
	}

	local_irq_save(flags);

	/* Clear any pending interrupts */
	dma_disable_intr(chan);

	/* UnRegister ISR */
	dma_state[chan].handler = NULL;
	dma_state[chan].dev_id = NULL;

	handlers = 0;
	for (i = 0; i < DMA_MAX_CHANNELS; i++) {
		if (dma_state[i].handler != NULL)
			handlers++;
	}
	local_irq_restore(flags);

	return 0;
}


/**
* @brief  Enable DMA interrupt for a DMA channel
*
* @param chan channel number
*
* @return	0 on success, else -EINVAL
*/
int dma_enable_irq(int chan)
{
	unsigned long flags;
	if (dma_state[chan].handler == NULL) {
		DMA_DBG(DBG_INFO,
			("DMA - ISR %d not registered, cannot enable.\n",
			 chan));
		return -EINVAL;
	}
	local_irq_save(flags);
	dma_enable_intr(chan);
	local_irq_restore(flags);
	return 0;
}


/**
* @brief Disable DMA interrupt for a DMA channel
*
* @param chan channel number
*
* @return
*/
int dma_disable_irq(int chan)
{
	unsigned long flags;
	if (dma_state[chan].handler == NULL) {
		DMA_DBG(DBG_INFO,
			("DMA - ISR %d not registered, cannot disable.\n",
			 chan));
		return -EINVAL;
	}
	local_irq_save(flags);
	dma_disable_intr(chan);
	local_irq_restore(flags);
	return 0;
}


/**
* @brief DMA Interrupt Handler, calls channel specific handler functions
*
* @param irq		irq number
* @param dev_id		private data argument
*
* @return 			IRQ_HANDLED
*/
static irqreturn_t dma_isr(int irq, void *dev_id)
{
	int chan;

	(void)irq;
	(void)dev_id;

	DMA_DBG(DBG_TRACE2, ("DMA - dma_isr()\n"));

	/* loop through 4 DMA chans
	 * calling registered handlers for each active ISR
	 */
	for (chan = 0; chan < DMA_MAX_CHANNELS; chan++) {
		if (readl(dma_base_addr + INT_STATUS) & (1 << (chan))) {
			if (readl(dma_base_addr + INT_ERROR_STATUS)
			    & (1 << (chan))) {
				dma_state[chan].dma_error++;
				DMA_DBG(DBG_ERROR,
					("Error on dma chan %d\n", chan));
			}
			/* clear the channel */
			dma_init_chan(chan);
			if (dma_state[chan].handler) {
				(*(dma_state[chan].handler)) (dma_state
							      [chan].dev_id);
				dma_state[chan].called++;
			}
		}
	}
	return IRQ_HANDLED;
}


/**
* @brief Get DMA source address for a channel
*
* @param chan channel number
*
* @return src address, 32-bit value
*/
u32 dma_get_src_addr(int chan)
{
	return readl(dma_base_addr + CHAN_SRC_ADDR(chan));
}


/**
* @brief Get DMA destination address for a channel
*
* @param chan  channel number
*
* @return dest address, 32-bit value
*/
u32 dma_get_dest_addr(int chan)
{
	return readl(dma_base_addr + CHAN_DEST_ADDR(chan));
}


/**
* @brief Get DMA LLI address for a channel
*
* @param chan channel number
*
* @return link address, 32-bit value
*/
u32 dma_get_link_addr(int chan)
{
	return readl(dma_base_addr + CHAN_LINK(chan));
}


/**
* @brief Enable DMA transfer for a channel
*
* @param chan channel number
*/
void dma_enable_chan(int chan)
{
	uint32_t val = readl(dma_base_addr + CHAN_CONFIG(chan));
	val |= REG_DMA_CONFIG_ENABLED;
	writel(val, dma_base_addr + CHAN_CONFIG(chan));
}


/**
* @brief Disable DMA transfer for a channel
*
* @param chan channel number
*/
void dma_disable_chan(int chan)
{
	uint32_t val = readl(dma_base_addr + CHAN_CONFIG(chan));
	val &= ~REG_DMA_CONFIG_ENABLED;
	writel(val, dma_base_addr + CHAN_CONFIG(chan));
}

/** @} */

arch_initcall(dma_init);

EXPORT_SYMBOL(dma_init_chan);
EXPORT_SYMBOL(dma_setup_chan);
EXPORT_SYMBOL(dma_poll_chan);
EXPORT_SYMBOL(dma_request_chan);
EXPORT_SYMBOL(dma_request_avail_chan);
EXPORT_SYMBOL(dma_free_chan);
EXPORT_SYMBOL(dma_request_irq);
EXPORT_SYMBOL(dma_free_irq);
EXPORT_SYMBOL(dma_enable_irq);
EXPORT_SYMBOL(dma_disable_irq);
EXPORT_SYMBOL(dma_get_dest_addr);
EXPORT_SYMBOL(dma_disable_chan);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("DMA Control Driver");
