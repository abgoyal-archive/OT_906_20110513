/*****************************************************************************
*  Copyright 2001 - 2007 Broadcom Corporation.  All rights reserved.
*
*  Unless you and Broadcom execute a separate written software license
*  agreement governing use of this software, this software is licensed to you
*  under the terms of the GNU General Public License version 2, available at
*  http://www.gnu.org/copyleft/gpl.html (the "GPL").
*
*  Notwithstanding the above, under no circumstances may you combine this
*  software in any way with any other Broadcom software provided under a
*  license other than the GPL, without Broadcom's express prior written
*  consent.
*
*****************************************************************************/

/****************************************************************************
*
*  lcd_common.c
*
*  PURPOSE:
*    This implements the code to use a Broadcom LCD host interface.
*
*  NOTES:
*    Uses device minor number to select panel:  0==main 1==sub
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/string.h>
#include <linux/module.h>

#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/interrupt.h>

#include <linux/version.h>
#include <linux/types.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/kernel_stat.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <linux/dma-mapping.h>
#ifdef CONFIG_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif
#include <linux/suspend.h>
#include <plat/dma.h>

#include <asm/byteorder.h>
#include <asm/irq.h>
#include <mach/reg_lcd.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/kthread.h>

#include <linux/broadcom/regaccess.h>
#if defined(CONFIG_BCM_IDLE_PROFILER_SUPPORT)
#include <linux/broadcom/idle_prof.h>
#endif
#include <linux/broadcom/cpu_sleep.h>
#include <asm/mach/irq.h>
#include <asm/io.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/hw.h>
#include <linux/broadcom/lcd.h>
#include <linux/broadcom/PowerManager.h>
#include <linux/broadcom/lcd_backlight.h>
#include <plat/syscfg.h>

#include <cfg_global.h>

#include <linux/broadcom/lcd_common.h>
#ifdef CONFIG_BRCM_KPANIC_UI_IND
#include <linux/broadcom/dump_start_qvga.h>
#include <linux/broadcom/dump_end_qvga.h>
#endif


/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */

static char gBanner[] __initdata = KERN_INFO "lcd: Broadcom LCD Driver: 0.01";

/* globals to: communicate with the update thread */
/*   control access to LCD registers */
/*  manage DMA channel */
static int gInitialized;
static long gUpdateThreadPid;
static struct task_struct *gUpdatetask;
static struct completion gUpdateExited;
static struct semaphore gdirtyRectSema;
static struct semaphore gUpdateSem;
static struct clk *gClk;
#if USE_DMA
static struct semaphore gDmaSem;
#endif

#ifdef CONFIG_BCM_LCD_TE_ENABLE
static struct semaphore gTESem;
#endif


/* forward func declarations */
static void lcd_exit(void);
static int lcd_init(void);
static int lcd_pwr_on(void);
static void lcd_init_all(void);
static void lcd_pwr_off_controller(void);
static int lcd_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg);
static int lcd_mmap(struct file *file, struct vm_area_struct *vma);
static int lcd_open(struct inode *inode, struct file *file);
static int lcd_release(struct inode *inode, struct file *file);
static void lcd_reset_controller(int level);
static void lcd_enable_ce(bool enable);

void lcd_display_test(LCD_dev_info_t *dev);
void lcd_display_rect(LCD_dev_info_t *dev, LCD_Rect_t *r);

#if USE_DMA
static void lcd_start_dma(LCD_dev_info_t *dev, LCD_DirtyRect_t *dirtyRect);
static irqreturn_t lcd_dma_isr(void *unused);
static int lcd_dmaCreateList(LCD_dev_info_t *dev);

static int gDmaChannel;
static DMA_Restore_t gDmaSavedLli;
#endif

#ifdef CONFIG_BRCM_KPANIC_UI_IND
static void lcd_pan_color(LcdColor_t color);
static void lcd_pan_screen(unsigned short *p_src, int length);
void lcd_draw_kpanic_dump(int dumpPic);
#endif


/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
***************************************************************************/

struct file_operations lcd_fops = {
	.owner = THIS_MODULE,
	.ioctl = lcd_ioctl,
	.mmap = lcd_mmap,
	.open = lcd_open,
	.release = lcd_release,
};

/* ---- Functions -------------------------------------------------------- */

static void lcd_dump_regs(void)
{
	pr_info("LCDC_CMD : 0x%08x\n", REG_LCD_CMDR);
	pr_info("LCDC_DATA: 0x%08x\n", REG_LCD_DATR);
	pr_info("LCDC_WTR : 0x%08x\n", REG_LCD_WTR);
	pr_info("LCDC_RTR : 0x%08x\n", REG_LCD_RTR);
	pr_info("LCDC_CR  : 0x%08x\n", REG_LCD_CR);
	pr_info("LCDC_SR  : 0x%08x\n", REG_LCD_SR);

}

void lcd_select_panel(LCD_panel_t panel)
{
	if (panel == LCD_sub_panel)
		REG_LCD_CR |= REG_LCD_CR_SELCD;
	else
		REG_LCD_CR &= ~REG_LCD_CR_SELCD;

	/* LCD_DEBUG("panel=%d REG_LCD_CR=0x%08X\n", panel, REG_LCD_CR); */

}

void lcd_write_cmd(uint32_t cmd)
{
	while (REG_LCD_SR & REG_LCD_SR_FIFO_FULL)
		udelay(0);

	REG_LCD_CMDR = cmd;
}

void lcd_write_data(uint32_t data)
{
	while (REG_LCD_SR & REG_LCD_SR_FIFO_FULL)
		udelay(0);

	REG_LCD_DATR = data;
}

void lcd_write_param(uint32_t cmd)
{
	extern LCD_Bus_t LCD_Bus;

	if (LCD_Bus == LCD_18BIT) {
		REG_LCD_CR &= ~REG_LCD_CR_ENABLE_16_TO_18_EXPANSION;
		WRITE_LCD_DATA(cmd);
		udelay(0);
		REG_LCD_CR |= REG_LCD_CR_ENABLE_16_TO_18_EXPANSION;
	} else
		WRITE_LCD_DATA(cmd);
}

/****************************************************************************
*
*  lcd_dirty_rows
*
*   Marks the indicated rows as dirty and arranges for them to be transferred
*   to the LCD.
*
***************************************************************************/

static void
lcd_dev_dirty_rows(LCD_dev_info_t *dev, LCD_DirtyRows_t *dirtyRows)
{

	/*  LCD_DEBUG("top = %u,  bottom = %u\n", dirtyRows->top, dirtyRows->bottom); */
	if ((dirtyRows->top > dirtyRows->bottom)
	    || ((dirtyRows->bottom - dirtyRows->top) >= dev->height)) {
		LCD_DEBUG("invalid dirty-rows params - ignoring\n");
		LCD_DEBUG("top = %u,  bottom = %u\n",
			  dirtyRows->top, dirtyRows->bottom);

		return;
	}
	if(down_interruptible(&gdirtyRectSema) != 0)
		return;
	/* Mark dirty rows */
	if (dirtyRows->top < dev->dirty_rect.top)
		dev->dirty_rect.top = dirtyRows->top;

	if (dirtyRows->bottom > dev->dirty_rect.bottom)
		dev->dirty_rect.bottom = dirtyRows->bottom;

	dev->dirty_rect.left = 0;
	dev->dirty_rect.right = dev->width - 1;

	up(&gUpdateSem);
/*   LCD_PUTS("done"); */

}				/* lcd_dirty_rows */

void lcd_dirty_rows(LCD_DirtyRows_t *dirtyRows)
{
	lcd_dev_dirty_rows(&LCD_device[LCD_main_panel], dirtyRows);
}

/****************************************************************************
*
*  lcd_dirty_rect
*
*   Marks the indicated rect as dirty and arranges for them to be transferred
*   to the LCD.
*
***************************************************************************/

static void
lcd_dev_dirty_rect(LCD_dev_info_t *dev, LCD_DirtyRect_t *dirtyRect)
{

	if ((dirtyRect->top > dirtyRect->bottom)
		|| ((dirtyRect->bottom - dirtyRect->top) >= dev->height)
		|| (dirtyRect->left > dirtyRect->right)
		|| ((dirtyRect->right - dirtyRect->left) >= dev->width)) {
		LCD_DEBUG("invalid dirty-rect params - ignoring\n");
		LCD_DEBUG("left = %u, top = %u, right = %u, bottom = %u\n",
				dirtyRect->left, dirtyRect->top,
				dirtyRect->right, dirtyRect->bottom);
		return;
	}

	if(down_interruptible(&gdirtyRectSema) != 0)
		return;
	/* Mark dirty rows */
	if (dirtyRect->top < dev->dirty_rect.top)
		dev->dirty_rect.top = dirtyRect->top;

	if (dirtyRect->bottom > dev->dirty_rect.bottom)
		dev->dirty_rect.bottom = dirtyRect->bottom;

	dev->dirty_rect.left = dirtyRect->left;
	dev->dirty_rect.right = dirtyRect->right;

	up(&gUpdateSem);
	/* LCD_PUTS("done"); */

}

void
lcd_dirty_rect(LCD_DirtyRect_t *dirtyRect)
{
	lcd_dev_dirty_rect(&LCD_device[LCD_main_panel], dirtyRect);
}

#if !USE_DMA

/* only to be called by lcd_update_thread() below!! */
#ifndef USE_PARTIAL_UPDATE
/****************************************************************************
*
*  lcd_send_data
*
*  Update LCD in non-DMA mode ( for testing purpose )
*  Currently supports 8-bit and 16-bit bus width.
*
***************************************************************************/
static void lcd_send_data(uint16_t *p, int len)
{
	int i;

	if (REG_LCD_CR & REG_LCD_CR_ENABLE_8_BIT_INTF) { /* 8 bit bus */
		for (i = 0; i < len; i++) {
			WRITE_LCD_DATA(p[i] >> 8);
			WRITE_LCD_DATA(p[i]);

			/* yield to other threads every N rows */
			if ((i & 0xF) == 0xF)
				yield();
		}
	} else {
		for (i = 0; i < len; i++) {
			WRITE_LCD_DATA(p[i]);

			/* yield to other threads every X rows */
			if ((i & 0xF) == 0xF)
				yield();
		}
	}
}
#else

/****************************************************************************
*
*  lcd_update_data
*
*  Update LCD in non-DMA mode within dirty region ( for testing purpose ).
*  Currently supports 8-bit and 16-bit bus width.
*
***************************************************************************/
static void
lcd_update_data(LCD_dev_info_t *dev, LCD_DirtyRect_t *dirtyRect)
{
	int i, j, stride;
	u32 source;
	uint16_t *p;

	stride = dev->width * dev->bits_per_pixel / 8;
	source = (u32)dev->frame_buffer.virtPtr + stride * dirtyRect->top +
			dirtyRect->left * dev->bits_per_pixel / 8;

	if (REG_LCD_CR & REG_LCD_CR_ENABLE_8_BIT_INTF) { /* 8 bit bus */

		for (j = dirtyRect->top; j <= dirtyRect->bottom; j++) {
			p = (uint16_t *)(source & ~1);

			for (i = dirtyRect->left;
				i <= dirtyRect->right; i++, p++) {
				WRITE_LCD_DATA(*p >> 8);
				WRITE_LCD_DATA(*p);
			}

			source += stride;

			/* yield to other threads every N rows */
			if ((j & 0xF) == 0xF)
				yield();
		}

	} else {

		for (j = dirtyRect->top; j <= dirtyRect->bottom; j++) {
			p = (uint16_t *)(source & ~1);

			for (i = dirtyRect->left;
				i <= dirtyRect->right; i++, p++)
				WRITE_LCD_DATA(*p);

			source += stride;

			/* yield to other threads every N rows */
			if ((j & 0xF) == 0xF)
				yield();
		}
	}
}
#endif

#else

/****************************************************************************
*
*  lcd_update_column
*
*  Update one column of LCD in DMA mode within dirty region.
*  Currently supports 8-bit and 16-bit bus width.
*
***************************************************************************/
static void
lcd_update_column(LCD_dev_info_t *dev,
			LCD_DirtyRect_t *dirtyRect, unsigned int column)
{
	int i, stride;
	u32 source;
	uint16_t *p;

	stride = dev->width * dev->bits_per_pixel / 8;
	source = (u32)dev->frame_buffer.virtPtr + stride * dirtyRect->top +
			column * dev->bits_per_pixel / 8;

	if (REG_LCD_CR & REG_LCD_CR_ENABLE_8_BIT_INTF) { /* 8 bit bus */

		for (i = dirtyRect->top; i <= dirtyRect->bottom; i++) {
			p = (uint16_t *)source;
			WRITE_LCD_DATA(*p >> 8);
			WRITE_LCD_DATA(*p);
			source += stride;
		}
	} else {

		for (i = dirtyRect->top; i <= dirtyRect->bottom; i++) {
			p = (uint16_t *)source;
			WRITE_LCD_DATA(*p);
			source += stride;
		}
	}
}

#endif /* !USE_DMA */

/****************************************************************************
*
*  lcd_update_thread
*
*   Worker thread to transfer data to the LCD.
*
***************************************************************************/
static int lcd_update_thread(void *data)
{
#ifdef CONFIG_HAS_WAKELOCK
	static struct wake_lock lcdfb_wake_lock;
#endif
	/* This thread doesn't need any user-level access, * so get rid of all
	   our resources */
	daemonize("lcdUpdate");
	/* LCD_PUTS("enter"); */
#ifdef CONFIG_HAS_WAKELOCK
	/* init lcdfb's wake lock */
	wake_lock_init(&lcdfb_wake_lock, WAKE_LOCK_SUSPEND, "lcdfb_wake_lock");
#endif
	/* Run until signal received */
	while (down_interruptible(&gUpdateSem) == 0) {
		int d;
		LCD_DirtyRect_t dirtyRect;
#ifdef CONFIG_HAS_WAKELOCK
		/* LCD_PUTS("wakeup"); */
		wake_lock(&lcdfb_wake_lock);
#endif
		/* check all panels here since they share registers */
		for (d = 0; d < LCD_num_panels; d++) {
			LCD_dev_info_t *dev = &LCD_device[d];

			/* Get dirty rows and reset global dirty row list */
			dirtyRect = dev->dirty_rect;
			dev->dirty_rect.top = (dev->height * NUM_DMA_BUFFERS);
			dev->dirty_rect.bottom = 0;

			up(&gdirtyRectSema);

			/* Transfer the rows to LCD */
			/* (actually, transfer all the rows as there does */
			/* not seem to be any LCD command for */
			/* a partial update) */

			if (gInitialized &&
				(dirtyRect.bottom >= dirtyRect.top)) {
#if USE_DMA
				int doDMA = 1;
#endif
				/* LCD_PUTS("refresh"); */

				/* refresh the panel */
				lcd_select_panel((LCD_panel_t) d);
				dev->row_start = dirtyRect.top % dev->height;
				dev->row_end = dirtyRect.bottom % dev->height;

#if USE_DMA
#ifdef CONFIG_BCM_LCD_TE_ENABLE
				sema_init(&gTESem,0);
				if (down_timeout(&gTESem, msecs_to_jiffies(35)))
					pr_info("WARNING: display te_irq miss \r\n");
#endif

				if (dirtyRect.left & 1) {
					dev->col_start = dirtyRect.left;
					dev->col_end = dirtyRect.left;
					lcd_setup_for_data(dev);
					lcd_update_column(dev,
						&dirtyRect,
						dirtyRect.left);
					dirtyRect.left += 1;
				}

				if (dirtyRect.right < dirtyRect.left) {
					doDMA = 0;
				} else {
					if (!((dirtyRect.right -
						dirtyRect.left) & 1)) {
						dev->col_start =
							dirtyRect.right;
						dev->col_end =
							dirtyRect.right;
						lcd_setup_for_data(dev);
						lcd_update_column(dev,
							&dirtyRect,
							dirtyRect.right);

						if (dirtyRect.right)
							dirtyRect.right -= 1;
						else
							doDMA = 0;
					}

					if (dirtyRect.right < dirtyRect.left)
						doDMA = 0;
				}

				if (doDMA) {
					dev->col_start = dirtyRect.left;
					dev->col_end = dirtyRect.right;
					lcd_setup_for_data(dev);

                                        /* ensure previous commands to */
				        /* LCD controller are complete */
				        while (!(REG_LCD_SR & REG_LCD_SR_FIFO_EMPTY))
					     udelay(0);

					/* start the DMA transfer */
					lcd_start_dma(dev, &dirtyRect);
				} else {
					/* no DMA processing to be done, */
					/* release the semaphore */
					up(&gDmaSem);
				}
#else /* USE_DMA */
/* for testing purpuse */
				dev->col_start = dirtyRect.left;
				dev->col_end = dirtyRect.right;

				lcd_setup_for_data(dev);

				/* ensure previous commands to */
				/* LCD controller are complete */
				while (!(REG_LCD_SR & REG_LCD_SR_FIFO_EMPTY))
					udelay(0);

#ifndef USE_PARTIAL_UPDATE
/* use simple API */
				lcd_send_data(dev->frame_buffer.virtPtr,
					      dev->width * dev->height);
#else
/* use API with dirty region for partial update */
				lcd_update_data(dev, &dirtyRect);
#endif

#endif /* USE_DMA */
			} else {
#if USE_DMA
				/* no DMA processing to be done, */
				/* release the semaphore */
				up(&gDmaSem);
#endif
			}
#if USE_DMA
			/* grab this here so we only try to */
			/* DMA to one panel at a time */
			/* we need to wait until the DMA transaction */
			/* is done to unlock wake lock */
			if (down_interruptible(&gDmaSem) != 0)
				break;
#endif
		}
#ifdef CONFIG_HAS_WAKELOCK
		/* LCD_PUTS("wait"); */
		wake_unlock(&lcdfb_wake_lock);
#endif
	}
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&lcdfb_wake_lock);
#endif
	complete_and_exit(&gUpdateExited, 0);
}				/* lcd_update_thread */

/****************************************************************************
*
*  lcd_enable_sub_backlight
*
*   Sets the LCD_BL_EN_M signal to 'level'.
*
***************************************************************************/

/* static */
void lcd_enable_sub_backlight(int level)
{
	LCD_DEBUG("%d\n", level);

	/* TODO - no separate sub-panel backlight control? */

}				/* lcd_enable_sub_backlight */

/****************************************************************************
*
*  lcd_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

/* static */
void __exit lcd_exit(void)
{
	LCD_PUTS("enter");

	lcd_backlight_enable(0);
	clk_disable(gClk);

	if (gUpdateThreadPid >= 0) {
		kill_proc_info(SIGTERM, SEND_SIG_PRIV, gUpdateThreadPid);
		wait_for_completion(&gUpdateExited);
	}

	gpio_free(GPIO_LCD_RESET);
	gpio_free(GPIO_LCD_BACKLIGHT);

}				/* lcd_exit */

static int lcd_alloc_fb(LCD_dev_info_t *dev)
{
	if (dev->frame_buffer.virtPtr != NULL)
		return 0;

	/* dma_alloc_writecombine allocates uncached, buffered memory, */
	/* that is io_remappable */
	/* dev->frame_buffer.sizeInBytes = dev->width * dev->height *
					 dev->bits_per_pixel / 8; */

	/* DBFrame : */
	dev->frame_buffer.sizeInBytes = NUM_DMA_BUFFERS * dev->width *
					dev->height * dev->bits_per_pixel / 8;

	dev->frame_buffer.virtPtr = dma_alloc_writecombine(NULL,
							   dev->frame_buffer.
							   sizeInBytes,
							   &dev->frame_buffer.
							   physPtr,
							   GFP_KERNEL);

	pr_info
	    ("[lcdcom] lcd_alloc_fb size=%#x virtPtr = 0x%lx, physPtr = 0x%lx\
		\r\n", dev->frame_buffer.sizeInBytes,
		(long)dev->frame_buffer.virtPtr,
		(long)dev->frame_buffer.physPtr);

	if (dev->frame_buffer.virtPtr == NULL)
		return -ENOMEM;

	if ((dev->frame_buffer.physPtr & ~PAGE_MASK) != 0) {
		panic("lcd_init: We didn't get a page aligned buffer");
		return -ENOMEM;
	}

	memset(dev->frame_buffer.virtPtr, 0, dev->frame_buffer.sizeInBytes);
	return 0;
}

void lcd_poweroff_panels(void);
#ifdef CONFIG_EARLYSUSPEND

/*Ladios Add for early suspend start*/
static void lcd_early_suspend(struct early_suspend *h)
{
	lcd_set_power(0);
}

static void lcd_late_resume(struct early_suspend *h)
{
	lcd_set_power(1);
}

static struct early_suspend lcd_early_suspend_desc = {
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
	.suspend = lcd_early_suspend,
	.resume = lcd_late_resume,
};
#endif
/*Ladios Add for early suspend end*/

static int lcd_pm_update(PM_CompPowerLevel compPowerLevel,
			 PM_PowerLevel sysPowerLevel);

/****************************************************************************
*
*  lcd_set_power
*
*     Called by backlight driver to turn power on/off
*
***************************************************************************/
void lcd_set_power(int onOff)
{
	/* pr_info("\r\n============[lcd_set_power] onOff=0x%x",onOff); */

	lcd_pm_update(onOff ? PM_COMP_PWR_ON : PM_COMP_PWR_OFF, 0);
}

/****************************************************************************
*
*  lcd_pm_update
*
*     Called by power manager to update component power level
*
***************************************************************************/
static
int lcd_pm_update(PM_CompPowerLevel compPowerLevel,
			PM_PowerLevel sysPowerLevel)
{
	static PM_CompPowerLevel powerLevel = PM_COMP_PWR_ON;
	/* Nothing to do if power level did not change */
	if (compPowerLevel == powerLevel)
		return 0;

	/* Save new power level */
	powerLevel = compPowerLevel;
	switch (powerLevel) {
	case PM_COMP_PWR_OFF:
	case PM_COMP_PWR_STANDBY:
		{
			/* pr_info("\r\n=========[PM_COMP_PWR_OFF]\r\n"); */
			lcd_enable_ce(true);
			lcd_poweroff_panels();
			lcd_enable_ce(false);
			clk_disable(gClk);
			break;
		}

	case PM_COMP_PWR_ON:
		{
			/* pr_info("\r\n=========[PM_COMP_PWR_ON]\r\n"); */
			clk_enable(gClk);
			lcd_enable_ce(true);
			lcd_init_panels();
			lcd_enable_ce(false);
			break;
		}
	default:
		break;
	}

	return 0;
}

#ifdef CONFIG_BCM_LCD_TE_ENABLE	

static irqreturn_t lcd_te_irq(int irq, void *handle)
{

	up(&gTESem);
	return IRQ_HANDLED;
}
#endif



/****************************************************************************
*
*  lcd_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

static int __init lcd_init(void)
{
	int rc;
	int i;
#ifdef CONFIG_BCM_LCD_TE_ENABLE	
	int te_irq;
#endif
	pr_info("************* %s\n", __FUNCTION__);
	LCD_PUTS("enter");
	pr_info("%s for %s\n", gBanner, LCD_panel_name);

	/* Register our device with Linux */
	rc = register_chrdev(BCM_LCD_MAJOR, "lcd", &lcd_fops);
	if (rc < 0) {
		pr_warning("lcd: register_chrdev failed for major %d\n",
		       BCM_LCD_MAJOR);
		return rc;
	}
	/* Allocate memory for the framebuffers and DMA. */
	for (i = 0; i < LCD_num_panels; i++) {
		LCD_dev_info_t *dev = &LCD_device[i];

		rc = lcd_alloc_fb(dev);

		if (rc)
			return rc;

#if USE_DMA
		if (lcd_dmaCreateList(dev) != 0) {
			LCD_DEBUG("LCD DBG_ERROR - \
				Failed to create DMA linked list\n");
			return -ENOMEM;
		}
#endif
	}

	gClk = clk_get(NULL, "lcd");
	clk_enable(gClk);

#if USE_DMA
	/* request DMA channel and DMA irq, and enable dma irq */
	gDmaChannel = DESIRED_DMA_CHANNEL;

	if (dma_request_chan(gDmaChannel, "LCD") != 0) {
		/* couldn't get desired DMA channel, */
		/* take whatever is available */
		if (dma_request_avail_chan(&gDmaChannel, "LCD") != 0) {
			LCD_DEBUG
			    ("LCD DBG_ERROR - Failed to get DMA channel\n");
			return -EINVAL;
		}
	}

	if (dma_request_irq(gDmaChannel, lcd_dma_isr, 0) != 0) {
		LCD_DEBUG("LCD DBG_ERROR - Failed to get dma irq\n");
		return -EINVAL;
	}

	sema_init(&gDmaSem, 0);
#ifdef CONFIG_BCM_LCD_TE_ENABLE
	sema_init(&gTESem, 0);
#endif

	if (dma_enable_irq(gDmaChannel) != 0) {
		LCD_DEBUG("cannot enable DMA IRQ\n");
		return -EINVAL;
	}
#endif /* USE_DMA */

	/* Create update thread */
	sema_init(&gUpdateSem, 0);
	sema_init(&gdirtyRectSema, 1);
	init_completion(&gUpdateExited);

	gpio_request(GPIO_LCD_RESET, "LCD Reset");
	gpio_request(GPIO_LCD_BACKLIGHT, "LCD Backlight");
#ifdef CONFIG_BCM_LCD_TE_ENABLE	
        gpio_request(GPIO_LCD_FLM, "LCD FLM");
        te_irq = gpio_to_irq(GPIO_LCD_FLM);
        gpio_direction_input(GPIO_LCD_FLM);	
#endif
	lcd_pwr_on();

	/* launch thread */
	gUpdatetask = kthread_run(lcd_update_thread, NULL, "lcdUpdate");
	gUpdateThreadPid = gUpdatetask->pid;

#if 0
	/* Register our device with the Power Manager */
	rc = pm_register_component(PM_COMP_LCD, &lcd_pm_ops);
	if (rc < 0) {
		pr_info("lcd: failed to register with power manager\n");
		return rc;
	}
#endif


#ifdef CONFIG_BCM_LCD_TE_ENABLE

        rc = request_irq(te_irq, lcd_te_irq, IRQF_TRIGGER_FALLING,
		"LCD_FLM", NULL);
        if (rc < 0) {
        	pr_info("[TCL]:TE interrupt request fail \r\n");
        }	
#endif
#if CONFIG_EARLYSUSPEND
	register_early_suspend(&lcd_early_suspend_desc);
#endif
	gInitialized = 1;
	LCD_PUTS("done");
	return 0;
}

#if 0
/****************************************************************************
*
*  lcd_pm_update
*
*     Called by power manager to update component power level
*
***************************************************************************/
static
int lcd_pm_update(PM_CompPowerLevel compPowerLevel,
			PM_PowerLevel sysPowerLevel)
{
	static PM_CompPowerLevel powerLevel = PM_COMP_PWR_OFF;

	/* Nothing to do if power level did not change */
	if (compPowerLevel == powerLevel)
		return 0;

	/* Save new power level */
	powerLevel = compPowerLevel;
	switch (powerLevel) {
	case PM_COMP_PWR_OFF:
	case PM_COMP_PWR_STANDBY:
		lcd_pwr_off_controller();
		break;
	case PM_COMP_PWR_ON:
		lcd_pwr_on();
		break;
	}

	return 0;
}

#endif

/****************************************************************************
*
*  lcd_pwr_on
*
*     Power on controller
*
***************************************************************************/
static int lcd_pwr_on(void)
{
	LCD_PUTS("enter");
	if (!gInitialized) {
		/* first init IOCR registers for LCD I/O */
		board_sysconfig(SYSCFG_LCD, SYSCFG_INIT);
		msleep(100);

		/* Configure the GPIO pins */
		LCD_PUTS("gpio");
		gpio_direction_output(GPIO_LCD_RESET, 1);
		gpio_direction_output(GPIO_LCD_BACKLIGHT, 1);


		/* Initialize our side of the controller hardware */
		LCD_PUTS("R/WTR");
		REG_LCD_RTR = LCD_Read_Timing;
		REG_LCD_WTR = LCD_Write_Timing;

		/* put control register into a known state */
		/* - also selects main panel */
		REG_LCD_CR = 0;

		if (LCD_Bus == LCD_18BIT)
			REG_LCD_CR |= REG_LCD_CR_ENABLE_16_TO_18_EXPANSION;
	}
#ifndef CONFIG_BCM_LCD_SKIP_INIT
	/* Initialize controller only if Bootloader doesn't do */
	lcd_init_all();
#endif
	return 0;
}				/* lcd_pwr_on */

static void lcd_enable_ce(bool enable)
{
	if (true == enable)
		REG_LCD_CR |= REG_LCD_CR_ENABLE_16_TO_18_EXPANSION;
	else
		REG_LCD_CR &= ~REG_LCD_CR_ENABLE_16_TO_18_EXPANSION;
}

static void lcd_init_all(void)
{
	LCD_PUTS("enter");

	if (gInitialized)
		return;

	/* drop then raise the RESET line */
#ifndef CONFIG_ARCH_BCM2153
	lcd_reset_controller(0);
	msleep(100);
	lcd_reset_controller(1);
	msleep(10);
#endif

	lcd_init_panels();

}				/* lcd_init_all */

/****************************************************************************
*
*  lcd_pwr_off_controller
*
*   Power off the LCD controller.
*
***************************************************************************/

static void lcd_pwr_off_controller(void)
{
	LCD_PUTS("enter");

	/* TODO */

}				/* lcd_pwr_off_controller */

/****************************************************************************
*
*  lcd_ioctl - TODO - lots of stuff needs to be filled in
*
***************************************************************************/

static int
lcd_ioctl(struct inode *inode, struct file *file,
	  unsigned int cmd, unsigned long arg)
{
	int err = 0;
	LCD_dev_info_t *dev = (LCD_dev_info_t *) file->private_data;

	LCD_DEBUG("[lcdcom] lcdioctl: %d type: '%c' cmd: 0x%x\r\n", dev->panel,
		  _IOC_TYPE(cmd), _IOC_NR(cmd));

	switch (cmd) {
	case LCD_IOCTL_RESET:
		lcd_reset_controller((int)arg);
		break;

	case LCD_IOCTL_ENABLE_BACKLIGHT:
		lcd_backlight_enable((int)arg);
		break;

	case LCD_IOCTL_ENABLE_SUB_BACKLIGHT:
		lcd_enable_sub_backlight((int)arg);
		break;

	case LCD_IOCTL_ENABLE_CS:
		break;

	case LCD_IOCTL_SCOPE_TIMEOUT:
		break;

	case LCD_IOCTL_INIT:
		lcd_init_panels();
		break;

	case LCD_IOCTL_INIT_ALL:
		lcd_init_all();
		break;

	case LCD_IOCTL_SETUP:
		break;

	case LCD_IOCTL_HOLD:
		break;

	case LCD_IOCTL_PULSE:
		break;

	case LCD_IOCTL_REG:
		{
			LCD_Reg_t r;

			if (copy_from_user(&r, (LCD_Reg_t *) arg, sizeof(r)) !=
			    0) {
				return -EFAULT;
			}
			/* lcd_write_cmd(r.reg, r.val); */
			break;
		}

	case LCD_IOCTL_RECT:
		{
			LCD_Rect_t r;

			if (copy_from_user(&r, (LCD_Rect_t *) arg,
				sizeof(r)) != 0) {
				return -EFAULT;
			}

			lcd_display_rect(dev, &r);

			break;
		}

	case LCD_IOCTL_COLOR_TEST:
		break;

	case LCD_IOCTL_DIRTY_ROWS:
		{
			LCD_DirtyRows_t dirtyRows;

			if (copy_from_user(&dirtyRows, (LCD_DirtyRows_t *) arg,
					   sizeof dirtyRows) != 0) {
				return -EFAULT;
			}

			lcd_dev_dirty_rows(dev, &dirtyRows);
			break;
		}

	case LCD_IOCTL_PRINT_REGS:
		lcd_dump_regs();
		break;

	case LCD_IOCTL_PRINT_DATA:
		break;

	case LCD_IOCTL_PWR_OFF:
		lcd_pwr_off_controller();
		break;

	case LCD_IOCTL_INFO:
		LCD_DEBUG("cmd=LCD_IOCTL_INFO arg=0x%08lX", arg);
		{
			LCD_Info_t lcdInfo;

			lcd_get_info(&lcdInfo);
			err =
			    copy_to_user((void *)arg, &lcdInfo,
					 sizeof(LCD_Info_t));

			break;
		}

	default:
		LCD_DEBUG("Unrecognized ioctl: '0x%x'\n", cmd);
		return -ENOTTY;
	}

	return err;

}				/* lcd_ioctl */

/* LCD driver hook for direct DMA CAM->LCD - used by camera_tc.c only */
void lcd_cam_hook(int action)
{
	LCD_DEBUG("action=%d\n", action);

	switch (action) {
	case 0:
		REG_LCD_CR &=
		    ~(REG_LCD_CR_ENABLE_DMA | REG_LCD_CR_ENABLE_DMA_WORDSWAP);
		break;

	case 1:
		lcd_select_panel(LCD_main_panel);
		lcd_setup_for_data(&LCD_device[LCD_main_panel]);

#ifdef __LITTLE_ENDIAN
		REG_LCD_CR |= (REG_LCD_CR_ENABLE_DMA);
#else
		REG_LCD_CR |=
		    (REG_LCD_CR_ENABLE_DMA | REG_LCD_CR_ENABLE_DMA_WORDSWAP);
#endif
		break;
	}
}

/****************************************************************************
*
*  lcd_get_info
*
*
*
***************************************************************************/

void lcd_get_info(LCD_Info_t *lcdInfo)
{
	LCD_dev_info_t *dev = &LCD_device[LCD_main_panel];

	lcdInfo->bitsPerPixel = dev->bits_per_pixel;
	lcdInfo->height = dev->height;
	lcdInfo->width = dev->width;
}

/****************************************************************************
*
*  lcd_mmap
*
*   Note that the bulk of this code came from the fb_mmap routine found in
*   drivers/video/fbmem.c
*
***************************************************************************/

static int lcd_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long offset;
	unsigned long start;
	unsigned long len;
	LCD_dev_info_t *dev;
	LCD_FrameBuffer_t *fb;

	/* vma->vm_start    is the start of the memory region, in user space */
	/* vma->vm_end      is one byte beyond the end of the memory region, in user space */
	/* vma->vm_pgoff    is the offset (in pages) within the vm_start to vm_end region */

	LCD_PUTS("enter");
	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT)) {
		pr_info("lcd: vm_pgoff is out of range\n");
		return -EINVAL;
	}

	if (file == NULL || file->private_data == NULL) {
		pr_info("lcd: bad file pointer or LCD-device pointer\n");
		return -EINVAL;
	}

	dev = (LCD_dev_info_t *) file->private_data;
	fb = &dev->frame_buffer;

	/* Convert offset into a byte offset, rather than a page offset */
	offset = vma->vm_pgoff << PAGE_SHIFT;
	start = (unsigned long)fb->physPtr;	/* already page-aligned */
	pr_info("[lcdcom] lcd_mmap %lx %lx  \r\n", offset, start);

	len = PAGE_ALIGN(start + fb->sizeInBytes);

	if (offset > len) {
		/* The pointer requested by the user isn't inside of our frame buffer */
		LCD_DEBUG("offset is too large, offset = %lu, len = %lu\n",
			  offset, len);
		return -EINVAL;
	}

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	offset += start;
	vma->vm_pgoff = offset >> PAGE_SHIFT;

	if (0 != io_remap_pfn_range(vma,
				    vma->vm_start,
				    offset >> PAGE_SHIFT,
				    vma->vm_end - vma->vm_start,
				    vma->vm_page_prot)) {
		LCD_DEBUG("remap_page_range failed\n");
		return -EAGAIN;
	}

	return 0;

}				/* lcd_mmap */

/****************************************************************************
*
*  lcd_open
*
***************************************************************************/

static int lcd_open(struct inode *inode, struct file *file)
{
	int minor = iminor(inode);

	LCD_DEBUG("major = %d, minor = %d\n", imajor(inode), minor);

	/* minor number must match values for LCD_panel_t */
	if (minor < 0 || minor >= LCD_num_panels) {
		pr_info("lcd: bad minor number %d; range is 0..%d\n",
		       minor, LCD_num_panels - 1);
		return -EINVAL;
	}
	/* set our private pointer to the correct LCD_dev_info_t */
	file->private_data = (void *)&LCD_device[minor];

	/* XXX - hack to see if LCD update task works */
	/* lcd_display_test((LCD_dev_info_t*)file->private_data); */

	return 0;

}				/* lcd_open */

/****************************************************************************
*
*  lcd_release
*
***************************************************************************/

static int lcd_release(struct inode *inode, struct file *file)
{
	LCD_PUTS("enter");

	return 0;

}				/* lcd_release */

/****************************************************************************
*
*  lcd_reset_controller
*
*   Resets the controller for use.
*
***************************************************************************/

static void lcd_reset_controller(int level)
{
	LCD_DEBUG("%d\n", level);

#ifndef CONFIG_ARCH_BCM2153
	gpio_set_value(GPIO_LCD_RESET, level);
#endif

}				/* lcd_reset_controller */

/****************************************************************************
*
*  lcd_get_framebuffer_addr
*
*   Gets the address of the primary frame buffer
*
***************************************************************************/

void *lcd_get_framebuffer_addr(int *frame_size, dma_addr_t * dma_addr)
{
	int rc;
	LCD_dev_info_t *dev = &LCD_device[LCD_main_panel];

	/*  Check if we have been initialized yet.  If not, another driver wants */
	/*  access to our framebuffer before we have been inited.  In this case, */
	/*  allocate the framebuffer now to avoid the other driver failing. */
	/* (lcd_alloc_fb() takes care not to reinitialize itself.) */

	rc = lcd_alloc_fb(dev);

	if (rc)
		return NULL;

	if (dma_addr)
		*dma_addr = dev->frame_buffer.physPtr;

	if (frame_size)
		*frame_size = dev->frame_buffer.sizeInBytes;

	pr_info("[lcdcom] lcd_get_frame %d 0x%x \r\n",
	       dev->frame_buffer.sizeInBytes,
	       (unsigned)dev->frame_buffer.virtPtr);

	return dev->frame_buffer.virtPtr;

}				/* lcd_get_framebuffer_addr */

#if USE_DMA
/****************************************************************************
*
*  lcd_dmaUpdateList
*
*  Update the linked lists of DMA descriptors
*
***************************************************************************/
inline DMA_LLI_t *
lcd_dmaUpdateList(LCD_dev_info_t *dev, LCD_DirtyRect_t *dirtyRect)
{
	int i, stride, width, last_width;
	DMA_LLI_t *list;
	u32 source;
	unsigned int top, bottom;

	stride = dev->width * BYTES_PER_PIXEL;
	source = dev->frame_buffer.physPtr + stride * dirtyRect->top +
			dirtyRect->left * BYTES_PER_PIXEL;
	width = 1 + dirtyRect->right - dirtyRect->left;

	/* The LLI data must be stored little endian when DMA is configured
	   for big endian. */
	/* update all but the last LLI */
	top = (NUM_DMA_LLI_BUFFERS == 1) ? dirtyRect->top %
					dev->height : dirtyRect->top;
	bottom = (NUM_DMA_LLI_BUFFERS == 1) ? dirtyRect->bottom %
					dev->height : dirtyRect->bottom;
	list = (DMA_LLI_t *)dev->dma_linked_list.virtPtr;

	if (width == dev->width) {
		/* maximum DMA size allowed per DMA, in byte */
		stride = (REG_DMA_CHAN_CTL_TRANSFER_SIZE_MASK & ~3) *
				DMA_DIVIDE_WIDTH;
		/* total data size in byte */
		width = width * (1 + bottom - top) * BYTES_PER_PIXEL;
		/* total number of DMA, less 1 */
		bottom = ((width + stride - 1) / stride) - 1;
		/* last DMA data size in byte */
		last_width = width - stride * bottom;
		/* last DMA index within the linked list */
		bottom += top;
		/* DMA transfer size per DMA */
		width = stride / DMA_DIVIDE_WIDTH;
		/* last DMA transfer size */
		last_width /= DMA_DIVIDE_WIDTH;

	} else {
		width = width * BYTES_PER_PIXEL / DMA_DIVIDE_WIDTH;
		last_width = width;
	}

	for (i = top; i < bottom; source += stride, i++) {
		list[i].source = DMA_SWAP(source);
		list[i].control = DMA_SWAP(width | DMA_CONTROL);
	}

	/* save LLI info for the end LLI link and control registers */
	gDmaSavedLli.list = list;
	gDmaSavedLli.row = i;
	gDmaSavedLli.link = list[i].link;
	gDmaSavedLli.valid = 1;

	/* update the last LLI */
	list[i].source = DMA_SWAP(source);
	list[i].link = 0;
	list[i].control = DMA_SWAP(last_width | DMA_CONTROL |
					REG_DMA_CHAN_CTL_TC_INT_ENABLE);

	return &list[top];
}

/****************************************************************************
*
*  lcd_start_dma
*
*   Initiates a DMA transfer of data from the frame buffer to the LCD
*
*   top = index of first row
*   bottom = index of last row
*
***************************************************************************/

static void
lcd_start_dma(LCD_dev_info_t *dev, LCD_DirtyRect_t *dirtyRect)
{
	DMA_LLI_t *startp;

	LCD_DEBUG("enter; top=%d bot=%d\n", dirtyRect->top, dirtyRect->bottom);

		/* enable DMA for the LCD controller */
#ifdef __LITTLE_ENDIAN
		REG_LCD_CR |= (REG_LCD_CR_ENABLE_DMA);
#else
		REG_LCD_CR |=
		    (REG_LCD_CR_ENABLE_DMA | REG_LCD_CR_ENABLE_DMA_WORDSWAP);
#endif
		startp = lcd_dmaUpdateList(dev, dirtyRect);

		dma_setup_chan(gDmaChannel, DMA_SWAP(startp->source),
			       DMA_SWAP(startp->dest), DMA_SWAP(startp->link),
			       DMA_SWAP(startp->control), DMA_CONFIG);

		
}				/* lcd_start_dma */

/****************************************************************************
*
*  lcd_dma_isr
*
*  This isr is triggered when a memory to LCD transfer has completed.
*
***************************************************************************/
static irqreturn_t lcd_dma_isr(void *unused)
{
	DMA_LLI_t *linkedList;

/*   LCD_PUTS("enter"); */
	/* disable DMA mode in LCD controller */
#ifdef __LITTLE_ENDIAN
	REG_LCD_CR &= ~(REG_LCD_CR_ENABLE_DMA);
#else
	REG_LCD_CR &= ~(REG_LCD_CR_ENABLE_DMA | REG_LCD_CR_ENABLE_DMA_WORDSWAP);
#endif

	/* restore the LLI at the end of the list */
	if (gDmaSavedLli.valid) {
		linkedList = gDmaSavedLli.list;
		linkedList[gDmaSavedLli.row].link = gDmaSavedLli.link;
		gDmaSavedLli.valid = 0;
	}
	/* show that DMA to LCD is complete */
	up(&gDmaSem);

/*   LCD_PUTS("handled"); */
	return IRQ_HANDLED;
}

/****************************************************************************
*
*  lcd_dmaCreateList
*
*  Create the linked lists of DMA descriptors
*
***************************************************************************/
static int lcd_dmaCreateList(LCD_dev_info_t *dev)
{
	int i;
	int width;
	DMA_LLI_t *list;

	LCD_PUTS("enter");

	/* allocate memory for DMA linked list (1 LLI per LCD row) */

	dev->dma_linked_list.sizeInBytes =
			dev->height * NUM_DMA_LLI_BUFFERS * DMA_BYTES_PER_LLI;

	dev->dma_linked_list.virtPtr = dma_alloc_coherent(NULL,
							  dev->dma_linked_list.
							  sizeInBytes,
							  &dev->dma_linked_list.
							  physPtr, GFP_KERNEL);

	LCD_DEBUG("virtPtr = 0x%lx, physPtr = 0x%lx\n",
		  (long)dev->dma_linked_list.virtPtr,
		  (long)dev->dma_linked_list.physPtr);

	if (dev->dma_linked_list.virtPtr == NULL) {
		LCD_DEBUG("cannot allocate DMA-safe memory for LLIs");
		return -ENOMEM;
	}

	list = (DMA_LLI_t *) dev->dma_linked_list.virtPtr;
	width = dev->width * BYTES_PER_PIXEL;

	/* The LLI data must be stored little endian when DMA is configured
	   for big endian. */

	/* create all but the last LLI */
	for (i = 0; i < ((dev->height * NUM_DMA_LLI_BUFFERS) - 1); i++) {
		list[i].source =
		    DMA_SWAP(dev->frame_buffer.physPtr + width * i);
		list[i].dest = DMA_SWAP(REG_LCD_DATR_PADDR);
		list[i].link = DMA_SWAP(dev->dma_linked_list.physPtr +
					DMA_BYTES_PER_LLI * (i + 1));
		list[i].control =
		    DMA_SWAP((width / DMA_DIVIDE_WIDTH) | DMA_CONTROL);
	}

	/* create the last LLI */
	list[i].source = DMA_SWAP(dev->frame_buffer.physPtr + width * i);
	list[i].dest = DMA_SWAP(REG_LCD_DATR_PADDR);
	list[i].link = 0;
	list[i].control = DMA_SWAP((width / DMA_DIVIDE_WIDTH) |
				   DMA_CONTROL |
				   REG_DMA_CHAN_CTL_TC_INT_ENABLE);

	gDmaSavedLli.valid = 0;

	LCD_PUTS("done");
	return 0;
}
#endif /* USE_DMA */

/* --------------------------------------------------------------------------
** Stubs.
*/

int lcd_is_dirty_row_update_supported(void)
{
	return 0;
}

int lcd_is_display_regions_supported(void)
{
	return 0;
}

#ifndef CONFIG_BCM_LCD_BACKLIGHT
void lcd_backlight_enable(LCD_BACKLIGHT_LEVEL level)
{
#if GPIO_LCD_BACKLIGHT != 17
	gpio_set_value(GPIO_LCD_BACKLIGHT, level ? 1 : 0);
#endif
}
#endif /* CONFIG_BCM_LCD_BACKLIGHT */

/* XXX - temp hacks */
void lcd_display_test(LCD_dev_info_t *dev)
{
	int i, j;
	uint16_t *fb;
	LCD_DirtyRows_t dirtyRows;

	fb = dev->frame_buffer.virtPtr;
	for (i = 50; i < 100; i++)
		for (j = 50; j < 100; j++)
			if (i < j)
				fb[i * dev->width + j] = LCD_COLOR_YELLOW >> 1;

	dirtyRows.top = 50;
	dirtyRows.bottom = 100;
	lcd_dev_dirty_rows(dev, &dirtyRows);
}

void lcd_display_rect(LCD_dev_info_t *dev, LCD_Rect_t *r)
{
	int i, j;
	uint16_t *fb;
	LCD_DirtyRows_t dirtyRows;

	fb = dev->frame_buffer.virtPtr;

	if (r->top > dev->height)
		r->top = dev->height;
	if (r->left > dev->width)
		r->left = dev->width;

	for (i = r->top; i < (r->top + r->height); i++)
		for (j = r->left; j < (r->left + r->width); j++)
			fb[i * dev->width + j] = r->color;

	dirtyRows.top = r->top;
	dirtyRows.bottom = r->top + r->height;

	if (dirtyRows.bottom >= dev->height)
		dirtyRows.bottom -= 1;

	lcd_dev_dirty_rows(dev, &dirtyRows);
}

#ifdef CONFIG_BRCM_KPANIC_UI_IND
static void lcd_pan_color(LcdColor_t color)
{
	int i;
        int length; 
	LCD_dev_info_t *dev = &LCD_device[0];
	
	dev->col_start = 0;
	dev->col_end = dev->width;
	dev->row_start = 0;
	dev->row_end = dev->height;
	length = dev->height * dev->width;
	lcd_setup_for_data(dev);
	  
        for (i = 0; i < length; i++)
	   WRITE_LCD_DATA(color);
	
}


static void lcd_pan_screen(unsigned short *p_src, int length)
{
	int i;
	unsigned short *p= p_src;
	LCD_dev_info_t *dev = &LCD_device[0];
	
	dev->col_start = 0;
	dev->col_end = dev->width;
	dev->row_start = 0;
	dev->row_end = dev->height;
	
	lcd_setup_for_data(dev);
	  
        for (i = 0; i < length; i++){
	    WRITE_LCD_DATA(*p);
	    p++;
   	}

	
}

void lcd_draw_kpanic_dump(int dumpPic)
{

 unsigned short *psrc;


 printk("[Warning]lcd_draw_kpanic_dump++ %d\r\n",dumpPic);


 lcd_pan_color(LCD_COLOR_BLACK);
 switch(dumpPic)
 {
    case DISPIC_KPANIC_START:
       psrc = &dump_start_qvga[0];
       lcd_pan_screen(psrc,sizeof(dump_start_qvga)/sizeof(unsigned short));
    break;

    case DISPIC_KPANIC_COMPLETE:
       psrc = &dump_end_qvga[0];
       lcd_pan_screen(psrc,sizeof(dump_end_qvga)/sizeof(unsigned short));
    break;

    default:
         printk("Warning: Not specify the dumping display \r\n");
 	break;

 }
 printk("[Warning]lcd_draw_kpanic_dump--\r\n");


}

#endif

/****************************************************************************/

fs_initcall(lcd_init);
module_exit(lcd_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom L2F50219P00 LCD Driver");
