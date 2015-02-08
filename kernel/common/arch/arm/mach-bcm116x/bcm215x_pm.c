/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/bcm215x_pm.c
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
 *   @file   bcm215x_pm.c
 *
 *   @brief  Power Management Driver for Broadcom BCM2153 chipset
 *
 ****************************************************************************/
/**
*   @defgroup   PMAPIGroup   PMU API's
*   @brief      This group defines the Power Management API
*
*****************************************************************************/
#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <mach/clkmgr.h>
#include <mach/reg_clkpwr.h>
#if defined(CONFIG_BCM_PEDESTAL_MIND_READER)
#include <mach/pd_mind_reader.h>
#endif
#include <linux/broadcom/IPCInterface.h>

#define ADDR_IRQ_IMR             (HW_IRQ_BASE + 0x0000)
#define ADDR_IRQ_IMR1            (HW_IRQ_BASE + 0x0100)

#if defined(CONFIG_ARCH_BCM2153)

#define PEDESTAL_CTRL_REG	(HW_SDIO1_BASE + 0x8000)	/*SDHOST2_CORECTRL */
#define PEDESTAL_CTRL_BIT	0x01
#define UARTA_UCR_REG		(HW_UART_A_BASE + 0x100)
#define UARTB_UCR_REG		(HW_UART_A_BASE + 0x108)
#define UARTA_PDEN_BIT		(1 << 14)
#define UARTB_PDEN_BIT		(1 << 7)

#elif defined(CONFIG_ARCH_BCM2157)

#define PEDESTAL_CTRL_REG	ADDR_CLKPWR_CLK_AP_POWER_MODES
#define PEDESTAL_CTRL_BIT	(1 << 1)
#define UARTA_UCR_REG		(HW_UART_A_BASE + 0x100)
#define UARTB_UCR_REG		(HW_UART_B_BASE + 0x100)
#define UARTA_PDEN_BIT		(1 << 6)
#define UARTB_PDEN_BIT		(1 << 6)

#else
#error bcm215x_pm: pedestal ctrl bits not defined !!!
#endif

extern void bcm215x_sleep(u32 bMaySleep);
extern void IPC_ApDisallowDeepSleep(void);
extern IPC_ReturnCode_T IPC_ApCheckDeepSleepAllowed(void);

struct bcm215x_pm {
	suspend_state_t target_state; /**< Power state of the SOC*/
};

static struct bcm215x_pm bcm215x_pm;
static bool bcm215x_sleep_enable = true;
static bool bcm215x_suspend_enable = true;

static void bcm215x_enable_pedestal_mode(void)
{
	u32 regVal;
	regVal = readl(PEDESTAL_CTRL_REG);

#if defined(CONFIG_ARCH_BCM2157)
	regVal |= PEDESTAL_CTRL_BIT;
#else
	regVal &= ~(PEDESTAL_CTRL_BIT);
#endif
	writel(regVal, PEDESTAL_CTRL_REG);
}

static void bcm215x_disable_pedestal_mode(void)
{
	u32 regVal;
	regVal = readl(PEDESTAL_CTRL_REG);

#if defined(CONFIG_ARCH_BCM2157)
	regVal &= ~(PEDESTAL_CTRL_BIT);
#else
	regVal |= PEDESTAL_CTRL_BIT;
#endif
	writel(regVal, PEDESTAL_CTRL_REG);
}

static void bcm215x_enable_dsm(void)
{
#if defined(CONFIG_ARCH_BCM2157)
	u32 regVal;
	regVal = readl(ADDR_CLKPWR_CLK_AP_POWER_MODES);
	regVal |= 1;
	writel(regVal, ADDR_CLKPWR_CLK_AP_POWER_MODES);
#else
	writel(1, ADDR_CLKPWR_PMSM_ENABLE);
#endif
}

static void bcm215x_disable_dsm(void)
{
#if defined(CONFIG_ARCH_BCM2157)
	u32 regVal;
	regVal = readl(ADDR_CLKPWR_CLK_AP_POWER_MODES);
	regVal &= ~1;
	writel(regVal, ADDR_CLKPWR_CLK_AP_POWER_MODES);
#else
	writel(0, ADDR_CLKPWR_PMSM_ENABLE);
#endif
}

#if defined(CONFIG_ARCH_BCM2153)

static void bcm215x_ipc_sem_access_delay(void)
{
	udelay(10);
}

static u32 bcm215x_ipc_dsm_allowed(void)
{
	return bcm215x_sleep_enable;
}

/** @addtogroup PMAPIGroup
	@{
*/
/**
* @brief       Initialization of Deep sleep handlers in IPC power saving structure with platfrom sleep methods
*
* @param ipc_ps        structure of type IPC_PlatformSpecificPowerSavingInfo_T.
*
* This method initializes the handlers to check/enable/disable deep sleep mode
* in the IPC power saving structures. CP can use the same for power managment functionalities.
*/
void pm_ipc_power_saving_init(IPC_PlatformSpecificPowerSavingInfo_T *ipc_ps)
{
	pr_info("Inside %s\n", __func__);
	ipc_ps->SemaphoreAccessDelayFPtr_T = bcm215x_ipc_sem_access_delay;
	ipc_ps->EnableHWDeepSleepFPtr_T = bcm215x_enable_dsm;
	ipc_ps->DisableHWDeepSleepFPtr_T = bcm215x_disable_dsm;
	ipc_ps->CheckDeepSleepAllowedFPtr_T = bcm215x_ipc_dsm_allowed;
}
/** @} */
EXPORT_SYMBOL(pm_ipc_power_saving_init);

#endif /*CONFIG_ARCH_BCM2153 */

static ssize_t bcm215x_pm_ctrl_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	bool sleep_ctrl = false;

	if (strcmp(__stringify(brcm_sleep_ctrl), attr->attr.name) == 0)
		sleep_ctrl = true;

	if (sleep_ctrl)
		return sprintf(buf, "%s\n",
			       (bcm215x_sleep_enable) ? "enabled" : "disabled");
	else
		return sprintf(buf, "%s\n",
			       (bcm215x_suspend_enable) ? "enabled" :
			       "disabled");
}

static ssize_t bcm215x_pm_ctrl_store(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     const char *buf, size_t n)
{
	bool sleep_ctrl = false;
	if (strcmp(__stringify(brcm_sleep_ctrl), attr->attr.name) == 0)
		sleep_ctrl = true;

	if (strncmp(buf, "enable", sizeof("enable") - 1) == 0) {
		if (sleep_ctrl)
			bcm215x_sleep_enable = true;
		else
			bcm215x_suspend_enable = true;
	}

	else if (strncmp(buf, "disable", sizeof("disable") - 1) == 0) {
		if (sleep_ctrl)
			bcm215x_sleep_enable = false;
		else
			bcm215x_suspend_enable = false;
	}

	else {
		pr_err("brcm pm ctrl: Invalid value\n");
		return -EINVAL;
	}

	return n;
}

static struct kobj_attribute brcm_sleep_ctrl =
__ATTR(brcm_sleep_ctrl, 0644, bcm215x_pm_ctrl_show, bcm215x_pm_ctrl_store);

static struct kobj_attribute brcm_suspend_ctrl =
__ATTR(brcm_suspend_ctrl, 0644, bcm215x_pm_ctrl_show, bcm215x_pm_ctrl_store);

extern bool brcm_clk_is_pedestal_allowed(void);

static void bcm215x_pm_idle(void)
{
#if defined(CONFIG_BOARD_EXPLORER)
	bool allow_pedestal = false;
#else
	bool allow_pedestal = true;
#endif /*CONFIG_BOARD_EXPLORER */
	u32 ori_uarta_ucr = 0, ori_uartb_ucr = 0;

	local_irq_disable();
	local_fiq_disable();
	if (need_resched()) {
		local_fiq_enable();
		local_irq_enable();
		return;
	}

	if (!brcm_clk_is_pedestal_allowed()
#ifdef CONFIG_HAS_WAKELOCK
			|| has_wake_lock(WAKE_LOCK_IDLE)
#endif
			)
		allow_pedestal = false;

	if (allow_pedestal) {
		ori_uarta_ucr = readl(UARTA_UCR_REG);	/*backup UARTA_UCR reg */
		ori_uartb_ucr = readl(UARTB_UCR_REG);	/*backup UARTB_UCR reg */
		/*UARTA Power down enable */
		writel((ori_uarta_ucr | UARTA_PDEN_BIT), UARTA_UCR_REG);
		/*UARTB Power down enable */
		writel((ori_uartb_ucr | UARTB_PDEN_BIT), UARTB_UCR_REG);
	} else {
		bcm215x_disable_pedestal_mode();
	}
	bcm215x_disable_dsm();	/* Make sure that only pedestal is allowed */
	bcm215x_sleep(false);

	if (allow_pedestal) {
		/* Restore UART_UCR & UART_IRCR registers */
		writel(ori_uarta_ucr, UARTA_UCR_REG);
		writel(ori_uartb_ucr, UARTB_UCR_REG);
	} else {
		bcm215x_enable_pedestal_mode();
	}

	local_fiq_enable();
	local_irq_enable();
}

static int bcm215x_pm_valid_state(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_ON:
		return 1;

	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		if (!bcm215x_suspend_enable) {
			pr_info("%s:Suspend is disabled.\
				 PM_SUSPEND_STANDBY/PM_SUSPEND_MEM\
				 not allowed\n", __func__);
			return 0;
		}
		return 1;

	default:
		return 0;
	}
}

/*
 * Called after processes are frozen, but before we shutdown devices.
 */
static int bcm215x_pm_begin(suspend_state_t state)
{
	bcm215x_pm.target_state = state;
	return 0;
}

static int bcm215x_pm_enter(suspend_state_t state)
{

	u32 oriIMR, oriIMR1, imr;
	u32 wakeup_source_IMR0 =
	    (1 << IRQ_KEYPAD) | (1 << IRQ_GPIO) | (1 << IRQ_GPTMR);
	u32 wakeup_source_IMR1 = 1 << (IRQ_IPC_C2A - 32);

	switch (state) {
	case PM_SUSPEND_MEM:
	case PM_SUSPEND_STANDBY:

#if defined(CONFIG_ARCH_BCM2153)
		IPC_ApCheckDeepSleepAllowed();
#endif
		/* Backup interrupt masks */
		oriIMR = readl(ADDR_IRQ_IMR);
		oriIMR1 = readl(ADDR_IRQ_IMR1);
		/*set wakeup source.
		 * Mask AP interrupts and enable AP wakeup source
		 * DSP interrupt mask bits should not be modified
		 */
		writel((wakeup_source_IMR0 | (oriIMR & (~BCM215X_VALID_SRC0))),
		       ADDR_IRQ_IMR);
		writel((wakeup_source_IMR1 | (oriIMR1 & (~BCM215X_VALID_SRC1))),
		       ADDR_IRQ_IMR1);

#if defined(CONFIG_BCM_PEDESTAL_MIND_READER)
		pd_mind_reader_log();
#endif

#if defined(CONFIG_ARCH_BCM2157)
		if (bcm215x_sleep_enable)
			bcm215x_enable_dsm();
#endif
		/* Finally go to sleep zzzzz */
		bcm215x_sleep(bcm215x_sleep_enable);

		/* Clear PMSM register */
		bcm215x_disable_dsm();
#if defined(CONFIG_ARCH_BCM2153)
		IPC_ApDisallowDeepSleep();
#endif

		/* Restore AP interrupt mask.
		 * DSP interrupt mask bits should not be modified
		 */
		imr = readl(ADDR_IRQ_IMR);
		writel(((imr & (~BCM215X_VALID_SRC0)) |
			(oriIMR & BCM215X_VALID_SRC0)), ADDR_IRQ_IMR);
		imr = readl(ADDR_IRQ_IMR1);
		writel(((imr & (~BCM215X_VALID_SRC1)) |
			(oriIMR1 & BCM215X_VALID_SRC1)), ADDR_IRQ_IMR1);

		break;

	case PM_SUSPEND_ON:
		break;
	default:
		break;
	}
	return 0;
}

static void bcm215x_pm_end(void)
{
	bcm215x_pm.target_state = PM_SUSPEND_ON;
}

static struct platform_suspend_ops bcm215x_pm_ops = {
	.valid = bcm215x_pm_valid_state,
	.begin = bcm215x_pm_begin,
	.enter = bcm215x_pm_enter,
	.end = bcm215x_pm_end,
};

static int __init bcm215x_pm_init(void)
{
	int ret = 0;
	pm_idle = bcm215x_pm_idle;
	pr_info("Inside %s\n", __func__);
	ret = sysfs_create_file(power_kobj, &brcm_sleep_ctrl.attr);
	ret |= sysfs_create_file(power_kobj, &brcm_suspend_ctrl.attr);
	if (ret)
		pr_err("brcm_sleep_ctrl : sysfs_create_file failed: %d\n", ret);

	bcm215x_pm.target_state = PM_SUSPEND_ON;
	suspend_set_ops(&bcm215x_pm_ops);
	/*Enable pedestal by default */
	bcm215x_enable_pedestal_mode();
	/*Disable sleep mode by default */
	bcm215x_disable_dsm();
	return 0;
}

module_init(bcm215x_pm_init);

static void __exit bcm215x_pm_exit(void)
{
	sysfs_remove_file(power_kobj, &brcm_sleep_ctrl.attr);
	sysfs_remove_file(power_kobj, &brcm_suspend_ctrl.attr);
}

module_exit(bcm215x_pm_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PM Driver for Broadcom bcm215x chip");
