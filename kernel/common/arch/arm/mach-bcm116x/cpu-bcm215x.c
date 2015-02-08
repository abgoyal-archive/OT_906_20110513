/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/cpu-bcm215x.c
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
 * BCM215X SoC specific driver definitions
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <linux/broadcom/bcm_fuse_memmap.h>
#include <linux/broadcom/IPCInterface.h>
#include <linux/gpio.h>

#include <asm/mach/map.h>
#include <mach/hardware.h>
#include <mach/setup.h>
#include <mach/clkmgr.h>

#if defined (CONFIG_BCM_FUSE_APPS_PROCESSOR)
static void __init start_cp(void)
{
	void __iomem *cp_boot_base;
	void __iomem *apcp_shmem;

	cp_boot_base = ioremap(CP_BOOT_BASE_PHYS, CP_BOOT_BASE_SIZE);
	if (!cp_boot_base) {
		pr_err("%s: ioremap error\n", __FUNCTION__);
		return;
	}

	apcp_shmem = ioremap_nocache(IPC_BASE, IPC_SIZE);
	if (!apcp_shmem) {
		pr_err("%s: ioremap shmem failed\n", __FUNCTION__);
		iounmap(cp_boot_base);
		return;
	}

	/* clear first (9) 32-bit words in shared memory */
	memset(apcp_shmem, 0, IPC_SIZE);

	/* Start the CP, Code taken from Nucleus BSP */
	writel(CP_START_ADDR, ((unsigned long)cp_boot_base) + 0x20);

	writel(0x00200000, ((unsigned long)HW_SYS_BASE) + 0x1c);
	writel(0x00200000, ((unsigned long)HW_SYS_BASE) + 0x1c);
	writel(0x00200000, ((unsigned long)HW_SYS_BASE) + 0x1c);
	writel(0x00200000, ((unsigned long)HW_SYS_BASE) + 0x1c);

	writel(0, ((unsigned long)HW_IRQ_BASE) + 0x38);

	iounmap(cp_boot_base);
	iounmap(apcp_shmem);

	pr_info("%s: BCM_FUSE CP Started....\n", __FUNCTION__);

	return;
}
#endif

struct bcm_gpio_port gpio_port = {
	.base = (void __iomem *)HW_GPIO_BASE,
	.irq = IRQ_GPIO,
};

void __init bcm215x_init_irq(void)
{
	bcm_intc_init((void __iomem *)(HW_IRQ_BASE + 0x0), 0,
		      BCM215X_VALID_SRC0, 0);
	bcm_intc_init((void __iomem *)(HW_IRQ_BASE + 0x100), 32,
		      BCM215X_VALID_SRC1, 0);

	bcm_gpio_init(&gpio_port);
}

/*
 * This function is called from the board init
 */
void __init bcm215x_platform_init(void)
{
#if defined (CONFIG_BCM_FUSE_APPS_PROCESSOR)
	start_cp();
#endif
	return;
}
