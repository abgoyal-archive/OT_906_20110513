/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/include/mach/setup.h
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
 * These symbols are needed for board-specific files to call their
 * own cpu-specific files
 */
#ifndef __ASM_ARCH_SETUP_H
#define __ASM_ARCH_SETUP_H

#include <linux/init.h>
#include <linux/gpio.h>
#include <asm/mach/time.h>

#include <plat/timer.h>

extern void bcm215x_map_io(void);
extern void bcm215x_platform_init(void);
extern void bcm215x_init_irq(void);
extern void bcm_timer_init(struct timer_config *bcm_timer_config);
extern void bcm_intc_init(void __iomem *base, unsigned int irq_start,
			  u32 vic_sources, u32 resume_sources);
extern int32_t bcm_gpio_init(struct bcm_gpio_port *port);

#endif /*  __ASM_ARCH_SETUP_H */
