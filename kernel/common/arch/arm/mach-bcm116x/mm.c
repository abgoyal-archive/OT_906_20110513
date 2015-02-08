/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/mm.c
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
 * BCM215X memory map definitions
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <mach/hardware.h>
#include <asm/mach/map.h>

#define __IO_DEV_DESC(x, sz)	{				\
	.virtual	= x,					\
	.pfn		= __phys_to_pfn(HW_IO_VIRT_TO_PHYS(x)), \
	.length		= sz,					\
	.type		= MT_DEVICE,				\
}

/* minimum static i/o mapping required to boot BCM215X platforms */
static struct map_desc bcm215x_io_desc[] __initdata = {
	/*
	 * TODO It turns out that the hardware is capable of mapping 1Mb blocks
	 * by using the first level mapping. So making the I/O space be one
	 * large space is better from a performance standpoint than making
	 * several spaces, each less than 1Mb.
	 */
	__IO_DEV_DESC(HW_ONAD_BASE, SZ_1M),
	__IO_DEV_DESC(HW_NAND_BASE, SZ_1M),
	__IO_DEV_DESC(HW_TV_OUTPUT_BASE, SZ_1M),
	__IO_DEV_DESC(HW_USBOTG_BASE, SZ_1M),
	__IO_DEV_DESC(HW_ITCM_BASE, SZ_1M),
	__IO_DEV_DESC(HW_DTCM_BASE, SZ_1M),
	__IO_DEV_DESC(HW_SMT_BASE, SZ_1M),
	__IO_DEV_DESC(HW_HEADSET_BASE, SZ_1M),
	__IO_DEV_DESC(HW_SRAM0_BASE, SZ_32M),
	__IO_DEV_DESC(HW_CRYPTO_BASE, SZ_1M),

};

void __init bcm215x_map_io(void)
{
	iotable_init(bcm215x_io_desc, ARRAY_SIZE(bcm215x_io_desc));
}
