/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm215xx/include/mach/sdio.h
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
 * This file provides the interface for all the SDMMC host
 */
#ifndef __ASM_ARCH_SDIO_H
#define __ASM_ARCH_SDIO_H

#include <plat/syscfg.h>

/* SDHOST_CORECTRL */
#define SDIO_SDHOST_CORECTRL_DATA_SWAP  (1 << 3)
#define SDIO_SDHOST_CORECTRL_STOP_HCLK  (1 << 2)
#define SDIO_SDHOST_CORECTRL_RESET      (1 << 1)
#define SDIO_SDHOST_CORECTRL_EN         (1 << 0)

/* SDHOST_CORESTAT */
#define SDIO_SDHOST_CORESTAT_LED        (1 << 8)
#define SDIO_SDHOST_CORESTAT_BUS_VOLT_MASK (0x7 << 5)
#define SDIO_SDHOST_CORESTAT_GET_BUS_VOLT(sdhost_corestat) ((sdhost_corestat & SDIO_SDHOST_CORESTAT_BUS_VOLT_MASK) >> 5)
#define SDIO_SDHOST_CORESTAT_BUS_PWR    (1 << 4)
#define SDIO_SDHOST_CORESTAT_WP         (1 << 1)
#define SDIO_SDHOST_CORESTAT_CD         (1 << 0)

/* SDHOST_COREIMR */
#define SDIO_SDHOST_COREIMR_DAT1        (1 << 1)
#define SDIO_SDHOST_COREIMR_IP          (1 << 0)

/* SDHOST_COREISR */
#define SDIO_SDHOST_COREISR_DAT1        (1 << 1)
#define SDIO_SDHOST_COREISR_IP          (1 << 0)

/* SDHOST_COREIMSR */
#define SDIO_SDHOST_COREIMSR_DAT1       (1 << 1)
#define SDIO_SDHOST_COREIMSR_IP         (1 << 0)

#define SDHOST_CORECTRL_SHIFT      0x00008000
#define SDHOST_CORECTRL_DATA_SWAP (1 << 3)	/* Swap Byte order for DMA/data register access */
#define SDHOST_CORECTRL_STOP_HLCK (1 << 2)	/* Stop the AHB clock going to the host controller */
#define SDHOST_CORECTRL_RESET     (1 << 1)	/* Software reset the entire host controller core */
#define SDHOST_CORECTRL_EN        (1 << 0)	/* Enable AHB clock gating module */

#define SDHOST_CORESTAT_SHIFT      0x00008004
#define SDHOST_CORESTAT_LED           (1 << 8)	/* Led_on output from the host controller IP (read only) */
#define SDHOST_CORESTAT_BUS_VOLT_MASK (7 << 5)	/* Bus_volt output from the host controller IP (read only) */
#define SDHOST_CORESTAT_BUS_PWR       (1 << 4)	/* Bus_pwr ouput from the host controller IP (read only) */
#define SDHOST_CORESTAT_WP            (1 << 1)	/* Write Protect status */
#define SDHOST_CORESTAT_CD            (1 << 0)	/* Card Detected */

#define SDHOST_COREIMR_SHIFT       0x00008008
#define SDHOST_COREIMR_DAT1       (1 << 1)	/* Enables detection on DAT1 in the absence of 0 SD clock and AHB clock.
						   DAT1 is used to signal interrupt in SDIO 1-bit mode. */
#define SDHOST_COREIMR_IP         (1 << 0)	/* Enable the interrupt from the controller */

#define SDHOST_COREISR_SHIFT       0x0000800C
#define SDHOST_COREISR_DAT1       (1 << 1)	/* Active high interrupt status on DAT1 input */
#define SDHOST_COREISR_IP         (1 << 0)	/* Active high interrupt satus from the controller */

#define SDHOST_COREIMSR_SHIFT      0x00008010
#define SDHOST_COREIMSR_DAT1      (1 << 1)	/* Masked Active high interrupt status on DAT1 input */
#define SDHOST_COREIMSR_IP        (1 << 0)	/* Masked Active high interrupt status from the controller */

#define SD3CMD_PULL_UP	  (1<<19)
#define SD3CMD_PULL_DOWN  (1<<20)
#define SD3DAT_PULL_UP	  (1<<22)
#define SD3DAT_PULL_DOWN  (1<<23)
#define SD1CMD_PULL_UP	  (1<<24)
#define SD1CMD_PULL_DOWN  (1<<25)

#define SD1DAT_PULL_UP    (1<<26)
#define SD1DAT_PULL_DOWN  (2<<26)

#define SD2CMD_PULL_UP	  (1<<28)
#define SD2CMD_PULL_DOWN  (2<<28)
#define SD2DAT_PULL_UP	  (1<<30)
#define SD2DAT_PULL_DOWN  (2<<30)

#define	SDCD_PULLDOWN	     (0 << 0)
#define	SDCD_PULLUP	     (1 << 0)
#define	SDCD_UPDOWN_DISABLE  (0 << 1)
#define	SDCD_UPDOWN_ENABLE   (1 << 1)

struct bcmsdhc_platform_data {
	u32 base_clk;
	u8 card_in_state;
	u8 cd_pullup_cfg;
	u8 irq_cd;
	int (*syscfg_interface) (uint32_t module, uint32_t op);
	int (*cfg_card_detect) (void __iomem *ioaddr, u8 ctrl_slot);
	int (*external_reset) (void __iomem *ioaddr, u8 ctrl_slot);
	int (*enable_int) (void __iomem *ioaddr, u8 ctrl_slot);
};

#endif /* __ASM_ARCH_SDIO_H */
