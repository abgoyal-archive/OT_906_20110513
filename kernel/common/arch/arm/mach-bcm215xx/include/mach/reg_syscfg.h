/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*       @file   arch/arm/mach-bcm215xx/include/mach/reg_syscfg.h
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

#ifndef __REG_BCM21553_SYSCFG_H__
#define __REG_BCM21553_SYSCFG_H__

#include <mach/io.h>
#include <mach/hardware.h>

#define HW_SYSCFG_BASE                      IO_ADDRESS(BCM21553_SYSCFG_BASE)

/* System configuration module register mapping */
#define ADDR_SYSCFG_IOCR3                           (HW_SYSCFG_BASE + 0x01C)
#ifdef CONFIG_BRCM_V3D
#define ADDR_SYSCFG_V3DRSTR                         (HW_SYSCFG_BASE + 0x030)
#endif
#define ADDR_SYSCFG_AHB_CLK_GATE_MASK               (HW_SYSCFG_BASE + 0x0D8)
#define ADDR_SYSCFG_AHB_CLK_GATE_FORCE              (HW_SYSCFG_BASE + 0x0DC)
#define ADDR_SYSCFG_AHB_CLK_GATE_MONITOR_RAW        (HW_SYSCFG_BASE + 0x0E0)
#define ADDR_SYSCFG_AHB_CLK_GATE_MONITOR            (HW_SYSCFG_BASE + 0x0E4)
#define ADDR_SYSCFG_VIDEO_CODEC_AHB_CLK_EN          (HW_SYSCFG_BASE + 0x100)
#define ADDR_SYSCFG_CAMARA_INTERFACE_AHB_CLK_EN     (HW_SYSCFG_BASE + 0x104)
#define ADDR_SYSCFG_USB_AHB_CLK_EN                  (HW_SYSCFG_BASE + 0x108)
#define ADDR_SYSCFG_GEA_AHB_CLK_EN                  (HW_SYSCFG_BASE + 0x10C)
#define ADDR_SYSCFG_CRYPTO_AHB_CLK_EN               (HW_SYSCFG_BASE + 0x110)
#define ADDR_SYSCFG_PKA_AHB_CLK_EN                  (HW_SYSCFG_BASE + 0x114)
#define ADDR_SYSCFG_UARTA_AHB_CLK_EN                (HW_SYSCFG_BASE + 0x118)
#define ADDR_SYSCFG_UARTB_AHB_CLK_EN                (HW_SYSCFG_BASE + 0x11C)
#define ADDR_SYSCFG_DA_AHB_CLK_EN                   (HW_SYSCFG_BASE + 0x120)
#define ADDR_SYSCFG_MPCLK_AHB_CLK_EN                (HW_SYSCFG_BASE + 0x124)
#define ADDR_SYSCFG_LCD_AHB_CLK_EN                  (HW_SYSCFG_BASE + 0x12C)
#define ADDR_SYSCFG_V3D_AHB_CLK_EN                  (HW_SYSCFG_BASE + 0x130)
#define ADDR_SYSCFG_DMAC_AHB_CLK_EN                 (HW_SYSCFG_BASE + 0x134)
#define ADDR_SYSCFG_SDIO1_AHB_CLK_EN                (HW_SYSCFG_BASE + 0x138)
#define ADDR_SYSCFG_SDIO2_AHB_CLK_EN                (HW_SYSCFG_BASE + 0x13C)
#define ADDR_SYSCFG_DES_AHB_CLK_EN                  (HW_SYSCFG_BASE + 0x144)
#define ADDR_SYSCFG_UARTC_AHB_CLK_EN                (HW_SYSCFG_BASE + 0x14C)
#define ADDR_SYSCFG_RNG_AHB_CLK_EN                  (HW_SYSCFG_BASE + 0x150)
#define ADDR_SYSCFG_SDIO3_AHB_CLK_EN                (HW_SYSCFG_BASE + 0x154)
#define ADDR_SYSCFG_FSUSBHOST_AHB_CLK_EN            (HW_SYSCFG_BASE + 0x15C)
#define ADDR_SYSCFG_MPHI_AHB_CLK_EN                 (HW_SYSCFG_BASE + 0x160)
#define ADDR_SYSCFG_DMAC_AHB_CLK_MODE               (HW_SYSCFG_BASE + 0x164)
#define ADDR_SYSCFG_HUCM_FW_CLK_EN                  (HW_SYSCFG_BASE + 0x168)
#define ADDR_SYSCFG_HTM_CLK_EN                      (HW_SYSCFG_BASE + 0x16C)
#define ADDR_SYSCFG_TESTABILITY_ACCESS              (HW_SYSCFG_BASE + 0x170)
#define ADDR_SYSCFG_DISABLE_OTP_REGION_READ_ACCESS  (HW_SYSCFG_BASE + 0x174)
#define ADDR_SYSCFG_DISABLE_OTP_REGION_WRITE_ACCESS (HW_SYSCFG_BASE + 0x178)
#define ADDR_SYSCFG_OTP_DEVICE_STATUS               (HW_SYSCFG_BASE + 0x17C)
#define ADDR_SYSCFG_IRDROP_MON3                     (HW_SYSCFG_BASE + 0x180)
#define ADDR_SYSCFG_IRDROP_MON4                     (HW_SYSCFG_BASE + 0x184)
#define ADDR_SYSCFG_IRDROP_MON5                     (HW_SYSCFG_BASE + 0x188)
#define ADDR_SYSCFG_IRDROP_MON6                     (HW_SYSCFG_BASE + 0x18C)
#define ADDR_SYSCFG_CIPHER_FW_CLK_EN                (HW_SYSCFG_BASE + 0x190)
#define ADDR_SYSCFG_SYSCONF_AHB_CLK_EXTEND0         (HW_SYSCFG_BASE + 0x1A0)
#define ADDR_SYSCFG_SYSCONF_AHB_CLK_EXTEND1         (HW_SYSCFG_BASE + 0x1A4)
#define ADDR_SYSCFG_OTP_CHIP_FEATURE_ID             (HW_SYSCFG_BASE + 0x1C0)
#define ADDR_SYSCFG_OTP_WCDMA_CAT                   (HW_SYSCFG_BASE + 0x1C4)
#define ADDR_SYSCFG_OTP_MM_FEAT_CFG                 (HW_SYSCFG_BASE + 0x1C8)
#define ADDR_SYSCFG_OTP_MM_FEAT_DIS                 (HW_SYSCFG_BASE + 0x1CC)
#define ADDR_SYSCFG_BRIDGE_INCR_EN                  (HW_SYSCFG_BASE + 0x1E0)
#define ADDR_SYSCFG_FPGA_VER                        (HW_SYSCFG_BASE + 0x1FC)
#define ADDR_SYSCFG_DP_AHB_CLK_EN                   (HW_SYSCFG_BASE + 0x200)

#define ADDR_SYSCFG_IOCR3_PHYS                      \
	(BCM21553_SYSCFG_BASE + 0x01C)
#define ADDR_SYSCFG_AHB_CLK_GATE_MASK_PHYS          \
	(BCM21553_SYSCFG_BASE + 0x0D8)
#define ADDR_SYSCFG_UARTA_AHB_CLK_EN_PHYS           \
	(BCM21553_SYSCFG_BASE + 0x118)

/*
 * System Configuration Controller Register Offsets
 */
#define SYSCFG_IOCR0_OFF			0x00
#define SYSCFG_IOCR1_OFF			0x04
#define SYSCFG_IOCR2_OFF			0x0c
#define SYSCFG_IOCR3_OFF			0x1c
#define SYSCFG_IRDROP_MON5_OFF		0x88

/*
 * Auxiliary Microphone Detection Register Offsets
 */
#define AUXMIC_PRB_CYC			0x00
#define AUXMIC_MSR_DLY			0x04
#define AUXMIC_MSR_INTVL		0x08
#define AUXMIC_CMC				0x0c
#define AUXMIC_MIC				0x10
#define AUXMIC_AUXEN			0x14
#define AUXMIC_MICINTH_ADJ		0x18
#define AUXMIC_MICINENTH_ADJ	0x1c
#define AUXMIC_MICONTH_ADJ		0x20
#define AUXMIC_MICONENTH_ADJ	0x24
#define AUXMIC_F_PWRDWN			0x28

/*
 * GPIO Register offsets
 */
#define GPIO_IOTR0_OFF		0x00
#define GPIO_IOTR1_OFF		0x04
#define GPIO_IOTR2_OFF		0x08
#define GPIO_IOTR3_OFF		0x0c
#define GPIO_GPOPS0_OFF		0x10
#define GPIO_GPIPEN0_OFF	0x20
#define GPIO_GPIPEN1_OFF	0x24
#define GPIO_GPIPUD0_OFF	0x28
#define GPIO_GPIPUD1_OFF	0x2C
/*
 * Register bit defines
 */

/* PERIPH_AHB_CLK_GATE_MON register bit defines */
#define PERIPH_AHB_CLK_GATE_MON_DMAC                (1 << 4)

#endif /*__REG_BCM21553_SYSCFG_H__*/
