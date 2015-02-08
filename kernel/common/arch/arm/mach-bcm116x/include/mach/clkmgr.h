/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/include/mach/clkmgr.h
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
 *   @file   clkmgr.h
 *
 *   @brief  BCM2153 side interface functions for clock frame work.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_MACH_BCM2153_CLOCK_H
#define __ARCH_ARM_MACH_BCM2153_CLOCK_H

int __init bcm2153_clk_mgr_init(void);

#define BCM_CLK_MAIN_PLL_STR_ID     "MAINPLL"
#define BCM_CLK_APPS_PLL_STR_ID     "APPSPLL"
#define BCM_CLK_ARM11_STR_ID        "ARM11"
#define BCM_CLK_CAMERA_STR_ID       "CAM"
#define BCM_CLK_I2S_INT_STR_ID      "i2s_int"
#define BCM_CLK_I2S_EXT_STR_ID      "i2s_ext"
#define BCM_CLK_DAM_STR_ID          "DAM"
#define BCM_CLK_PDP_STR_ID          "PDM"
#define BCM_CLK_SDIO1_STR_ID        "bcm_sdhc.1"
#define BCM_CLK_SDIO2_STR_ID        "bcm_sdhc.2"
#define BCM_CLK_SM_STR_ID           "SM"
#define BCM_CLK_SPI0_STR_ID          "bcm_spi.0"
#define BCM_CLK_UARTA_STR_ID        "serial8250.0"
#define BCM_CLK_UARTB_STR_ID        "serial8250.1"
#define BCM_CLK_GP_STR_ID           "GP"
#define BCM_CLK_MSPRO_STR_ID        "MSPRO"
#define BCM_CLK_I2C1_STR_ID         "i2cbcm.0"
#define BCM_CLK_I2C2_STR_ID         "i2cbcm.1"
#define BCM_CLK_USB_STR_ID          "USB"
#define BCM_CLK_DMAC_STR_ID			"dmac"
#define BCM_CLK_GE_STR_ID			"ge"
#define BCM_CLK_LCD_STR_ID			"lcd"

#ifdef CONFIG_ARCH_BCM2157
#define BCM_CLK_PWM_STR_ID		"pwm"
#define BCM_CLK_VCODEC_STR_ID	"vcodec"
#define BCM_CLK_RNG_STR_ID		"rng"
#define BCM_CLK_MPHI_STR_ID		"mphi"
#define BCM_CLK_CMI_STR_ID		"CMI"
#endif

#endif /* __ARCH_ARM_MACH_BCM2153_CLOCK_H */
