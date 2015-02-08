/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/include/mach/reg_clkpwr.h
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
*
*****************************************************************************
*
*  PURPOSE:
*     This file contains definitions for the system configuration control registers:
*
*  NOTES:
*
*****************************************************************************/

#if !defined(__ASM_ARCH_REG_CLKPWR_H)
#define __ASM_ARCH_REG_CLKPWR_H

/* ---- Include Files ---------------------------------------------------- */
#include <mach/hardware.h>

/* ---- Constants and Types ---------------------------------------------- */
									     /* Default #Bits Description */
#define REG_CLKPWR_CLK_ARMAHB_MODE      __REG32(HW_CLKPWR_BASE + 0x00)	/* 0 3bit */
#define REG_CLKPWR_CLK_CAMCK_ENABLE     __REG32(HW_CLKPWR_BASE + 0x04)	/* 0 1bit */
#define REG_CLKPWR_CLK_CAMCK_MODE       __REG32(HW_CLKPWR_BASE + 0x08)	/* 0 3bit */
#define REG_CLKPWR_CLK_DSP_MODE         __REG32(HW_CLKPWR_BASE + 0x0C)	/* 0 2bit */
#define REG_CLKPWR_CLK_I2S_INT_ENABLE   __REG32(HW_CLKPWR_BASE + 0x10)	/* 0 1bit */
#define REG_CLKPWR_CLK_I2S_INT_MODE     __REG32(HW_CLKPWR_BASE + 0x14)	/* 0 2bit */
#define REG_CLKPWR_CLK_I2S_EXT_ENABLE   __REG32(HW_CLKPWR_BASE + 0x18)	/* 0 1bit */
#define REG_CLKPWR_CLK_DAMCK_ENABLE     __REG32(HW_CLKPWR_BASE + 0x1C)	/* 0 1bit */
#define REG_CLKPWR_CLK_MONITOR_ENABLE   __REG32(HW_CLKPWR_BASE + 0x20)	/* 0 1bit */
#define REG_CLKPWR_CLK_MONITOR_SELECT   __REG32(HW_CLKPWR_BASE + 0x24)	/* 0 5bit */
#define REG_CLKPWR_CLK_PDPDCK_ENABLE    __REG32(HW_CLKPWR_BASE + 0x28)	/* 0 1bit */
#define REG_CLKPWR_CLK_PDPDCK_DIV       __REG32(HW_CLKPWR_BASE + 0x2C)	/* 5 3bit  Clk Output = 156 / n where n is equal to (reg + 1)*2 */
#define REG_CLKPWR_CLK_SDIO0_ENABLE     __REG32(HW_CLKPWR_BASE + 0x30)	/* 0 1bit */
#define REG_CLKPWR_CLK_SDIO0_DIV        __REG32(HW_CLKPWR_BASE + 0x34)	/* 2 14bit Clk Output = 104 / n where n is equal to (reg + 1)*2 */
#define REG_CLKPWR_CLK_SDIO1_ENABLE     __REG32(HW_CLKPWR_BASE + 0x38)	/* 0 1bit */
#define REG_CLKPWR_CLK_SDIO1_DIV        __REG32(HW_CLKPWR_BASE + 0x3C)	/* 2 14bit Clk Output = 104 / n where n is equal to (reg + 1)*2 */
#define REG_CLKPWR_CLK_SMCLK_ENABLE     __REG32(HW_CLKPWR_BASE + 0x40)	/* 0 1bit */
#define REG_CLKPWR_CLK_SMCLK_MODE       __REG32(HW_CLKPWR_BASE + 0x44)	/* 1 2bit */
#define REG_CLKPWR_CLK_SPI_ENABLE       __REG32(HW_CLKPWR_BASE + 0x48)	/* 0 1bit */
#define REG_CLKPWR_CLK_SPI_DIV          __REG32(HW_CLKPWR_BASE + 0x4C)	/* 1 3bit   Clk Output = 156 / n where n is equal to (reg + 1)*2 */
#define REG_CLKPWR_CLK_UARTA_ENABLE     __REG32(HW_CLKPWR_BASE + 0x50)	/* 1 1bit */
#define REG_CLKPWR_CLK_UARTA_N          __REG32(HW_CLKPWR_BASE + 0x54)	/* 1 9bit   Clk Output = 156 * n / m */
#define REG_CLKPWR_CLK_UARTA_M          __REG32(HW_CLKPWR_BASE + 0x58)	/* C 9bit */
#define REG_CLKPWR_CLK_UARTB_ENABLE     __REG32(HW_CLKPWR_BASE + 0x5C)	/* 1 1bit */
#define REG_CLKPWR_CLK_UARTB_N          __REG32(HW_CLKPWR_BASE + 0x60)	/* 1 9bit   Clk Output = 156 * n / m */
#define REG_CLKPWR_CLK_UARTB_M          __REG32(HW_CLKPWR_BASE + 0x64)	/* C 9bit */
#define REG_CLKPWR_CLK_UARTC_ENABLE     __REG32(HW_CLKPWR_BASE + 0x68)	/* 1 1bit */
#define REG_CLKPWR_CLK_UARTC_N          __REG32(HW_CLKPWR_BASE + 0x6C)	/* 1 9bit   Clk Output = 156 * n / m */
#define REG_CLKPWR_CLK_UARTC_M          __REG32(HW_CLKPWR_BASE + 0x70)	/* C 9bit */
#define REG_CLKPWR_DSPSRST_ENABLE       __REG32(HW_CLKPWR_BASE + 0x74)	/* 0 1bit   Enable DSP soft reset */
#define REG_CLKPWR_IRPC_ENABLE          __REG32(HW_CLKPWR_BASE + 0x9C)	/* 1 1bit   Power Down IR Transceiver during deep sleep mode */
#define REG_CLKPWR_MCLK_STABLE_TIME     __REG32(HW_CLKPWR_BASE + 0x80)	/* 0 11bit  Setting time of 13MHz oscillator in number of 32kHz clock */
#define REG_CLKPWR_PC_0_ENABLE          __REG32(HW_CLKPWR_BASE + 0x84)	/* 1 1bit   Power Down 13Mhz oscillator during deep sleep mode */
#define REG_CLKPWR_PCHOSTPD_ENABLE      __REG32(HW_CLKPWR_BASE + 0x88)	/* 0 1bit   Enable the use of deep sleep when in PCMCIA mode */
#define REG_CLKPWR_PLL48_STABLE_TIME    __REG32(HW_CLKPWR_BASE + 0x8C)	/* 0 7bit   Setting time of 48MHz PLL in number of 32kHz clock */
#define REG_CLKPWR_PLL48CNTEN           __REG32(HW_CLKPWR_BASE + 0x90)	/* 0 1bit   Enable the setting time counter of 48MHz PLL */
#define REG_CLKPWR_PLL_STABLE_TIME      __REG32(HW_CLKPWR_BASE + 0x94)	/* 0 7bit   Setting time of Main PLL in number of 32kHz clock */
#define REG_CLKPWR_PLLCNTEN             __REG32(HW_CLKPWR_BASE + 0x98)	/* 0 1bit   Enable the setting time counter of Main PLL */
#define REG_CLKPWR_PMSM_ENABLE          __REG32(HW_CLKPWR_BASE + 0x9C)	/* 0 1bit   Enable the Power Management State Machine */
#define REG_CLKPWR_PWDPLL_ENABLE        __REG32(HW_CLKPWR_BASE + 0xA0)	/* 1 1bit   Power Down the Main PLL during deep sleep mode */
#define REG_CLKPWR_USBPLL_ENABLE        __REG32(HW_CLKPWR_BASE + 0xA4)	/* 0 1bit   Power Up the 48MHz PLL */
#define REG_CLKPWR_USBPLL_OEN           __REG32(HW_CLKPWR_BASE + 0xA8)	/* 0 1bit   Turn on the output gate of the 48MHz PLL */
#define REG_CLKPWR_CLK_GPCK_ENABLE      __REG32(HW_CLKPWR_BASE + 0xAC)	/* 0 1bit */
#define REG_CLKPWR_CLK_GPCK_DIV         __REG32(HW_CLKPWR_BASE + 0xB0)	/* 5 3bit   Clk Output = 156 / n where n is equal to (reg + 1)*2 */
#define REG_CLKPWR_CLK_MSPRO_ENABLE     __REG32(HW_CLKPWR_BASE + 0xB4)	/* 0 1bit   Memory stick pro enable 1=enabled */
#define REG_CLKPWR_CLK_MSPRO_DIV        __REG32(HW_CLKPWR_BASE + 0xB8)	/* 3 3bit   Clk Output = 156 / n where n is equal to (reg + 1)*2 */
#define REG_CLKPWR_CLK_ATCLK_MODE       __REG32(HW_CLKPWR_BASE + 0xBC)	/* 0 1bit   AT clock mode */

#ifdef CONFIG_ARCH_BCM2157

#define REG_CLKPWR_CLK_I2S_FRAC_MODE		REG32(HW_CLKPWR_BASE + 0x1A8)
#define REG_CLKPWR_CLK_CMI_ENABLE		REG32(HW_CLKPWR_BASE + 0x12C)	/* 0 1bit */
#define REG_CLKPWR_CLK_CMI_MODE			REG32(HW_CLKPWR_BASE + 0x130)	/* 0 3bit */
#define REG_CLKPWR_CLK_PWM_ENABLE		REG32(HW_CLKPWR_BASE + 0x1C4)	/* 0 1bit */
#define REG_CLKPWR_CLK_PWM_SOFT_RESET		REG32(HW_CLKPWR_BASE + 0x1C8)	/* 0 1bit */

#endif

/* REG_CLKPWR_CLK_ARMAHB_MODE bits -  ARM11 ARM9  AHB */
#define REG_CLKPWR_CLK_ARMAHB_MODE_ARM52_AHB52_MHZ          0	/*    52    52    52 */
#define REG_CLKPWR_CLK_ARMAHB_MODE_ARM78_AHB78_MHZ          1	/*    78    78    78 */
#define REG_CLKPWR_CLK_ARMAHB_MODE_ARM104_AHB52_MHZ         2	/*   104    52    52 */
#define REG_CLKPWR_CLK_ARMAHB_MODE_ARM104_AHB104_MHZ        3	/*   104   104   104 */
#define REG_CLKPWR_CLK_ARMAHB_MODE_ARM156_AHB78_MHZ         4	/*   156    78    78 */
#define REG_CLKPWR_CLK_ARMAHB_MODE_ARM13_AHB13_MHZ          5	/*    13    13    13 */
#define REG_CLKPWR_CLK_ARMAHB_MODE_ARM156_AHB52_MHZ         6	/*   156    52    52 */
#define REG_CLKPWR_CLK_ARMAHB_MODE_ARM208_AHB104_MHZ        7	/*   208   104   104 */

/* REG_CLKPWR_CLK_CAMCK_MODE bits */
#define REG_CLKPWR_CLK_CAMCK_MODE_12MHZ                 0
#define REG_CLKPWR_CLK_CAMCK_MODE_13MHZ                 1
#define REG_CLKPWR_CLK_CAMCK_MODE_24MHZ                 2
#define REG_CLKPWR_CLK_CAMCK_MODE_26MHZ                 3
#define REG_CLKPWR_CLK_CAMCK_MODE_48MHZ                 4

/* REG_CLKPWR_CLK_DSP_MODE bits */
#define REG_CLKPWR_CLK_DSP_MODE_52MHZ                   0
#define REG_CLKPWR_CLK_DSP_MODE_78MHZ                   1
#define REG_CLKPWR_CLK_DSP_MODE_104MHZ                  2
#define REG_CLKPWR_CLK_DSP_MODE_124P8MHZ                3

/* REG_CLKPWR_CLK_I2S_INT_MODE bits */
#define REG_CLKPWR_CLK_I2S_INT_MODE_12MHZ               0
#define REG_CLKPWR_CLK_I2S_INT_MODE_6MHZ                1
#define REG_CLKPWR_CLK_I2S_INT_MODE_3MHZ                2
#define REG_CLKPWR_CLK_I2S_INT_MODE_1_5MHZ              3

/* REG_CLKPWR_CLK_MONITOR_SELECT bits */
#define REG_CLKPWR_CLK_MONITOR_SELECT_ANA_CLK_156MHZ    0x00
#define REG_CLKPWR_CLK_MONITOR_SELECT_ANA_CLK_104MHZ    0x01
#define REG_CLKPWR_CLK_MONITOR_SELECT_ANA_CLK_78MHZ     0x02
#define REG_CLKPWR_CLK_MONITOR_SELECT_ANA_CLK_52MHZ     0x03
#define REG_CLKPWR_CLK_MONITOR_SELECT_ANA_CLK_26MHZ     0x04
#define REG_CLKPWR_CLK_MONITOR_SELECT_ANA_CLK_13MHZ     0x05
#define REG_CLKPWR_CLK_MONITOR_SELECT_ANA_CLK_1MHZ      0x06
#define REG_CLKPWR_CLK_MONITOR_SELECT_ANA_CLK_88KHZ     0x07
#define REG_CLKPWR_CLK_MONITOR_SELECT_ANA_CLK_40KHZ     0x08
#define REG_CLKPWR_CLK_MONITOR_SELECT_ANA_CLK_32KHZ     0x09
#define REG_CLKPWR_CLK_MONITOR_SELECT_ANA_CLK_48MHZ     0x0a
#define REG_CLKPWR_CLK_MONITOR_SELECT_CLK_ARM           0x0b
#define REG_CLKPWR_CLK_MONITOR_SELECT_CLK_AHB           0x0c
#define REG_CLKPWR_CLK_MONITOR_SELECT_CLK_DSP           0x0d
#define REG_CLKPWR_CLK_MONITOR_SELECT_CLK_I2S           0x0e
#define REG_CLKPWR_CLK_MONITOR_SELECT_CLK_SDIO0         0x0f
#define REG_CLKPWR_CLK_MONITOR_SELECT_CLK_SDIO1         0x10
#define REG_CLKPWR_CLK_MONITOR_SELECT_CLK_SPI           0x11
#define REG_CLKPWR_CLK_MONITOR_SELECT_CLK_UARTA         0x12
#define REG_CLKPWR_CLK_MONITOR_SELECT_CLK_UARTB         0x13
#define REG_CLKPWR_CLK_MONITOR_SELECT_CLK_UARTC         0x14
#define REG_CLKPWR_CLK_MONITOR_SELECT_CLK_SMCLK         0x15
#define REG_CLKPWR_CLK_MONITOR_SELECT_ANA_CLK_208MHZ    0x16
#define REG_CLKPWR_CLK_MONITOR_SELECT_ANA_CLK_124P8MHZ  0x17

/* REG_CLKPWR_CLK_SMCLK_MODE bits */
#define REG_CLKPWR_CLK_SMCLK_MODE_AHB                   0
#define REG_CLKPWR_CLK_SMCLK_MODE_AHB_DIV_2             1
#define REG_CLKPWR_CLK_SMCLK_MODE_AHB_DIV_3             2
#define REG_CLKPWR_CLK_SMCLK_MODE_AHB_DIV_4             3

/* REG_CLKPWR_CLK_ATCLK_MODE bits */
#define REG_CLKPWR_CLK_ATCLK_MODE_52MHZ                 0
#define REG_CLKPWR_CLK_ATCLK_MODE_78MHZ                 1
#define REG_CLKPWR_CLK_ATCLK_MODE_104MHZ                2
#define REG_CLKPWR_CLK_ATCLK_MODE_124PT8MHZ             3
#define REG_CLKPWR_CLK_ATCLK_MODE_156MHZ                4
#define REG_CLKPWR_CLK_ATCLK_MODE_208MHZ                5
#define REG_CLKPWR_CLK_ATCLK_MODE_13MHZ                 6

/*********** Defines all addresses for 2153, and register field settings. **************/
/* Address definitions of all registers. */
#define ADDR_CLKPWR_CLK_ARMAHB_MODE      (HW_CLKPWR_BASE + 0x00)
#define ADDR_CLKPWR_CLK_CAMCK_ENABLE     (HW_CLKPWR_BASE + 0x04)
#define ADDR_CLKPWR_CLK_CAMCK_MODE       (HW_CLKPWR_BASE + 0x08)
#define ADDR_CLKPWR_CLK_DSP_MODE         (HW_CLKPWR_BASE + 0x0C)
#define ADDR_CLKPWR_CLK_I2S_INT_ENABLE   (HW_CLKPWR_BASE + 0x10)
#define ADDR_CLKPWR_CLK_I2S_INT_MODE     (HW_CLKPWR_BASE + 0x14)
#define ADDR_CLKPWR_CLK_I2S_EXT_ENABLE   (HW_CLKPWR_BASE + 0x18)
#define ADDR_CLKPWR_CLK_DAMCK_ENABLE     (HW_CLKPWR_BASE + 0x1C)
#define ADDR_CLKPWR_CLK_MONITOR_ENABLE   (HW_CLKPWR_BASE + 0x20)
#define ADDR_CLKPWR_CLK_MONITOR_SELECT   (HW_CLKPWR_BASE + 0x24)
#define ADDR_CLKPWR_CLK_PDPDCK_ENABLE    (HW_CLKPWR_BASE + 0x28)
#define ADDR_CLKPWR_CLK_PDPDCK_DIV       (HW_CLKPWR_BASE + 0x2C)
#define ADDR_CLKPWR_CLK_SDIO1_ENABLE     (HW_CLKPWR_BASE + 0x30)
#define ADDR_CLKPWR_CLK_SDIO1_DIV        (HW_CLKPWR_BASE + 0x34)
#define ADDR_CLKPWR_CLK_SDIO2_ENABLE     (HW_CLKPWR_BASE + 0x38)
#define ADDR_CLKPWR_CLK_SDIO2_DIV        (HW_CLKPWR_BASE + 0x3C)
#define ADDR_CLKPWR_CLK_SMCLK_ENABLE     (HW_CLKPWR_BASE + 0x40)
#define ADDR_CLKPWR_CLK_SMCLK_MODE       (HW_CLKPWR_BASE + 0x44)
#define ADDR_CLKPWR_CLK_SPI_ENABLE       (HW_CLKPWR_BASE + 0x48)
#define ADDR_CLKPWR_CLK_SPI_DIV          (HW_CLKPWR_BASE + 0x4C)
#define ADDR_CLKPWR_CLK_UARTA_ENABLE     (HW_CLKPWR_BASE + 0x50)
#define ADDR_CLKPWR_CLK_UARTA_N          (HW_CLKPWR_BASE + 0x54)
#define ADDR_CLKPWR_CLK_UARTA_M          (HW_CLKPWR_BASE + 0x58)
#define ADDR_CLKPWR_CLK_UARTB_ENABLE     (HW_CLKPWR_BASE + 0x5C)
#define ADDR_CLKPWR_CLK_UARTB_N          (HW_CLKPWR_BASE + 0x60)
#define ADDR_CLKPWR_CLK_UARTB_M          (HW_CLKPWR_BASE + 0x64)
#define ADDR_CLKPWR_CLK_UARTC_ENABLE     (HW_CLKPWR_BASE + 0x68)
#define ADDR_CLKPWR_CLK_UARTC_N          (HW_CLKPWR_BASE + 0x6C)
#define ADDR_CLKPWR_CLK_UARTC_M          (HW_CLKPWR_BASE + 0x70)
#define ADDR_CLKPWR_DSPSRST_ENABLE       (HW_CLKPWR_BASE + 0x74)
#define ADDR_CLKPWR_IRPC_ENABLE          (HW_CLKPWR_BASE + 0x9C)
#define ADDR_CLKPWR_MCLK_STABLE_TIME     (HW_CLKPWR_BASE + 0x80)
#define ADDR_CLKPWR_PC_0_ENABLE          (HW_CLKPWR_BASE + 0x84)
#define ADDR_CLKPWR_PCHOSTPD_ENABLE      (HW_CLKPWR_BASE + 0x88)
#define ADDR_CLKPWR_PLL48_STABLE_TIME    (HW_CLKPWR_BASE + 0x8C)
#define ADDR_CLKPWR_PLL48CNTEN           (HW_CLKPWR_BASE + 0x90)
#define ADDR_CLKPWR_PLL_STABLE_TIME      (HW_CLKPWR_BASE + 0x94)
#define ADDR_CLKPWR_PLLCNTEN             (HW_CLKPWR_BASE + 0x98)
#define ADDR_CLKPWR_PMSM_ENABLE          (HW_CLKPWR_BASE + 0x9C)
#define ADDR_CLKPWR_PWDPLL_ENABLE        (HW_CLKPWR_BASE + 0xA0)
#define ADDR_CLKPWR_USBPLL_ENABLE        (HW_CLKPWR_BASE + 0xA4)
#define ADDR_CLKPWR_USBPLL_OEN           (HW_CLKPWR_BASE + 0xA8)
#define ADDR_CLKPWR_CLK_GPCK_ENABLE      (HW_CLKPWR_BASE + 0xAC)
#define ADDR_CLKPWR_CLK_GPCK_DIV         (HW_CLKPWR_BASE + 0xB0)
#define ADDR_CLKPWR_CLK_MSPRO_ENABLE     (HW_CLKPWR_BASE + 0xB4)
#define ADDR_CLKPWR_CLK_MSPRO_DIV        (HW_CLKPWR_BASE + 0xB8)
#define ADDR_CLKPWR_CLK_SEL_MODE         (HW_CLKPWR_BASE + 0xF4)
#define ADDR_CLKPWR_CLK_PLL_STATUS       (HW_CLKPWR_BASE + 0x128)

#ifdef CONFIG_ARCH_BCM2157

#define ADDR_CLKPWR_CLK_APPSPLL_DIVIDERS		(HW_CLKPWR_BASE + 0xF8)
#define ADDR_CLKPWR_CLK_APPSPLL_FRAC_DIVIDERS		(HW_CLKPWR_BASE + 0xFC)
#define ADDR_CLKPWR_CLK_APPSPLL_MODE			(HW_CLKPWR_BASE + 0x100)
#define ADDR_CLKPWR_CLK_I2S_FRAC_MODE			(HW_CLKPWR_BASE + 0x1A8)
#define ADDR_CLKPWR_CLK_CMI_ENABLE			(HW_CLKPWR_BASE + 0x12C)
#define ADDR_CLKPWR_CLK_CMI_MODE			(HW_CLKPWR_BASE + 0x130)
#define ADDR_CLKPWR_CLK_PWM_ENABLE			(HW_CLKPWR_BASE + 0x1c4)
#define ADDR_CLKPWR_CLK_PWM_SOFT_RESET			(HW_CLKPWR_BASE + 0x1c8)
#define ADDR_CLKPWR_CLK_AP_POWER_MODES			(HW_CLKPWR_BASE + 0x1F4)
#define ADDR_CLKPWR_CLK_PM_SLEEP_REQ_MON		(HW_CLKPWR_BASE + 0x1F8)
#define ADDR_CLKPWR_CLK_PM_SLEEP_REQ_MASK		(HW_CLKPWR_BASE + 0x1FC)
#define ADDR_CLKPWR_CLK_PM_SLEEP_REQ_FORCE		(HW_CLKPWR_BASE + 0x200)
#define ADDR_CLKPWR_CLK_SC_EX_DBG_MON_CTRL		(HW_CLKPWR_BASE + 0x2B0)
#define ADDR_CLKPWR_CLK_SYSCLK_DEBUG_MON1		(HW_CLKPWR_BASE + 0x2B4)
#define ADDR_CLKPWR_CLK_SYSCLK_DEBUG_MON2		(HW_CLKPWR_BASE + 0x2B8)
#define ADDR_CLKPWR_CLK_PLL_ARESET_CTRL			(HW_CLKPWR_BASE + 0x2BC)

/*SDIO DIV maks*/
#define CLK_SDIO_DIV_48_24_SEL		(1<<11)
#define CLK_SDIO_DIV_24_SEL		(1<<12)
#define CLK_SDIO_DIV_48_24_EN		(1<<13)
#define CLK_SDIO_DIV_DIVIDER_MASK	0x7FF

/*SPI DIV*/
#define CLK_SPI_DIV_104_EN		(1<<3)

#endif

#define CLK_ARMAHB_MODE_FREQ_MASK                           0xFFFFFFF0	/* and with this value to clear freq set bits. */
#define CLK_ARMAHB_MODE_ARM11_52_ARM9_52_AHB_52_VCO_624     0x00000000
#define CLK_ARMAHB_MODE_ARM11_78_ARM9_78_AHB_78_VCO_624     0x00000001
#define CLK_ARMAHB_MODE_ARM11_104_ARM9_52_AHB_52_VCO_624    0x00000002
#define CLK_ARMAHB_MODE_ARM11_104_ARM9_104_AHB_104_VCO_624  0x00000003
#define CLK_ARMAHB_MODE_ARM11_156_ARM9_78_AHB_78_VCO_624    0x00000004
#define CLK_ARMAHB_MODE_ARM11_13_ARM9_13_AHB_13_VCO_624     0x00000005
#define CLK_ARMAHB_MODE_ARM11_156_ARM9_52_AHB_52_VCO_624    0x00000006
#define CLK_ARMAHB_MODE_ARM11_208_ARM9_104_AHB_104_VCO_624  0x00000007
#define CLK_ARMAHB_MODE_ARM11_312_ARM9_208_AHB_104_VCO_624  0x00000008

/* CLK_CAMCK */
#define CLK_CAMCK_ENABLE 0x00000001
#define CLK_CAMCK_DISABLE 0

#define CLK_CAMCK_MODE_MASK              0xFFFFFFF8
#define CLK_CAMCK_MODE_12                0x00000000
#define CLK_CAMCK_MODE_13                0x00000001
#define CLK_CAMCK_MODE_24                0x00000002
#define CLK_CAMCK_MODE_26                0x00000003
#define CLK_CAMCK_MODE_48                0x00000004

/* CLK_DSP_MODE */
#define CLK_DSP_MODE_MASK                0xFFFFFFF8
#define CLK_DSP_MODE_52                  0x00000000
#define CLK_DSP_MODE_78                  0X00000001
#define CLK_DSP_MODE_104                 0x00000002
#define CLK_DSP_MODE_125                 0x00000003
#define CLK_DSP_MODE_156                 0x00000004

/* CLK_I2S_INT */
#define CLK_I2S_INT_ENABLE               0x00000001
#define CLK_I2S_INT_DISABLE              0x00000000

#define CLK_I2S_INT_MODE_12              0x00000000
#define CLK_I2S_INT_MODE_6               0x00000001
#define CLK_I2S_INT_MODE_3               0x00000002
#define CLK_I2S_INT_MODE_1_5             0x00000003

#define CLK_I2S_EXT_ENABLE               0x00000001
#define CLK_I2S_EXT_DISABLE              0x00000000

/* CLK_DAMCK */
#define CLK_DAMCK_ENABLE                 0x00000001
#define CLK_DAMCK_DISABLE                0x00000000

/* CLK_MONITR */
#define CLK_MONITOR_ENABLE               0X00000001
#define CLK_MONITOR_DISABLE              0X00000000

/* CLK_MONITOR_SELECT : for debugging purposes. */
#define CLK_MONITOR_SELECT_156               0x00000000
#define CLK_MONITOR_SELECT_104               0x00000001
#define CLK_MONITOR_SELECT_78                0x00000002
#define CLK_MONITOR_SELECT_52                0x00000003
#define CLK_MONITOR_SELECT_26                0x00000004
#define CLK_MONITOR_SELECT_13                0x00000005
#define CLK_MONITOR_SELECT_1                 0x00000006
#define CLK_MONITOR_SELECT_32K               0x00000009
#define CLK_MONITOR_SELECT_48M               0x0000000A
#define CLK_MONITOR_SELECT_CLK_ARM           0x0000000B
#define CLK_MONITOR_SELECT_CLK_AHB           0x0000000C
#define CLK_MONITOR_SELECT_CLK_DSP           0x0000000D
#define CLK_MONITOR_SELECT_CLK_I2S           0x0000000E
#define CLK_MONITOR_SELECT_CLK_SDIO1         0x0000000F
#define CLK_MONITOR_SELECT_CLK_SDIO2         0x00000010
#define CLK_MONITOR_SELECT_CLK_SPI           0x00000011
#define CLK_MONITOR_SELECT_CLK_UARTA         0x00000012
#define CLK_MONITOR_SELECT_CLK_UARTB         0x00000013
#define CLK_MONITOR_SELECT_CLK_UARTC         0x00000014
#define CLK_MONITOR_SELECT_CLK_SM            0x00000015

/* CLK_PDPCK : Peripheral clock. */
#define CLK_PDPCK_ENABLE                     0x00000001
#define CLK_PDPCK_DISABLE                    0x00000000

/* CLK_PDPCK_DIV */

/* CLK_SDIO1 */
#define CLK_SDIO1_ENABLE                     0x00000001
#define CLK_SDIO1_DISABLE                    0x00000000

#define CLK_SDIO1_DIV_MASK                   0xFFFFC000	/* and with this value to clear div bits. */
/* CLK SDIO2 */
#define CLK_SDIO2_ENABLE                     0x00000001
#define CLK_SDIO2_DISABLE                    0x00000000

#define CLK_SDIO2_DIV_MASK                   0xFFFFC000	/* and with this value to clear div bits. */

/* CLK UARTS. */
#define CLK_UART_ENABLE                      0x00000001
#define CLK_UART_DISABLE                     0

#define CLK_UART_N_M_MASK                    0xFFFFFE00	/* and with this to clear n, m fields of n,m registers. */

#endif
