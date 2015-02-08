/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/include/mach/reg_sys.h
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

#if !defined(__ASM_ARCH_REG_SYS_H)
#define __ASM_ARCH_REG_SYS_H

/* ---- Include Files ---------------------------------------------------- */
#include <mach/hardware.h>

/* ---- Constants and Types ---------------------------------------------- */

#define REG_SYS_IOCR0	    __REG32(HW_SYS_BASE + 0x00)	/* IO control register0 */
#define REG_SYS_IOCR1       __REG32(HW_SYS_BASE + 0x04)	/* IO control register1 */
#define REG_SYS_SUCR        __REG32(HW_SYS_BASE + 0x08)	/* Startup mode register */
#define REG_SYS_IOCR2       __REG32(HW_SYS_BASE + 0x0c)	/* IO control register2 */
#define REG_SYS_IOCR3       __REG32(HW_SYS_BASE + 0x1c)	/* IO control register3 */
#define REG_SYS_IOCR4       __REG32(HW_SYS_BASE + 0x20)	/* IO control register4 */
#define REG_SYS_IOCR5       __REG32(HW_SYS_BASE + 0x24)	/* IO control register5 */
#define REG_SYS_IOCR6       __REG32(HW_SYS_BASE + 0x28)	/* IO control register6 */
#define REG_SYS_IOCR7       __REG32(HW_SYS_BASE + 0x2c)	/* IO control register7 */
#define REG_SYS_IOCR8       __REG32(HW_SYS_BASE + 0x30)	/* IO control register8 */
#define REG_SYS_PIDR        __REG32(HW_SYS_BASE + 0x10)	/* Product ID register */
#define REG_SYS_DSPCTRL     __REG32(HW_SYS_BASE + 0x14)	/* DSP Control register */
#define REG_SYS_PUMR        __REG32(HW_SYS_BASE + 0x18)	/* Power Up Mode register */
#define REG_SYS_MCR         __REG32(HW_SYS_BASE + 0x40)	/* BootROM remap register */
#define REG_SYS_MRR         __REG32(HW_SYS_BASE + 0x44)	/* BootROM restore register */
#define REG_SYS_RAMCTRL     __REG32(HW_SYS_BASE + 0x48)	/* RAM Control register */
#define REG_SYS_SECCTRL     __REG32(HW_SYS_BASE + 0x50)	/* Security Control register */
#define REG_SYS_SECSTAT     __REG32(HW_SYS_BASE + 0x54)	/* Security Status register */
#define REG_SYS_ANACR0      __REG32(HW_SYS_BASE + 0x80)	/* Analog configuration registers */
#define REG_SYS_ANACR1      __REG32(HW_SYS_BASE + 0x84)
#define REG_SYS_ANACR2      __REG32(HW_SYS_BASE + 0x88)
#define REG_SYS_ANACR3      __REG32(HW_SYS_BASE + 0x8c)
#define REG_SYS_ANACR4      __REG32(HW_SYS_BASE + 0x90)
#define REG_SYS_ANACR5      __REG32(HW_SYS_BASE + 0x94)
#define REG_SYS_ANACR6      __REG32(HW_SYS_BASE + 0x98)
#define REG_SYS_ANACR7      __REG32(HW_SYS_BASE + 0x9c)
#define REG_SYS_ANACR8      __REG32(HW_SYS_BASE + 0xa0)
#define REG_SYS_ANACR9      __REG32(HW_SYS_BASE + 0xa4)
#define REG_SYS_ANACR10     __REG32(HW_SYS_BASE + 0xa8)
#define REG_SYS_ANACR11     __REG32(HW_SYS_BASE + 0xac)
#define REG_SYS_ANACR12     __REG32(HW_SYS_BASE + 0xb0)
#define REG_SYS_ANACR13     __REG32(HW_SYS_BASE + 0xb4)
#define REG_SYS_ANACR14     __REG32(HW_SYS_BASE + 0xb8)
#define REG_SYS_ANACR15     __REG32(HW_SYS_BASE + 0xbc)
#define REG_SYS_IRDROP0     __REG32(HW_SYS_BASE + 0xc0)
#define REG_SYS_IRDROP1     __REG32(HW_SYS_BASE + 0xc4)
#define REG_SYS_IRDROP2     __REG32(HW_SYS_BASE + 0xc8)
#define REG_SYS_IOCR10      __REG32(HW_SYS_BASE + 0x204)	/* IO control register10 */

#ifdef CONFIG_ARCH_BCM2157
#define ADDR_SYSCFG_AHB_CLK_GATE_MASK				(HW_SYS_BASE + 0xD8)
#define ADDR_SYSCFG_AHB_CLK_GATE_FORCE				(HW_SYS_BASE + 0xDC)
#define ADDR_SYSCFG_AHB_CLK_GATE_MONITOR_RAW			(HW_SYS_BASE + 0xE0)
#define ADDR_SYSCFG_AHB_CLK_GATE_MONITOR			(HW_SYS_BASE + 0xE4)
#define ADDR_SYSCFG_VIDEO_CODEC_AHB_CLK_EN			(HW_SYS_BASE + 0x100)
#define ADDR_SYSCFG_CAMERA_INTERFACE_AHB_CLK_EN			(HW_SYS_BASE + 0x104)
#define ADDR_SYSCFG_USB_AHB_CLK_EN				(HW_SYS_BASE + 0x108)
#define ADDR_SYSCFG_GEA_AHB_CLK_EN				(HW_SYS_BASE + 0x10C)
#define ADDR_SYSCFG_CRYPTO_AHB_CLK_EN				(HW_SYS_BASE + 0x110)
#define ADDR_SYSCFG_UARTA_AHB_CLK_EN				(HW_SYS_BASE + 0x118)
#define ADDR_SYSCFG_UARTB_AHB_CLK_EN				(HW_SYS_BASE + 0x11C)
#define ADDR_SYSCFG_DA_AHB_CLK_EN				(HW_SYS_BASE + 0x120)
#define ADDR_SYSCFG_MPCLK_AHB_CLK_EN				(HW_SYS_BASE + 0x124)
#define ADDR_SYSCFG_LCD_AHB_CLK_EN				(HW_SYS_BASE + 0x12C)
#define ADDR_SYSCFG_GE_AHB_CLK_EN				(HW_SYS_BASE + 0x130)
#define ADDR_SYSCFG_DMAC_AHB_CLK_EN				(HW_SYS_BASE + 0x134)
#define ADDR_SYSCFG_SDIO1_AHB_CLK_EN				(HW_SYS_BASE + 0x138)
#define ADDR_SYSCFG_SDIO2_AHB_CLK_EN				(HW_SYS_BASE + 0x13C)
#define ADDR_SYSCFG_DES_AHB_CLK_EN				(HW_SYS_BASE + 0x144)
#define ADDR_SYSCFG_RNG_AHB_CLK_EN				(HW_SYS_BASE + 0x150)
#define ADDR_SYSCFG_MPHI_AHB_CLK_EN				(HW_SYS_BASE + 0x160)
#define ADDR_SYSCFG_TESTABILITY_ACCESS				(HW_SYS_BASE + 0x170)
#define ADDR_SYSCFG_DISABLE_OTP_REGION_READ_ACCESS		(HW_SYS_BASE + 0x174)
#define ADDR_SYSCFG_DISABLE_OTP_REGION_WRITE_ACCESS		(HW_SYS_BASE + 0x178)
#define ADDR_SYSCFG_OTP_DEVICE_STATUS				(HW_SYS_BASE + 0x17C)
#define ADDR_SYSCFG_CIPHER_FW_CLK_EN				(HW_SYS_BASE + 0x190)
#define ADDR_SYSCFG_SYSCONF_AHB_CLK_EXTEND0			(HW_SYS_BASE + 0x1A0)
#define ADDR_SYSCFG_SYSCONF_AHB_CLK_EXTEND1			(HW_SYS_BASE + 0x1A4)
#define ADDR_SYSCFG_OTP_WCDMA_CAT				(HW_SYS_BASE + 0x1C4)
#endif

#define AUXMIC_PRB_CYC_REG              (&__REG32(AUXMIC_DETECT_BASE))
#define AUXMIC_MSR_DLY_REG              (&__REG32(AUXMIC_DETECT_BASE + 0x04))
#define AUXMIC_MSR_INTVL_REG            (&__REG32(AUXMIC_DETECT_BASE + 0x08))
#define AUXMIC_CMC_REG                  (&__REG32(AUXMIC_DETECT_BASE + 0x0C))	/* Continuous Measurement Control Register */
#define AUXMIC_MIC_REG                  (&__REG32(AUXMIC_DETECT_BASE + 0x10))	/* Measurement Interval Control Register */
#define AUXMIC_AUXEN_REG                (&__REG32(AUXMIC_DETECT_BASE + 0x14))
#define AUXMIC_MICINTH_ADJ_REG          (&__REG32(AUXMIC_DETECT_BASE + 0x18))
#define AUXMIC_MICINENTH_ADJ_REG        (&__REG32(AUXMIC_DETECT_BASE + 0x1C))
#define AUXMIC_MICONTH_ADJ_REG          (&__REG32(AUXMIC_DETECT_BASE + 0x20))
#define AUXMIC_MICONENTH_ADJ_REG        (&__REG32(AUXMIC_DETECT_BASE + 0x24))
#define AUXMIC_F_PWRDWN_REG             (&__REG32(AUXMIC_DETECT_BASE + 0x28))
#define AUXMIC_MICINTH_DEF_REG          (&__REG32(AUXMIC_DETECT_BASE + 0x30))
#define AUXMIC_MICINENTH_DEF_REG        (&__REG32(AUXMIC_DETECT_BASE + 0x34))
#define AUXMIC_MICONTH_DEF_REG          (&__REG32(AUXMIC_DETECT_BASE + 0x38))
#define AUXMIC_MICONENTH_DEF_REG        (&__REG32(AUXMIC_DETECT_BASE + 0x3C))
#define AUXMIC_MICINTH_REG              (&__REG32(AUXMIC_DETECT_BASE + 0x40))
#define AUXMIC_MICONTH_REG              (&__REG32(AUXMIC_DETECT_BASE + 0x44))

/* REG_SYS_IOCR0 bits */
#define REG_SYS_IOCR0_SPI_RXD           0x80000000	/* Select SPI RXD MUX */
						   /* 0: SD1DAT0 */
						   /* 1: LCDRE */
#define REG_SYS_IOCR0_LCD_MASK          0x30000000	/* lcd/spi */
						   /* 0: LCD pin select(Z80) */
						   /* 1: LCD pin select(M68) */
						   /* 2: SPI pin select */
						   /* 3: SPI pin select */
#define REG_SYS_IOCR0_GPEN_MASK         0x0ffff000	/* GPEN/GPIO/RF/TX/RX */
						   /* refer to pin mux table */
#define REG_SYS_IOCR0_PCM               0x00000800	/* 0:PCM, 1:GPIO[51:48] */
#define REG_SYS_IOCR0_BUZZER            0x00000400	/* 0:GPIO16 */
						   /* 1:Buzzer */
#define REG_SYS_IOCR0_BKLIGHT           0x00000200	/* 0:GPIO17 1: Backlight */
#define REG_SYS_IOCR0_1WIRE	        0x00000080	/* 0:GPIO20 1: D1W */
#define REG_SYS_IOCR0_GPIO_I2S          0x00000040	/* GPIO / I2S */
						   /* 0:I2S 1:GPIO[47:44] */
#define REG_SYS_IOCR0_MSPRO_SD2         0x00000020	/* MSPRO/SD2 */
						   /* 0:SD2 */
						   /* 1:MSPRO */
#define REG_SYS_IOCR0_SPI_GPIO_MASK     0x00000018	/* SDIO/SPI/GPIO */
						   /* 0:SD1 */
						   /* 1:SPI pin select */
						   /* 2 or 3:GPIO[37:32] */
#define REG_SYS_IOCR0_SPI_UARTA_MUX     0x00000004	/* 0:UARTA functionality */
						   /* 1:SPI functionality */
#define REG_SYS_IOCR0_UARTA_DAI         0x00000002	/* UART-A/DAI */
						   /* 0:UART_A DPDCD select */
						   /* 1:Monitor Clock on DPDCD */
#define REG_SYS_IOCR0_CAM_TVO           0x00000001	/* Camera/TVO dac select */
						   /* 0:Camera 1:TVO */

/* REG_SYS_IOCR1 bits */
#define REG_SYS_IOCR1_KEY_COL_MASK	0x0000FF00	/* 0: GPIO[15:8] */
						   /* 1: Keypad Column [7:0] */
#define REG_SYS_IOCR1_KEY_ROW_MASK	0x000000FF	/* 0: GPIO[7:0] */
						   /* 1: Keypad Row [7:0] */

/* REG_SYS_IOCR2 bits */
#define REG_SYS_IOCR2_SD2DAT_PULL_MASK	0xC0000000	/* SD2DAT pull */
						   /* 00: no pull 01: pullup */
						   /* 10: pulldown */
						   /* 11:Illegal cause leakage */
#define REG_SYS_IOCR2_SD2CMD_PULL_MASK	0x30000000	/* SD2CMD pull */
						   /* 00: no pull 01: pullup */
						   /* 10: pulldown */
						   /* 11: Illegal cause leakage */
#define REG_SYS_IOCR2_SD2CK_PULL_MASK 	0x0C000000	/* SD2CK pull */
						   /* 00: no pull 01: pullup */
						   /* 10: pulldown */
						   /* 11: Illegal cause leakage */
#define REG_SYS_IOCR2_WCDMA_DBG_CLK2 	0x02000000	/* 0: RFSLEEPCTRL2 */
						   /* 1: Wcdma Debug Clock 2 */
#define REG_SYS_IOCR2_WCDMA_DBG_CLK1 	0x01000000	/* 0: RFSLEEPCTRL1 */
						   /* 1: Wcdma Debug Clock 1 */
#define REG_SYS_IOCR2_I2C2_OEB_SHIFT	(23)	/* OEB ctrl I2C2 SCL n SDA */
						  /* 0: pads on 1: pads off */
#define REG_SYS_IOCR2_I2C2_OEB_MASK	(1 << REG_SYS_IOCR2_I2C2_OEB_SHIFT)
#define REG_SYS_IOCR2_I2C2_SLEW_SHIFT	(21)	/* Slew control for I2C2 */
						  /* SCL and SDA pads */
						  /* Max. in ns (rise; fall) */
						  /* for 10-100 pF load */
						  /* 00 (42;39) 01 (84;80) */
						  /* 10 (124;121) 11 (29;22) */
#define REG_SYS_IOCR2_I2C2_SLEW_MASK 	(3 << REG_SYS_IOCR2_I2C2_SLEW_SHIFT)
#define REG_SYS_IOCR2_DAT_PULLUP 	0x00001000	/* Pull up for UMI DAT */
						   /* 0: disabled 1: enabled */
#define REG_SYS_IOCR2_GPIO31_16_SLEW 	0x00000800	/* Slew ctrl on GPIO31-16 */
						   /* 0 Normal slew */
						   /* 1 Slewed output */
#define REG_SYS_IOCR2_GPIO15_0_SLEW 	0x00000400	/* Slew ctrl on GPIO15-0 */
						   /* 0 Normal slew */
						   /* 1 Slewed output */
#define REG_SYS_IOCR2_VBUS_SEL 		0x00000080	/* Select PMU mode for OTG */
						   /* 0 Broadcom PMU support */
						   /* 1 Maxim PMU support */
#define REG_SYS_IOCR2_SIMDAT_HYSEN	0x00000040	/* SIMDAT pad hysteresis */
						   /* 0: disabled 1:enabled */
#define REG_SYS_IOCR2_OSC2_SELECT_MASK	0x00000018	/* Select sel1, sel0 of OSC2 */
#define REG_SYS_IOCR2_OSC2_ENABLE	0x00000008	/* Enable OSC1 o/p on GPIO25 */
#define REG_SYS_IOCR2_OSC1_SELECT	0x00000006	/* Select sel1, sel0 of OSC1 */
#define REG_SYS_IOCR2_OSC1_ENABLE	0x00000001	/* Enable OSC1 o/p on RFGPIO0 */

/* REG_SYS_SUCR bits */
#define REG_SYS_SUCR_SPARE	0x80000000	/* 0: USB download 1: UART download */
#define REG_SYS_SUCR_DEV_BOOT   0x40000000	/* DAT30 0: Rom Boot 1: NOR Boot */
#define REG_SYS_SUCR_DOWNLOAD	0x20000000	/* Download when NORBOOT==NOR */
					   /* 0 NOR Flash Boot */
					   /* 1 Download via UART or USB */
					   /* Nandtype when NORBOOT == NAND */
					   /* 0 512-byte page size */
					   /* 1 2048-byte page size */
#define REG_SYS_SUCR_AP		0x10000000	/* Audio Precision Testing (DAT28) */
					   /* 0 Normal mode */
					   /* 1 Enable Audio Precision */
#define REG_SYS_SUCR_TCKEN	0x08000000	/* TCKEN strap value (DAT27) */
					   /* 1 PDPCK, GPCK pads are disabled */
					   /* and turned into inputs */
					   /* 0 PDPCK, GPCK clks will be available */
#define REG_SYS_SUCR_NORBOOT	0x04000000	/* NORBOOT strap value (DAT26) */
					   /* 0x0 = NAND Boot */
					   /* 0x1 = NOR  Boot */
#define REG_SYS_SUCR_NAND8	0x02000000	/* NANDFlash Bus Width (DAT25) */
					   /* 0 16-bit  1 8-bit */
#define REG_SYS_SUCR_VCOBYPASS	0x00800000	/* VCO_BYPASS strap value (DAT24) */
					   /* 0 Normal mode-Clocks from PLL */
					   /* 1 PLL Bypassed-Clocks from GPIO3 */
#define REG_SYS_SUCR_DSPXPM	0x00800000	/* DSPXPM strap value (DAT23) */
#define REG_SYS_SUCR_DVLP	0x00400000	/* DVLP strap value (DAT22) */
					   /* 0 Production 1 Development */
#define REG_SYS_SUCR_JTAG_SEL   0x0001C000	/* JTAG Multi-Core daisy chain */
					   /* strap value of DAT20-18 */
					   /* 000 LV JTAP only (default) */
					   /* 001 LV JTAP only */
					   /* 010 LV JTAP only */
					   /* 011 Partial daisy(ARM11-ARM9) */
					   /* 100 Full daisy */
					   /* (LV_JTAP-DSP-ARM11-ARM9) */
					   /* 101 ARM11 110 ARM9 111 DSP */
#define REG_SYS_SUCR_D3G_DIGRF	0x00020000	/* DVLP strap value (DAT22) */
					   /* 0 3G_DIGRF not enabled */
					   /* 1 3G_DIGRF enabled */
#define REG_SYS_SUCR_BE		0x00010000	/* Endianess strap value (DAT16) */
					   /* 0 Chip in Little Endian mode */
					   /* 1 Chip in Big Endian mode */
#define REG_SYS_SUCR_SRST_STAT	0x00000010	/* Soft Reset Status: */
					   /* 0 Hard reset */
					   /* 1 Soft reset from ARM/DSP */
					   /* or Watchdog reset */
#define REG_SYS_SUCR_BOOTSRC	0x00000004	/* Boot Source Select: */
					   /* 0 Boot from internal boot ROM. */
					   /* 1 Boot from external NOR flash. */
#define REG_SYS_SUCR_DLM	0x00000001	/* Download program through UART. */

/* REG_SYS_PIDR bits */
#define REG_SYS_PIDR_PRODUCTID_MASK         0xf0	/* Product ID */
#define REG_SYS_PIDR_REVID_MASK             0x0f	/* Revision ID */

/* REG_SYS_DSPCTRL bits */
#define REG_SYS_DSPCTRL_DSPRST              0x80	/* 0:operating 1:reset */
#define REG_SYS_DSPCTRL_SYNCEXTPRAM         0x40	/* Obsolete: DSP only supports */
						 /* synchronous external PRAM */
#define REG_SYS_DSPCTRL_JTAGINTWAKE         0x20	/* Control whether DSP JTAG */
						 /* interrupt wakes up DSP from sleep mode */
						 /* 0 - DSP JTAG interrupt does not not wake DSP */
						 /* 1 = DSP JTAG interrupt  wakes up DSP */

/* REG_SYS_PUMR bits */
#define REG_SYS_PUMR_PUMODE_MASK             0xFF	/* Power up mode */

#define REG_SYS_IOCR3_DA_DIS_SHIFT         (16)	/* 0 Normal operation */
						/* 1 I2S_SDI Ouput Disabled */
#define REG_SYS_IOCR3_DA_DIS_MASK       (1 << REG_SYS_IOCR3_DA_DIS_SHIFT)

#define REG_SYS_IOCR3_CAMD_PD_SHIFT	   (10)	/* 0 CAMD0-7 pd not selected */
						/* 1 CAMD0-7 pulldown */
#define REG_SYS_IOCR3_CAMD_PD_MASK	(1 << REG_SYS_IOCR3_CAMD_PD_SHIFT)
#define REG_SYS_IOCR3_CAMD_PU_SHIFT	   (9)	/* 0 CAMD0-7 pu not selected */
						/* 1 CAMD0-7 pullup */
#define REG_SYS_IOCR3_CAMD_PU_MASK	(1 << REG_SYS_IOCR3_CAMD_PU_SHIFT)
#define REG_SYS_IOCR3_CAMHVS_PD_SHIFT 	   (8)	/* 0 CAMHS/CAMVS pd not sel */
						/* 1 CAMHS/CAMVS pulldown */
#define REG_SYS_IOCR3_CAMHVS_PD_MASK 	(1 << REG_SYS_IOCR3_CAMHVS_PD_SHIFT)
#define REG_SYS_IOCR3_CAMHVS_PU_SHIFT	   (7)	/* 0 CAMHS/CAMVS pu not sel */
						/* 1 CAMHS/CAMVS pullup */
#define REG_SYS_IOCR3_CAMHVS_PU_MASK	(1 << REG_SYS_IOCR3_CAMHVS_PU_SHIFT)
#define REG_SYS_IOCR3_CAMDCK_PD_SHIFT      (6)	/* 0 CAMDCK pd not selected */
						/* 1 CAMDCK pulldown */
#define REG_SYS_IOCR3_CAMDCK_PD_MASK    (1 << REG_SYS_IOCR3_CAMDCK_PD_SHIFT)
#define REG_SYS_IOCR3_CAMDCK_PU_SHIFT 	   (5)	/* 0 CAMDCK pu not selected */
						/* 1 CAMDCK pullup */
#define REG_SYS_IOCR3_CAMDCK_PU_MASK 	(1 << REG_SYS_IOCR3_CAMDCK_PU_SHIFT)
#define REG_SYS_IOCR3_CAMCK_DIS_SHIFT	   (4)	/* 0 Normal operation */
						/* CAMCK output disabled,pd */
#define REG_SYS_IOCR3_CAMCK_DIS_MASK	(1 << REG_SYS_IOCR3_CAMCK_DIS_SHIFT)

#define REG_SYS_IOCR4_DAT_PULL_SHIFT		(31)
#define REG_SYS_IOCR4_LCD_SLEW_SHIFT		(30)
#define REG_SYS_IOCR4_SDIO2_DRIVE_SHIFT		(27)
#define REG_SYS_IOCR4_ETM_DRIVE_SHIFT		(24)
#define REG_SYS_IOCR4_LCD_DRIVE_SHIFT		(21)
#define REG_SYS_IOCR4_SIM_DRIVE_SHIFT		(18)
#define REG_SYS_IOCR4_RF_DRIVE_SHIFT		(15)
#define REG_SYS_IOCR4_CAMERA_DRIVE_SHIFT	(12)
#define REG_SYS_IOCR4_SDMCLK_DRIVE_SHIFT	(9)
#define REG_SYS_IOCR4_MBCK_DRIVE_SHIFT		(6)
#define REG_SYS_IOCR4_DAT31_15_DRIVE_SHIFT	(3)
#define REG_SYS_IOCR4_MEM_DRIVE_SHIFT		(0)
#define REG_SYS_IOCR4_DRIVE_MASK		(0x7)
#define REG_SYS_IOCR4_DRIVE_2mA			(0x1)
#define REG_SYS_IOCR4_DRIVE_4mA			(0x2)
#define REG_SYS_IOCR4_DRIVE_6mA			(0x4)
#define REG_SYS_IOCR4_DRIVE_8mA			(0x5)
#define REG_SYS_IOCR4_DRIVE_10mA		(0x6)
#define REG_SYS_IOCR4_DRIVE_12mA		(0x7)

/* REG_SYS_MCR bits - None: any write will remove the bootrom and remap the address 0x0 to external nor flash */

/* REG_SYS_MRR bits */
/* This register is to restore the bootrom mapping back. It is done by first writing pattern1 */
/* and then writing pattern2. */
#define REG_SYS_MRR_PATTERN1                0x5a5a5a5a
#define REG_SYS_MRR_PATTERN2                0x10c510c5

/* REG_SYS_RAMCTRL bits */
#define REG_SYS_RAMCTRL_SRAM_SAM_MASK       0xc
#define REG_SYS_RAMCTRL_SRAM_STBY_MASK      0x3

/* REG_SYS_SECCTRL bits */
/* These bits are one-way sticky.  Coming out of reset, the default is 0. */
/* Once a "1" is written to this bit,  it will be stuck high and will */
/* not be resetable to "0" unless the entire system is reset. */
#define REG_SYS_SECCTRL_CRYPTO_DIS          (1 << 4)	/* 0:Enable access to the DES and Crypto (AES/SHA1) */
						      /* 1:Disable access to the DES and Crypto (AES/SHA1) */
#define REG_SYS_SECCTRL_PKE_DIS             (1 << 3)	/* 0:Enable access to the PKE (Public Key Encryption Engine) */
						      /* 1:Disable access to the PKE (Public Key Encryption Engine) */
#define REG_SYS_SECCTRL_OTP_DIS             (1 << 2)	/* 0:Enable access to the OTP (secure storage) */
						      /* 1:Disable access to the OTP (secure storage) */
#define REG_SYS_SECCTRL_RTC_DIS_WR          (1 << 1)	/* 0:Enable write access to the RTC (real-time counter) */
						      /* 1:Disable write access to the RTC (real-time counter) */
#define REG_SYS_SECCTRL_BRM_DIS_RD          (1 << 0)	/* 0:Enable read access to the boot ROM */
						      /* 1:Disable read access to the boot ROM */
/* REG_SYS_SECSTAT bits */
#define REG_SYS_SECSTAT_JTAG_DIS            (1 << 10)	/* nvm_glb_disable_jtag (READ-ONLY) */
						      /* This is the post XORed signal to disable the JTAG logic. */

#define REG_SYS_SECSTAT_TSTDBG_DIS          (1 << 9)	/* nvm_glb_disable_scan_bist_debug (READ-ONLY) */
						      /* This is the post XORed signal to disable the scan, */
						      /* bist, and visibility (snoop) debug logic. */
#define REG_SYS_SECSTAT_CRYPTO_DIS          (1 << 8)	/* nvm_glb_disable_crypto (READ-ONLY) */
#define REG_SYS_SECSTAT_SEC_CONF_MASK       0xff	/* Secure Configuration Status (READ-ONLY) */
						      /* This is the 8-bit value of the system configuration bits. */
						      /* Coming out of reset, data is read out of address 0 of the NVM, */
						      /* and either the upper or lower byte is latched, based on the valid bits. */
						      /* Writing to the NVM will not affect these bits; these bits are only */
						      /* updated when coming out of reset. */
#define REG_SYS_SECSTAT_OTP_BLANK           0x03	/* OTP blank pattern */

#define REG_SYS_ANACR0_PWDDIG               (1<<11)
#define REG_SYS_ANACR0_PWDMIC               (1<<12)
#define REG_SYS_ANACR0_PWDAUXMIC            (1<<13)

#define REG_SYS_ANACR10_MICBIAS_ON          (1<<17)

#define REG_SYS_ANACR0_SIMVCC_SHIFT         8
#define REG_SYS_ANACR0_SIMVCC_MASK          0xFFFFE0FF

#define REG_SYS_ANACR0_SIM_LDO_1_2          0	/* 0 = 1.2V */
#define REG_SYS_ANACR0_SIM_LDO_1_3          1	/* 1 = 1.3V */
#define REG_SYS_ANACR0_SIM_LDO_1_4          2	/* 2 = 1.4V */
#define REG_SYS_ANACR0_SIM_LDO_1_5          3	/* 3 = 1.5V */
#define REG_SYS_ANACR0_SIM_LDO_1_6          4	/* 4 = 1.6V */
#define REG_SYS_ANACR0_SIM_LDO_1_7          5	/* 5 = 1.7V */
#define REG_SYS_ANACR0_SIM_LDO_1_8          6	/* 6 = 1.8V */
#define REG_SYS_ANACR0_SIM_LDO_1_9          7	/* 7 = 1.9V */
#define REG_SYS_ANACR0_SIM_LDO_2_0          8	/* 8 = 2.0V */
#define REG_SYS_ANACR0_SIM_LDO_2_1          9	/* 9 = 2.1V */
#define REG_SYS_ANACR0_SIM_LDO_2_2          10	/* A = 2.2V */
#define REG_SYS_ANACR0_SIM_LDO_2_3          11	/* B = 2.3V */
#define REG_SYS_ANACR0_SIM_LDO_2_4          12	/* C = 2.4V */
#define REG_SYS_ANACR0_SIM_LDO_2_5          13	/* D = 2.5V */
#define REG_SYS_ANACR0_SIM_LDO_2_6          14	/* E = 2.6V */
#define REG_SYS_ANACR0_SIM_LDO_2_7          15	/* F = 2.7V */
#define REG_SYS_ANACR0_SIM_LDO_2_8          16	/* 0x10 = 2.8V */
#define REG_SYS_ANACR0_SIM_LDO_2_9          17	/* 0x11 = 2.9V */
#define REG_SYS_ANACR0_SIM_LDO_3_0          18	/* 0x12 = 3.0V */
#define REG_SYS_ANACR0_SIM_LDO_3_1          19	/* 0x13 = 3.1V */
#define REG_SYS_ANACR0_SIM_LDO_3_2          20	/* 0x14 = 3.2V */

/* Bit definitions for Analog Configuration Register9 on baseband processor related to USB Device Mode */
#define ANACR9_USB_PPC_PWR_OFF_ANALOG_DRIVERS	0x00000001
#define ANACR9_USB_SELECT_DEVICE_MODE		0x00000002	/* If this bit is 0 then it selects host mode */
#define ANACR9_USB_SELECT_OTG_MODE		0x00000004	/* If this bit is 0 then it selects standard USB 2.0 mode */
#define ANACR9_USB_PLL_CAL_ENABLE	       	0x00000008
#define ANACR9_USB_PLL_POWER_ON		       	0x00000010	/* If this bit is 0 then it turns PLL power down */
#define ANACR9_USB_UTMI_SOFT_RESET_DISABLE      0x00000020	/* If this bit is 0 then it enables UTMI soft reset */
#define ANACR9_USB_UTMI_STOP_DIGITAL_CLOCKS     0x00000040	/* If this bit is 0 then port comes out of reset in power down */
#define ANACR9_USB_RESERVED_CLK_DURING_SUSP	0x40000000
#define ANACR9_USB_IDLE_ENABLE	              	0x00008000	/* If 1 HSOTG PHY goes to sleep mode */
#define ANACR9_PLL_SUSPEND_ENABLE		0x00000100	/* 0: USB suspend; 1: Normal mode */

/* REG_SYS_IRDROP bits */
/* The IRDROP monitors work by counting the number of pulses of a ring oscillator during 4 cycles of a 32.76KHz clock. */
/* The frequency of the ring oscillator varies between 1MHz and 3MHz depending upon the voltage at the oscillator. */
/* Software should enable the oscillator by writing MON_EN and waiting an appropriate amount of time before reading */
/* the CNT_OUT value. Given the frequencies of operation, the value of CNT_OUT will vary between 16F (no IR drop) */
/* and 7A (maximum IR drop). */
#define REG_SYS_IRDROP_OSC_EN               (1 << 11)	/* Not used */
#define REG_SYS_IRDROP_MON_EN               (1 << 10)	/* Write 1 to enable counter. Clear to 0 before re-initialization. */
#define REG_SYS_IRDROP_CNT_OUT_MASK         0x3ff	/* Software can read this value to determine IRDROP (Read-Only) */

#endif
