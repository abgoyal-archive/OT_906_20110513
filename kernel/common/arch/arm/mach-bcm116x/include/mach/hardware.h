/*
 *
 *  This file contains the hardware definitions of the BCM116X.
 *
 *  Copyright (C) 1999 ARM Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/include/mach/hardware.h
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

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>
#include <mach/memory.h>

/* Hardware addresses of major areas.
 *  *_START is the physical address
 *  *_SIZE  is the size of the region
 *  *_BASE  is the virtual address
 */
#define RAM_START               PHYS_OFFSET

/* #define RAM_SIZE                SZ_32M */
#define RAM_SIZE                (CFG_GLOBAL_RAM_SIZE - CFG_GLOBAL_RAM_SIZE_RESERVED)
#define RAM_BASE                PAGE_OFFSET

#define IO_START                0x08000000	/* PA of IO */
#define IO_BASE                 0xF8000000	/* VA of IO */

#define pcibios_assign_all_busses()	1

#ifndef __ARMEB__
#define BCM_BYTE_OFFSET	0
#define BCM_WORD_OFFSET	0
#else
#define BCM_BYTE_OFFSET	3
#define BCM_WORD_OFFSET	2
#endif

#define __REG32(x)   (*((volatile u32 *)(x)))
#define __REG16(x)   (*((volatile u16 *)(x)))
#define __REG8(x) (*((volatile u8  *)(x)))

#ifndef HW_IO_PHYS_TO_VIRT
#define HW_IO_PHYS_TO_VIRT(x)  ((x) - IO_START + IO_BASE)
#define HW_IO_VIRT_TO_PHYS(x)  ((x) - IO_BASE + IO_START)
#endif

/* used in C code, so cast to proper type */
#define io_p2v(x) (__io(x) - IO_START + IO_BASE)
#define io_v2p(x) (__io(x) - IO_BASE + IO_START)

/* used in asm code, so no casts */
#define IO_ADDRESS(x) ((x) - IO_START + IO_BASE)

#define HW_ONAD_BASE       HW_IO_PHYS_TO_VIRT(0x20000)	/* 32 M max */

#define HW_NOR0_BASE       HW_IO_PHYS_TO_VIRT(0x00000000)	/* 32 M max */
#define HW_NOR1_BASE       HW_IO_PHYS_TO_VIRT(0x02000000)	/* 32 M */
#define HW_NOR2_BASE       HW_IO_PHYS_TO_VIRT(0x04000000)	/* 32 M */
#define HW_SRAM0_BASE      HW_IO_PHYS_TO_VIRT(0x06000000)	/* 16 M */
#define HW_SRAM1_BASE      HW_IO_PHYS_TO_VIRT(0x07000000)	/* 16 M */
#define HW_NAND_BASE       HW_IO_PHYS_TO_VIRT(0x08000000)	/* NAND Flash */
#define HW_AHB_BASE        HW_IO_PHYS_TO_VIRT(0x08010000)	/* 64 K */
#define HW_DMA_BASE        HW_IO_PHYS_TO_VIRT(0x08020000)	/* 64 K */
#define HW_LCD_INTF_BASE   HW_IO_PHYS_TO_VIRT(0x08030000)	/* 64 K */
#define HW_GRAPHICS_BASE   HW_IO_PHYS_TO_VIRT(0x08040000)	/* 64 K */
#define HW_CAMERA_BASE     HW_IO_PHYS_TO_VIRT(0x08050000)	/* 64 K */
#define HW_DCT_BASE        HW_IO_PHYS_TO_VIRT(0x08060000)	/* DCT/IDCT/Quantization functions */
#define HW_EDGE_MP_BASE    HW_IO_PHYS_TO_VIRT(0x08070000)	/* Edge Message Processor */
#define HW_UMI_BASE        HW_IO_PHYS_TO_VIRT(0x08090000)	/* Universal Memory Interface Ctrl Register */
#define HW_DATAPACKER_BASE HW_IO_PHYS_TO_VIRT(0x080a0000)	/* Data Packer */
#define HW_DES_ENGINE_BASE HW_IO_PHYS_TO_VIRT(0x080b0000)	/* DES Engine */
#define HW_TV_OUTPUT_BASE  HW_IO_PHYS_TO_VIRT(0x08100000)	/* TV Output */
#define HW_SDIO0_BASE      HW_IO_PHYS_TO_VIRT(0x08110000)	/* SDIO 0 */
#define HW_SDIO1_BASE      HW_IO_PHYS_TO_VIRT(0x08120000)	/* SDIO 1 */
#define HW_SMI_BASE        HW_IO_PHYS_TO_VIRT(0x08130000)	/* Shared Memory Interface Control Register */
#define HW_DSP_DMA_BASE    HW_IO_PHYS_TO_VIRT(0x08138000)	/* DSP-DMA Interface Control Register */
#define HW_CLKPWR_BASE     HW_IO_PHYS_TO_VIRT(0x08140000)	/* Clock/Power Management */
#define HW_COLORSPACE_BASE HW_IO_PHYS_TO_VIRT(0x08170000)	/* Colorspace Conversion */
#define HW_CRC_BASE        HW_IO_PHYS_TO_VIRT(0x08180000)	/* CRC Base */
#define HW_JPEG_ACCEL_BASE HW_IO_PHYS_TO_VIRT(0x08190000)	/* Jpeg Accelerator */
#define HW_HANTRO_DEC_BASE HW_IO_PHYS_TO_VIRT(0x081a0000)	/* Hantro Decoder */
#define HW_HANTRO_ENC_BASE HW_IO_PHYS_TO_VIRT(0x081a8000)	/* Hantro Encoder */
#define HW_USBOTG_BASE     HW_IO_PHYS_TO_VIRT(0x08200000)	/* USB On-the-go */
#define HW_ITCM_BASE       HW_IO_PHYS_TO_VIRT(0x08400000)	/* Instruction tightly coupled memory */
#define HW_DTCM_BASE       HW_IO_PHYS_TO_VIRT(0x08700000)	/* Data tightly coupled memory */
#define HW_SMT_BASE        HW_IO_PHYS_TO_VIRT(0x08800000)	/* Sleep mode timer */
#define HW_IRQ_BASE        HW_IO_PHYS_TO_VIRT(0x08810000)	/* Interrupt controller */
#define HW_UART_A_BASE     HW_IO_PHYS_TO_VIRT(0x08820000)	/* UART A base */
#define HW_UART_B_BASE     HW_IO_PHYS_TO_VIRT(0x08821000)	/* UART B base */
#define HW_UART_C_BASE     HW_IO_PHYS_TO_VIRT(0x08822000)	/* UART C base */
#define HW_PTIMER_BASE     HW_IO_PHYS_TO_VIRT(0x08830000)	/* Periodic timer (buzzer/backlight) */
#define HW_AUXADC_BASE     HW_IO_PHYS_TO_VIRT(0x08830020)	/* Auxiliary ADC control */
#define HW_GPTIMER_BASE    HW_IO_PHYS_TO_VIRT(0x08830100)	/* General purpose timer */
#define HW_SIM_BASE        HW_IO_PHYS_TO_VIRT(0x08860000)	/* SIM interface */
#define HW_SLOWCLK_BASE    HW_IO_PHYS_TO_VIRT(0x08870010)	/* Slow clock calibration */
#define HW_SYS_BASE        HW_IO_PHYS_TO_VIRT(0x08880000)	/* System Configuration */
#define HW_GPRS_BASE       HW_IO_PHYS_TO_VIRT(0x08890000)	/* GPRS encryption */
#define HW_WDT_BASE        HW_IO_PHYS_TO_VIRT(0x088A0000)	/* Watchdog timer */
#define HW_WDT_BASE1       HW_IO_PHYS_TO_VIRT(0x088A0010)	/* Watchdog timer 2 */
#define HW_I2C_BASE        HW_IO_PHYS_TO_VIRT(0x088A0020)	/* I2C interface */
#define HW_I2C2_BASE       HW_IO_PHYS_TO_VIRT(0x088B0020)	/* I2C interface */
#define HW_I2S_BASE        HW_IO_PHYS_TO_VIRT(0x088C0000)	/* I2S interface (aka Digital audio interface) */
#define HW_GPIO_BASE       HW_IO_PHYS_TO_VIRT(0x088CE000)	/* GPIO interface */
#define HW_KEYPAD_BASE     HW_IO_PHYS_TO_VIRT(0x088CE080)	/* Keypad control */
#define HW_SPI_BASE        HW_IO_PHYS_TO_VIRT(0x088D0000)	/* SPI interface */
#define HW_PCMCIA_BASE     HW_IO_PHYS_TO_VIRT(0x088E0000)	/* PCMCIA interface */
#define HW_USB_DEVICE_BASE HW_IO_PHYS_TO_VIRT(0x088F0000)	/* USB Device interface */
#define HW_SCRATCHRAM_BASE HW_IO_PHYS_TO_VIRT(0x28000000)	/* Scratch Ram base - 64 KB, 16MB allocated to 0x28ffffff */
#define HW_SDRAM_BASE      HW_IO_PHYS_TO_VIRT(0x80000000)	/* SDRAM base - 64 MB max */
#define HW_SDRAM2_BASE     HW_IO_PHYS_TO_VIRT(0x84000000)	/* SDRAM2 base - 64 MB max */
#define HW_CRYPTO_BASE     HW_IO_PHYS_TO_VIRT(0x0c080000)	/* Crypto base, */
#define HW_SECURITY_APB_BASE HW_IO_PHYS_TO_VIRT(0x0c0c0000)	/* Security APB base */
#define HW_PKI_BASE        HW_IO_PHYS_TO_VIRT(0x0c0c8000)	/* PKI base */
#define HW_OTP_BASE        HW_IO_PHYS_TO_VIRT(0x0c0c9000)	/* OTP base */
#define HW_RNG_BASE        HW_IO_PHYS_TO_VIRT(0x0c0ca000)	/* RNG base */
#define HW_SECURE_RTC_BASE HW_IO_PHYS_TO_VIRT(0x0c0cb000)	/* Secure RTC base */
#define HW_ARM9_BOOT_BASE  HW_IO_PHYS_TO_VIRT(0xffff0000)	/* ARM9 Boot Module base */
#define HW_HEADSET_BASE    HW_IO_PHYS_TO_VIRT(0x08900000)	/* Headset address mapping */
#define AUXMIC_DETECT_BASE HW_IO_PHYS_TO_VIRT(0x08911000)	/* AuxMic Detect */
#define HW_ANACR0_REG_BASE HW_IO_PHYS_TO_VIRT(0x08880080)	/* Analog register */
#ifdef CONFIG_ARCH_BCM2157
#define HW_PWM_BASE	   HW_IO_PHYS_TO_VIRT(0x08940000)
#endif

#endif
