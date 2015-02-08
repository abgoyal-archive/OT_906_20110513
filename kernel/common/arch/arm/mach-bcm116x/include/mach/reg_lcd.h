/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/include/mach/reg_lcd.h
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
*  reg_lcd.h
*
*  PURPOSE:
*
*     This file contains definitions for the LCD registers
*
*  NOTES:
*
*****************************************************************************/

#if !defined(__ASM_ARCH_REG_LCD_H)
#define __ASM_ARCH_REG_LCD_H

/* ---- Include Files ---------------------------------------------------- */

#include <mach/hardware.h>

/* ---- Constants and Types ---------------------------------------------- */

#define REG_LCD_CMDR    __REG32(HW_LCD_INTF_BASE + 0x0000)
#define REG_LCD_DATR    __REG32(HW_LCD_INTF_BASE + 0x0004)
#define REG_LCD_RREQ    __REG32(HW_LCD_INTF_BASE + 0x0008)
#define REG_LCD_WTR     __REG32(HW_LCD_INTF_BASE + 0x0010)
#define REG_LCD_RTR     __REG32(HW_LCD_INTF_BASE + 0x0014)
#define REG_LCD_CR      __REG32(HW_LCD_INTF_BASE + 0x0018)
#define REG_LCD_SR      __REG32(HW_LCD_INTF_BASE + 0x001C)
#define REG_LCD_BLCR    __REG8(HW_PTIMER_BASE    + 0x000C)

/*
 * LCD_BLCR - LCD backlight control register
 */
#define REG_LCD_BLCR_DUTYMASK   0x00F8	/* Duty cycle */
#define REG_LCD_BLCR_DUTYSHFT   0x0003	/* active period = duty * 4 / 128 */
#define REG_LCD_BLCR_FREQ_6KHZ  0x0000	/* 6.3475 khz */
#define REG_LCD_BLCR_FREQ_12KHZ 0x0002	/* 12.695 khz */
#define REG_LCD_BLCR_FREQ_25KHZ 0x0004	/* 25.390 khz */
#define REG_LCD_BLCR_FREQ_50KHZ 0x0006	/* 50.780 khz */
#define REG_LCD_BLCR_MOD_ON     0x0001	/* Modulate on */

/* physical address of the LCD data register */
#define REG_LCD_DATR_PADDR  HW_IO_VIRT_TO_PHYS(HW_LCD_INTF_BASE + 0x0004)

/*
 * LCD_WTR - LCD Write Timing Register
 */

#define REG_LCD_WTR_SETUP_SHIFT  0
#define REG_LCD_WTR_SETUP_MASK   (0xFF << REG_LCD_WTR_SETUP_SHIFT)

#define REG_LCD_WTR_PULSE_SHIFT  8
#define REG_LCD_WTR_PULSE_MASK   (0xFF << REG_LCD_WTR_PULSE_SHIFT)

#define REG_LCD_WTR_HOLD_SHIFT  16
#define REG_LCD_WTR_HOLD_MASK   (0xFF << REG_LCD_WTR_HOLD_SHIFT)

#define REG_LCD_WTR_VAL(setup, pulse, hold)   \
   (((setup) << REG_LCD_WTR_SETUP_SHIFT)      \
   | ((pulse) << REG_LCD_WTR_PULSE_SHIFT)      \
   | ((hold)  << REG_LCD_WTR_HOLD_SHIFT))

/*
 * LCD_RTR - LCD Read Timing Register
 */

#define REG_LCD_RTR_SETUP_SHIFT  0
#define REG_LCD_RTR_SETUP_MASK   (0xFF << REG_LCD_RTR_SETUP_SHIFT)

#define REG_LCD_RTR_PULSE_SHIFT  8
#define REG_LCD_RTR_PULSE_MASK   (0xFF << REG_LCD_RTR_PULSE_SHIFT)

#define REG_LCD_RTR_HOLD_SHIFT  16
#define REG_LCD_RTR_HOLD_MASK   (0xFF << REG_LCD_RTR_HOLD_SHIFT)

#define REG_LCD_RTR_VAL(setup, pulse, hold)   \
   (((setup) << REG_LCD_RTR_SETUP_SHIFT)      \
   | ((pulse) << REG_LCD_RTR_PULSE_SHIFT)      \
   | ((hold)  << REG_LCD_RTR_HOLD_SHIFT))

/*
 * LCD_CR - LCD Control Register
 */
#define REG_LCD_CR_ENABLE_DMA                   (1u << 31)
#define REG_LCD_CR_ENABLE_8_BIT_INTF            (1u << 30)
#define REG_LCD_CR_ENABLE_16_TO_18_EXPANSION    (1u << 29)
#define REG_LCD_CR_ENABLE_DMA_BYTESWAP          (1u << 28)
#define REG_LCD_CR_ENABLE_DMA_WORDSWAP          (1u << 27)
#define REG_LCD_CR_SELCD                        (1u << 26)
#define REG_LCD_CR_SEL_HANTRO_MODE              (1u << 25)
/* bit 24 reserved */
#define REG_LCD_CR_DBI_B                        (1u << 23)
#define REG_LCD_CR_COLOR_ENDIAN                 (1u << 22)	/* 1==big */
#define REG_LCD_CR_INPUT_COLOR_MODE             (3u << 20)
#define REG_LCD_CR_COLOR_MODE                   (7u << 17)
#define REG_LCD_CR_THCLK_CNT                    (3u << 15)
#define REG_LCD_CR_DBI_C                        (1u << 14)
#define REG_LCD_CR_DBI_C_TYPE                   (1u << 13)	/* 1==4wire */

/*
 * LCDC_SR - LCD Status Register
 */
#define REG_LCD_SR_FIFO_EMPTY		(1u << 31)
#define REG_LCD_SR_FIFO_FULL		(1u << 30)
#define REG_LCD_SR_FIFO_READ_PTR	(0x1fu << 25)
#define REG_LCD_SR_FIFO_WRITE_PTR	(0x7u << 22)
#define REG_LCD_SR_READ_READY		(1u << 21)
#define REG_LCD_SR_LCD_BUSY		(1u << 20)
#define REG_LCD_SR_RREQ			(1u << 19)

#endif /* __ASM_ARCH_REG_LCD_H */
