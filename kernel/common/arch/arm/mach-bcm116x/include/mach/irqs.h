/*
 *  Copyright (C) 1999 ARM Limited
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
* 	@file	arch/arm/mach-bcm116x/include/mach/irqs.h
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

#ifndef ASM_ARCH_IRQS_H
#define ASM_ARCH_IRQS_H

#define NUM_INTERNAL_GPIO	64

#if defined(CONFIG_GPIO_BCM59035)
#define ARCH_NR_GPIOS		(NUM_INTERNAL_GPIO + 2)
#else
#define ARCH_NR_GPIOS		NUM_INTERNAL_GPIO
#endif

#define NUM_INTERNAL_IRQS	64
#define NR_IRQS 		(NUM_INTERNAL_IRQS + ARCH_NR_GPIOS)

#define IRQ_INTC_START          0	/* first INTC interrupt is 0 */
#define IRQ_WAKETMR                 0	/* Sleep Mode Timer Wake-Up interrupt */
#define IRQ_DSP                     1	/* RIP Communications interrupt */
#define IRQ_UARTA                   2	/* UART-A interrupt */
#define IRQ_UARTB                   3	/* UART-B interrupt */
#define IRQ_SIM                     4	/* SIM Interface interrupt */
#define IRQ_KEYPAD                  5	/* Keypad Key Depressed interrupt */
#define IRQ_GPIO                    6	/* GPIO interrupt */
#define IRQ_I2C                     7	/* I2C interrupt */
#define IRQ_NANDECC                 8	/* NAND ECC done/ReadyBusy interrupt */
#define IRQ_WDOG1                   9	/* Watchdog Timer interrupt */
#define IRQ_RTC                     10	/* Periodic Timer interrupt */
#define IRQ_GPTMR                   11	/* General Purpose Time (A/B) interrupt */
#define IRQ_MICON                   12	/* MICON Headset button interrupt (after debouncing) */
#define IRQ_UARTC                   13	/* I2C2 (2nd I2C) interrupt */
#define IRQ_MPFIQ                   14	/* Message Processor interrupt */
#define IRQ_15                      15	/* reserved (= Sleep Mode control) */
#define IRQ_GRAPH                   16	/* Graphics Engine interrupt */
#define IRQ_VSYNC                   17	/* Camera VSYNC interrupt */
#define IRQ_AUDIO                   18	/* External Audio DAC interrupt */
#define IRQ_DSPFIQ                  19	/* RIP FIQ interrupt, Triggers FIQ */
#define IRQ_DMA                     20	/* DMA interrupt */
#define IRQ_USB                     21	/* USB 2.0 OTG (HSOTG) interrupt */
#define IRQ_CAM                     22	/* Camera module interrupt */
#define IRQ_HSB                     23	/* MICON raw headset button status interrupt */
#define IRQ_SPI                     24	/* SPI interrupt */
#define IRQ_TVO                     25	/* TVO Vsync interrupt */
#define IRQ_SDIO2                   26	/* SDIO2 interrupt */
#define IRQ_SDIO1                   27	/* SDIO1 interrupt */
#define IRQ_GEA3                    28	/* GEA3 interrupt */
#define IRQ_29                      29	/* reserved interrupt */
#define IRQ_MICIN                   30	/* MICIN interrupt */
#define IRQ_MSPRO                   31	/* MSPRO interrupt */
#define IRQ_WDOG2                   63	/* Watchdog Timer 2 */

#define IRQ_UNKNOWN                 -1
/* Unmask HSB_IRQ for headset */
#define BCM215X_VALID_SRC0 	0xCFF73DED	/* bitmask of valid interrupts */
#if defined(CONFIG_ARCH_BCM2153)
#define BCM215X_VALID_SRC1 	0xB0064C00	/* bitmask of valid interrupts */
#elif defined(CONFIG_ARCH_BCM2157)
#define BCM215X_VALID_SRC1 	0xD8064C00	/* bitmask of valid interrupts */
#else
#error BCM215X_VALID_SRC1 not defined for this CPU!!!
#endif

#define IRQ_HANTRO_ENC              42	/* Hantro encoder interrupt */
#define IRQ_HANTRO_DEC              43	/* Hantro decoder interrupt */
#define IRQ_ARM_PMU                 46	/* ARM PMU (Performance Monitor Unit) interrupt */
#define IRQ_GPT2                    62  /* general timer 2 */

#define IRQ_IPC_A2C		(60)
#if defined(CONFIG_ARCH_BCM2153)
#define IRQ_IPC_C2A		(61)
#elif defined(CONFIG_ARCH_BCM2157)
#define IRQ_IPC_C2A		(59)
#else
#error IRQ_IPC_C2A not defined for this CPU!!!
#endif

/*
 * IRQs 32 thru 95 are mapped onto GPIO lines 0 thru 63
 *
 * So, to request
 */

#define  IRQ_GPIO_0                 NUM_INTERNAL_IRQS

#define  GPIO_TO_IRQ(gpio)          ((gpio) + IRQ_GPIO_0)
#define  IRQ_TO_GPIO(irq)           ((irq) - IRQ_GPIO_0)

#define  gpio_to_irq(gpio)          GPIO_TO_IRQ(gpio)

/*
 * In Kernel 2.6.32.9, On system wide suspend context the device
 * drivers are refrained from receiving the interrupts, it masks all the
 * interrupts except for the timer ones and sets the IRQ_DISABLED flag
 * for each of the driver. If the interrupt is a WAKEUP source then there
 * is no way to get out of the cuffs set by kernel as the handle_level_irq()
 * exits if the interrupt status is set to IRQF_DISABLED :-(,
 * In case of 2.6.35.7 this is taken care by defining a new IRQ status flag
 * IRQF_NO_SUSPEND which will ensure that respective IRQ is not disabled.
 * As a work around for this issue in 2.6.32.9 the keypad interrupt WAKEUP
 * source is described as IRQF_TIMER
 */
#define IRQF_NO_SUSPEND IRQF_TIMER

#endif /* ASM_ARCH_IRQS_H */
