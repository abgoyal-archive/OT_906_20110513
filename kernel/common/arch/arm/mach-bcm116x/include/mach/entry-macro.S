/*
 *
 * Low-level IRQ helper macros for BCM116X-based platforms
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/include/mach/entry-macro.S
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

#include <mach/irqs.h>
#include <mach/hardware.h>

	.macro	disable_fiq
	.endm

	.macro	get_irqnr_preamble, base, tmp
	ldr	\base, =HW_IRQ_BASE
	.endm

	.macro	arch_ret_to_user, tmp1, tmp2
	.endm

	.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp

	/* This stanza gets the irq mask from one of two status registers */
	mov	\irqnr, #0
	ldr	\irqstat, [\base, #0x00c]	@ get masked status
	cmp	\irqstat, #0
	bne	1001f
	add	\irqnr, \irqnr, #32
	ldr	\irqstat, [\base, #0x10c]	@ get masked status
	teq	\irqstat, #0
	beq	1003f

1001:	tst	\irqstat, #15
	bne	1002f
	add	\irqnr, \irqnr, #4
	movs	\irqstat, \irqstat, lsr #4
	bne	1001b
1002:	tst	\irqstat, #1
	bne	1003f
	add	\irqnr, \irqnr, #1
	movs	\irqstat, \irqstat, lsr #1
	bne	1002b
1003:	/* EQ will be set if no irqs pending */
	.endm

	.macro	irq_prio_table
	.endm

