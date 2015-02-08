/*
 *
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

/*
 * Map IO 1 to 1 with physical addresses.
 * Do not move this below the include of hardware.h
 */
/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/include/mach/uncompress.h
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

#define HW_IO_PHYS_TO_VIRT(x)  (x)

#include <mach/hardware.h>
#include <mach/reg_uart.h>

#include <linux/serial_reg.h>	/* For UART_LSR_TEMT constant */

/*
 * This does not append a newline
 */
static inline void putc(int c)
{
	while ((REG_UART_A_LSR & UART_LSR_TEMT) == 0) {
		barrier();
	}

	REG_UART_A_FIFO = c;
}

static inline void flush(void)
{
	while ((REG_UART_A_LSR & UART_LSR_TEMT) == 0) {
		barrier();
	}
}

#define arch_decomp_setup()
#define arch_decomp_wdog()
