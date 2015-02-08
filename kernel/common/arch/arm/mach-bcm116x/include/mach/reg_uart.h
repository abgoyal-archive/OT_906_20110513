/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/include/mach/reg_uart.h
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

/* --------------------------------------------------------------------------- */
/*  BCM2153 UART Register definitions */
/* --------------------------------------------------------------------------- */

#ifndef __ASM_ARCH_REG_UART_H
#define __ASM_ARCH_REG_UART_H

/* ---- Include Files ---------------------------------------- */
#include <mach/hardware.h>

#ifndef __ARMEB__
#define REG_UART_OFFSET	0	/* little-endian */
#else
#define REG_UART_OFFSET	3	/* big-endian */
#endif

/* ---- Constants and Types ---------------------------------- */
/* UART registers */
#define REG_UART_A_FIFO     __REG8(HW_UART_A_BASE + 0x0000)
#define REG_UART_A_IER      __REG8(HW_UART_A_BASE + 0x0004 + REG_UART_OFFSET)
#define REG_UART_A_ISR      __REG8(HW_UART_A_BASE + 0x0008 + REG_UART_OFFSET)
#define REG_UART_A_FCR      __REG8(HW_UART_A_BASE + 0x0008 + REG_UART_OFFSET)
#define REG_UART_A_LCR      __REG8(HW_UART_A_BASE + 0x000c + REG_UART_OFFSET)
#define REG_UART_A_MCR      __REG8(HW_UART_A_BASE + 0x0010 + REG_UART_OFFSET)
#define REG_UART_A_LSR      __REG8(HW_UART_A_BASE + 0x0014 + REG_UART_OFFSET)
#define REG_UART_A_MSR      __REG8(HW_UART_A_BASE + 0x0018 + REG_UART_OFFSET)
#define REG_UART_A_DLL      __REG8(HW_UART_A_BASE + 0x0000 + REG_UART_OFFSET)
#define REG_UART_A_DLM      __REG8(HW_UART_A_BASE + 0x0004 + REG_UART_OFFSET)

#define REG_UART_B_FIFO     __REG8(HW_UART_B_BASE + 0x0000)
#define REG_UART_B_IER      __REG8(HW_UART_B_BASE + 0x0004 + REG_UART_OFFSET)
#define REG_UART_B_ISR      __REG8(HW_UART_B_BASE + 0x0008 + REG_UART_OFFSET)
#define REG_UART_B_FCR      __REG8(HW_UART_B_BASE + 0x0008 + REG_UART_OFFSET)
#define REG_UART_B_LCR      __REG8(HW_UART_B_BASE + 0x000c + REG_UART_OFFSET)
#define REG_UART_B_MCR      __REG8(HW_UART_B_BASE + 0x0010 + REG_UART_OFFSET)
#define REG_UART_B_LSR      __REG8(HW_UART_B_BASE + 0x0014 + REG_UART_OFFSET)
#define REG_UART_B_MSR      __REG8(HW_UART_B_BASE + 0x0018 + REG_UART_OFFSET)
#define REG_UART_B_DLL      __REG8(HW_UART_B_BASE + 0x0000 + REG_UART_OFFSET)
#define REG_UART_B_DLM      __REG8(HW_UART_B_BASE + 0x0004 + REG_UART_OFFSET)

#define REG_UART_C_FIFO     __REG8(HW_UART_C_BASE + 0x0000)
#define REG_UART_C_IER      __REG8(HW_UART_C_BASE + 0x0004 + REG_UART_OFFSET)
#define REG_UART_C_ISR      __REG8(HW_UART_C_BASE + 0x0008 + REG_UART_OFFSET)
#define REG_UART_C_FCR      __REG8(HW_UART_C_BASE + 0x0008 + REG_UART_OFFSET)
#define REG_UART_C_LCR      __REG8(HW_UART_C_BASE + 0x000c + REG_UART_OFFSET)
#define REG_UART_C_MCR      __REG8(HW_UART_C_BASE + 0x0010 + REG_UART_OFFSET)
#define REG_UART_C_LSR      __REG8(HW_UART_C_BASE + 0x0014 + REG_UART_OFFSET)
#define REG_UART_C_MSR      __REG8(HW_UART_C_BASE + 0x0018 + REG_UART_OFFSET)
#define REG_UART_C_DLL      __REG8(HW_UART_C_BASE + 0x0000 + REG_UART_OFFSET)
#define REG_UART_C_DLM      __REG8(HW_UART_C_BASE + 0x0004 + REG_UART_OFFSET)

#define REG_UART_UCR        __REG16(HW_UART_A_BASE + 0x100 + 2)	/* UART Configuration Register */

/*UART UCR Register Address */
#define ADDR_UART_UCR       (HW_UART_A_BASE + 0x100)
#define ADDR_UART_IRCR      (HW_UART_A_BASE + 0x108)

/* UCR bits */
#define REG_UARTA_PDEN     (1 << 14)	/* UARTA power down enable. */
#define REG_UARTB_PDEN     (1 << 7)	/*  UARTB power down enable. */

#define REG_UART_IER_RXINT              0x01
#define REG_UART_IER_TXINT              0x02
#define REG_UART_FCR_FIFO_ENABLE        0x01
#define REG_UART_FCR_RX_FIFO_RESET      0x02
#define REG_UART_FCR_TX_FIFO_RESET      0x04
#define REG_UART_LCR_DLAB               0x80
#define REG_UART_LSR_RX_RDY             0x01
#define REG_UART_LSR_OE                 0x02
#define REG_UART_LSR_PE                 0x04
#define REG_UART_LSR_FE                 0x08
#define REG_UART_LSR_BI                 0x10
#define REG_UART_LSR_TX_EMPTY           0x20
#define REG_UART_LSR_TX_IDLE            0x40
#define REG_UART_LSR_RXFIFOERR          0x80
#define REG_UART_MCR_LOOP_BACK          0x10
#define REG_UART_LCR_PARITY_ODD         0x08
#define REG_UART_LCR_PARITY_EVEN        0x18
#define REG_UART_LCR_PARITY_NONE        0x00
#define REG_UART_LCR_5BIT_1STOP         0x00
#define REG_UART_LCR_6BIT_1STOP         0x01
#define REG_UART_LCR_7BIT_1STOP         0x02
#define REG_UART_LCR_8BIT_1STOP         0x03
#define REG_UART_LCR_5BIT_1P5STOP       0x04
#define REG_UART_LCR_6BIT_2STOP         0x05
#define REG_UART_LCR_7BIT_2STOP         0x06
#define REG_UART_LCR_8BIT_2STOP         0x07

/* ---- Variables Externs ------------------------------------ */

/* ---- Function Prototypes ---------------------------------- */
#endif
