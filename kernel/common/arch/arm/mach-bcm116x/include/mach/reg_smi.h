/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/include/mach/reg_smi.h
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
*  REG_SMI.h
*
*  PURPOSE:
*
*     This file contains definitions for the shared memory interface.
*
*  NOTES:
*
*****************************************************************************/

#if !defined(__ASM_ARCH_REG_SMI_H)
#define __ASM_ARCH_REG_SMI_H

/* ---- Include Files ---------------------------------------------------- */

#include <mach/hardware.h>

/* ---- Constants and Types ---------------------------------------------- */

#define REG_SMI_CR0     __REG32(HW_SMI_BASE + 0x00)	/* Control register 0 */
#define REG_SMI_CR1     __REG32(HW_SMI_BASE + 0x04)	/* Control register 1 */
#define REG_SMI_CR2     __REG32(HW_SMI_BASE + 0x08)	/* Control register 2 */
#define REG_SMI_CR3     __REG32(HW_SMI_BASE + 0x0C)	/* Control register 3 */

/* REG_SMI_CR bits */
#define REG_SMI_BASEADDR_MASK    (0x7ffff << 13)	/* Base address mask */
#define REG_SMI_PAGESIZE_MASK    (0xf << 1)	/* Page size mask */
#define REG_SMI_PREF_BUFEN       (0x1 << 0)	/* Pre-fetch buffer enable */

#endif /* __ASM_ARCH_REG_SMI_H */
