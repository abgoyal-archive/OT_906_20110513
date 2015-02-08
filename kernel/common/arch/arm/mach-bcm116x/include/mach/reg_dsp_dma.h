/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/include/mach/reg_dsp_dma.h
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
*  REG_DSP_DMA.h
*
*  PURPOSE:
*
*     This file contains definitions for the shared memory interface.
*
*  NOTES:
*
*****************************************************************************/

#if !defined(__ASM_ARCH_REG_DSP_DMA_H)
#define __ASM_ARCH_REG_DSP_DMA_H

/* ---- Include Files ---------------------------------------------------- */

#include <mach/hardware.h>

/* ---- Constants and Types ---------------------------------------------- */

#define REG_DSP_DMA_PROG     __REG32(HW_DSP_DMA_BASE + 0x00)	/* DSP program DMA control register */
#define REG_DSP_DMA_DATA     __REG32(HW_DSP_DMA_BASE + 0x04)	/* DSP data DMA control register */

#endif /* __ASM_ARCH_REG_DSP_DMA_H */
