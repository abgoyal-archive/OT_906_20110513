/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/include/mach/reg_camera.h
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
*  reg_camera.h
*
*  PURPOSE:
*
*     This file contains definitions for the camera controller registers.
*
*  NOTES:
*
*****************************************************************************/

#if !defined(__ASM_ARCH_REG_CAMERA_H)
#define __ASM_ARCH_REG_CAMERA_H

/* ---- Include Files ---------------------------------------------------- */

#include <mach/hardware.h>

/* ---- Constants and Types ---------------------------------------------- */

#define REG_CAM_DATR    __REG32(HW_CAMERA_BASE + 0x0000)
#define REG_CAM_CR      __REG32(HW_CAMERA_BASE + 0x4000)
#define REG_CAM_WVR     __REG32(HW_CAMERA_BASE + 0x4004)
#define REG_CAM_WHR     __REG32(HW_CAMERA_BASE + 0x4008)
#define REG_CAM_CC1R    __REG32(HW_CAMERA_BASE + 0x400C)
#define REG_CAM_CC2R    __REG32(HW_CAMERA_BASE + 0x4010)
#define REG_CAM_CC3R    __REG16(HW_CAMERA_BASE + 0x4014 + 2)
#define REG_CAM_CR2     __REG32(HW_CAMERA_BASE + 0x4018)
#define REG_CAM_WVRS    __REG32(HW_CAMERA_BASE + 0x401C)	/* CAM Window Vertical Register Start */
#define REG_CAM_WVRE    __REG32(HW_CAMERA_BASE + 0x4020)	/* CAM Window Vertical Register End */

/* physical address of the camera data register */
#define REG_CAM_DATR_PADDR  HW_IO_VIRT_TO_PHYS(HW_CAMERA_BASE + 0x0000)

/*
 * CAM_CR - camera control register
 */

#define REG_CAM_CR_DCKINV                       (1 << 31)
#define REG_CAM_CR_VSINV                        (1 << 30)
#define REG_CAM_CR_HSINV                        (1 << 29)
#define REG_CAM_CR_CAMDCK                       (1 << 28)
#define REG_CAM_CR_RST                          (1 << 27)
#define REG_CAM_CR_DITH                         (1 << 26)
#define REG_CAM_CR_SYNC                         (1 << 25)
#define REG_CAM_CR_IY                           (1 << 24)
#define REG_CAM_CR_SDFV(sdfv)                 ((sdfv & 0xff) << 16)
#define REG_CAM_CR_SDFH(sdfh)                 ((sdfh & 0xff) << 8)
#define REG_CAM_CR_SDFV_MASK                    (0xff << 16)
#define REG_CAM_CR_SDFH_MASK                    (0xff << 8)
#define REG_CAM_CR_IUV                          (1 << 7)
#define REG_CAM_CR_YUV                          (1 << 6)
#define REG_CAM_CR_FMT_MASK                     (3 << 4)
#define REG_CAM_CR_FMT_DIRECT                   (0 << 4)
#define REG_CAM_CR_FMT_RGB565                   (1 << 4)
#define REG_CAM_CR_FMT_RGB555                   (2 << 4)
#define REG_CAM_CR_FMT_RGB444                   (3 << 4)
#define REG_CAM_CR_BYPASS_EN                    (1 << 3)
#define REG_CAM_CR_LBSEL                        (1 << 2)
#define REG_CAM_CR_LBTST                        (1 << 1)
#define REG_CAM_CR_EN                           (1 << 0)

/*
 * CAM_WVR - camera window vertical register
 */

#if defined(CONFIG_ARCH_BCM116X)
#define REG_CAM_WVR_SET(line) ((line / 2) & 0x1ffff)
#else
#define REG_CAM_WVR_SET(first, last) (((first / 2) & 0x3ff) | (((last / 2) & 0x3ff) << 16))
#endif

/*
 * CAM_WHR - camera window horizontal register
 */

#if defined (CONFIG_ARCH_BCM116X)
#define REG_CAM_WHR_SET(first, last) (((first / 2) & 0x7fff) | (((last / 2) & 0x7fff) << 16))
#else
#define REG_CAM_WHR_SET(first, last) (((first / 2) & 0x3ff) | (((last / 2) & 0x3ff) << 16))
#endif

/*
 * CAM_CCxR - camera colour conversion registers
 */

#define REG_CAM_CC1R_SET(r0, r1) ((r0 & 0x3ff) | ((r1 & 0x1ff) << 16))
#define REG_CAM_CC2R_SET(g0, g1) ((g0 & 0x3ff) | ((g1 & 0x1ff) << 16))
#define REG_CAM_CC3R_SET(b1)    (b1 & 0x3ff)

/*
 * CAM_CR2 - control register 2
 */

#define REG_CAM_CR2_BYTESWAP                    (1 << 0)
#define REG_CAM_CR2_WORDSWAP                    (1 << 1)
#define REG_CAM_CR2_YUVFULLRANGE                (1 << 2)
#define REG_CAM_CR2_HSYNCCONTROL                (1 << 3)

#endif /* __ASM_ARCH_REG_CAMERA_H */
