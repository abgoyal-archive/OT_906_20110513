/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/include/mach/auxadc.h
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
*  auxadc.h
*
*  PURPOSE:
*
*  This file defines the interface to the AUX ADC driver.
*
*  NOTES:
*
*****************************************************************************/

#if !defined(_BCM116X_AUXADC_H)
#define _BCM116X_AUXADC_H

/* ---- Include Files ---------------------------------------------------- */
/* ---- Constants and Types ---------------------------------------------- */
/* ---- Variable Externs ------------------------------------------------- */
/* ---- Function Prototypes ---------------------------------------------- */

#if defined(__KERNEL__)
int auxadc_access(int regID);
#endif /* __KERNEL__ */

#endif /* _BCM116X_AUXADC_H */
