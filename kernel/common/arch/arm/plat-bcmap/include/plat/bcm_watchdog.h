/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/plat-bcmap/include/plat/bcm_watchdog.h
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

/**
 *
 *   @file   bcm_watchdog.h
 *
 *   @brief  Broadcom platform header for watchdog.
 *
 ****************************************************************************/

#ifndef __BCM_WATCHDOG__
#define __BCM_WATCHDOG__

struct bcm_watchdog {
	unsigned int *memaddr;
};

extern int wdt_kopen(int inten);
extern int wdt_krelease(void);
extern int wdt_kwrite(size_t len);
extern int wdt_ksettimeout(int secs);
extern int wdt_kstart(void);
#endif /*__BCM_WATCHDOG__*/
