/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	include/linux/broadcom/IPCInterface.h
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

/* needed for now so proper types file is included in IPCInterface.h */
#ifndef UNDER_LINUX
#define UNDER_LINUX 1
#endif

#if defined(CONFIG_ARCH_BCM2157)
#include "2157SDB/IPCInterface.h"
#elif defined(CONFIG_ARCH_BCM2153)
#include "2153SDB/IPCInterface.h"
#endif
