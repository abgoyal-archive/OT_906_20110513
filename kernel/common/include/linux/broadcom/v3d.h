/*******************************************************************************
Copyright 2010 Broadcom Corporation.  All rights reserved.

Unless you and Broadcom execute a separate written software license agreement
governing use of this software, this software is licensed to you under the
terms of the GNU General Public License version 2, available at
http://www.gnu.org/copyleft/gpl.html (the "GPL").

Notwithstanding the above, under no circumstances may you combine this software
in any way with any other Broadcom software provided under a license other than
the GPL, without Broadcom's express prior written consent.
*******************************************************************************/
#ifndef _V3D_H_
#define _V3D_H_
#include <linux/ioctl.h>

#define V3D_DEV_NAME	"v3d"
#define BCM_V3D_MAGIC	'V'
#ifdef __KERNEL__
//#define V3D_MEMPOOL_SIZE	SZ_32M
#define V3D_MEMPOOL_SIZE	SZ_8M
#endif

typedef struct {
	void *ptr;		// virtual address
	unsigned int addr;	// physical address
	unsigned int size;
} mem_t;

enum {
	V3D_CMD_GET_MEMPOOL = 0x80,
	V3D_CMD_WAIT_IRQ,
	V3D_CMD_LAST
};

#define V3D_IOCTL_GET_MEMPOOL	_IOR(BCM_V3D_MAGIC, V3D_CMD_GET_MEMPOOL, mem_t)
#define V3D_IOCTL_WAIT_IRQ	_IOR(BCM_V3D_MAGIC, V3D_CMD_WAIT_IRQ, unsigned int)

#endif
