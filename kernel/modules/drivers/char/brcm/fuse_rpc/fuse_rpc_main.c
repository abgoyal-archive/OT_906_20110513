/****************************************************************************
*
*     Copyright (c) 2009 Broadcom Corporation
*
*   Unless you and Broadcom execute a separate written software license 
*   agreement governing use of this software, this software is licensed to you 
*   under the terms of the GNU General Public License version 2, available 
*    at http://www.gnu.org/licenses/old-licenses/gpl-2.0.html (the "GPL"). 
*
*   Notwithstanding the above, under no circumstances may you combine this 
*   software in any way with any other Broadcom software provided under a license 
*   other than the GPL, without Broadcom's express prior written consent.
*
****************************************************************************/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <asm/system.h>
#include <linux/broadcom/bcm_major.h>
#include "rpc_internal_api.h"
#include "rpc_debug.h"
#ifdef CONFIG_BRCM_FUSE_RPC_CIB
#include "rpc_sync_api.h"
#endif

static int __init bcm_fuse_rpc_init_module(void);
static void __exit bcm_fuse_rpc_exit_module(void);

static struct class *rpc_class;

extern RPC_Result_t RPC_SYS_EndPointRegister(RpcProcessorType_t processorType);

static int rpc_open(struct inode *inode, struct file *filp)
{

    return 0;
}

static int rpc_release(struct inode *inode, struct file *filp)
{

    return 0;
}

int rpc_read(struct file *filp, char __user *buf, size_t size, loff_t *offset)
{
    int rc = 0;

    return rc;
}

int rpc_write(struct file *filp, const char __user *buf, size_t size, loff_t *offset)
{
    int rc = 0;

    return rc;
}

static int rpc_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, UInt32 arg)
{
    int rc = 0;
    return(rc);
}


static struct file_operations rpc_ops = 
{
	.owner = THIS_MODULE,
	.open  = rpc_open,
	.read  = rpc_read,
	.write = rpc_write,
	.ioctl = rpc_ioctl,
	.mmap	 = NULL,
	.release = rpc_release,
};


/****************************************************************************
*
*  RPC_Init(void);
*
*  Register RPC module.
*
***************************************************************************/
UInt32 RPC_Init(void)
{

    if(RPC_SYS_Init(NULL) == RESULT_ERROR)
    {
        RPC_DEBUG(DBG_ERROR, "RPC device init fail...!\n");
        return 1;
    }
	if(RPC_SYS_EndPointRegister(RPC_APPS) == RPC_RESULT_ERROR)
    {
        RPC_DEBUG(DBG_ERROR, "registering the RPC device fail...!\n");
        return 1;
    }

#ifdef CONFIG_BRCM_FUSE_RPC_CIB
    // initialize the synchronous RPC interface
	if( RPC_SyncInitialize() != RESULT_OK )
	{
        RPC_DEBUG(DBG_ERROR, "RPC_SyncInitialize fail...!\n");
        return 1;
	}
#endif

    return(0);
}

/****************************************************************************
*
*  bcm_fuse_rpc_init_module(void);
*
*  Init module.
*
***************************************************************************/
static int __init bcm_fuse_rpc_init_module(void)
{
    int ret = 0;

    pr_info("RPC Support 1.00 (BUILD TIME "__DATE__" "__TIME__")\n" );

    if (( ret = register_chrdev( BCM_RPC_MAJOR, "bcm_rpc", &rpc_ops )) < 0 )
    {
        RPC_DEBUG(DBG_ERROR, "rpc: register_chrdev failed for major %d\n", BCM_RPC_MAJOR);
        goto out;
    }

    rpc_class = class_create(THIS_MODULE, "bcm_rpc");
    if (IS_ERR(rpc_class)) {
        return PTR_ERR(rpc_class);
    }

    device_create(rpc_class, NULL, MKDEV(BCM_RPC_MAJOR, 0),NULL, "bcm_rpc");

    RPC_DEBUG(DBG_INFO, "%s driver(major %d) installed.\n", "bcm_rpc", BCM_RPC_MAJOR);

    /** Init RPC Driver */
    ret = RPC_Init();

    if (ret) 
    {
        ret = -1;
        RPC_DEBUG(DBG_ERROR, "KRIL_Init fail...!\n");
    }

out:
    return ret;
}

/****************************************************************************
*
*  bcm_fuse_rpc_exit_module(void);
*
*  Exit module.
*
***************************************************************************/
static void __exit bcm_fuse_rpc_exit_module(void)
{

    return;
}


module_init(bcm_fuse_rpc_init_module);
module_exit(bcm_fuse_rpc_exit_module);
