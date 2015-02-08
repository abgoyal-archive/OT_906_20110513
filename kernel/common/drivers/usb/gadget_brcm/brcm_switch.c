/*******************************************************************************************
Copyright 2010 Broadcom Corporation.  All rights reserved.
This program is the proprietary software of Broadcom Corporation and/or its licensors, and may 
only be used, duplicated, modified or distributed pursuant to the terms and conditions of a 
separate, written license agreement executed between you and Broadcom (an "Authorized License").

Except as set forth in an Authorized License, Broadcom grants no license(express or implied), 
right to use, or waiver of any kind with respect to the Software, and Broadcom expressly reserves 
all rights in and to the Software and all intellectual property rights therein.  
IF YOU HAVE NO AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO USE THIS SOFTWARE IN ANY WAY, AND 
SHOULD IMMEDIATELY NOTIFY BROADCOM AND DISCONTINUE ALL USE OF THE SOFTWARE.
  
 Except as expressly set forth in the Authorized License,
1. This program, including its structure, sequence and organization, constitutes the valuable 
trade secrets of Broadcom, and you shall use all reasonable efforts to protect the confidentiality 
thereof, and to use this information only in connection with your use of Broadcom integrated 
circuit products.

2. TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS 
AND BROADCOM MAKES NO PROMISES, REPRESENTATIONS OR WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, 
OR OTHERWISE, WITH RESPECT TO THE SOFTWARE.  BROADCOM SPECIFICALLY DISCLAIMS ANY AND ALL IMPLIED 
WARRANTIES OF TITLE, MERCHANTABILITY, NONINFRINGEMENT, FITNESS FOR A PARTICULAR PURPOSE, LACK OF 
VIRUSES, ACCURACY OR COMPLETENESS, QUIET ENJOYMENT, QUIET POSSESSION OR CORRESPONDENCE TO DESCRIPTION. 
YOU ASSUME THE ENTIRE RISK ARISING OUT OF USE OR PERFORMANCE OF THE SOFTWARE.

3. TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT SHALL BROADCOM OR ITS LICENSORS BE LIABLE 
FOR (i) CONSEQUENTIAL, INCIDENTAL, SPECIAL, INDIRECT, OR EXEMPLARY DAMAGES WHATSOEVER ARISING OUT 
OF OR IN ANY WAY RELATING TO YOUR USE OF OR INABILITY TO USE THE SOFTWARE EVEN IF BROADCOM HAS BEEN 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGES; OR (ii) ANY AMOUNT IN EXCESS OF THE AMOUNT ACTUALLY PAID 
FOR THE SOFTWARE ITSELF OR U.S. $1, WHICHEVER IS GREATER. THESE LIMITATIONS SHALL APPLY 
NOTWITHSTANDING ANY FAILURE OF ESSENTIAL PURPOSE OF ANY LIMITED REMEDY.
*******************************************************************************************/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/suspend.h>
#include <linux/kthread.h>
#include "../../../arch/arm/include/asm/memory.h"
#include "android.h"

enum {
	MSC_ONLY_USB_CONF	= 1,
	ADB_MSC_USB_CONF,	
	RNDIS_ETHER_USB_CONF,
	RESTORE_ADB_MODE,
	CHECK_USB_MODE
};

struct proc_dir_entry *pentry_brcm_switch;
unsigned char g_check_usb_mode = 0;

static int my_atoi(const char *name)
{
    int val = 0;

    for (;; name++) {
        switch (*name) {
            case '0' ... '9':
                val = 10*val+(*name-'0');
                break;
            default:
                return val;
        }
    }
}

//Test the APIs
static void brcm_switch_msc_only(void* data)
{
#ifdef CONFIG_USB_ANDROID
	Android_switch_usb_conf(MSC_ONLY_USB_CONF - 1);
#endif
}

static void brcm_switch_adb_msc(void* data)
{		
#ifdef CONFIG_USB_ANDROID
	Android_switch_usb_conf(ADB_MSC_USB_CONF - 1);				
#endif
}

static void brcm_switch_eth(void* data)
{
#ifdef CONFIG_USB_ANDROID
	Android_switch_usb_conf(RNDIS_ETHER_USB_CONF - 1);
#endif
}

static void brcm_switch_restore_adb(void* data)
{
#ifdef CONFIG_USB_ANDROID
	Android_switch_usb_conf(RESTORE_ADB_MODE - 1);
#endif
}

static ssize_t brcm_switch_read(struct file *file, char *buf, int count, loff_t *ppos)
{
	int	val, len, err;

	if (g_check_usb_mode) {
		g_check_usb_mode = 0;
		pr_info("checking the current USB mod\n");
#ifdef CONFIG_USB_ANDROID
		val = Android_switch_usb_conf(CHECK_USB_MODE - 1) + 1;
#endif
		len = sizeof(val);
		pr_info("val = %d \n",val);
		err = copy_to_user(buf, &val, len);
		if (err != 0)
			return -EFAULT;
		return len;
	} else {
		pr_info("\nTest Modes:\n");
        	pr_info("MSC_ONLY_MODE: echo 1 > /proc/brcm_switch\n");
        	pr_info("ADB_MSC_MODE: echo 2 > /proc/brcm_switch\n");
        	pr_info("RNDIS_ETH_MODE: echo 3 > /proc/brcm_switch\n");
       		return 0;
	}      
}

static ssize_t brcm_switch_write(struct file *file, const char *buffer, unsigned long count, void *data)
{	
	struct task_struct *brcm_switch_task = NULL;
	int thread_mode = my_atoi(buffer);    
		
	switch (thread_mode) {
		case MSC_ONLY_USB_CONF:
			pr_info("\nMSC_ONLY_MODE\n");
			brcm_switch_task = kthread_run(brcm_switch_msc_only, 0, "msc_only");
			break;
		case ADB_MSC_USB_CONF:
			pr_info("\nADB_MSC_MODE\n");
			brcm_switch_task = kthread_run(brcm_switch_adb_msc, 0, "adb_msc");
			break;
		case RNDIS_ETHER_USB_CONF:
			pr_info("\nRNDIS_ETH_MODE\n");			
			brcm_switch_task = kthread_run(brcm_switch_eth, 0, "rndis");
			break;
		case RESTORE_ADB_MODE:
			pr_info("\nRESTORE_ADB_MODE\n");			
			brcm_switch_task = kthread_run(brcm_switch_restore_adb, 0, "restore_adb");
			break;
		case CHECK_USB_MODE:
			g_check_usb_mode = 1;
                	return 1;	
		default:
			pr_info("\nThe input mode (%d) is NOT supported!!!!\n");		
			return 1;
	}
	
	if (IS_ERR(brcm_switch_task )) {
		pr_err("Failed to create brcm_switch_task -- ERROR=%x\n", PTR_ERR(brcm_switch_task ));
		brcm_switch_task =  NULL;
	}
       return 1;
}

static struct file_operations brcm_switch_ops = {
  .owner =  THIS_MODULE,
  .read = brcm_switch_read,
  .write = brcm_switch_write,
};

static void brcm_switch_exit(void);

static int brcm_switch_init(void)
{
    pr_alert("printk: brcm_switch_init\n");
    pr_alert("cat /proc/brcm_switch (To list the test modes and usage.....)");
    pentry_brcm_switch = proc_create("brcm_switch", 0666, NULL, &brcm_switch_ops);
      
   if (!pentry_brcm_switch)
              return -EPERM;
          return 0;
}

static void brcm_switch_exit(void)
{
    pr_alert("\nprintk: brcm_switch_exit\n");
    remove_proc_entry("brcm_switch", NULL);
}

module_init(brcm_switch_init)
module_exit(brcm_switch_exit)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin Hwang");
MODULE_DESCRIPTION("BRCM SWITCH");

