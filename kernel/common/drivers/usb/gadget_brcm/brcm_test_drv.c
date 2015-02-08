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
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/suspend.h>
#include <linux/brcm_console.h>
#include "../../../arch/arm/include/asm/memory.h"

struct proc_dir_entry *pentry_brcm_usb_test;

struct delayed_work g_delay_workq, g_delay_workq_usbcalled;

#define TST_SZ 1023
char s1[256] = "aaaaaaaaaabbbbbbbbbbbbbbbccccccccccccccddddddddddddddddddeeeeeeeeeeeeeeeeeeeee";
char s2[256] = "1111111111222222222222333333333333333333444444444444444444455555555555555555666666666666666666666777777777777778888888888888888888899999999";
char test1[TST_SZ];
unsigned char print_it;

extern int brcm_klogging(char *data, int length);
extern char brcm_netconsole_register_callbacks(struct brcm_netconsole_callbacks *_cb);

static int netconsole_start_cb (void)
{
	pr_info("%s\n",__func__);
	return 0;
}

static int netconsole_stop_cb(void)
{
	pr_info("%s\n",__func__);
	return 0;
}

static struct brcm_netconsole_callbacks brcm_netcon_callbacks = {
        .start = netconsole_start_cb,
        .stop = netconsole_stop_cb,        
};

static void brcm_usb_test_logging(void)
{
      static int i = 0;
      
       for (i=0;i<TST_SZ;i++)
                test1[i] = i;

      /*  brcm_klogging(test1, TST_SZ);*/
	brcm_klogging(s1, strlen(s1));
	brcm_klogging(s2, strlen(s2));
}

static void brcm_usb_test_klogging_task(void* data)
{
   while (print_it) {
	brcm_usb_test_logging();
	msleep(10);
   }
}

static int brcm_usb_test_read(struct inode *inode, struct file *file)
{

      pr_info("start brcm_printk test... \n");
      print_it = 1;
       /*create thread to test the klogging*/
	kernel_thread(brcm_usb_test_klogging_task, 0, 0);
      return 0;
}

static int brcm_usb_test_write(struct inode *inode, struct file *file)
{
	print_it = 0;

	pr_info("brcm_usb_test_write....\n");
	brcm_netconsole_register_callbacks(&brcm_netcon_callbacks);

	return 0;
}

static int brcm_usb_test_open(struct inode *inode, struct file *file)
{	
	return 0;
}

static struct file_operations brcm_usb_test_ops = {
  .owner =  THIS_MODULE,
  .read = brcm_usb_test_read,
  .write = brcm_usb_test_write,
  .open = brcm_usb_test_open,
};

static void brcm_usb_test_exit(void);

static int brcm_usb_test_init(void)
{
	pr_alert("printk: brcm usb test init\n");
	pentry_brcm_usb_test = proc_create("brcm_usb_test", 0666, NULL, &brcm_usb_test_ops);	
	
	if (!pentry_brcm_usb_test)
		return -EPERM;
	return 0;
}

static void brcm_usb_test_exit(void)
{
    pr_alert("printk: brcm usb test exit\n");
    remove_proc_entry("brcm_usb_test", NULL);
}

module_init(brcm_usb_test_init)
module_exit(brcm_usb_test_exit)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin Hwang");
MODULE_DESCRIPTION("BRCM USB TEST");
