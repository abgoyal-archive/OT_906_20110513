/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*******************************************************************************************
Copyright 2010 Broadcom Corporation.  All rights reserved.

Unless you and Broadcom execute a separate written software license agreement governing use
of this software, this software is licensed to you under the terms of the GNU General Public
License version 2, available at http://www.gnu.org/copyleft/gpl.html (the "GPL").

Notwithstanding the above, under no circumstances may you combine this software in any way
with any other Broadcom software provided under a license other than the GPL, without
Broadcom's express prior written consent.
*******************************************************************************************/

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>

#include <linux/usb/android.h>
#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/kthread.h>

#include "f_mass_storage.h"
#include "f_adb.h"
#include "f_ether.h"
#include "f_rndis.h"
#include "dwc_otg_cil.h"

#include "gadget_chips.h"
#include "u_ether.h"

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

/* #define TEST_USB_BC10 */

/* RNDIS Ethernet Settings... */
/*-------------------------------------------------------------------------*/
#define RNDIS_VENDOR_NUM       0x0A5C  /* Broadcom */
#define RNDIS_PRODUCT_NUM      0xABCD  /* Ethernet/RNDIS Gadget */

static unsigned char hostaddr[ETH_ALEN];
/*-------------------------------------------------------------------------*/

/* Android Settings.... */
/*-------------------------------------------------------------------------*/
static const char longname[] = "Gadget Android";

/* Default vendor and product IDs, overridden by platform data */
#define VENDOR_ID			0x18D1
#define MSC_PRODUCT_ID			0x0005
#define ADB_PRODUCT_ID			0x0002
#define ENABLE				1
#define DISABLE				0
#define SET_RNDIS_ETH_USB_CONF		0
#define SET_ANDROID_USB_CONF		1
#define RETRY_ENUM			10
#define RENUM_TIME			3000
#define INT_USB_TIME			1000
#define DISCONNECT_TIME_FOR_PC		10
/* #define SUPPORT_REMOTE_WAKEUP */

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif


/* USB modes */
enum {
	MSC_ONLY_MOD = 0,
	ADB_MSC_MOD,
	ADB_RNDIS_MOD,
	RESTORE_ADB_MODE,
	CHK_CUR_USB_MOD,
	ADB_RNDIS_MOD_OFF
};

struct android_dev {
	struct usb_gadget *gadget;
	struct usb_composite_dev *cdev;

	int product_id;
	int adb_product_id;
	int version;

	int adb_enabled;
	int nluns;
};

static atomic_t adb_enable_excl;
static struct android_dev *_android_dev;
static unsigned char cur_adb_mode = 0;

/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

/* String Table */
static struct usb_string strings_dev[] = {
	/* These dummy values should be overridden by platform data */
	[STRING_MANUFACTURER_IDX].s = "BRCM",
	[STRING_PRODUCT_IDX].s = "Android Phone",
	[STRING_SERIAL_IDX].s = "BRCM2153",
	{}
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(MSC_PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};
/*-------------------------------------------------------------------------*/
/* 							External Functions 								     */
/*-------------------------------------------------------------------------*/
extern int dwc_otg_cil_GetPostConfigCurrentIn2maUnit(void);
extern void dwc_otg_cil_USBInitialized(void);
extern void dwc_otg_pcd_stop_usb(void);
extern USB_Charger_Type_t dwc_otg_cil_Usb_Charger_Detection_BC10(void);
extern USB_Charger_Type_t dwc_otg_cil_Get_Usb_Charger_Type(void);
extern void dwc_otg_cil_Reset_Usb_Charger_Type(void);
extern void Disable_MSC_function(void);
/*-------------------------------------------------------------------------*/
/* 							Local Functions 								     */
/*-------------------------------------------------------------------------*/
static void android_reenum_thread(void *data);
static void android_init_usb_thread (void *data);

/* Global variables */
static unsigned char g_retry;
static bool g_need_reenum, g_usb_cable_connected;

enum {
	INIT_USB = 0,
	USB_REENUM
};

/**
* android_kernel_thread(unsigned char thread_mode) - Create and run the kernel thread.
* @thread_mode: specify the thread mode to be processed
*
*/
static void android_kernel_thread (unsigned char thread_mode)
{
	struct task_struct *usb_enum_task;
	
	switch (thread_mode) {
	case INIT_USB:
		usb_enum_task = kthread_run(android_init_usb_thread, 0, "android_init_usb");
		break;
	case USB_REENUM:
		usb_enum_task = kthread_run(android_reenum_thread, 0, "android_usb_enum");
		break;
	default:
		pr_info("Not support...\n");
		break;
	}
	
	if (IS_ERR(usb_enum_task)) {
		pr_err("Failed to create usb_enum_task -- ERROR=%x\n", PTR_ERR(usb_enum_task));
		usb_enum_task = NULL;
	}
}

/**
 * android_start_usb() - Start USB initialization at the kenel driver initialization
 * Note: the default usb config is MSC only mode and it will initiailize the usb gadget driver.
 *
 */
static void android_start_usb (void)
{
	struct android_dev *dev = _android_dev;

	if (dev->cdev && dev->cdev->gadget)
		usb_gadget_connect(dev->cdev->gadget);
	android_kernel_thread(INIT_USB);
}

/**
 * android_force_reenum() - Force to enumeration the USB device
 *
 */
static void android_force_reenum(void)
{
	struct android_dev *dev = _android_dev;

	pr_info("android_force_reenum\n");
	
	/* This is for USB cable unplugged during the driver initialization...
	     So the USB gadget speed is UNKNOWN....*/
	if (dev->cdev && dev->cdev->gadget &&
		dev->cdev->gadget->speed == USB_SPEED_UNKNOWN) {		
		if ( dwc_otg_cil_Usb_Charger_Detection_BC10() == DEDICATED_CHARGER) {
			dwc_otg_pcd_stop_usb();
			dwc_otg_cil_USBInitialized();
			return;	
		}		
		msleep(INT_USB_TIME);
	}

	if (dev->cdev && dev->cdev->gadget ) {
		pr_info("Performing USB enum...\n");
		usb_gadget_disconnect(dev->cdev->gadget);
		/* should have the enough delay for some PCs to release the drivers*/
		msleep(DISCONNECT_TIME_FOR_PC); 
		g_need_reenum = TRUE;
		android_kernel_thread(USB_REENUM);
		usb_gadget_connect(dev->cdev->gadget);
	} 

}

/**
 * android_init_usb_thread() - Initialize the USB setting for the Android ADB/MSC device.
 * Note: the default usb config is MSC only mode and it will initiailize the usb gadget driver.
 *
 */
static void android_init_usb_thread (void *data)
{

	struct android_dev *dev = _android_dev;
	pr_info("android_init_usb_thread ...\n");
	/* Init the USB.....*/
	if (dev->cdev && dev->cdev->gadget) {
		msleep(INT_USB_TIME);
		if (dev->cdev->gadget->speed == USB_SPEED_UNKNOWN) {
			pr_info("speed == USB_SPEED_UNKNOWN...\n");
			usb_gadget_disconnect(dev->cdev->gadget);
			dwc_otg_cil_USBInitialized();
		} else {
			pr_info("speed == %d\n", dev->cdev->gadget->speed);
			g_need_reenum = TRUE;
			android_kernel_thread(USB_REENUM);
			dwc_otg_cil_USBInitialized();
		}

	} else {
		pr_info("Android_init_usb -- failed ...\n");
		pr_info("dev->cdev = %x, dev->cdev->gadget = %x\n",
				dev->cdev, dev->cdev->gadget);
		return;
	}
}

/**
 * android_reenum_thread() - the thread will perform the re-enum if the USB REST is comming after 3 seconds.
 *
 * Note:  Doing so, it is because we want to prevent the current dwc otg driver to handle the IN EP disable interrupt case.
 *		In the IN EP diable interrupt case, it will re-xmit the data but it does not handle well and sometimes, it will go to the suspend state.
 */
static void android_reenum_thread(void *data)
{
	struct android_dev *dev = _android_dev;
	
	pr_info("android_reenum_thread\n");

	 /* for the WALL charger or USB cable unplugged suddendly... */
	if (g_retry > RETRY_ENUM) {
		pr_info("g_retry > RETRY_ENUM\n");
		g_retry = 0;
		g_need_reenum = FALSE;
		usb_gadget_disconnect(dev->cdev->gadget);
		return;
	}		

	if (!g_usb_cable_connected) {
		pr_info("g_usb_cable_connected=%d\n", g_usb_cable_connected);	
		g_retry = 0;
		g_need_reenum = FALSE;
		usb_gadget_disconnect(dev->cdev->gadget);
		return;
	}

	msleep(RENUM_TIME);
	if (g_need_reenum) {
		g_retry++;
		android_force_reenum();
		return;
	} 
	
}

void Android_usb_cable_connection(bool is_connected)
{
	pr_info("Android_USB_Cable_Connection = %d\n", is_connected);
	g_usb_cable_connected = is_connected;
}

/**
 * Android_cancel_reenum() - USB RESET interrupt routine called this function to stop the re-enum if USB RESET
 *							interrupt is triggerred less than 3 seconds. After cable unplugged, call it to stop
 *                                              the re-enum.
 *
 * Note: If USB cable is unplugged, call this function first to stop any potential issue.
 */
void Android_cancel_reenum(void)
{
	pr_info("Android_cancel_reenum\n");
	g_need_reenum = FALSE;
	g_retry = 0;
}

/**
 * Android_PMU_USB_Start() - PMU call this function to start the USB opertion
 *
 */
void Android_PMU_USB_Start(void)
{		
#ifdef TEST_USB_BC10	
	if ( dwc_otg_cil_Usb_Charger_Detection_BC10() == DEDICATED_CHARGER)	
		return;	
#endif	
	android_force_reenum();		
}

/*
  *Android_Perform_BC10_Dectection() - PMU call this function to detect the USB BC type.
  * 
  *@return the current BC type
  *
  *Note: It must be called before calling Android_PMU_USB_Start()
  *
 */
USB_Charger_Type_t Android_Perform_BC10_Dectection(void)
{
	return dwc_otg_cil_Usb_Charger_Detection_BC10();	
}

/*
  *Android_Get_USB_Charger_type() - PMU call this function to get the current USB BC type 
  * 									without performing hardware detection.
  * 
  *@return the current USB BC type
  *
  *
 */
USB_Charger_Type_t Android_Get_USB_BC_Type(void)
{
	return dwc_otg_cil_Get_Usb_Charger_Type();
}

/*
  *Android_Get_USB_Charger_type() - PMU call this function to set the current USB BC type to 
  *									"NO_CHARGER".
  * 									
 */
void Android_Reset_USB_BC_Type(void)
{
	dwc_otg_cil_Reset_Usb_Charger_Type();
}

/**
 * Android_switch_usb_conf() - switch the USB configuration
 * @new_usb_conf: The value of the USB configuration
 *
 */
int Android_switch_usb_conf (unsigned char new_usb_conf)
{
	static unsigned char cur_usb_conf = 0xff;
	struct android_dev *dev = _android_dev;

	if (new_usb_conf == CHK_CUR_USB_MOD) {
                pr_info("The current usb configuration is %d", cur_usb_conf);
                return cur_usb_conf;
        }

	if (new_usb_conf == RESTORE_ADB_MODE) {
                pr_info("restore adb mode enable = %d", cur_adb_mode );
                new_usb_conf = cur_adb_mode;
        }

	if (cur_usb_conf == new_usb_conf) {
		pr_info("The usb configuration has not been changed!\n");
		return cur_usb_conf;
	}

	switch (new_usb_conf) {
	case ADB_RNDIS_MOD:
		device_desc.idVendor = __constant_cpu_to_le16(RNDIS_VENDOR_NUM);
		device_desc.idProduct = __constant_cpu_to_le16(RNDIS_PRODUCT_NUM);
		device_desc.bDeviceClass         = USB_CLASS_COMM;
		Disable_MSC_function();
		set_current_usb_config(SET_RNDIS_ETH_USB_CONF);
		rndis_interface_enable(ENABLE);
		dev->adb_enabled = DISABLE;
		dev->cdev->desc.bDeviceClass         = USB_CLASS_COMM;
		break;	
	case MSC_ONLY_MOD:
		device_desc.idVendor = __constant_cpu_to_le16(VENDOR_ID);
		device_desc.idProduct = __constant_cpu_to_le16(MSC_PRODUCT_ID);
		device_desc.bDeviceClass         = USB_CLASS_MASS_STORAGE;
		set_current_usb_config(SET_ANDROID_USB_CONF);
		adb_interface_enable(DISABLE);
		dev->cdev->desc.bDeviceClass         = USB_CLASS_MASS_STORAGE;
		dev->adb_enabled = DISABLE;
		adb_function_enable(DISABLE);
		cur_adb_mode = 0;
		break;
	case ADB_MSC_MOD:
		device_desc.idVendor = __constant_cpu_to_le16(VENDOR_ID);
		device_desc.idProduct = __constant_cpu_to_le16(ADB_PRODUCT_ID);
		device_desc.bDeviceClass         = USB_CLASS_PER_INTERFACE;
		set_current_usb_config(SET_ANDROID_USB_CONF);
		adb_interface_enable(ENABLE);
		dev->adb_enabled = ENABLE;
		adb_function_enable(ENABLE);
		dev->cdev->desc.bDeviceClass         = USB_CLASS_PER_INTERFACE;
		cur_adb_mode = 1;
		break;		
	default:
		pr_info("The USB configuration is supported.....\n");
		break;
	}

	dev->cdev->desc.idProduct = device_desc.idProduct;
	dev->cdev->desc.idVendor = device_desc.idVendor;
	cur_usb_conf = new_usb_conf;
	g_retry = 0;		
	android_force_reenum();
	return new_usb_conf;
}
EXPORT_SYMBOL(Android_switch_usb_conf);

/*-------------------------------------------------------------------------*/

/*
 * We may not have an RNDIS configuration, but if we do it needs to be
 * the first one present.  That's to make Microsoft's drivers happy,
 * and to follow DOCSIS 1.0 (cable modem standard).
 */
static int __init rndis_do_config(struct usb_configuration *c)
{
	/* FIXME alloc iConfiguration string, set it in c->strings */
	int ret;
	pr_info("rndis_do_config: \n");

	ret = rndis_bind_config(c, hostaddr);
	 if (ret)
		return ret;
	return ret;
}

static struct usb_configuration rndis_config_driver = {
	 .label			= "RNDIS",
	.bind			= rndis_do_config,
	.bConfigurationValue	= 2,
	/* .iConfiguration = DYNAMIC */
	.bmAttributes		= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower		= 0xFA, /* 500ma */
};

static int __init android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret;
	pr_debug("android_bind_config\n");
	ret = mass_storage_function_add(dev->cdev, c, dev->nluns);
	if (ret)
		return ret;
	return adb_function_add(dev->cdev, c);
}

static struct usb_configuration android_config = {
	.label		= "android",
	.bind		= android_bind_config,
	.bConfigurationValue = 1,
	.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower	= 0xFA, /* 500ma */
};

static int __init android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum;
	int			id;
	int			ret;

	pr_info("android_bind %d\n", dev->adb_enabled);

	/* set up network link layer */
	ret = gether_setup(cdev->gadget, hostaddr); /* debug.... */

	if (ret < 0)
		goto fail;

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;
	
#ifdef SUPPORT_REMOTE_WAKEUP
	if (gadget->ops->wakeup)
		android_config.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
#endif

	/* Get the Max power */
	android_config.bMaxPower = dwc_otg_cil_GetPostConfigCurrentIn2maUnit();

	/* register our configuration */
	ret = usb_add_config(cdev, &android_config);
	if (ret) {
		pr_err("usb_add_config failed\n");
		return ret;
	}

	ret = usb_add_config(cdev, &rndis_config_driver);
	if (ret) {
		pr_err("usb_add_config failed\n");
		return ret;
	}

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	usb_gadget_set_selfpowered(gadget);
	dev->cdev = cdev;
	g_need_reenum = FALSE;
	g_retry = 0;
	g_usb_cable_connected = TRUE;
	android_start_usb();
	return 0;

fail:
	gether_cleanup();
	return ret;
}

static void enable_adb(struct android_dev *dev, int enable)
{	
	pr_info("enable_adb: enable = %d\n", enable);
	
	if (enable == ADB_RNDIS_MOD) 
		return;
	
	/* set product ID to the appropriate value */
	if (enable)
	 	Android_switch_usb_conf(ADB_MSC_MOD);
	else
		Android_switch_usb_conf(MSC_ONLY_MOD);

	cur_adb_mode = enable;
}


void android_enable_function(struct usb_function *f, int enable)
{
	struct android_dev *dev = _android_dev;
	
	pr_info("android_enable_function enable = %d\n", enable);
	
	if (!strcmp(f->name, "rndis")) {		
		if (enable)
			Android_switch_usb_conf(ADB_RNDIS_MOD);
		else
			enable_adb(_android_dev, 2);		
	}

}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.bind		= android_bind,
	.enable_function = android_enable_function,
};

static int adb_enable_open(struct inode *ip, struct file *fp)
{
	if (atomic_inc_return(&adb_enable_excl) != 1) {
		atomic_dec(&adb_enable_excl);
		return -EBUSY;
	}
	return 0;
}

static int adb_enable_release(struct inode *ip, struct file *fp)
{
	pr_info("disabling adb\n");
	enable_adb(_android_dev, 0);
	atomic_dec(&adb_enable_excl);
	return 0;
}

static int adb_enable_ioctl(struct inode *inode, struct file *fp,
							unsigned int cmd, unsigned long arg)
{
	void __user	*argp = (void __user *)arg;
	int		val;

	
	switch (cmd) {
	case MSC_ONLY_MOD:
		pr_info("enabling adb MSC only mode\n");
		enable_adb(_android_dev, 0);
		break;
	case ADB_MSC_MOD:
		pr_info("enabling adb ADB mode\n");
		enable_adb(_android_dev, 1);
		break;
	case ADB_RNDIS_MOD:
		pr_info("enabling adb RNDIS mode\n");
		Android_switch_usb_conf(ADB_RNDIS_MOD);
		break;
	case CHK_CUR_USB_MOD:
		pr_info("checking the current USB mod\n");
		val = Android_switch_usb_conf(CHK_CUR_USB_MOD);
		if(put_user(val, (int __user *) argp)) {
			pr_err("put_user for check cur USB mod --- failed\n");
		}			
		break;
	case ADB_RNDIS_MOD_OFF:
		enable_adb(_android_dev, 2);
		break;
	default:
		pr_info("adb ioctl unknown cmd %d\n", cmd);
		break;
	}
	return 0;
}

static struct file_operations adb_enable_fops = {
	.owner =   THIS_MODULE,
	.open =    adb_enable_open,
	.release = adb_enable_release,
	.ioctl = adb_enable_ioctl,
};

static struct miscdevice adb_enable_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_adb_enable",
	.fops = &adb_enable_fops,
};

static int __init android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;

	pr_info("android_probe pdata: %p\n", pdata);

	if (pdata) {
		if (pdata->vendor_id)
			device_desc.idVendor =
				__constant_cpu_to_le16(pdata->vendor_id);
		if (pdata->product_id) {
			dev->product_id = pdata->product_id;
			device_desc.idProduct =
				__constant_cpu_to_le16(pdata->product_id);
		}
		if (pdata->adb_product_id)
			dev->adb_product_id = pdata->adb_product_id;
		if (pdata->version)
			dev->version = pdata->version;

		if (pdata->product_name)
			strings_dev[STRING_PRODUCT_IDX].s = pdata->product_name;
		if (pdata->manufacturer_name)
			strings_dev[STRING_MANUFACTURER_IDX].s =
					pdata->manufacturer_name;
		if (pdata->serial_number)
			strings_dev[STRING_SERIAL_IDX].s = pdata->serial_number;
		dev->nluns = pdata->nluns;
	}

	return 0;
}

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb", },
	.probe = android_probe,
};

/*-------------------------------------------------------------------------*/
static int __init android_init(void)
{
	struct android_dev *dev;
	int ret;

	pr_info("android init\n");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* set default values, which should be overridden by platform data */
	dev->product_id = MSC_PRODUCT_ID;
	dev->adb_product_id = ADB_PRODUCT_ID;
	_android_dev = dev;

	pr_info("adb android enabled  %d\n", dev->adb_enabled);

	ret = platform_driver_register(&android_platform_driver);
	if (ret)
		return ret;
	ret = misc_register(&adb_enable_device);
	if (ret) {
		platform_driver_unregister(&android_platform_driver);
		return ret;
	}

	ret = usb_composite_register(&android_usb_driver);
	if (ret) {
		misc_deregister(&adb_enable_device);
		platform_driver_unregister(&android_platform_driver);
	}
	return ret;
}

static void  __exit android_cleanup(void)
{
	pr_info("cleanup\n");
	usb_composite_unregister(&android_usb_driver);
	misc_deregister(&adb_enable_device);
	platform_driver_unregister(&android_platform_driver);
	kfree(_android_dev);
	_android_dev = NULL;

}

module_init(android_init);
module_exit(android_cleanup);
