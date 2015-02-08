/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/input/misc/ponkey-bcm59035.c
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/mfd/bcm59035/bcm59035.h>

#define PONKEY_DEBUG 1
#ifdef PONKEY_DEBUG
#define PMU_PONKEY_LOG(fmt, args...) \
    do { \
	pr_info("BCM59035-PONKEY: " fmt "\n", ##args); \
    } while (0)
#else
#define PMU_PONKEY_LOG(fmt, args...)
#endif

/*
*  Private data
*/
struct bcm59035_ponkey {
	struct bcm59035 *bcm59035;
	struct input_dev *dev;
};

static void bcm59035_ponkey_isr(int irq, void *data)
{
	/* Android counts the hold time for KEY_END so we can just repost key press and realease here */
	struct bcm59035_ponkey *bcm59035_ponkey = data;
	switch (irq) {
	case BCM59035_IRQID_INT1_PONKEYF:
		PMU_PONKEY_LOG("isr: PONKEYF irq");
		input_report_key(bcm59035_ponkey->dev, KEY_END, 1);
		input_sync(bcm59035_ponkey->dev);
		break;
	case BCM59035_IRQID_INT1_PONKEYH:
		PMU_PONKEY_LOG("isr: PONKEYH irq");
		break;
	case BCM59035_IRQID_INT1_PONKEYR:
		PMU_PONKEY_LOG("isr: PONKEYR irq");
		input_report_key(bcm59035_ponkey->dev, KEY_END, 0);
		input_sync(bcm59035_ponkey->dev);
		break;
	}
}

static int __devinit bcm59035_ponkey_probe(struct platform_device *pdev)
{
	struct bcm59035 *bcm59035 = dev_get_drvdata(pdev->dev.parent);
	struct bcm59035_ponkey *bcm59035_ponkey;
	int ret = 0, err;
	u8 rval;

	PMU_PONKEY_LOG("probe()++");
	bcm59035_ponkey = kzalloc(sizeof(struct bcm59035_ponkey), GFP_KERNEL);
	if (bcm59035_ponkey == NULL) {
		PMU_PONKEY_LOG("probe: falied allocate memory");
		ret = -ENOMEM;
		goto err;
	}
	bcm59035_ponkey->bcm59035 = bcm59035;

	bcm59035_ponkey->dev = input_allocate_device();
	if (!bcm59035_ponkey->dev) {
		PMU_PONKEY_LOG("probe: Can't allocate input dev");
		ret = -ENOMEM;
		goto err;
	}

	set_bit(EV_KEY, bcm59035_ponkey->dev->evbit);
	set_bit(KEY_END, bcm59035_ponkey->dev->keybit);
	bcm59035_ponkey->dev->name = "bcm59035_ponkey";
	bcm59035_ponkey->dev->phys = "bcm59035_ponkey/input0";
	bcm59035_ponkey->dev->id.bustype = BUS_HOST;
	bcm59035_ponkey->dev->id.vendor = 0x0001;
	bcm59035_ponkey->dev->id.product = 0x00010;
	bcm59035_ponkey->dev->id.version = 0x0100;
	/* bcm59035_ponkey->dev->dev.parent = &pdev->dev; */

	/* Set debounce time for PONKEY press and release
	 * to 50ms. This is to avoid any spurious interrupt.
	 * Set KEYLOCK to refrain PMU from shutting down system
	*/
	err = bcm59035->read_dev(bcm59035, BCM59035_REG_PONKEYBDB, &rval);
	PMU_PONKEY_LOG("probe: PONKEYBDB reg val =  0x%x", rval);
	if (err)
		PMU_PONKEY_LOG("probe: can not read PONKEYBDB reg");
	else
		PMU_PONKEY_LOG("probe: PONKEYBDB reg val =  0x%x", rval);

	rval &= ~BCM59035_PONKEYBDB_PONKEYBRF_MASK;
	rval |= ((PONKYB_100_DB << BCM59035_PONKEYBDB_PONKEYBRF_SHIFT) |
					BCM59035_PONKEYBDB_KEYLOCK);

	err = bcm59035->write_dev(bcm59035, BCM59035_REG_PONKEYBDB, rval);
	if (err)
		PMU_PONKEY_LOG("probe: can not write on PONKEYBDB reg");
	else
		PMU_PONKEY_LOG("probe: Written val on PONKEYBDB 0x%x", rval);


	ret =
	    bcm59035_request_irq(bcm59035, BCM59035_IRQID_INT1_PONKEYF, false,
				 bcm59035_ponkey_isr, bcm59035_ponkey);
	ret |=
	    bcm59035_request_irq(bcm59035, BCM59035_IRQID_INT1_PONKEYH, false,
				 bcm59035_ponkey_isr, bcm59035_ponkey);
	ret |=
	    bcm59035_request_irq(bcm59035, BCM59035_IRQID_INT1_PONKEYR, false,
				 bcm59035_ponkey_isr, bcm59035_ponkey);
	if (ret) {
		PMU_PONKEY_LOG("probe: IRQ registration failed");
		ret = -ENOMEM;
		goto err;
	}

	ret = input_register_device(bcm59035_ponkey->dev);
	if (ret) {
		PMU_PONKEY_LOG("probe: Input device register failed");
		goto err_irq;
	}

	platform_set_drvdata(pdev, bcm59035_ponkey);
	ret = bcm59035_enable_irq(bcm59035, BCM59035_IRQID_INT1_PONKEYF);
	ret |= bcm59035_enable_irq(bcm59035, BCM59035_IRQID_INT1_PONKEYH);
	ret |= bcm59035_enable_irq(bcm59035, BCM59035_IRQID_INT1_PONKEYR);
	if (ret) {
		PMU_PONKEY_LOG("probe: IRQ Enabling failed");
		goto err_irq;
	}

	return 0;

err_irq:
	bcm59035_free_irq(bcm59035, BCM59035_IRQID_INT1_PONKEYF);
	bcm59035_free_irq(bcm59035, BCM59035_IRQID_INT1_PONKEYH);
	bcm59035_free_irq(bcm59035, BCM59035_IRQID_INT1_PONKEYR);
err:
	if (bcm59035_ponkey)
		kfree(bcm59035_ponkey);
	if (bcm59035_ponkey->dev)
		input_free_device(bcm59035_ponkey->dev);
	return ret;
}

static int __devexit bcm59035_ponkey_remove(struct platform_device *pdev)
{
	struct bcm59035_ponkey *bcm59035_ponkey = platform_get_drvdata(pdev);
	bcm59035_free_irq(bcm59035_ponkey->bcm59035,
			  BCM59035_IRQID_INT1_PONKEYF);
	bcm59035_free_irq(bcm59035_ponkey->bcm59035,
			  BCM59035_IRQID_INT1_PONKEYH);
	bcm59035_free_irq(bcm59035_ponkey->bcm59035,
			  BCM59035_IRQID_INT1_PONKEYR);
	if (bcm59035_ponkey->dev)
		input_free_device(bcm59035_ponkey->dev);
	if (bcm59035_ponkey)
		kfree(bcm59035_ponkey);
	return 0;
}

static struct platform_driver bcm59035_ponkey_driver = {
	.probe = bcm59035_ponkey_probe,
	.remove = __devexit_p(bcm59035_ponkey_remove),
	.driver = {
		   .name = "bcm59035-ponkey",
		   .owner = THIS_MODULE,
		   },
};

static int __init bcm59035_ponkey_init(void)
{
	PMU_PONKEY_LOG("ponkey_init()++");
	return platform_driver_register(&bcm59035_ponkey_driver);
}

module_init(bcm59035_ponkey_init);

static void __exit bcm59035_ponkey_exit(void)
{
	platform_driver_unregister(&bcm59035_ponkey_driver);
}

module_exit(bcm59035_ponkey_exit);

MODULE_ALIAS("platform:bcm59035-ponkey");
MODULE_DESCRIPTION("BCM59035 POWER ON pin");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("TKG");
