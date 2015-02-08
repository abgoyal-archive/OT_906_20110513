/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/gpio/bcm59035-gpio.c
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
 *   @file   bcm59035-gpio.c
 *
 *   @brief  GPIO interface for Broadcom BCM59035 PMU
 *
 ****************************************************************************/
#include <linux/mfd/bcm59035/bcm59035.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/seq_file.h>

struct bcm59035_gpio {
	struct bcm59035 *bcm59035;
	struct gpio_chip gpio_chip;
};

static int bcm59035_gpio_direction_in(struct gpio_chip *chip, unsigned offset)
{
	u8 regVal;
	struct bcm59035_gpio *bcm59035_gpio =
	    container_of(chip, struct bcm59035_gpio, gpio_chip);
	struct bcm59035 *bcm59035 = bcm59035_gpio->bcm59035;
	int ret;

	if (offset < 0 || offset > 1) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"bcm59035_gpio_direction_in : Invalid offset\n");
		return -EINVAL;
	}

	ret = bcm59035->read_dev(bcm59035, BCM59035_REG_GPIOCTRL, &regVal);
	if (ret < 0) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"bcm59035_gpio_direction_in : PMU read error !!! \n");
		return ret;
	}

	if (offset == 0) {
		regVal &= ~(0x03);	/* clear bit 0 & 1 */
		regVal |= 0x02;	/* set dir. as input */
	} else {
		regVal &= ~(0x03 << 3);	/* clear bit 3 & 4 */
		regVal |= (0x02 << 3);	/* set dir. as input */
	}

	return bcm59035->write_dev(bcm59035, BCM59035_REG_GPIOCTRL, regVal);
}

static int bcm59035_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct bcm59035_gpio *bcm59035_gpio =
	    container_of(chip, struct bcm59035_gpio, gpio_chip);
	struct bcm59035 *bcm59035 = bcm59035_gpio->bcm59035;
	int ret;
	u8 regVal;

	if (offset < 0 || offset > 1) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"bcm59035_gpio_get : Invalid offset\n");
		return -EINVAL;
	}

	ret = bcm59035->read_dev(bcm59035, BCM59035_REG_GPIOCTRL, &regVal);
	if (ret < 0) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"bcm59035_gpio_get : PMU read error !!! \n");
		return ret;
	}

	if (offset == 0)
		return regVal & (1 << 2);
	else
		return regVal & (1 << 5);
}

static int bcm59035_gpio_direction_out(struct gpio_chip *chip,
				       unsigned offset, int value)
{
	u8 regVal, bitPos;
	struct bcm59035_gpio *bcm59035_gpio =
	    container_of(chip, struct bcm59035_gpio, gpio_chip);
	struct bcm59035 *bcm59035 = bcm59035_gpio->bcm59035;
	int ret;

	if (offset < 0 || offset > 1) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"bcm59035_gpio_direction_out : Invalid offset\n");
		return -EINVAL;
	}

	ret = bcm59035->read_dev(bcm59035, BCM59035_REG_GPIOCTRL, &regVal);
	if (ret < 0) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"bcm59035_gpio_direction_out : PMU read error !!! \n");
		return ret;
	}

	if (offset == 0) {
		regVal &= ~(0x03);	/* clear bit 0 & 1 */
		regVal |= 0x01;	/* set dir. as o/p */
		bitPos = 2;
	} else {
		regVal &= ~(0x03 << 3);	/* clear bit 3 & 4 */
		regVal |= (0x01 << 3);	/* set dir. as o/p */
		bitPos = 5;
	}

	if (value)
		regVal |= (1 << bitPos);
	else
		regVal &= ~(1 << bitPos);

	return bcm59035->write_dev(bcm59035, BCM59035_REG_GPIOCTRL, regVal);
}

static void bcm59035_gpio_set(struct gpio_chip *chip, unsigned offset,
			      int value)
{
	struct bcm59035_gpio *bcm59035_gpio =
	    container_of(chip, struct bcm59035_gpio, gpio_chip);
	struct bcm59035 *bcm59035 = bcm59035_gpio->bcm59035;
	int ret;
	u8 bitPos, regVal;

	if (offset < 0 || offset > 1) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"bcm59035_gpio_set : Invalid offset\n");
		return;
	}

	ret = bcm59035->read_dev(bcm59035, BCM59035_REG_GPIOCTRL, &regVal);
	if (ret < 0) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"bcm59035_gpio_set : PMU read error !!! \n");
		return;
	}

	bitPos = (offset == 0) ? 2 : 5;
	if (value)
		regVal |= (1 << bitPos);
	else
		regVal &= ~(1 << bitPos);

	bcm59035->write_dev(bcm59035, BCM59035_REG_GPIOCTRL, regVal);

}

#ifdef CONFIG_DEBUG_FS
static void bcm59035_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	struct bcm59035_gpio *bcm59035_gpio =
	    container_of(chip, struct bcm59035_gpio, gpio_chip);
	struct bcm59035 *bcm59035 = bcm59035_gpio->bcm59035;
	u8 regVal, i, dirBits;
	const char *label, *dir;

	if (bcm59035->read_dev(bcm59035, BCM59035_REG_GPIOCTRL, &regVal) < 0) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"bcm59035_gpio_dbg_show : PMU read error !!! \n");
		return;
	}

	for (i = 0; i < chip->ngpio; i++) {
		label = gpiochip_is_requested(chip, i);
		if (!label)
			label = "Unrequested";

		seq_printf(s, " gpio-%-3d (%-20.20s) ", chip->base, label);
		dirBits = (i == 0) ? (regVal & 0x3) : ((regVal >> 3) & 0x3);
		switch (dirBits) {
		case 0x0:
			dir = "hi-Z";
			break;
		case 0x01:
			dir = "out";
			break;
		case 0x02:
			dir = "in";
			break;
		default:
			dir = "ERROR!!!";
			break;
		}
		seq_printf(s, " %s %s\n", dir,
			   (bcm59035_gpio_get(chip, i) ? "high" : "low"));
	}
}
#else
#define bcm59035_gpio_dbg_show NULL
#endif

static int __devinit bcm59035_gpio_probe(struct platform_device *pdev)
{
	struct bcm59035 *bcm59035 = dev_get_drvdata(pdev->dev.parent);
	struct bcm59035_gpio *bcm59035_gpio;
	int ret;

	PMU_LOG(DEBUG_PMU_INFO, "Inside %s\n", __FUNCTION__);

	if (!bcm59035) {
		PMU_LOG(DEBUG_PMU_ERROR, "bcm59035 == NULL !!!\n");
		return -EINVAL;
	}

	bcm59035_gpio = kzalloc(sizeof(struct bcm59035_gpio), GFP_KERNEL);
	if (bcm59035_gpio == NULL) {
		PMU_LOG(DEBUG_PMU_ERROR, "Memory error !!!\n");
		return -ENOMEM;
	}

	bcm59035_gpio->bcm59035 = bcm59035;
	bcm59035_gpio->gpio_chip.label = "bcm59035-gpio";
	bcm59035_gpio->gpio_chip.owner = THIS_MODULE;
	bcm59035_gpio->gpio_chip.direction_input = bcm59035_gpio_direction_in;
	bcm59035_gpio->gpio_chip.get = bcm59035_gpio_get;
	bcm59035_gpio->gpio_chip.direction_output = bcm59035_gpio_direction_out;
	bcm59035_gpio->gpio_chip.set = bcm59035_gpio_set;
	bcm59035_gpio->gpio_chip.dbg_show = bcm59035_gpio_dbg_show;
	bcm59035_gpio->gpio_chip.can_sleep = 0;
	bcm59035_gpio->gpio_chip.ngpio = BCM59035_NUM_GPIO;
	bcm59035_gpio->gpio_chip.dev = &pdev->dev;

	if (bcm59035->pdata)
		bcm59035_gpio->gpio_chip.base = bcm59035->pdata->gpio_base;
	else
		bcm59035_gpio->gpio_chip.base = -1;

	ret = gpiochip_add(&bcm59035_gpio->gpio_chip);
	if (ret < 0) {
		PMU_LOG(DEBUG_PMU_ERROR, "Could not register gpiochip, %d\n",
			ret);
		kfree(bcm59035_gpio);
		return ret;
	}
	platform_set_drvdata(pdev, bcm59035_gpio);
	return ret;
}

static int __devexit bcm59035_gpio_remove(struct platform_device *pdev)
{
	struct bcm59035_gpio *bcm59035_gpio = platform_get_drvdata(pdev);
	int ret;

	ret = gpiochip_remove(&bcm59035_gpio->gpio_chip);
	if (ret == 0)
		kfree(bcm59035_gpio);

	return ret;
}

static struct platform_driver bcm59035_gpio_driver = {
	.driver.name = "bcm59035-gpio",
	.driver.owner = THIS_MODULE,
	.probe = bcm59035_gpio_probe,
	.remove = __devexit_p(bcm59035_gpio_remove),
};

static int __init bcm59035_gpio_init(void)
{
	return platform_driver_register(&bcm59035_gpio_driver);
}

subsys_initcall(bcm59035_gpio_init);

static void __exit bcm59035_gpio_exit(void)
{
	platform_driver_unregister(&bcm59035_gpio_driver);
}

module_exit(bcm59035_gpio_exit);

MODULE_DESCRIPTION("GPIO interface for Broadcom BCM59035 PMU");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bcm59035-gpio");
