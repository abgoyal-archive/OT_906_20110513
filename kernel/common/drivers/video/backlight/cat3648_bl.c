/*
*linux/drivers/video/backlight/cat3648_bl.c
*
*This program is free software; you can redistribute it and/or modify
*it under the terms of the GNU General Public License version 2 as
*published by the Free Software Foundation.
*/

/*******************************************************************************
*Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*@file	drivers/video/backlight/cat3648_bl.c
*
*Unless you and Broadcom execute a separate written software license agreement
*governing use of this software, this software is licensed to you under the
*terms of the GNU General Public License version 2, available at
*http://www.gnu.org/copyleft/gpl.html (the "GPL").
*
*Notwithstanding the above, under no circumstances may you combine this
*software in any way with any other Broadcom software provided under a license
*other than the GPL, without Broadcom's express prior written consent.
*******************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/cat3648_bl.h>
#include <plat/gpio.h>
#include <linux/delay.h>
#include <linux/broadcom/lcd.h>
#include <mach/reg_sys.h>
#include <linux/broadcom/regaccess.h>
#include <plat/timer.h>

struct cat3648_bl_data {
	struct platform_device *pdev;
	unsigned int ctrl_pin;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend_desc;
        unsigned int backLightSuspend;
#endif
};

static void cat3648_disable_backlight(unsigned int ctrl_pin)
{
	gpio_set_value(ctrl_pin, 0);

	/* Delay of TLR to be provided, as specified in datasheet */
	udelay(500);
}


void cat3648_set_brightness(unsigned int ctrl_pin, int brightness)
{

/*We seperate CAT3648 as 8 level for make it clean in each level change. The level[] comes from
human eye observed and can be addjusted accord to different scenario.
*/
	u8 level[8] = {29, 25, 22, 18, 14, 10, 8, 4};
	int i, pulse;
	static int cur_brightness = 31;
	/* regaccess_and_bits( &REG_SYS_IOCR0, ~REG_SYS_IOCR0_BKLIGHT );*/

	if (brightness == 31){
	gpio_set_value(ctrl_pin, 1);
	}
        else
      	{
	printk(":cat3648_set_brightness  brightness=%d  \r\n", brightness);
	brightness = (brightness / 4);
	if (brightness > (sizeof(level) - 1))
			brightness = sizeof(level) - 1;

	pulse = (level[brightness] - cur_brightness+32)%32;

	for (i = 0; i < pulse; ++i) {
			gpio_set_value(ctrl_pin, 0);

	udelay(1);
	gpio_set_value(ctrl_pin, 1);
	}

	/* Delay of TLR to be provided, as specified in datasheet */
	udelay(200);
	cur_brightness = level[brightness];
      	}
}


static int cat3648_backlight_update_status(struct backlight_device *bl)
{
	struct cat3648_bl_data *cat3648 = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	//printk(":cat3648_backlight_update_status  brightness=%d  %d\r\n", brightness,cat3648->backLightSuspend);

	if(cat3648->backLightSuspend==1)
	     return 0;
	if (brightness == 0)
		cat3648_disable_backlight(cat3648->ctrl_pin);
	else{
		regaccess_and_bits(&REG_SYS_IOCR0, ~REG_SYS_IOCR0_BKLIGHT);
		cat3648_set_brightness(cat3648->ctrl_pin, brightness);
}
	return 0;
}

static int cat3648_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static struct backlight_ops cat3648_backlight_ops = {
	.update_status	= cat3648_backlight_update_status,
	.get_brightness	= cat3648_backlight_get_brightness,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cat3648_backlight_earlysuspend(struct early_suspend *desc)
{
	struct cat3648_bl_data *cat3648 = container_of(desc, struct cat3648_bl_data,
				early_suspend_desc);
	
	//printk(" cat3648_backlight_earlysuspend++  %d \r\n",cat3648->backLightSuspend);
	cat3648_disable_backlight(cat3648->ctrl_pin);
	cat3648->backLightSuspend=1;
}

static void cat3648_backlight_earlyresume(struct early_suspend *desc)
{
        
	struct cat3648_bl_data *cat3648 = container_of(desc, struct cat3648_bl_data,
					early_suspend_desc);
	struct backlight_device *bl = platform_get_drvdata(cat3648->pdev);
	cat3648->backLightSuspend=0;

	
	//printk(" cat3648_backlight_earlyresume++   %d\r\n", cat3648->backLightSuspend);

	backlight_update_status(bl);
	
}
#else
#ifdef CONFIG_PM
static int cat3648_backlight_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct cat3648_bl_data *cat3648 = dev_get_drvdata(&bl->dev);

	cat3648_disable_backlight(cat3648->ctrl_pin);
	return 0;
}

static int cat3648_backlight_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_update_status(bl);
	return 0;
}
#else
#define cat3648_backlight_suspend  NULL
#define cat3648_backlight_resume   NULL
#endif
#endif

static int cat3648_backlight_probe(struct platform_device *pdev)
{
	struct platform_cat3648_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct cat3648_bl_data *cat3648;
	int ret;

	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	cat3648 = kzalloc(sizeof(*cat3648), GFP_KERNEL);
	if (!cat3648) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	cat3648->ctrl_pin = data->ctrl_pin;

	bl = backlight_device_register(pdev->name, &pdev->dev,
			cat3648, &cat3648_backlight_ops);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	gpio_direction_output(data->ctrl_pin, 1);
	cat3648_set_brightness(data->ctrl_pin, data->max_brightness);
#ifdef CONFIG_HAS_EARLYSUSPEND
	cat3648->pdev = pdev;
	cat3648->early_suspend_desc.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	cat3648->early_suspend_desc.suspend = cat3648_backlight_earlysuspend;
	cat3648->early_suspend_desc.resume = cat3648_backlight_earlyresume;
	register_early_suspend(&cat3648->early_suspend_desc);
#endif

	bl->props.max_brightness = data->max_brightness;
	bl->props.brightness = data->dft_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);
	return 0;

err_bl:
	kfree(cat3648);
err_alloc:
	return ret;
}

static int cat3648_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct cat3648_bl_data *cat3648 = dev_get_drvdata(&bl->dev);

	backlight_device_unregister(bl);
	cat3648_disable_backlight(cat3648->ctrl_pin);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&cat3648->early_suspend_desc);
#endif

	kfree(cat3648);
	return 0;
}


static struct platform_driver cat3648_backlight_driver = {
	.driver		= {
		.name	= "cat3648-backlight",
		.owner	= THIS_MODULE,
	},
	.probe		= cat3648_backlight_probe,
	.remove		= cat3648_backlight_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend        = cat3648_backlight_suspend,
	.resume         = cat3648_backlight_resume,
#endif
};

static int __init cat3648_backlight_init(void)
{
	return platform_driver_register(&cat3648_backlight_driver);
}
module_init(cat3648_backlight_init);

static void __exit cat3648_backlight_exit(void)
{
	platform_driver_unregister(&cat3648_backlight_driver);
}
module_exit(cat3648_backlight_exit);

MODULE_DESCRIPTION("cat3648 based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cat3648-backlight");


