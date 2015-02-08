/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/video/backlight/lcdfb_backlight.c
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

/*
*
*   lcdfb_backlight.c
*
*   This file contains code which enables backlight support through the
*   backlight device.
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/broadcom/lcd.h>
#include <linux/broadcom/lcd_backlight.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/broadcom/cpu_sleep.h>

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */

static char gBanner[] __initdata = KERN_INFO "LCDFB-Backlight: 0.1\n";

static LCD_BACKLIGHT_LEVEL gPrevLevel;

/* ---- Private Variables ------------------------------------------------ */

/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*   Return the current backlight brightness; accounting for power, blanking
*   etc.
*
****************************************************************************/

static int lcdfb_backlight_get_brightness(struct backlight_device *bl_dev)
{

	(void)bl_dev;

	return lcd_backlight_curr_level();

}				/* lcdfb_backlight_get_brightness */

/****************************************************************************
*
*   Callback which is called whenever a property has been changed
*
****************************************************************************/

static int lcdfb_backlight_update_status(struct backlight_device *bl_dev)
{
	LCD_BACKLIGHT_LEVEL level;

	level = bl_dev->props.brightness;

	if ((bl_dev->props.power != FB_BLANK_UNBLANK)
	    || (bl_dev->props.fb_blank != FB_BLANK_UNBLANK)) {
		pr_info("\r\n[lcdfb_backlight_update_status set level = 0]");
		level = 0;
	}

#if defined(CONFIG_FB_LCD)
/*      lcd_set_power(bl_dev->props.power == FB_BLANK_UNBLANK); */
	lcd_set_power(level ? 1 : 0);
#endif

	lcd_backlight_enable(level);

	return 0;

}				/* lcdfb_backlight_update_status */

/****************************************************************************
*
*   Data structure which defines the backlight operations
*
****************************************************************************/

static struct backlight_ops lcdfb_backlight_ops = {
	.update_status = lcdfb_backlight_update_status,
	.get_brightness = lcdfb_backlight_get_brightness,
};

#if defined CONFIG_HAS_EARLYSUSPEND
static void backlight_early_suspend(struct early_suspend *h)
{
	pr_info("[lcdfb_backlight_suspend]+++\n");
	gPrevLevel = lcd_backlight_curr_level();

	if (gPrevLevel != 0) {
		pr_info("\r\n[lcdfb_backlight_suspend] Action");
		lcd_backlight_enable(0);
	}
	pr_info("[lcdfb_backlight_suspend]---\n");
}

static void backlight_late_resume(struct early_suspend *h)
{
	pr_info("[lcdfb_backlight_resume]+++\n");

	if ((lcd_backlight_curr_level() == 0) && (gPrevLevel != 0)) {
		pr_info("\r\n[lcdfb_backlight_resume] Action");
		lcd_backlight_enable(gPrevLevel);
	}
	pr_info("[lcdfb_backlight_resume]---\n");

}

static struct early_suspend backlight_early_suspend_desc = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = backlight_early_suspend,
	.resume = backlight_late_resume,
};
#endif

/****************************************************************************
*
*   Function called to initialize our driver.
*
****************************************************************************/

static int __init lcdfb_backlight_probe(struct platform_device *dev)
{
	struct backlight_device *bl_dev;

	lcd_backlight_init();

	bl_dev =
	    backlight_device_register("lcdfb-bl", &dev->dev, NULL,
				      &lcdfb_backlight_ops);
	if (IS_ERR(bl_dev)) {
		pr_err(
		       "lcdfb: Registration of backlight device failed\n");
		return PTR_ERR(bl_dev);
	}

	platform_set_drvdata(dev, bl_dev);

	bl_dev->props.max_brightness = lcd_backlight_max_level();
	bl_dev->props.brightness = bl_dev->props.max_brightness;
	bl_dev->props.power = FB_BLANK_UNBLANK;
	bl_dev->props.fb_blank = FB_BLANK_UNBLANK;

	backlight_update_status(bl_dev);

#if defined CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&backlight_early_suspend_desc);
#endif

	pr_info("%s", gBanner);

	return 0;

}				/* lcdfb_backlight_probe */

/****************************************************************************
*
*   Function called to remove our driver.
*
****************************************************************************/

static int lcdfb_backlight_remove(struct platform_device *dev)
{
	struct backlight_device *bl_dev = platform_get_drvdata(dev);

	bl_dev->props.brightness = 0;
	bl_dev->props.power = 0;
	backlight_update_status(bl_dev);

	backlight_device_unregister(bl_dev);

	lcd_backlight_deinit();

	return 0;

}				/* lcdfb_backlight_remove */

/****************************************************************************
*
*   Function called to shutdown driver whenever system is shutting down.
*
****************************************************************************/

static void lcdfb_backlight_shutdown(struct platform_device *dev)
{
	struct backlight_device *bl_dev = platform_get_drvdata(dev);

	bl_dev->props.brightness = 0;
	bl_dev->props.power = 0;
	backlight_update_status(bl_dev);

	backlight_device_unregister(bl_dev);

	lcd_backlight_deinit();

}				/* lcdfb_backlight_shutdown */

/****************************************************************************
*
*   Data structure which defines the backlight driver
*
****************************************************************************/

static struct platform_driver lcdfb_backlight_driver = {
	.probe = lcdfb_backlight_probe,
	.remove = lcdfb_backlight_remove,
	.shutdown = lcdfb_backlight_shutdown,
	.driver = {
		   .name = "lcdfb-bl",
		   },
};

/****************************************************************************
*
*   Module initialization and cleanup.
*
****************************************************************************/

static int __init lcdfb_backlight_init(void)
{

	return platform_driver_register(&lcdfb_backlight_driver);
}

static void __exit lcdfb_backlight_exit(void)
{
	platform_driver_unregister(&lcdfb_backlight_driver);
}

/****************************************************************************/

/* module_init( lcdfb_backlight_init); */
fs_initcall(lcdfb_backlight_init);

module_exit(lcdfb_backlight_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom LCD Backlight Driver");
MODULE_LICENSE("GPL");
