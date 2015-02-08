/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*       @file   drivers/video/backlight/bcm_pwm_keypad_bl.c
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/pwm_backlight.h>
#include <linux/platform_device.h>
#include <linux/mfd/bcm59035/bcm59035.h>

#include <plat/pwm/consumer.h>
#include <plat/bcm_pwm_keypad_bl.h>

/*
 * data structure private to this driver
 */
struct pwm_bl_driver_data {
	int pwm_ctrl_gpio;
	bool pwm_ctrl_gpio_active_state;
	int brightness;
	struct pwm_device *pwm;
	unsigned int period;
	int (*notify)(int brightness);
};

/****************************************************************************
 * backlight ops
 ****************************************************************************/

static int pwm_backlight_update_status(struct backlight_device *bl)
{
	struct pwm_bl_driver_data *d = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (d->notify)
		brightness = d->notify(brightness);

	/* If the new brightness value is same as the one
	 * currently configured, then no need to take any
	 * action.
	 */
	if (d->brightness == brightness)
		goto update_stat_exit;

	if (brightness == 0) {
		pwm_config(d->pwm, 0, d->period);
		pwm_disable(d->pwm);

		if (d->pwm_ctrl_gpio >= 0) {
			gpio_set_value(d->pwm_ctrl_gpio,
				!d->pwm_ctrl_gpio_active_state);
		}
	} else {
		pwm_enable(d->pwm);

		pwm_config(d->pwm, brightness * d->period / max,
			d->period);

		if (d->pwm_ctrl_gpio >= 0) {
			gpio_set_value(d->pwm_ctrl_gpio,
				d->pwm_ctrl_gpio_active_state);
		}

	}

	d->brightness = brightness;

update_stat_exit:
	return 0;
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static struct backlight_ops pwm_backlight_ops = {
	.update_status = pwm_backlight_update_status,
	.get_brightness = pwm_backlight_get_brightness,
};

/****************************************************************************
 * power management routines
 ****************************************************************************/

#ifdef CONFIG_PM
static int bcm_pwm_keypad_bl_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_driver_data *d = dev_get_drvdata(&bl->dev);

	pwm_config(d->pwm, 0, d->period);
	pwm_disable(d->pwm);

	if (d->pwm_ctrl_gpio >= 0) {
		gpio_set_value(d->pwm_ctrl_gpio,
			!d->pwm_ctrl_gpio_active_state);
	}

	d->brightness = 0;

	return 0;
}

static int bcm_pwm_keypad_bl_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_update_status(bl);
	return 0;
}
#endif /* CONFIG_PM */

/****************************************************************************
 * init routines
 ****************************************************************************/

static int bcm_pwm_keypad_bl_probe(struct platform_device *pdev)
{
	struct keypad_bl_drv_pdata *pdata = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct pwm_bl_driver_data *d;
	int ret;

	dev_info(&pdev->dev, "%s\n", __func__);

	if (!pdata) {
		dev_err(&pdev->dev, "%s: platform data not set\n", __func__);
		return -EINVAL;
	}

	if (pdata->bl_pdata.init) {
		ret = pdata->bl_pdata.init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	d = kzalloc(sizeof(struct pwm_bl_driver_data), GFP_KERNEL);
	if (!d) {
		dev_err(&pdev->dev, "%s: failed to allocate driver data\n",
			__func__);
		ret = -ENOMEM;
		goto err_alloc;
	}

	d->period = pdata->bl_pdata.pwm_period_ns;
	d->notify = pdata->bl_pdata.notify;
	d->pwm_ctrl_gpio = pdata->pwm_ctrl_gpio;
	d->pwm_ctrl_gpio_active_state = pdata->pwm_ctrl_gpio_active_state;

	/* request for the gpio to control pwm enable/disable */
	if (d->pwm_ctrl_gpio >= 0) {
		ret = gpio_request(d->pwm_ctrl_gpio, "keypad-backlight");
		if (ret != 0) {
			dev_err(&pdev->dev, "%s: request for gpio %d failed\n",
			__func__, d->pwm_ctrl_gpio);
			goto err_request;
		}

		ret = gpio_direction_output(d->pwm_ctrl_gpio,
			!pdata->pwm_ctrl_gpio_active_state);
		if (ret != 0) {
			dev_err(&pdev->dev, "%s: request for gpio %d dir "
			 "failed\n", __func__, d->pwm_ctrl_gpio);
			goto err_request;
		}
	}

	/* request for the pwm channel */
	d->pwm = pwm_request(pdata->bl_pdata.pwm_id, "keypad-backlight");
	if (IS_ERR(d->pwm)) {
		dev_err(&pdev->dev, "%s: keypad backlight request failed\n",
			__func__);
		ret = PTR_ERR(d->pwm);
		goto err_request;
	}

	/* set the pwm channel to led mode */
	ret = pwm_setmode(d->pwm, BCM59035_PWMON);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: configuring pwm mode "
			"failed: %d\n", __func__, ret);
		goto err_set_mode;
	}

	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev,
				       d, &pwm_backlight_ops);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "%s: failed to register backlight\n",
			__func__);
		ret = PTR_ERR(bl);
		goto err_bl_register;
	}

	bl->props.max_brightness = pdata->bl_pdata.max_brightness;
	bl->props.brightness = pdata->bl_pdata.dft_brightness;
	backlight_update_status(bl);
	d->brightness = bl->props.brightness;

	platform_set_drvdata(pdev, bl);
	return 0;

err_set_mode:
err_bl_register:
	pwm_free(d->pwm);

err_request:
	kfree(d);

err_alloc:
	if (pdata->bl_pdata.exit)
		pdata->bl_pdata.exit(&pdev->dev);

	return ret;
}

static int __devexit bcm_pwm_keypad_bl_remove(struct platform_device *pdev)
{
	struct keypad_bl_drv_pdata *pdata = pdev->dev.platform_data;
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_driver_data *d = dev_get_drvdata(&bl->dev);

	backlight_device_unregister(bl);
	pwm_config(d->pwm, 0, 0);
	pwm_disable(d->pwm);
	pwm_free(d->pwm);

	if (d->pwm_ctrl_gpio >= 0)
		gpio_free(d->pwm_ctrl_gpio);

	kfree(d);

	if (pdata->bl_pdata.exit)
		pdata->bl_pdata.exit(&pdev->dev);

	return 0;
}

static struct platform_driver bcm_pwm_keypad_bl_driver = {
	.probe = bcm_pwm_keypad_bl_probe,
	.remove = __devexit_p(bcm_pwm_keypad_bl_remove),

#ifdef CONFIG_PM
	.suspend = bcm_pwm_keypad_bl_suspend,
	.resume = bcm_pwm_keypad_bl_resume,
#endif

	.driver = {
		.name = "pwm-keypad-backlight",
	},
};

static int __init bcm_pwm_keypad_bl_init(void)
{
	pr_debug("%s\n", __func__);
	return platform_driver_register(&bcm_pwm_keypad_bl_driver);
}
late_initcall(bcm_pwm_keypad_bl_init);

static void __exit bcm_pwm_keypad_bl_exit(void)
{
	platform_driver_unregister(&bcm_pwm_keypad_bl_driver);
}
module_exit(bcm_pwm_keypad_bl_exit);

MODULE_ALIAS("platform:pwm-backlight");
MODULE_DESCRIPTION("Broadcom PWM Keypad Backlight Driver");
MODULE_LICENSE("GPL");
