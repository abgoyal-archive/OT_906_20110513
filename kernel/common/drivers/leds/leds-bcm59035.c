/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*	@file	drivers/led/leds-bcm59035.c
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
 * drivers/led/leds-bcm59035.c
 *
 * BCM59035 PMU based LED control
 *
 * based on leds-pwm.c by Luotao Fu @ Pengutronix (l.fu@pengutronix.de)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/gpio.h>
#include <linux/mfd/bcm59035/bcm59035.h>

#include <plat/pwm/consumer.h>

#define BCM59035_LED_ALWAYS_ON							  (0xFFFFFFFF)
/* bcm59035 has two pwm channels. this structure tracks the usage of
 * both these channels. led pwm channel id is subtracted from the
 * 'start_id' field to index into the 'usage_count' array. the usage_count
 * ensures that a pwm channel is not requested more than once and also
 * the pwm channel is released only when all the leds have released
 * the channel (this is the case in Brava where two leds are connected
 * to the same pwm channel).
 */
struct pwm_channel_info {
#define BCM59035_MAX_PWM_CHANNELS    2

	int start_id;
	struct pwm_device *pwm[BCM59035_MAX_PWM_CHANNELS];
	int in_use[BCM59035_MAX_PWM_CHANNELS];
};
static struct pwm_channel_info pwm_channel_info;

/* data on each led */
struct bcm59035_led_data {
	struct led_classdev	cdev;
	struct pwm_device	*pwm;
	int			gpio;
	int			gpio_active_on;
};

/* each led will have one instance of this data structure */
static struct bcm59035_led_data *leds_data;

#define to_led_bcm59035_data(led)			\
	container_of(led, struct bcm59035_led_data, cdev)


/**
 * Helper function to compute LED blink pattern
 */
static void bcm59035_get_blink_pattern(unsigned long delay_on,
										unsigned long delay_off,
										int* pattern, int* delay)
{
	pr_info("%s: delay_on: %lu delay_off:%lu\n",
		__func__, delay_on,delay_off);

	if(BCM59035_LED_ALWAYS_ON == delay_on)
	{
		*pattern = BCM59035_LED_PATTERN_ON_ALWAYS;
		*delay = BCM59035_LED_REPEAT_PERIOD_8_0S;
		return;
	}
	if(delay_on <= 50)
	{
		*pattern = BCM59035_LED_PATTERN_ON50_OFF_REST;
	}
	else if(delay_on <= 100)
	{
		*pattern = BCM59035_LED_PATTERN_ON100_OFF_REST;
	}
	else if(delay_on <= 200)
	{
		*pattern = BCM59035_LED_PATTERN_ON200_OFF_REST;
	}
	else
	{
		*pattern = BCM59035_LED_PATTERN_ON500_OFF_REST;
	}

	if(delay_off <= 1000)
	{
		*delay = (delay_off == 1000) ? BCM59035_LED_REPEAT_PERIOD_1_0S :
									BCM59035_LED_REPEAT_PERIOD_0_4S;
	}
	else if(delay_off <= 2000)
	{
		*delay = BCM59035_LED_REPEAT_PERIOD_2_0S;
	}
	else if(delay_off <= 4000)
	{
		*delay = BCM59035_LED_REPEAT_PERIOD_4_0S;
	}
	else if(delay_off <= 6000)
	{
		*delay = BCM59035_LED_REPEAT_PERIOD_6_0S;
	}
	else
	{
		*delay = BCM59035_LED_REPEAT_PERIOD_8_0S;
	}

}

/*********************************************************************
 *                        LED SUBSYSTEM CALLBACKS                    *
 *********************************************************************/

 /**
 * Callback function to set LED blink pattern
 */
static int bcm59035_led_set_blink(struct led_classdev *led_cdev,
				unsigned long *delay_on,
				unsigned long *delay_off)
{
	struct bcm59035_led_data *led_dat =
		container_of(led_cdev, struct bcm59035_led_data, cdev);
	int pattern = 0;
	int delay = 0;
	if (*delay_on == 0 && *delay_off == 0) {
		/* Special case: the leds subsystem requires a default user
		 * friendly blink pattern for the LED.

		 Probe function takes of  default setting...just return*/
		return 0;
	}

	bcm59035_get_blink_pattern(*delay_on,*delay_off,
								&pattern,&delay);
	pr_info("%s: pattern: %d delay:%d\n",
		__func__, pattern,delay);

	pwm_config(led_dat->pwm, pattern, delay);
	return 0;
}

/**
 * Callback function to set LED brightness
 */
static void bcm59035_led_bcm_brightness_set(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	struct bcm59035_led_data *led_dat =
		container_of(led_cdev, struct bcm59035_led_data, cdev);

	pr_info("%s: brightness: %d\n",
		__func__, brightness);
	if (brightness == LED_OFF) {
		pwm_disable(led_dat->pwm);
		if (led_dat->gpio >= 0) {
			gpio_set_value(led_dat->gpio,
				!led_dat->gpio_active_on);
		}
	} else {
		pwm_enable(led_dat->pwm);
		if (led_dat->gpio >= 0) {
			gpio_set_value(led_dat->gpio,
				led_dat->gpio_active_on);
		}
	}

	/* if this api was called from led core's suspend handler,
	 * don't save the brightness value in the handle.
	 */
	if (!(led_cdev->flags & LED_SUSPENDED))
		led_cdev->brightness = brightness;
}

/*********************************************************************
 *                           INIT ROUTINES                           *
 *********************************************************************/
/**
 * LED mode probe handler
 */
static int bcm59035_led_probe(struct platform_device *pdev)
{
	struct bcm59035 *bcm59035 = dev_get_drvdata(pdev->dev.parent);
	struct bcm59035_led_pdata *pdata;
	struct bcm59035_led *cur_led;
	struct bcm59035_led_data *led_dat;
	int i, ret = 0, chan_index;

	dev_info(&pdev->dev, "%s\n", __func__);

	pdata = bcm59035->pdata->led;
	if (!pdata) {
		dev_err(&pdev->dev, "%s: error: platform data not set!\n",
			__func__);
		return -EINVAL;
	}

	leds_data = kzalloc(pdata->num_leds * sizeof(struct bcm59035_led_data),
		GFP_KERNEL);
	if (!leds_data)
		return -ENOMEM;

	/* backup the pwm channel start id */
	pwm_channel_info.start_id = pdata->start_id;

	for (i = 0; i < pdata->num_leds; i++) {
		cur_led = &pdata->leds[i];
		led_dat = &leds_data[i];

		/* request for the gpio to control pwm enable/disable */
		if (cur_led->gpio >= 0) {
			ret = gpio_request(cur_led->gpio, cur_led->name);
			if (ret != 0) {
				dev_err(&pdev->dev, "%s: request for gpio"
					" %d failed\n", __func__,
					cur_led->gpio);
				goto err;
			}

			if (cur_led->init_state == MODE_PWM_OFF_LED_OFF)
				ret = gpio_direction_output(cur_led->gpio,
					!cur_led->gpio_active_on);
			else
				ret = gpio_direction_output(cur_led->gpio,
					cur_led->gpio_active_on);

			if (ret != 0) {
				dev_err(&pdev->dev, "%s: request for gpio %d"
				" dir failed\n", __func__, cur_led->gpio);
				goto err;
			}

			led_dat->gpio = cur_led->gpio;
			led_dat->gpio_active_on = cur_led->gpio_active_on;
		}

		/* request the pwm channel only if it has not already been
		 * requested for.
		 */
		chan_index = cur_led->pwm_id - pwm_channel_info.start_id;
		if (pwm_channel_info.in_use[chan_index] == 0) {
			led_dat->pwm = pwm_request(cur_led->pwm_id,
				cur_led->name);
			if (IS_ERR(led_dat->pwm)) {
				dev_err(&pdev->dev, "%s: unable to request "
					"pwm %d\n", __func__, cur_led->pwm_id);
				goto err;
			}

			if (cur_led->init_state == MODE_PWM_OFF_LED_OFF) {
				/* keep the channels disabled */
				pwm_disable(led_dat->pwm);

				/* set the pwm channel to led mode */
				ret = pwm_setmode(led_dat->pwm,
					BCM59035_LEDON);
				if (ret < 0) {
					dev_err(&pdev->dev, "%s: configuring "
						"led mode failed: %d\n",
						__func__, cur_led->pwm_id);
					goto err;
				}
			}

			/* mark the channel as in use */
			pwm_channel_info.in_use[chan_index] = 1;
			pwm_channel_info.pwm[chan_index] = led_dat->pwm;
		} else {
			/* if the channel is already registered, the new
			 * led device must use the same channel handle.
			 */
			led_dat->pwm = pwm_channel_info.pwm[chan_index];
		}

		led_dat->cdev.name = cur_led->name;
		led_dat->cdev.brightness_set = bcm59035_led_bcm_brightness_set;
		led_dat->cdev.blink_set = bcm59035_led_set_blink;
		led_dat->cdev.brightness = LED_OFF;
		led_dat->cdev.max_brightness = cur_led->max_brightness;
		led_dat->cdev.flags = cur_led->flags;
		led_dat->cdev.default_trigger = cur_led->default_trigger;
		/* register with the led core */
		ret = led_classdev_register(&pdev->dev, &led_dat->cdev);
		if (ret < 0) {
			pwm_free(led_dat->pwm);
			goto err;
		}
	}

	platform_set_drvdata(pdev, leds_data);
	dev_info(&pdev->dev, "%s: success\n", __func__);

	return 0;

err:
	if (i > 0) {
		for (i = i - 1; i >= 0; i--) {
			led_classdev_unregister(&leds_data[i].cdev);
			pwm_free(leds_data[i].pwm);
		}
	}

	kfree(leds_data);

	return ret;
}

static int __devexit bcm59035_led_remove(struct platform_device *pdev)
{
	struct bcm59035 *bcm59035 = dev_get_drvdata(pdev->dev.parent);
	struct bcm59035_led_pdata *pdata = bcm59035->pdata->led;
	struct bcm59035_led_data *leds_data;
	int i;

	leds_data = platform_get_drvdata(pdev);

	for (i = 0; i < pdata->num_leds; i++) {
		led_classdev_unregister(&leds_data[i].cdev);
		pwm_free(leds_data[i].pwm);
	}

	memset(&pwm_channel_info, 0, sizeof(struct pwm_channel_info));
	kfree(leds_data);

	return 0;
}

static struct platform_driver bcm59035_led_driver = {
	.probe		= bcm59035_led_probe,
	.remove		= __devexit_p(bcm59035_led_remove),
	.driver		= {
		.name	= "bcm59035-led",
		.owner	= THIS_MODULE,
	},
};

static int __init bcm59035_led_init(void)
{
	return platform_driver_register(&bcm59035_led_driver);
}

static void __exit bcm59035_led_exit(void)
{
	platform_driver_unregister(&bcm59035_led_driver);
}

module_init(bcm59035_led_init);
module_exit(bcm59035_led_exit);

MODULE_ALIAS("platform:bcm59035-led");
MODULE_DESCRIPTION("BCM59035 LED");
MODULE_LICENSE("GPL");
