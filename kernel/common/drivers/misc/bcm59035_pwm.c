/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*       @file   drivers/misc/bcm59035_pwm.c
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
#include <linux/platform_device.h>
#include <linux/mfd/bcm59035/bcm59035.h>

#include <mach/pwm.h>
#include <plat/pwm/pwm_core.h>

/* maximum number of channels supported by this driver */
#define BCM59035_MAX_PWM_CHANNELS    2

/*
 * BCM59035 PWM register bit masks
 */
#define PWMLEDCTRL_PWMHP_MASK        0x3F
#define PWMLEDCTRL_PWMLP_MASK        0x3F
#define PWMLEDCTRL_PWMDIV_MASK       0x0C
#define PWMLEDCTRL_PWMDIV_SHIFT      2

#define FSYSTEM_HZ                   (2 * 1000000)

/*
 * descriptors for pwm channels, one for each channel.
 */
struct pwm_device_desc {
	struct pwm_desc desc;
	struct pwm_channel *chan;

	/* current channel mode */
	int chan_mode;

	/* channel register addresses */
	u8 mode_reg;
	u8 high_period_reg;
	u8 low_period_reg;
	u8 pad_reg;
};

/*
 * data structure private to this driver. this data is common to
 * all pwm channel instances.
 */
struct pwm_driver_data {
	/* start id of the set of pwm channels supported by this
	 * driver. subsequent ids are assumed to be sequential
	 * from the start id.
	 */
	int start_id;

	/* handle to the mfd core driver */
	struct bcm59035 *bcm59035;

	/* flag to indicate whether a channel is in use or not */
	int in_use[BCM59035_MAX_PWM_CHANNELS];
	/* lock for above usage flags */
	struct mutex pwm_lock;
};

/* helper to initialize pwm_device_desc */
#define PWM_DESC_INIT(o, m, h, l, p) \
{                                    \
	.desc = {                    \
		.ops = o,            \
	},                           \
	.mode_reg = m,               \
	.high_period_reg = h,        \
	.low_period_reg = l,         \
	.pad_reg = p,                \
}

/****************************************************************************
 * private driver data
 ****************************************************************************/

static struct pwm_driver_data drv_data;

static struct pwm_channel_ops pwm_chan_ops;
static struct pwm_device_desc dev_desc[BCM59035_MAX_PWM_CHANNELS] = {
	PWM_DESC_INIT(&pwm_chan_ops, BCM59035_REG_PWMLEDCTRL1,
		BCM59035_REG_PWMLEDCTRL2, BCM59035_REG_PWMLEDCTRL4,
		BCM59035_REG_PWMLEDCTRL10),

	PWM_DESC_INIT(&pwm_chan_ops, BCM59035_REG_PWMLEDCTRL6,
		BCM59035_REG_PWMLEDCTRL7, BCM59035_REG_PWMLEDCTRL9,
		BCM59035_REG_PWMLEDCTRL10),
};

/****************************************************************************
 * internal api
 ****************************************************************************/

/*
 * if the number is not a multiple of 4, round it up to the nearest multiple
 * of 4
 */
int to_multiple_of_4(int n)
{
	int mod4 = n & (4 - 1);  /* mod = n % 4 */

	if (mod4)
		n = n + (4 - mod4);

	return n;
}

/*
 * table of input value ranges for different fsystem division factors:
 * fsystem = 2MHz
 *
 * the columns in this table are computed as follows:
 *
 * div - obtained from BCM59025 datasheet
 * fpwm_hz = 2 * 1000 * 1000 / div
 * tpwm_ns = 1000000000 / fpwm_hz
 * min_ns = 4 * tpwm_ns
 * max_ns = 252 * tpwm_ns
 *
 * -------------------------------------------------------
 * div fpwm_hz tpwm_ns min_ns           max_ns
 * -------------------------------------------------------
 * 4   500000  2000    8000(8us)        504000(504us)
 * 16  125000  8000    32000(32us)      2016000 (2.016ms)
 * 64  31250   32000   128000(128us)    8064000(8.064ms)
 * 512 3906    256016  1024064(1.024ms) 64516032(64.516ms)
 * -------------------------------------------------------
 *
 * the input values of duty_ns and period_ns are compared with the ranges
 * listed in the above table. if both duty cycle and period values fall
 * within one of these ranges, the corresponding division factor is
 * returned. -EINVAL is returned otherwise.
 */
static int get_div_factor(int d, int p)
{
	if (d < 0 || d > 64516032 || p < 0 || p > 64516032 || d > p)
		return -EINVAL;

	if (d >= 0 && d <= 504000 && p >= 0 && p <= 504000)
		return 4;
	if (d >= 32000 && d <= 2016000 && p >= 32000 && p <= 2016000)
		return 16;
	if (d >= 128000 && d <= 8064000 && p >= 128000 && p <= 8064000)
		return 64;
	if (d >= 1024064 && d <= 64516032 && p >= 1024064 && p <= 64516032)
		return 512;

	return -EINVAL;
}

static int get_chan_time_configs(int duty_ns, int period_ns, int *high_out,
	int *low_out, int *div_out)
{
	u32 fpwm, tpwm_ns;
	int high, low, period;
	int div;

	/*
	 * sanity check the args so that calculations further down
	 * will not fail. also calculate the correct fsystem division
	 * factor based on the input duty cycle and period.
	 */
	div = get_div_factor(duty_ns, period_ns);
	if (div < 0)
		return -EINVAL;

	/*
	 * calculate pwm clock period in ns. high and low periods are set to
	 * multiple of pwm clock period.
	 */
	fpwm = FSYSTEM_HZ / div;

	/* tpwm_ns = 1000000000 / fpwm; */
	tpwm_ns = 1000000000;
	do_div(tpwm_ns, fpwm);

	/* convert high_period = duty_ns to multiple of tpwm */
	high = duty_ns;
	do_div(high, tpwm_ns);

	/* convert period to multiple of tpwm */
	period = period_ns;
	do_div(period, tpwm_ns);

	/* compute low period */
	low = period - high;

	/* fit high and low to nearest multiple of four (since BCM59035 pwm
	 * allows configuring the high and low period in 0tpwm, 4tpwm, 8tpwm
	 * and so on).
	 */
	high = to_multiple_of_4(high);
	low = to_multiple_of_4(low);

	/* finally convert high and low to a value between 0 & 63. if the
	 * values do not fall in this range flag error.
	 */
	high = high / 4;
	low = low / 4;

	*high_out = high;
	*low_out = low;
	*div_out = div;

	return 0;
}

int validate_pattern(int pattern)
{
	int ret = -EINVAL;

	switch (pattern) {
	case BCM59035_LED_PATTERN_ON50_OFF_REST:
	case BCM59035_LED_PATTERN_ON100_OFF_REST:
	case BCM59035_LED_PATTERN_ON200_OFF_REST:
	case BCM59035_LED_PATTERN_ON500_OFF_REST:
	case BCM59035_LED_PATTERN_ON50_OFF50_ON50_OFF_REST:
	case BCM59035_LED_PATTERN_ON100_OFF100_ON100_OFF_REST:
	case BCM59035_LED_PATTERN_ON200_OFF200_ON200_OFF_REST:
	case BCM59035_LED_PATTERN_ON_ALWAYS:
		ret = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

int validate_repeat_period(int repeat_period)
{
	int ret = -EINVAL;

	switch (repeat_period) {
	case BCM59035_LED_REPEAT_PERIOD_0_4S:
	case BCM59035_LED_REPEAT_PERIOD_1_0S:
	case BCM59035_LED_REPEAT_PERIOD_1_2S:
	case BCM59035_LED_REPEAT_PERIOD_2_0S:
	case BCM59035_LED_REPEAT_PERIOD_2_6S:
	case BCM59035_LED_REPEAT_PERIOD_4_0S:
	case BCM59035_LED_REPEAT_PERIOD_6_0S:
	case BCM59035_LED_REPEAT_PERIOD_8_0S:
		ret = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

/****************************************************************************
 * pwm core interface routines
 ****************************************************************************/

static int bcm59035_pwm_config(struct pwm_channel *chan, int duty_ns,
	int period_ns)
{
	struct bcm59035 *bcm59035 = drv_data.bcm59035;
	int index = chan->desc->pwm_id - drv_data.start_id;
	int ret = 0;
	int high, low, div;
	u8 mode, ledctrl;

	dev_dbg(chan->dev, "%s\n", __func__);

	/*
	 * configure the pwm controller
	 */
	mode = dev_desc[index].chan_mode;
	if (mode & BCM59035_LEDON) {
		if ((validate_pattern(duty_ns) != 0) ||
			(validate_repeat_period(period_ns) != 0)) {
			dev_err(chan->dev, "%s: Invalid LED mode for pwm "
				"channel %d failed\n", __func__,
				chan->desc->pwm_id);
			return -EINVAL;
		}

		ledctrl = ((period_ns) | (duty_ns << 3));
		ret = bcm59035->write_dev(bcm59035,
			dev_desc[index].high_period_reg, ledctrl);

	} else if (mode & BCM59035_PWMON) {
		/*
		 * Convert duty cycle and period from ns to multiples of tpwm
		 */
		ret = get_chan_time_configs(duty_ns, period_ns, &high, &low,
			&div);
		if (ret != 0)
			return ret;

		/* configure the high and low periods */
		ret = bcm59035->write_dev(bcm59035,
			dev_desc[index].high_period_reg, high);
		ret |= bcm59035->write_dev(bcm59035,
			dev_desc[index].low_period_reg, low);

		/* configure the division factor */
		mode &= ~PWMLEDCTRL_PWMDIV_MASK;
		mode |= (div << PWMLEDCTRL_PWMDIV_SHIFT);
		ret |= bcm59035->write_dev(bcm59035, dev_desc[index].mode_reg,
			mode);
	}

	return ret;
}

static int bcm59035_pwm_setmode(struct pwm_channel *chan, int mode)
{
	struct bcm59035 *bcm59035 = drv_data.bcm59035;
	int index = chan->desc->pwm_id - drv_data.start_id;
	u8 current_mode;
	u8 pad;
	int ret = 0;

	dev_dbg(chan->dev, "%s\n", __func__);

	/* configure mode register */
	ret |= bcm59035->read_dev(bcm59035, dev_desc[index].mode_reg,
		&current_mode);
	current_mode &= ~BCM59035_LEDON_MASK;
	current_mode |= (mode & BCM59035_LEDON_MASK);
	ret |= bcm59035->write_dev(bcm59035, dev_desc[index].mode_reg,
		current_mode);

	/* save the mode */
	dev_desc[index].chan_mode = current_mode;

	/* configure pad register */
	ret |= bcm59035->read_dev(bcm59035, dev_desc[index].pad_reg, &pad);
	if (index == 0) {
		if (mode == BCM59035_LEDON)
			pad |= BCM59035_PAD_PWMLED1_BIT;
		else
			pad &= ~BCM59035_PAD_PWMLED1_BIT;
	} else {
		if (mode == BCM59035_LEDON)
			pad |= BCM59035_PAD_PWMLED2_BIT;
		else
			pad &= ~BCM59035_PAD_PWMLED2_BIT;
	}
	ret |= bcm59035->write_dev(bcm59035, dev_desc[index].pad_reg, pad);

	return ret;
}

static int bcm59035_pwm_request(struct pwm_channel *chan)
{
	dev_dbg(chan->dev, "%s\n", __func__);

	return 0;
}

static void bcm59035_pwm_free(struct pwm_channel *chan)
{
	dev_dbg(chan->dev, "%s\n", __func__);
}

static int bcm59035_pwm_enable(struct pwm_channel *chan)
{
	struct bcm59035 *bcm59035 = drv_data.bcm59035;
	int index = chan->desc->pwm_id - drv_data.start_id;
	int ret;
	u8 reg_val;

	dev_dbg(chan->dev, "%s\n", __func__);

	ret = bcm59035->write_dev(bcm59035, dev_desc[index].mode_reg,
		dev_desc[index].chan_mode);
	if (ret != 0)
		dev_err(chan->dev, "%s: enabling pwm channel %d failed\n",
			__func__, chan->desc->pwm_id);

	/* increment the pwm channels usage count */
	mutex_lock(&drv_data.pwm_lock);
	drv_data.in_use[index] = 1;
	mutex_unlock(&drv_data.pwm_lock);

	/* turn on the LED block */
	bcm59035->read_dev(bcm59035, BCM59035_REG_PWMLEDCTRL5, &reg_val);
	reg_val &= ~BCM59035_PWMLEDCTRL5_PWMLED_PDN;
	bcm59035->write_dev(bcm59035, BCM59035_REG_PWMLEDCTRL5, reg_val);

	return ret;
}

static void bcm59035_pwm_disable(struct pwm_channel *chan)
{
	struct bcm59035 *bcm59035 = drv_data.bcm59035;
	int index = chan->desc->pwm_id - drv_data.start_id;
	int ret;
	int pwm_inuse;
	u8 reg_val;

	dev_dbg(chan->dev, "%s\n", __func__);

	ret = bcm59035->write_dev(bcm59035, dev_desc[index].mode_reg,
		MODE_PWM_OFF_LED_OFF);
	if (ret != 0)
		dev_err(chan->dev, "%s: disabling pwm channel %d failed\n",
			__func__, chan->desc->pwm_id);

	/* test to see if all the pwm channels are disabled. if so disable
	 * the whole PWM LED block.
	 */
	mutex_lock(&drv_data.pwm_lock);
	drv_data.in_use[index] = 0;
	pwm_inuse = drv_data.in_use[0] | drv_data.in_use[1];
	mutex_unlock(&drv_data.pwm_lock);

	if (!pwm_inuse) {
		/* turn off the LED block. according to the datasheet,
		 * this saves about 220uA.
		 */
		bcm59035->read_dev(bcm59035, BCM59035_REG_PWMLEDCTRL5,
			&reg_val);
		reg_val |= BCM59035_PWMLEDCTRL5_PWMLED_PDN;
		bcm59035->write_dev(bcm59035, BCM59035_REG_PWMLEDCTRL5,
			reg_val);
	}
}

static struct pwm_channel_ops pwm_chan_ops = {
	.config = bcm59035_pwm_config,
	.request = bcm59035_pwm_request,
	.free = bcm59035_pwm_free,
	.enable = bcm59035_pwm_enable,
	.disable = bcm59035_pwm_disable,
	.setmode = bcm59035_pwm_setmode,
};

/****************************************************************************
 * power management routines
 ****************************************************************************/

static int bcm59035_pwm_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	pr_debug("%s\n", __func__);

	return 0;
}

static int bcm59035_pwm_resume(struct platform_device *pdev)
{
	pr_debug("%s\n", __func__);

	return 0;
}

/****************************************************************************
 * init routines
 ****************************************************************************/

/* this function turns off the PWM-LED block in the BCM59035 PMU */
static void bcm59035_pwm_module_disable(void)
{
	struct bcm59035 *bcm59035 = drv_data.bcm59035;
	u8 reg_val;

	/* turn off the LED block.according to the datasheet, this saves about
	 * 220uA.
	 */
	bcm59035->read_dev(bcm59035, BCM59035_REG_PWMLEDCTRL5, &reg_val);
	reg_val |= BCM59035_PWMLEDCTRL5_PWMLED_PDN;
	bcm59035->write_dev(bcm59035, BCM59035_REG_PWMLEDCTRL5, reg_val);
}

/*
 * initialize the pwm channel with the input configs
 */
static int bcm59035_pwm_channel_init(int index,
	struct bcm59035_pwm_channel_data *d)
{
	struct bcm59035 *bcm59035 = drv_data.bcm59035;
	int ret = 0;
	int high, low, div;

	/* if the channel is already ON (turned ON by bootloader or early
	 * init code), then don't reconfigure it
	 */
	if (d->init_state != MODE_PWM_OFF_LED_OFF)
		return ret;

	/*
	 * Convert duty cycle and period from ns to multiples of tpwm
	 */
	ret = get_chan_time_configs(d->duty_ns, d->period_ns, &high, &low,
		&div);
	if (ret != 0)
		return ret;

	/* configure pwm mode */
	d->mode &= ~PWMLEDCTRL_PWMDIV_MASK;
	d->mode |= (div << PWMLEDCTRL_PWMDIV_SHIFT);
	ret = bcm59035->write_dev(bcm59035, dev_desc[index].mode_reg, d->mode);

	/* save the current mode */
	dev_desc[index].chan_mode = d->mode;

	/* configure high period */
	ret |= bcm59035->write_dev(bcm59035, dev_desc[index].high_period_reg,
		high);

	/* configure low period */
	ret |= bcm59035->write_dev(bcm59035, dev_desc[index].low_period_reg,
		low);

	/* configure pad settings */
	ret |= bcm59035->write_dev(bcm59035, dev_desc[index].pad_reg,
		d->pad_config);

	return ret;
}

static int bcm59035_pwm_probe(struct platform_device *pdev)
{
	struct bcm59035 *bcm59035 = dev_get_drvdata(pdev->dev.parent);
	struct pwm_device_desc *desc;
	struct bcm59035_pwm_pdata *pdata;
	int i;
	int ret = 0;
	int module_disable;

	dev_info(&pdev->dev, "%s\n", __func__);

	/* get the pwm driver platform data */
	pdata = bcm59035->pdata->pwm;

	/* save the start id of the pwm channels and mfd driver handle
	 * for this driver. mfd driver handler is used to access the i2c
	 * read/write routines exported by the mfd core. start id is
	 * used to generate the correct device id to index into the
	 * device descriptor structure, private to this driver.
	 */
	drv_data.start_id = pdata->start_id;
	drv_data.bcm59035 = bcm59035;

	/* register all the pwm channels with the pwm core */
	for (i = 0; i < BCM59035_MAX_PWM_CHANNELS; i++) {
		struct pwm_channel *chan;

		/* get the channel descriptor */
		desc = &dev_desc[i];

		/* save the id in the descriptor */
		desc->desc.pwm_id = pdata->start_id + i;

		/* register the pwm channel with the core */
		chan = pwm_register(&desc->desc, &pdev->dev, desc);
		if (IS_ERR(chan)) {
			ret = PTR_ERR(chan);
			dev_err(&pdev->dev, "%s: pwm device %d registration"
				" failed\n", __func__, desc->desc.pwm_id);
			goto pwm_register_fail;
		}

		/* store the hook into the core layer */
		desc->chan = chan;
	}

	/*
	 * configure the pwm channels listed in the platform data
	 */
	module_disable = 1;
	for (i = 0; i < BCM59035_MAX_PWM_CHANNELS; i++) {
		int index = pdata->data[i].id - pdata->start_id;

		if (bcm59035_pwm_channel_init(index, &pdata->data[i]) != 0) {
			ret = -EIO;
			dev_err(&pdev->dev, "%s: pwm channel init for %d"
				" failed\n", __func__,
				dev_desc[i].desc.pwm_id);
			goto pwm_channel_init_fail;
		}

		if (pdata->data[i].init_state != MODE_PWM_OFF_LED_OFF)
			module_disable = 0;
	}

	if (module_disable == 1)
		bcm59035_pwm_module_disable();

	mutex_init(&drv_data.pwm_lock);
	return ret;

pwm_channel_init_fail:
	i = BCM59035_MAX_PWM_CHANNELS;

pwm_register_fail:
	while (--i >= 0) {
		desc = &dev_desc[i];
		pwm_unregister(desc->chan);
	}

	mutex_destroy(&drv_data.pwm_lock);
	return ret;
}

static int __devexit bcm59035_pwm_remove(struct platform_device *pdev)
{
	int i;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	for (i = 0; i < BCM59035_MAX_PWM_CHANNELS; i++) {
		if (pwm_unregister(dev_desc[i].chan) != 0) {
			dev_err(&pdev->dev, "%s: pwm device %d unregistration"
				" failed\n", __func__,
				dev_desc[i].desc.pwm_id);
			return -EBUSY;
		}

		dev_desc[i].chan = NULL;
		dev_desc[i].desc.pwm_id = 0;
	}

	drv_data.start_id = 0;
	drv_data.bcm59035 = NULL;

	mutex_destroy(&drv_data.pwm_lock);
	return 0;
}

static struct platform_driver bcm59035_pwm_driver = {
	.probe = bcm59035_pwm_probe,
	.remove = __devexit_p(bcm59035_pwm_remove),

#ifdef CONFIG_PM
	.suspend = bcm59035_pwm_suspend,
	.resume = bcm59035_pwm_resume,
#endif

#ifdef CONFIG_EARLY_SUSPEND
#endif

	.driver = {
		.name = "bcm59035-pwm",
	},
};

static int __init bcm59035_pwm_init(void)
{
	return platform_driver_register(&bcm59035_pwm_driver);
}
module_init(bcm59035_pwm_init);

static void __exit bcm59035_pwm_exit(void)
{
	platform_driver_unregister(&bcm59035_pwm_driver);
}
module_exit(bcm59035_pwm_exit);

MODULE_ALIAS("platform:bcm59035-pwm");
MODULE_DESCRIPTION("BCM59035 PWM");
MODULE_LICENSE("GPL");
