/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/rtc/rtc-bcm59035.c
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
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>
#include <linux/mfd/bcm59035/bcm59035.h>
#include <asm/io.h>

struct bcm59035_rtc {
	struct bcm59035 *bcm59035;
	struct rtc_device *rtc;
	int alarm_enabled;
};

/*
 * Read current time and date in RTC
 */
static int bcm59035_rtc_readtime(struct device *dev, struct rtc_time *tm)
{
	struct bcm59035_rtc *bcm59035_rtc = dev_get_drvdata(dev);
	struct bcm59035 *bcm59035 = bcm59035_rtc->bcm59035;
	u8 regVal;
	int ret = 0;

	ret = bcm59035->read_dev(bcm59035, BCM59035_REG_RTCYR, &regVal);
	tm->tm_year = regVal + 100;

	ret |= bcm59035->read_dev(bcm59035, BCM59035_REG_RTCMT, &regVal);
	tm->tm_mon = regVal;

	ret |= bcm59035->read_dev(bcm59035, BCM59035_REG_RTCDT, &regVal);
	tm->tm_mday = regVal;

	ret |= bcm59035->read_dev(bcm59035, BCM59035_REG_RTCHR, &regVal);
	tm->tm_hour = regVal;

	ret |= bcm59035->read_dev(bcm59035, BCM59035_REG_RTCMN, &regVal);
	tm->tm_min = regVal;

	ret |= bcm59035->read_dev(bcm59035, BCM59035_REG_RTCSC, &regVal);
	tm->tm_sec = regVal;

	pr_debug("%s: year = %d\n", __func__, tm->tm_year);
	pr_debug("%s: mon  = %d\n", __func__, tm->tm_mon);
	pr_debug("%s: mday = %d\n", __func__, tm->tm_mday);
	pr_debug("%s: wday = %d\n", __func__, tm->tm_wday);
	pr_debug("%s: hour = %d\n", __func__, tm->tm_hour);
	pr_debug("%s: min  = %d\n", __func__, tm->tm_min);
	pr_debug("%s: sec  = %d\n", __func__, tm->tm_sec);

	return ret;
}

/*
 * Set current time and date in RTC
 */
static int bcm59035_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct bcm59035_rtc *bcm59035_rtc = dev_get_drvdata(dev);
	struct bcm59035 *bcm59035 = bcm59035_rtc->bcm59035;
	int ret = 0;

	pr_debug("%s: year = %d\n", __func__, tm->tm_year);
	pr_debug("%s: mon  = %d\n", __func__, tm->tm_mon);
	pr_debug("%s: mday = %d\n", __func__, tm->tm_mday);
	pr_debug("%s: wday = %d\n", __func__, tm->tm_wday);
	pr_debug("%s: hour = %d\n", __func__, tm->tm_hour);
	pr_debug("%s: min  = %d\n", __func__, tm->tm_min);
	pr_debug("%s: sec  = %d\n", __func__, tm->tm_sec);

	ret |= bcm59035->write_dev(bcm59035, BCM59035_REG_RTCYR,
				(tm->tm_year - 100));
	/* pmu expects 1.. numbering here */
	ret |= bcm59035->write_dev(bcm59035, BCM59035_REG_RTCMT, tm->tm_mon);
	ret |= bcm59035->write_dev(bcm59035, BCM59035_REG_RTCDT, tm->tm_mday);
	ret |= bcm59035->write_dev(bcm59035, BCM59035_REG_RTCHR, tm->tm_hour);
	ret |= bcm59035->write_dev(bcm59035, BCM59035_REG_RTCMN, tm->tm_min);
	ret |= bcm59035->write_dev(bcm59035, BCM59035_REG_RTCSC, tm->tm_sec);

	return ret;
}

static int bcm59035_rtc_alarm_irq_enable(struct device *dev, unsigned enabled)
{
	int ret = 0;
	struct bcm59035_rtc *bcm59035_rtc = dev_get_drvdata(dev);
	struct bcm59035 *bcm59035 = bcm59035_rtc->bcm59035;

	pr_debug("%s: enabled: %d\n", __func__, enabled);

	if (enabled)
		ret = bcm59035_enable_irq(bcm59035, BCM59035_IRQID_INT1_RTCA1);
	else
		ret = bcm59035_disable_irq(bcm59035, BCM59035_IRQID_INT1_RTCA1);

	if (ret < 0)
		pr_err("%s: irq enable/disable failed: %d\n", __func__, ret);

	return ret;
}

static int bcm59035_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct bcm59035_rtc *bcm59035_rtc = dev_get_drvdata(dev);
	struct bcm59035 *bcm59035 = bcm59035_rtc->bcm59035;
	u8 regVal;
	int ret = 0;

	ret |= bcm59035->read_dev(bcm59035, BCM59035_REG_RTCYR_A1, &regVal);
	alm->time.tm_year = regVal + 100;

	ret |= bcm59035->read_dev(bcm59035, BCM59035_REG_RTCMT_A1, &regVal);
	alm->time.tm_mon = regVal;

	ret |= bcm59035->read_dev(bcm59035, BCM59035_REG_RTCDT_A1, &regVal);
	alm->time.tm_mday = regVal;

	ret |= bcm59035->read_dev(bcm59035, BCM59035_REG_RTCHR_A1, &regVal);
	alm->time.tm_hour = regVal;

	ret |= bcm59035->read_dev(bcm59035, BCM59035_REG_RTCMN_A1, &regVal);
	alm->time.tm_min = regVal;

	ret |= bcm59035->read_dev(bcm59035, BCM59035_REG_RTCSC_A1, &regVal);
	alm->time.tm_sec = regVal;

	if (ret) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"bcm59035_rtc_read_alarm: PMU read error !!!\n");
	}

	if (bcm59035_rtc->alarm_enabled)
		alm->enabled = 1;
	else
		alm->enabled = 0;

	pr_debug("%s: alm->year    = %d\n", __func__, alm->time.tm_year);
	pr_debug("%s: alm->mon     = %d\n", __func__, alm->time.tm_mon);
	pr_debug("%s: alm->mday    = %d\n", __func__, alm->time.tm_mday);
	pr_debug("%s: alm->wday    = %d\n", __func__, alm->time.tm_wday);
	pr_debug("%s: alm->hour    = %d\n", __func__, alm->time.tm_hour);
	pr_debug("%s: alm->min     = %d\n", __func__, alm->time.tm_min);
	pr_debug("%s: alm->sec     = %d\n", __func__, alm->time.tm_sec);
	pr_debug("%s: alm->enabled = %d\n", __func__, alm->enabled);
	pr_debug("%s: alm->pending = %d\n", __func__, alm->pending);

	return ret;
}

static int bcm59035_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct bcm59035_rtc *bcm59035_rtc = dev_get_drvdata(dev);
	struct bcm59035 *bcm59035 = bcm59035_rtc->bcm59035;
	int ret = 0;

	if (alm->enabled) {
		pr_debug("%s: alm->year     = %d\n", __func__,
			alm->time.tm_year);
		pr_debug("%s: alm->mon      = %d\n", __func__,
			alm->time.tm_mon);
		pr_debug("%s: alm->mday     = %d\n", __func__,
			alm->time.tm_mday);
		pr_debug("%s: alm->wday     = %d\n", __func__,
			alm->time.tm_wday);
		pr_debug("%s: alm->hour     = %d\n", __func__,
			alm->time.tm_hour);
		pr_debug("%s: alm->min      = %d\n", __func__,
			alm->time.tm_min);
		pr_debug("%s: alm->sec      = %d\n", __func__,
			alm->time.tm_sec);
		pr_debug("%s: alm->enabled  = %d\n", __func__,
			alm->enabled);
		pr_debug("%s: alm->pending  = %d\n", __func__,
			alm->pending);

		ret |= bcm59035->write_dev(bcm59035, BCM59035_REG_RTCYR_A1,
					(alm->time.tm_year - 100));
		/* pmu expects 1.. numbering here */
		ret |= bcm59035->write_dev(bcm59035, BCM59035_REG_RTCMT_A1,
					alm->time.tm_mon);
		ret |= bcm59035->write_dev(bcm59035, BCM59035_REG_RTCDT_A1,
					alm->time.tm_mday);
		ret |= bcm59035->write_dev(bcm59035, BCM59035_REG_RTCHR_A1,
					alm->time.tm_hour);
		ret |= bcm59035->write_dev(bcm59035, BCM59035_REG_RTCMN_A1,
					alm->time.tm_min);
		ret |= bcm59035->write_dev(bcm59035, BCM59035_REG_RTCSC_A1,
					alm->time.tm_sec);
	}

	bcm59035_rtc_alarm_irq_enable(dev, alm->enabled);

	return ret;
}

static void bcm59035_rtc_isr(int irq, void *data)
{
	unsigned long events = 0;
	struct bcm59035_rtc *bcm59035_rtc = data;
	PMU_LOG(DEBUG_PMU_INFO, "----bcm59035_rtc_isr----\n");
	events |= RTC_IRQF | RTC_AF;
	rtc_update_irq(bcm59035_rtc->rtc, 1, events);
}

static const struct rtc_class_ops bcm59035_rtc_ops = {
	.read_time = bcm59035_rtc_readtime,
	.set_time = bcm59035_rtc_set_time,
	.read_alarm = bcm59035_rtc_read_alarm,
	.set_alarm = bcm59035_rtc_set_alarm,
	.alarm_irq_enable = bcm59035_rtc_alarm_irq_enable,
	.update_irq_enable = bcm59035_rtc_alarm_irq_enable,
};

#ifdef CONFIG_PM
static int bcm59035_rtc_suspend(struct device *dev)
{
	return 0;
}

static int bcm59035_rtc_resume(struct device *dev)
{
#if defined(RTC_ANDROID_ALARM_WORKAROUND)
	/* This option selects temporary fix for alarm handling in 'Android'
	 * environment. This option enables code to disable alarm in the
	 * 'resume' handler of RTC driver. In the normal mode,
	 * android handles all alarms in software without using the RTC chip.
	 * Android sets the alarm in the rtc only in the suspend path (by
	 * calling .set_alarm with struct rtc_wkalrm->enabled set to 1).
	 * In the resume path, android tries to disable alarm by calling
	 * .set_alarm with struct rtc_wkalrm->enabled' field set to 0.
	 * But unfortunately, it memsets the rtc_wkalrm struct to 0, which
	 * causes the rtc lib to flag error and control does not reach this
	 * driver. Hence this workaround.
	 */
	bcm59035_rtc_alarm_irq_enable(dev, 0);
#endif

	return 0;
}

#else
#define bcm59035_rtc_suspend NULL
#define bcm59035_rtc_resume NULL
#endif

static int bcm59035_rtc_probe(struct platform_device *pdev)
{
	struct bcm59035 *bcm59035 = dev_get_drvdata(pdev->dev.parent);
	struct bcm59035_rtc *bcm59035_rtc;
	int ret = 0;

	PMU_LOG(DEBUG_PMU_INFO, "Inside %s\n", __FUNCTION__);

	bcm59035_rtc = kzalloc(sizeof(struct bcm59035_rtc), GFP_KERNEL);
	if (bcm59035_rtc == NULL) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"bcm59035_rtc_probe failed!! ..no memory!!!\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, bcm59035_rtc);
	bcm59035_rtc->bcm59035 = bcm59035;

	device_init_wakeup(&pdev->dev, 1);

	ret =
	    bcm59035_request_irq(bcm59035, BCM59035_IRQID_INT1_RTCA1, false,
				 bcm59035_rtc_isr, bcm59035_rtc);
	if (ret < 0) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"bcm59035_rtc_probe IRQ register failed !!!\n");
		kfree(bcm59035_rtc);
		return ret;
	}
	bcm59035_rtc->rtc = rtc_device_register(pdev->name, &pdev->dev,
						&bcm59035_rtc_ops, THIS_MODULE);
	if (IS_ERR(bcm59035_rtc->rtc)) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"bcm59035_rtc_probe:rtc_device_register failed !!!\n");
		kfree(bcm59035_rtc);
		return PTR_ERR(bcm59035_rtc->rtc);

	}
	return 0;

}

static int __devexit bcm59035_rtc_remove(struct platform_device *pdev)
{
	struct bcm59035_rtc *bcm59035_rtc = platform_get_drvdata(pdev);

	bcm59035_free_irq(bcm59035_rtc->bcm59035, BCM59035_IRQID_INT1_RTCA1);
	rtc_device_unregister(bcm59035_rtc->rtc);
	kfree(bcm59035_rtc);

	return 0;
}

static struct dev_pm_ops bcm59035_rtc_pm_ops = {
	.suspend = bcm59035_rtc_suspend,
	.resume = bcm59035_rtc_resume,

	.thaw = bcm59035_rtc_resume,
	.restore = bcm59035_rtc_resume,

	.poweroff = bcm59035_rtc_suspend,
};

static struct platform_driver bcm59035_rtc_driver = {
	.probe = bcm59035_rtc_probe,
	.remove = __devexit_p(bcm59035_rtc_remove),
	.driver = {
		   .name = "bcm59035-rtc",
		   .pm = &bcm59035_rtc_pm_ops,
		   },
};

static int __init bcm59035_rtc_init(void)
{
	return platform_driver_register(&bcm59035_rtc_driver);
}

module_init(bcm59035_rtc_init);

static void __exit bcm59035_rtc_exit(void)
{
	platform_driver_unregister(&bcm59035_rtc_driver);
}

module_exit(bcm59035_rtc_exit);

MODULE_DESCRIPTION("RTC driver for the Broadcom BCM59035 PMU");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bcm59035-rtc");

