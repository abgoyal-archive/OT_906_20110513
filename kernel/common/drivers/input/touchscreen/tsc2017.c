/*
 * drivers/input/touchscreen/tsc2017.c
 *
 * Copyright (c) 2008 MtekVision Co., Ltd.
 *	Kwangwoo Lee <kwlee@mtekvision.com>
 *
 * Using code from:
 *  - ads7846.c
 *	Copyright (c) 2005 David Brownell
 *	Copyright (c) 2006 Nokia Corporation
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
 *	Copyright (C) 2004 Texas Instruments
 *	Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

/*******************************************************************************
*Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*@file	drivers/input/touchscreen/tsc2017.c
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
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c/tsc2017.h>
#include <linux/earlysuspend.h>

#include <linux/time.h>

#include <linux/gpio.h>
#include <linux/delay.h>

#define PRINTKEY_SUPPORT


#define TS_POLL_DELAY			1 /* ms delay between samples */
#define TS_POLL_PERIOD			20 /* ms delay between samples */

#define TSC2017_MEASURE_TEMP0		(0x0 << 4)
#define TSC2017_MEASURE_AUX		(0x2 << 4)
#define TSC2017_MEASURE_TEMP1		(0x4 << 4)
#define TSC2017_ACTIVATE_XN		(0x8 << 4)
#define TSC2017_ACTIVATE_YN		(0x9 << 4)
#define TSC2017_ACTIVATE_YP_XN		(0xa << 4)
#define TSC2017_SETUP			(0xb << 4)
#define TSC2017_MEASURE_X		(0xc << 4)
#define TSC2017_MEASURE_Y		(0xd << 4)
#define TSC2017_MEASURE_Z1		(0xe << 4)
#define TSC2017_MEASURE_Z2		(0xf << 4)

#define TSC2017_POWER_OFF_IRQ_EN	(0x0 << 2)
#define TSC2017_ADC_ON_IRQ_DIS0		(0x1 << 2)
#define TSC2017_ADC_OFF_IRQ_EN		(0x2 << 2)
#define TSC2017_ADC_ON_IRQ_DIS1		(0x3 << 2)

#define TSC2017_12BIT			(0x0 << 1)
#define TSC2017_8BIT			(0x1 << 1)

#define	MAX_12BIT			((1 << 12) - 1)

#define ADC_ON_12BIT	(TSC2017_12BIT | TSC2017_ADC_ON_IRQ_DIS0)

#define READ_Y		(ADC_ON_12BIT | TSC2017_MEASURE_Y)
#define READ_Z1		(ADC_ON_12BIT | TSC2017_MEASURE_Z1)
#define READ_Z2		(ADC_ON_12BIT | TSC2017_MEASURE_Z2)
#define READ_X		(ADC_ON_12BIT | TSC2017_MEASURE_X)
#define PWRDOWN		(TSC2017_12BIT | TSC2017_POWER_OFF_IRQ_EN)

#ifdef PRINTKEY_SUPPORT
#define KEY_MENUS    229

struct ts_printkey_data {
	unsigned int code;
	u16	left;
	u16  right;
	u16  up;
	u16  down;
};

#endif

struct ts_event {
	u16	x;
	u16	y;
	u16	z1, z2;
};

struct tsc2017 {
	struct tsc2017_platform_data *pdata;
	struct input_dev	*input;
	char			phys[32];
	struct delayed_work	work;

	struct i2c_client	*client;
	struct early_suspend	early_suspend;

	u16			model;
	u16			x_plate_ohms;

	bool			pendown;
	int			irq;

	int			(*get_pendown_state)(void);
	void			(*clear_penirq)(void);

#ifdef PRINTKEY_SUPPORT
	struct ts_printkey_data *pk_data;
	u16                pk_num;
#endif
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tsc2017_early_suspend(struct early_suspend *h);
static void tsc2017_late_resume(struct early_suspend *h);
#endif

/*#undef BCM_TS_EVENT_REGULATION*/
#define BCM_TS_EVENT_REGULATION

#ifdef BCM_TS_EVENT_REGULATION
/* regulate event report rate */
#define TS_EVENT_RATE	12/* maximum number of touch screen event per second */
#define TS_EVENT_REGULATE_WINDOW (HZ / TS_EVENT_RATE)	/* minimum jiffies between two reported touch screen events */
uint32_t ts_last_time;
#endif  /* BCM_TS_EVENT_REGULATION */

static inline int tsc2017_xfer(struct tsc2017 *tsc, u8 cmd)
{
	s32 data;
	u16 val;

	data = i2c_smbus_read_word_data(tsc->client, cmd);
	if (data < 0) {
		dev_err(&tsc->client->dev, "i2c io error: %d\n", data);
		return data;
	}

	/* The protocol and raw data format from i2c interface:
	 * S Addr Wr [A] Comm [A] S Addr Rd [A] [DataLow] A [DataHigh] NA P
	 * Where DataLow has [D11-D4], DataHigh has [D3-D0 << 4 | Dummy 4bit].
	 */
	val = swab16(data) >> 4;

	dev_dbg(&tsc->client->dev, "data: 0x%x, val: 0x%x\n", data, val);

	return val;
}

static void tsc2017_read_values(struct tsc2017 *tsc, struct ts_event *tc)
{
	/* y- still on; turn on only y+ (and ADC) */
	tc->y = tsc2017_xfer(tsc, READ_Y);

	/* turn y- off, x+ on, then leave in lowpower */
	tc->x = tsc2017_xfer(tsc, READ_X);
	/*printk("[TCL]:tsc2017_read_values  (x,y)=(%d,%d)   \r\n",tc->x,tc->y);*/
	/* turn y+ off, x- on; we'll use formula #1 */
	tc->z1 = tsc2017_xfer(tsc, READ_Z1);
	tc->z2 = tsc2017_xfer(tsc, READ_Z2);

	/* Prepare for next touch reading - power down ADC, enable PENIRQ */
	tsc2017_xfer(tsc, PWRDOWN);
}

static u32 tsc2017_calculate_pressure(struct tsc2017 *tsc, struct ts_event *tc)
{
	u32 rt = 0;

	/* range filtering */
	if (tc->x == MAX_12BIT)
		tc->x = 0;

	if (likely(tc->x && tc->z1)) {
		/* compute touch pressure resistance using equation #1 */
		rt = tc->z2 - tc->z1;
		rt *= tc->x;
		rt *= tsc->x_plate_ohms;
		rt /= tc->z1;
		rt = (rt + 2047) >> 12;
	}

	return rt;
}

static void tsc2017_send_up_event(struct tsc2017 *tsc)
{
	struct input_dev *input = tsc->input;

	dev_dbg(&tsc->client->dev, "UP\n");

	input_report_key(input, BTN_TOUCH, 0);
	input_report_abs(input, ABS_PRESSURE, 0);
	input_sync(input);
}
#define to_delayed_work(_work) container_of(_work, struct delayed_work, work)

#define REPORT_X	(ts->pdata->get_x_value(ts->pdata, tc.x, MAX_12BIT))
#define REPORT_Y	(ts->pdata->get_y_value(ts->pdata, tc.y, MAX_12BIT))

static void tsc2017_send_key_event(struct input_dev *input, unsigned int code)
{
	input_report_key(input, code, 1);
	input_sync(input);

	input_report_key(input, code, 0);
	input_sync(input);

}

static void tsc2017_work(struct work_struct *work)
{
	struct tsc2017 *ts =
		container_of(to_delayed_work(work), struct tsc2017, work);
	struct ts_event tc;
	u32 rt;
#ifdef BCM_TS_EVENT_REGULATION
	uint32_t ts_this_time;
#endif
	/*
	 * NOTE: We can't rely on the pressure to determine the pen down
	 * state, even though this controller has a pressure sensor.
	 * The pressure value can fluctuate for quite a while after
	 * lifting the pen and in some cases may not even settle at the
	 * expected value.
	 *
	 * The only safe way to check for the pen up condition is in the
	 * work function by reading the pen signal state (it's a GPIO
	 * and IRQ). Unfortunately such callback is not always available,
	 * in that case we have rely on the pressure anyway.
	 */
	if (ts->get_pendown_state) {
		if (unlikely(!ts->get_pendown_state())) {
			tsc2017_send_up_event(ts);
			ts->pendown = false;
			goto out;
		}

		dev_dbg(&ts->client->dev, "pen is still down\n");
	}

	tsc2017_read_values(ts, &tc);

	rt = tsc2017_calculate_pressure(ts, &tc);
	if (rt > MAX_12BIT) {
		/*
		 * Sample found inconsistent by debouncing or pressure is
		 * beyond the maximum. Don't report it to user space,
		 * repeat at least once more the measurement.
		 */
		dev_dbg(&ts->client->dev, "ignored pressure %d\n", rt);
		goto out;

	}

	if (rt) {
		/*static int ts_counter;*/
		struct input_dev *input = ts->input;

#ifdef	PRINTKEY_SUPPORT
		u16 usCount;
		/* Check to see if this point is belong to print key area*/
		if (ts->pdata->validy_up > tc.y) {
			if (!ts->pendown) {
				/*start to check which print key been pressed*/
				for (usCount = 0; usCount < ts->pk_num; usCount++) {
					if ((tc.y > (ts->pk_data+usCount)->up) && \
					    (tc.y < (ts->pk_data+usCount)->down) && \
					    (tc.x > (ts->pk_data+usCount)->left) && \
					    (tc.x < (ts->pk_data+usCount)->right)) {
						tsc2017_send_key_event(input, (ts->pk_data+usCount)->code);
						/*printk(" Send Printkey %d  \r\n", (ts->pk_data+usCount)->code);*/
						ts->pendown = true;
						break;
					}
				}
			}
		} else {
#endif

			if (!ts->pendown) {
				dev_dbg(&ts->client->dev, "DOWN\n");

				input_report_key(input, BTN_TOUCH, 1);
				ts->pendown = true;


			}

#ifdef BCM_TS_EVENT_REGULATION
		ts_this_time = jiffies;
		if (ts_last_time == 0)
				ts_last_time = ts_this_time - TS_EVENT_REGULATE_WINDOW;

		if (ts_this_time - ts_last_time >= TS_EVENT_REGULATE_WINDOW) {
			ts_last_time = ts_this_time;

			input_report_abs (input, ABS_X, REPORT_X);
			input_report_abs (input, ABS_Y, REPORT_Y);
			input_report_abs(input, ABS_PRESSURE, rt);
			input_sync(input);

			dev_dbg(&ts->client->dev, "point(%4d,%4d), pressure (%4u)\n",
				REPORT_X, REPORT_Y, rt);

			/*printk("ts#=%d ts_this_time=%u\n", ts_counter++, ts_this_time);*/
		}
#else /* BCM_TS_EVENT_REGULATION*/
		input_report_abs (input, ABS_X, REPORT_X);
		input_report_abs (input, ABS_Y, REPORT_Y);
		input_report_abs(input, ABS_PRESSURE, rt);
		input_sync(input);

		dev_dbg(&ts->client->dev, "point(%4d,%4d), pressure (%4u)\n",
			REPORT_X, REPORT_Y, rt);
#endif
#ifdef PRINTKEY_SUPPORT
		}
#endif
	} else if (!ts->get_pendown_state && ts->pendown) {
		/*
		 * We don't have callback to check pendown state, so we
		 * have to assume that since pressure reported is 0 the
		 * pen was lifted up.
		 */
		tsc2017_send_up_event(ts);
		ts->pendown = false;
	}

 out:
	if (ts->pendown)
		schedule_delayed_work(&ts->work,
				      msecs_to_jiffies(TS_POLL_PERIOD));
	else
		enable_irq(ts->irq);  
}

static irqreturn_t tsc2017_irq(int irq, void *handle)
{
	struct tsc2017 *ts = handle;

	if (!ts->get_pendown_state || likely(ts->get_pendown_state())) {
		disable_irq_nosync(ts->irq);
		schedule_delayed_work(&ts->work,
				      msecs_to_jiffies(TS_POLL_DELAY));
	}

	if (ts->clear_penirq)
		ts->clear_penirq();

	return IRQ_HANDLED;
}

static void tsc2017_free_irq(struct tsc2017 *ts)
{
	free_irq(ts->irq, ts);
	if (cancel_delayed_work_sync(&ts->work)) {
		/*
		 * Work was pending, therefore we need to enable
		 * IRQ here to balance the disable_irq() done in the
		 * interrupt handler.
		 */
		enable_irq(ts->irq);
	}
}

static int tsc2017_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct tsc2017 *ts;

	ts = i2c_get_clientdata(client);
	cancel_work_sync(&ts->work.work);

	tsc2017_xfer(ts, TSC2017_ADC_ON_IRQ_DIS0);
	if(ts->pendown)
		ts->pendown =false;
	else	
	disable_irq(ts->irq);
	return 0;

}

static int tsc2017_resume(struct i2c_client *client)
{
	struct tsc2017 *ts;

	ts = i2c_get_clientdata(client);
       ts->pdata->reset();
	enable_irq(ts->irq); 
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tsc2017_early_suspend(struct early_suspend *h)
{
				struct tsc2017 *ts;
				ts = container_of(h, struct tsc2017, early_suspend);
				tsc2017_suspend(ts->client, PMSG_SUSPEND);
}

static void tsc2017_late_resume(struct early_suspend *h)
{
				struct tsc2017 *ts;
				ts = container_of(h, struct tsc2017, early_suspend);
				tsc2017_resume(ts->client);
}
#endif

static int __devinit tsc2017_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct tsc2017 *ts;
	struct tsc2017_platform_data *pdata = pdata = client->dev.platform_data;
	struct input_dev *input_dev;

#ifdef PRINTKEY_SUPPORT

	int iCount;
																					/* code ,left ,right ,up ,down*/
	struct ts_printkey_data pk_data_tlb[] = {\
																					{KEY_MENUS, 580, 840, 220, 400},\
																					{KEY_SEARCH, 1900, 2300, 220, 400},\
																					{KEY_BACK, 3200, 3699, 220, 400} };
#endif
	int err;

	printk("tsc2017_probe \n");
	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	ts = kzalloc(sizeof(struct tsc2017), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->pdata = pdata;
	ts->client = client;
	ts->irq = client->irq;
	ts->input = input_dev;
	INIT_DELAYED_WORK(&ts->work, tsc2017_work);

	ts->model             = pdata->model;
	ts->x_plate_ohms      = pdata->x_plate_ohms;
	ts->get_pendown_state = pdata->get_pendown_state;
	ts->clear_penirq      = pdata->clear_penirq;

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&client->dev));

	input_dev->name = "TSC2017 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

#ifdef PRINTKEY_SUPPORT
	ts->pk_data = kmalloc(sizeof(pk_data_tlb), GFP_KERNEL);
	if (!ts->pk_data) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	memcpy(ts->pk_data, pk_data_tlb, sizeof(pk_data_tlb));

	ts->pk_num = sizeof(pk_data_tlb)/sizeof(struct ts_printkey_data);

	for (iCount = 0; iCount < ts->pk_num; iCount++)
		input_dev->keybit[BIT_WORD(ts->pk_data[iCount].code)] = BIT_MASK(ts->pk_data[iCount].code);
#endif

	input_set_abs_params(input_dev, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_12BIT, 0, 0);

	if (pdata->init_platform_hw)
		pdata->init_platform_hw();

	err = request_irq(ts->irq, tsc2017_irq, 0,
			client->dev.driver->name, ts);
	if (err < 0) {
		dev_err(&client->dev, "irq %d busy?\n", ts->irq);
		goto err_free_mem;
	}

	/* Prepare for touch readings - power down ADC and enable PENIRQ */
	err = tsc2017_xfer(ts, PWRDOWN);
	if (err < 0)
		goto err_free_irq;

	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	i2c_set_clientdata(client, ts);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = tsc2017_early_suspend;
	ts->early_suspend.resume = tsc2017_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	return 0;

 err_free_irq:
	tsc2017_free_irq(ts);
	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();
 err_free_mem:
	input_free_device(input_dev);
#ifdef PRINTKEY_SUPPORT
	if (ts->pk_data) {
		kfree(ts->pk_data);
	}
#endif

	kfree(ts);
	return err;
}

static int __devexit tsc2017_remove(struct i2c_client *client)
{
	struct tsc2017	*ts = i2c_get_clientdata(client);
	struct tsc2017_platform_data *pdata = client->dev.platform_data;

	tsc2017_free_irq(ts);

	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();

	input_unregister_device(ts->input);

#ifdef PRINTKEY_SUPPORT
	kfree(ts->pk_data);
#endif

	kfree(ts);

	return 0;
}

static struct i2c_device_id tsc2017_idtable[] = {
	{ "tsc2017", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tsc2017_idtable);

static struct i2c_driver tsc2017_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "tsc2017"
	},
	.id_table = tsc2017_idtable,
	.probe = tsc2017_probe,
	.remove = __devexit_p(tsc2017_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = tsc2017_suspend,
	.resume = tsc2017_resume,
#endif

};

static int __init tsc2017_init(void)
{
	return i2c_add_driver(&tsc2017_driver);
}

static void __exit tsc2017_exit(void)
{
	i2c_del_driver(&tsc2017_driver);
}

module_init(tsc2017_init);
module_exit(tsc2017_exit);

MODULE_AUTHOR("Kwangwoo Lee <kwlee@mtekvision.com>");
MODULE_DESCRIPTION("TSC2017 TouchScreen Driver");
MODULE_LICENSE("GPL");
