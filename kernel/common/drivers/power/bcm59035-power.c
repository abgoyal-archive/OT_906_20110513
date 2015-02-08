/*******************************************************************************
* Copyright 2010 Broadcom Corporation. All rights reserved.
*
*	@file	drivers/power/bcm59035-power.c
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
 * @file bcm59035-power.c
 *
 * @brief Power Driver for Broadcom BCM59035 PMU
 *
 ****************************************************************************/
/**
*   @defgroup   BCM59035PowerAPIGroup   Power Driver  API's
*   @brief      This group defines the Power Driver for BCM59035 PMU API
*
*****************************************************************************/
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/mfd/bcm59035/bcm59035.h>
#include <linux/broadcom/types.h>
#include <linux/broadcom/bcm_kril_Interface.h>
#include <linux/broadcom/bcm_fuse_sysparm.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#if defined(CONFIG_HAS_WAKELOCK)
#include <linux/wakelock.h>
#endif /*CONFIG_HAS_WAKELOCK*/
#include <mach/auxadc.h>
#include <plat/bcm_auxadc.h>
#include <mach/irqs.h>
#include <linux/stringify.h>
#include <linux/ktime.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif /*CONFIG_HAS_EARLYSUSPEND*/

#define SUCCESS 0		/* request is successfull */
#define BCM59035_LOG_CHARGING_TIME

#define USB_PRE_ENUM_CUR		BCM59035_CHARGING_CUR_95MA
#define ADC_RUNNING_AVG_SHIFT 3	/* # of shifts used to divide the sum to get the average. */
#define ADC_RUNNING_AVG_SIZE (1 << ADC_RUNNING_AVG_SHIFT)	/* # of samples to perform voltage running sum */

/*Macros to control schedule frequency of charging monitor work queue - with
 * and without charger present */
#define BATTERY_LVL_MON_INTERVAL_WHILE_CHARGING		60000 /* One minute */
#define	BATTERY_LVL_MON_INTERVAL			300000 /* Five minutes */
#define BAT_TEMP_EXCEED_LIMIT_COUNT_MAX			3
#define BATTERY_CHARGING_HYSTERESIS 7

#define ADC_MAX_RETRY 3
#define ADC_ERROR_VALUE 1023

static unsigned long charging_time_sec;
module_param(charging_time_sec, ulong, S_IRUGO);
MODULE_PARM_DESC(charging_time_sec, "Time from start of charge to end of \
charge: sec:nsec");

static unsigned long charging_time_nsec;
module_param(charging_time_nsec, ulong, S_IRUGO);
MODULE_PARM_DESC(charging_time_nsec, "Time from start of charge to end of \
charge: sec:nsec");

struct bcm59035_power {
	struct bcm59035 *bcm59035; /**< Pointer to private data of PMU core driver*/
	struct power_supply wall; /**< linux power supply structure for WAC*/
	struct power_supply usb; /**< linux power supply structure for USB*/
	struct power_supply battery; /**< linux power supply structure for
	Battery*/

	enum power_supply_type power_src; /**< Type of power supply*/
	int charging_status; /**< used to store current status of charging*/

	u8 batt_percentage; /**< used to store battery level in percentage*/
	u8 batt_health; /**< Battery Health status*/
	u16 batt_adc_avg; /**< Averaged ADC value of battery level*/
	u32 batt_voltage; /**< current battery voltage */

	int temp_running_sum; /**< used to calculate avegared temperature*/
	int tem_read_inx; /**< used to calculate avegared temperature**/
	int temp_reading[ADC_RUNNING_AVG_SIZE]; /**< used to calculate
	avegared temperature**/
	int batt_temp_adc_avg; /**< used to store avegared temperature adc
	value**/
	u32 temp_exceed_limit_count; /**< number of times temperature has
	exceeded the maximum limit*/

	struct delayed_work batt_lvl_mon_wq; /**< work queue to monitor
	charging*/
#if defined(CONFIG_HAS_WAKELOCK)
	struct wake_lock usb_charger_wl; /**< wakelock while USB charger is
	active*/
	struct wake_lock temp_adc_wl; /**< wakelock while adc access is in
	progress*/
#endif /*CONFIG_HAS_WAKELOCK*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend_desc;
#endif /*CONFIG_HAS_EARLYSUSPEND*/
#ifdef BCM59035_LOG_CHARGING_TIME
	ktime_t charging_start_time; /**< start time of charging*/
	ktime_t charging_end_time; /**< end time of charging*/
	ktime_t charging_time; /**< Total time taken for charging*/
#endif
	struct mutex power_lock;
};

typedef enum {
	EM_BATTMGR_CHARGER_PLUG_IN_EVENT,	/* /< Charger plug in event for both USB and Wall (basic notification of */
	EM_BATTMGR_CHARGER_PLUG_OUT_EVENT,	/* /< Charger plug out event */
	EM_BATTMGR_ENDOFCHARGE_EVENT,	/* /< End of Charge event. Battery is full - charging is done. */
	EM_BATTMGR_BATT_EXTREME_TEMP_EVENT,	/* /< BATT temp is outside window (safety) or extreme temperature */
	EM_BATTMGR_LOW_BATT_EVENT,	/* /< BATT low is detected */
	EM_BATTMGR_EMPTY_BATT_EVENT,	/* /< BATT empty is detected */
	EM_BATTMGR_BATTLEVEL_CHANGE_EVENT	/* /< BATT level change is detected */
} HAL_EM_BATTMGR_Event_en_t;
typedef struct {
	HAL_EM_BATTMGR_Event_en_t eventType;	/* /< The event type */
	u8 inLevel;		/* /< The battery level, 0~N, depend the sysparm */
	u16 inAdc_avg;		/* /< Adc value in mV. Ex, 4000 is 4.0V, 3800 is 3.8V */
	u8 inTotal_levels;	/* /< total levels */
} HAL_EM_BatteryLevel_t;

static bool usb_driver_init;
static int usb_enum_current = 460;
static struct platform_device *power_device;

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TEMP, /* Temp prop is register only if a valid temp adc channel is specified */
};

static enum power_supply_property wall_props[] = {
	POWER_SUPPLY_PROP_ONLINE
};

static enum power_supply_property usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE
};

static int bcm59035_usb_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	int ret = 0;
	struct bcm59035_power *bcm59035_power =
	 dev_get_drvdata(psy->dev->parent);
	if (unlikely(!bcm59035_power)) {
		PMU_LOG(DEBUG_PMU_ERROR, "%s: invalid driver data !!!\n",
			__func__);
		return -EINVAL;
	}
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval =
		 (bcm59035_power->power_src ==
		 POWER_SUPPLY_TYPE_USB) ? 1 : 0;
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int bcm59035_wall_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	int ret = 0;
	struct bcm59035_power *bcm59035_power =
	 dev_get_drvdata(psy->dev->parent);

	if (unlikely(!bcm59035_power)) {
		PMU_LOG(DEBUG_PMU_ERROR, "%s: invalid driver data !!!\n",
			__func__);
		return -EINVAL;
	}
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval =
		 (bcm59035_power->power_src ==
		 POWER_SUPPLY_TYPE_MAINS) ? 1 : 0;
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int bcm59035_battery_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	int ret = 0;
	struct bcm59035_power *bcm59035_power =
	 dev_get_drvdata(psy->dev->parent);
	struct bcm59035_power_pdata *pdata;

	if (unlikely(!bcm59035_power || !bcm59035_power->bcm59035)) {
		PMU_LOG(DEBUG_PMU_ERROR, "%s: invalid driver data !!!\n",
			__func__);
		return -EINVAL;
	}
	pdata = bcm59035_power->bcm59035->pdata->power;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bcm59035_power->charging_status;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = pdata->batt_technology;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bcm59035_power->batt_percentage;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if(pdata->batt_adc_tbl.num_entries)
			val->intval = pdata->batt_adc_tbl.bat_vol[pdata->batt_adc_tbl.num_entries-1] * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		if(pdata->batt_adc_tbl.num_entries)
			val->intval = pdata->batt_adc_tbl.bat_vol[0] * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bcm59035_power->batt_voltage;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (bcm59035_power->power_src == POWER_SUPPLY_TYPE_BATTERY) ? 1 : 0;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bcm59035_power->batt_health;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		if(bcm59035_power->bcm59035->pdata->pmu_event_cb)
				val->intval = bcm59035_power->bcm59035->pdata->pmu_event_cb(PMU_EVENT_BATT_TEMP_TO_DEGREE_C,
									bcm59035_power->batt_temp_adc_avg);
		break;

	default:
		pr_info("Bat: property:%d is not implemented\n", psp);
		ret = -EINVAL;
		break;
	}
	return ret;
}

static s8 bcm59035_get_batt_capacity(u16 batt_adc, struct batt_adc_tbl* batt_adc_tbl,u8 prev_per, u8* new_per)
{
	int inx;
	u32 per = 100;
	pr_info("%s:batt_adc = %d\n", __func__,batt_adc);

	if(batt_adc < batt_adc_tbl->bat_adc[0])
	{
		*new_per = 0;
		return -1; /*batt too low*/
	}

	else if(batt_adc > batt_adc_tbl->bat_adc[batt_adc_tbl->num_entries - 1])
	{
		*new_per = 100;
		return 1; /*beyond max limit*/
	}
	pr_info(KERN_INFO "%s:num_entries = %d\n",__func__, batt_adc_tbl->num_entries);
	for(inx = 0; inx < batt_adc_tbl->num_entries -1;inx++)
	{
		pr_info(KERN_INFO "%s:lvl = %d, adc = %d\n",__func__,inx, batt_adc_tbl->bat_adc[inx]);
		if(batt_adc >= batt_adc_tbl->bat_adc[inx] && batt_adc < batt_adc_tbl->bat_adc[inx+1])
		{
			per = inx*100 + ((batt_adc-batt_adc_tbl->bat_adc[inx])*100)/(batt_adc_tbl->bat_adc[inx+1]-batt_adc_tbl->bat_adc[inx]);
			per /= batt_adc_tbl->num_entries-1;
			break;
		}
	}
	/*round to nearest multiple of 5. Needed to adjust minor changes in battery % due to load*/
	if(prev_per != 0xFF)
	{
		if(per < prev_per)
			per = ((per+5)/5)*5;
		else
			per = (per/5)*5;
	}

	*new_per = (u8)per;
	return 0;
}

static void bcm59035_ril_adc_notify_cb(unsigned long msg_type, int result,
				 void *dataBuf, unsigned long dataLength)
{
	HAL_EM_BatteryLevel_t *batt_lvl = (HAL_EM_BatteryLevel_t *) dataBuf;
	struct bcm59035_power *bcm59035_power;
	struct bcm59035_power_pdata *pdata;
	u8 bat_per = 0;
	s8 bat_state;

	if (batt_lvl == NULL || power_device == NULL)
	{
		PMU_LOG(DEBUG_PMU_ERROR,
			"%s:Invalid params ...\n", __func__);
		return;
	}
	bcm59035_power = platform_get_drvdata(power_device);
	if (bcm59035_power == NULL)
	{
		PMU_LOG(DEBUG_PMU_ERROR, "%s:Device not init\n",
			__func__);
		return;
	}
	pdata = bcm59035_power->bcm59035->pdata->power;

	PMU_LOG(DEBUG_PMU_INFO,"%s:eventType = %d\n",__func__,batt_lvl->eventType);
	switch(batt_lvl->eventType)
	{
	case EM_BATTMGR_BATTLEVEL_CHANGE_EVENT:

		if(bcm59035_power->power_src != POWER_SUPPLY_TYPE_BATTERY &&
			batt_lvl->inAdc_avg >= (BATTERY_CHARGING_HYSTERESIS<<2) &&
			(batt_lvl->inAdc_avg - BATTERY_CHARGING_HYSTERESIS) > bcm59035_power->batt_adc_avg)
		{
			bcm59035_power->batt_adc_avg = batt_lvl->inAdc_avg - BATTERY_CHARGING_HYSTERESIS;
		}
		else
		{
			bcm59035_power->batt_adc_avg = batt_lvl->inAdc_avg;
		}

		PMU_LOG(DEBUG_PMU_INFO,"%s:inAdc_avg = %d\n",__func__,batt_lvl->inAdc_avg);
		bat_state = bcm59035_get_batt_capacity(bcm59035_power->batt_adc_avg,
						&pdata->batt_adc_tbl, bcm59035_power->batt_percentage,&bat_per);
		if(bat_state > 0)
		{
			bcm59035_power->batt_health =  POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		}
		else if(bcm59035_power->batt_health == POWER_SUPPLY_HEALTH_OVERVOLTAGE)
		{
			bcm59035_power->batt_health =  POWER_SUPPLY_HEALTH_GOOD;
		}
		if(bcm59035_power->charging_status == POWER_SUPPLY_STATUS_FULL)
		{
			bat_per = 100;
		}
		if (bcm59035_power->batt_percentage != bat_per)
		{
			bcm59035_power->batt_percentage  = bat_per;
			/*"N mV" = (("8-Bit_ADC_code" - 30) * 4) + 3400*/
			bcm59035_power->batt_voltage = (bcm59035_power->batt_adc_avg - 30)*4 + 3400 - 12;
			if(bat_state <= 0 && bcm59035_power->batt_voltage > pdata->batt_adc_tbl.bat_vol[pdata->batt_adc_tbl.num_entries-1])
				bcm59035_power->batt_voltage = pdata->batt_adc_tbl.bat_vol[pdata->batt_adc_tbl.num_entries-1];
			bcm59035_power->batt_voltage*= 1000;

			PMU_LOG(DEBUG_PMU_INFO, "Battery percentage : %d, volt = %d\n", bcm59035_power->batt_percentage,bcm59035_power->batt_voltage);
			power_supply_changed(&bcm59035_power->battery);
		}
		break;

	case EM_BATTMGR_EMPTY_BATT_EVENT:
		pr_info(KERN_INFO "%s: low batt  event\n",__func__);
		bcm59035_get_batt_capacity(pdata->batt_adc_tbl.bat_adc[0],
		&pdata->batt_adc_tbl, 0xFF, &bcm59035_power->batt_percentage);
		bcm59035_power->batt_voltage = pdata->batt_adc_tbl.bat_vol[0]*1000;
		PMU_LOG(DEBUG_PMU_INFO, "Battery percentage : %d, volt = %d\n", bcm59035_power->batt_percentage,bcm59035_power->batt_voltage);
		bcm59035_power->batt_health = POWER_SUPPLY_HEALTH_DEAD;
		power_supply_changed(&bcm59035_power->battery);
		break;

	default:
		break;
	}
}

static void bcm59035_set_usb_rc_current(struct bcm59035_power *bcm59035_power,
					u8 rc_current)
{
	u8 regVal;
	struct bcm59035 *bcm59035 = bcm59035_power->bcm59035;
	/* MBCCTRL6 control the USB charging current */
	bcm59035->read_dev(bcm59035, BCM59035_REG_MBCCTRL6, &regVal);
	regVal &= ~BCM59035_MBCCTRL6_RC1_MASK;
	regVal |= (BCM59035_MBCCTRL6_RC1_MASK & rc_current);
	bcm59035->write_dev(bcm59035, BCM59035_REG_MBCCTRL6, regVal);
}

static void bcm59035_set_wall_rc_current(struct bcm59035_power *bcm59035_power,
					 u8 rc_current)
{
	u8 regVal;
	struct bcm59035 *bcm59035 = bcm59035_power->bcm59035;

	bcm59035->read_dev(bcm59035, BCM59035_REG_MBCCTRL4, &regVal);
	regVal &= ~BCM59035_MBCCTRL4_RC1_MASK;
	regVal |= rc_current & BCM59035_MBCCTRL4_RC1_MASK;
	bcm59035->write_dev(bcm59035, BCM59035_REG_MBCCTRL4, regVal);
}

/** @addtogroup BCM59035PowerAPIGroup
	@{
*/
/**
* @brief Get the current USB enumeration current set by PMU
*
* @return value of the USB enumeration current
*
* This method returns the USB enumeration current set by PMU.
*/
int pmu_get_usb_enum_current(void)
{
	PMU_LOG(DEBUG_PMU_INFO, "Inside %s\n", __func__);
	return usb_enum_current;
}


/**
* @brief Set the USB enumeration current to PMU
*
* @param pre_enum boolean indicating if the USB pre-enumeration current to be set to PMU
*
* This method sets the USB enumeration current to either pre enumeration
* current value or to the USB rapid charge current value set in platform based
* on the argument passed.
*/
void pmu_set_usb_enum_current(bool pre_enum)
{
	struct bcm59035_power_pdata *pdata;
	struct bcm59035_power *bcm59035_power;

	PMU_LOG(DEBUG_PMU_INFO, "Inside %s:pre_enum = %d\n", __func__,
		pre_enum);

	if (power_device == NULL) {
		PMU_LOG(DEBUG_PMU_ERROR, "%s:Device not init\n", __func__);
		return;
	}
	bcm59035_power = platform_get_drvdata(power_device);
	if (bcm59035_power == NULL) {
		PMU_LOG(DEBUG_PMU_ERROR, "%s:Device not init\n", __func__);
		return;
	}

	if (pre_enum)
		bcm59035_set_usb_rc_current(bcm59035_power, USB_PRE_ENUM_CUR);
	else {
		pdata = bcm59035_power->bcm59035->pdata->power;
		bcm59035_set_usb_rc_current(bcm59035_power,
					 pdata->usb.rc_current);
	}

}
/** @} */
EXPORT_SYMBOL(pmu_get_usb_enum_current);
EXPORT_SYMBOL(pmu_set_usb_enum_current);

static enum power_supply_type bcm59035_get_power_supply_type(struct
							 bcm59035_power
							 *bcm59035_power)
{
	u8 regVal;
	int ret;
	struct bcm59035 *bcm59035 = bcm59035_power->bcm59035;

	ret = bcm59035->read_dev(bcm59035, BCM59035_REG_ENV1, &regVal);
	if (!ret) {
		if (BCM59035_ENV1_CGPD & regVal)
			return POWER_SUPPLY_TYPE_MAINS;
		else if (BCM59035_ENV1_UBPD & regVal)
			return POWER_SUPPLY_TYPE_USB;
	}
	return POWER_SUPPLY_TYPE_BATTERY;
}

static void bcm59035_start_charging(struct bcm59035_power *bcm59035_power,
				 int charger_type)
{
	u8 regVal,mbcctrl2;
	struct bcm59035 *bcm59035 = bcm59035_power->bcm59035;
	struct bcm59035_power_pdata *pdata = bcm59035->pdata->power;
	u8 usb_current = pdata->usb.rc_current;

	bcm59035->read_dev(bcm59035, BCM59035_REG_MBCCTRL2, &mbcctrl2);

	if (bcm59035_power->bcm59035->flags & BCM59035_USE_DEDICATED_USB_CHARGER)
	{
		/*if BCM59035_USE_DEDICATED_USB_CHARGER is not enabled,
			charging timeout params are set in init function
		*/
		mbcctrl2 &= ~BCM59035_MBCCTRL2_USB_ELAPSED_TIMER_MASK;
		bcm59035->read_dev(bcm59035, BCM59035_REG_MBCCTRL9, &regVal);
		if (charger_type == POWER_SUPPLY_TYPE_USB)
		{
			regVal &= ~BCM59035_MBCCTRL9_SP_TYP;
			/*Set charging elapsed time for USB charger	*/
			mbcctrl2 |= (pdata->usb.max_charging_time << BCM59035_MBCCTRL2_USB_ELAPSED_TIMER_OFFSET);
		}
		else
		{
			regVal |= BCM59035_MBCCTRL9_SP_TYP;
			usb_current = pdata->wac.rc_current;
			/*Set charging elapsed time for wall charger	*/
			mbcctrl2 |= (pdata->wac.max_charging_time << BCM59035_MBCCTRL2_USB_ELAPSED_TIMER_OFFSET);
		}
		bcm59035->write_dev(bcm59035, BCM59035_REG_MBCCTRL9, regVal);
	}

	if (charger_type == POWER_SUPPLY_TYPE_USB ||
		bcm59035_power->bcm59035->flags & BCM59035_USE_DEDICATED_USB_CHARGER)
	{
		bcm59035_set_usb_rc_current(bcm59035_power, usb_current);
		bcm59035->read_dev(bcm59035, BCM59035_REG_MBCCTRL8, &regVal);
		regVal |= BCM59035_MBCCTRL8_VUBGRRC;
		bcm59035->write_dev(bcm59035, BCM59035_REG_MBCCTRL8, regVal);	/* Enable rapid charge mode and enable USB charger */
	}
	else
	{
		mbcctrl2 |= BCM59035_MBCCTRL2_VCHGRRC;
		bcm59035_set_wall_rc_current(bcm59035_power,
					 pdata->wac.rc_current);
	}
	mbcctrl2 |= BCM59035_MBCCTRL2_MBCHOSTEN; /*enable charging*/
	bcm59035->write_dev(bcm59035, BCM59035_REG_MBCCTRL2, mbcctrl2); /*write back mbcctrl2 register */

	bcm59035_enable_irq(bcm59035_power->bcm59035,
			 BCM59035_IRQID_INT2_CHGEOC);


	bcm59035_enable_irq(bcm59035_power->bcm59035,
			 BCM59035_IRQID_INT2_MBCCHGERR);

	bcm59035_power->charging_status = POWER_SUPPLY_STATUS_CHARGING;
	bcm59035_power->power_src = charger_type;
	power_supply_changed((charger_type ==
			 POWER_SUPPLY_TYPE_USB) ? &bcm59035_power->
			 usb : &bcm59035_power->wall);
	power_supply_changed(&bcm59035_power->battery);

#ifdef BCM59035_LOG_CHARGING_TIME
	bcm59035_power->charging_start_time = ktime_get();
#endif
}

static void bcm59035_stop_charging(struct bcm59035_power *bcm59035_power,
				 bool updatePwrSrc)
{
	u8 regVal;
	enum power_supply_type old_pwr_src;
	struct bcm59035 *bcm59035 = bcm59035_power->bcm59035;

	bcm59035->read_dev(bcm59035, BCM59035_REG_MBCCTRL2, &regVal);
	regVal &= ~(BCM59035_MBCCTRL2_MBCHOSTEN | BCM59035_MBCCTRL2_VCHGRRC);
	bcm59035->write_dev(bcm59035, BCM59035_REG_MBCCTRL2, regVal);
	bcm59035->read_dev(bcm59035, BCM59035_REG_MBCCTRL8, &regVal);
	regVal &= ~BCM59035_MBCCTRL8_VUBGRRC;	/*Disable RC */
	bcm59035->write_dev(bcm59035, BCM59035_REG_MBCCTRL8, regVal);

	bcm59035_power->charging_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	if (updatePwrSrc) {
		pr_info("%s:updatePwrSrc\n", __func__);
		old_pwr_src = bcm59035_power->power_src;
		bcm59035_power->power_src = POWER_SUPPLY_TYPE_BATTERY;
		bcm59035_power->charging_status =
		 POWER_SUPPLY_STATUS_DISCHARGING;
		if (old_pwr_src == POWER_SUPPLY_TYPE_USB) {
			power_supply_changed(&bcm59035_power->usb);
		} else if (old_pwr_src == POWER_SUPPLY_TYPE_MAINS) {
			power_supply_changed(&bcm59035_power->wall);
		}
	}
	bcm59035_disable_irq(bcm59035_power->bcm59035,
				 BCM59035_IRQID_INT2_CHGEOC);
	bcm59035_disable_irq(bcm59035_power->bcm59035,
			 BCM59035_IRQID_INT2_MBCCHGERR);

	power_supply_changed(&bcm59035_power->battery);
}

/****************************************************************************
*
* bcm59035_get_batt_temperature
*
* returns: temperature measurement in ADC units or -1 on error
*
***************************************************************************/
static int bcm59035_get_batt_temperature(struct bcm59035_power *bcm59035_power)
{
	int temp;
	int i;
	struct bcm59035_power_pdata *pdata =
	 bcm59035_power->bcm59035->pdata->power;

	if (pdata->temp_adc_channel < 0)
	    return -EINVAL;
	/* get 10 bit ADC output */
	temp = auxadc_access(pdata->temp_adc_channel);
	if (temp <= 0) {
		PMU_LOG(DEBUG_PMU_ERROR, "%s:Error reading ADC\n",
			__func__);
		return -1;
	}
	/* If it is the very first measurement taken, initialize the buffer elements to the same value */
	if (bcm59035_power->batt_temp_adc_avg == 0) {
		bcm59035_power->temp_running_sum = 0;
		for (i = 0; i < ADC_RUNNING_AVG_SIZE; i++) {
			temp = auxadc_access(pdata->temp_adc_channel);
			bcm59035_power->temp_reading[i] = temp;
			bcm59035_power->temp_running_sum += temp;
		}
		bcm59035_power->tem_read_inx = 0;
	}
	/* Keep the sum running forwards */
	bcm59035_power->temp_running_sum -=
	 bcm59035_power->temp_reading[bcm59035_power->tem_read_inx];
	bcm59035_power->temp_reading[bcm59035_power->tem_read_inx] = temp;
	bcm59035_power->temp_running_sum += temp;
	bcm59035_power->tem_read_inx =
	 (bcm59035_power->tem_read_inx + 1) % ADC_RUNNING_AVG_SIZE;

	/* Divide the running sum by number of measurements taken to get the average */
	bcm59035_power->batt_temp_adc_avg =
	 bcm59035_power->temp_running_sum >> ADC_RUNNING_AVG_SHIFT;

	return bcm59035_power->batt_temp_adc_avg;
}

static void bcm59035_batt_lvl_mon_wq(struct work_struct *work)
{
	int temp;
	struct bcm59035_power *bcm59035_power =
	 container_of(work, struct bcm59035_power, batt_lvl_mon_wq.work);
	struct bcm59035_power_pdata *pdata =
	 bcm59035_power->bcm59035->pdata->power;
	int mon_interval = BATTERY_LVL_MON_INTERVAL_WHILE_CHARGING; /*init to charging case */

	if (bcm59035_power->charging_status == POWER_SUPPLY_STATUS_DISCHARGING ||
		bcm59035_power->charging_status == POWER_SUPPLY_STATUS_FULL)
	{
		mon_interval = BATTERY_LVL_MON_INTERVAL;
	}

	/*Do temperature monitoring if a valid temp adc channel is specified */
	if (pdata->temp_adc_channel >= 0)
	{
#if defined(CONFIG_HAS_WAKELOCK)
		wake_lock(&bcm59035_power->temp_adc_wl);
#endif
		temp = bcm59035_get_batt_temperature(bcm59035_power);
#if defined(CONFIG_HAS_WAKELOCK)
		wake_unlock(&bcm59035_power->temp_adc_wl);
#endif
		if (temp < 0)
		{
			PMU_LOG(DEBUG_PMU_ERROR, "%s: Error reading temp\n", __func__);
		}
		else
		{
			if ((temp >= pdata->temp_high_limit) || (temp <= pdata->temp_low_limit))
			{
				PMU_LOG(DEBUG_PMU_INFO, "%s: Batt temp(%d) beyond limit\n", __func__, temp);
				if(bcm59035_power->temp_exceed_limit_count < BAT_TEMP_EXCEED_LIMIT_COUNT_MAX)
						bcm59035_power->temp_exceed_limit_count++;

				if(bcm59035_power->temp_exceed_limit_count == BAT_TEMP_EXCEED_LIMIT_COUNT_MAX &&
					POWER_SUPPLY_HEALTH_GOOD == bcm59035_power->batt_health)
				{
					bcm59035_power->batt_health = (temp >= pdata->temp_high_limit) ?
												POWER_SUPPLY_HEALTH_OVERHEAT : POWER_SUPPLY_HEALTH_COLD;
					if(POWER_SUPPLY_STATUS_CHARGING == bcm59035_power->charging_status)
					{
						PMU_LOG(DEBUG_PMU_INFO, "%s: Temp out of range - Charging stopped\n", __func__);
						bcm59035_stop_charging(bcm59035_power, false);
					}
					if(bcm59035_power->bcm59035->pdata->pmu_event_cb)
						bcm59035_power->bcm59035->pdata->pmu_event_cb(PMU_EVENT_BATT_TEMP_BEYOND_LIMIT, temp);
				}
			}
			else if(bcm59035_power->temp_exceed_limit_count)
			{
				if (--bcm59035_power->temp_exceed_limit_count == 0)
				{
					if(bcm59035_power->batt_health != POWER_SUPPLY_HEALTH_GOOD)
					{
						bcm59035_power->batt_health = POWER_SUPPLY_HEALTH_GOOD;
						if(POWER_SUPPLY_STATUS_NOT_CHARGING == bcm59035_power->charging_status)
							bcm59035_start_charging(bcm59035_power, bcm59035_power->power_src);
					}
				}
			}
		}
	}

	if (KRIL_DevSpecific_Cmd(BCM_POWER_CLIENT,
			RIL_DEVSPECIFICPARAM_BCM_PMU_GET_BATT_ADC, NULL,0) == false)
	{
		PMU_LOG(DEBUG_PMU_ERROR, "%s: KRIL_DevSpecific_Cmd failed\n",
			__func__);
	}
	queue_delayed_work(bcm59035_power->bcm59035->pmu_workqueue, &bcm59035_power->batt_lvl_mon_wq,
	msecs_to_jiffies(mon_interval));
}

static void bcm59035_power_isr(int intr, void *data)
{
	u8 regVal;
	struct bcm59035_power *bcm59035_power = data;
	struct bcm59035 *bcm59035 = bcm59035_power->bcm59035;
	int charger_type = POWER_SUPPLY_TYPE_BATTERY;
	PMU_LOG(DEBUG_PMU_INFO, "Inside %s:interrupt id = %u\n", __func__,
		intr);
	/* This function is invoked from pmu_usb_driver_initialized() as well.
	   Acquire lock to avoid sync. issues.
	*/
	mutex_lock(&bcm59035_power->power_lock);

	switch (intr) {
	case BCM59035_IRQID_INT2_CHGINS:

		PMU_LOG(DEBUG_PMU_INFO, "%s:Wall Charger detected\n",
			__func__);
		if (bcm59035->pdata->pmu_event_cb)
			bcm59035->pdata->pmu_event_cb(PMU_EVENT_CHARGER_INSERT,
			POWER_SUPPLY_TYPE_MAINS);
		if (KRIL_DevSpecific_Cmd(BCM_POWER_CLIENT,
		    RIL_DEVSPECIFICPARAM_BCM_PMU_GET_BATT_ADC, NULL, 0) == false)
				PMU_LOG(DEBUG_PMU_ERROR, "%s: KRIL_DevSpecific_Cmd failed\n", __func__);
		bcm59035_start_charging(bcm59035_power,
					POWER_SUPPLY_TYPE_MAINS);
		bcm59035_power->temp_exceed_limit_count = 0;

		break;

	case BCM59035_IRQID_INT2_USBINS:
		bcm59035_power->temp_exceed_limit_count = 0;
		if (usb_driver_init) {
			PMU_LOG(DEBUG_PMU_INFO, "%s:USB Charger detected\n",
				__func__);
			if (bcm59035_power->bcm59035->
			 flags & BCM59035_USE_DEDICATED_USB_CHARGER) {
				if (bcm59035->pdata->pmu_event_cb)
					charger_type = bcm59035->pdata->
							pmu_event_cb(PMU_EVENT_GET_CHARGER_TYPE, 1);
			}
			else {
				charger_type = POWER_SUPPLY_TYPE_USB;
			}
			PMU_LOG(DEBUG_PMU_INFO, "%s:bcm59035_get_power_supply_type: %d !!!\n",
					__func__, bcm59035_get_power_supply_type(bcm59035_power));
			if (charger_type != POWER_SUPPLY_TYPE_BATTERY &&
					bcm59035_get_power_supply_type(bcm59035_power)!= POWER_SUPPLY_TYPE_BATTERY)
			{
				if (KRIL_DevSpecific_Cmd(BCM_POWER_CLIENT,
					RIL_DEVSPECIFICPARAM_BCM_PMU_GET_BATT_ADC, NULL, 0) == false)
						PMU_LOG(DEBUG_PMU_ERROR, "%s: KRIL_DevSpecific_Cmd failed\n", __func__);

				if (charger_type == POWER_SUPPLY_TYPE_USB) {
#if defined(CONFIG_HAS_WAKELOCK)
					wake_lock(&bcm59035_power->usb_charger_wl);
#endif
					mdelay(4000);
				}
				bcm59035_start_charging(bcm59035_power, charger_type);
				if (bcm59035->pdata->pmu_event_cb)
					bcm59035->pdata->pmu_event_cb(PMU_EVENT_CHARGER_INSERT, charger_type);
			}
		}
		else {
			PMU_LOG(DEBUG_PMU_INFO,
			"%s:USB driver not initialized yet !!!\n",
			__func__);
		}
		break;

	case BCM59035_IRQID_INT2_CHGRM:

		PMU_LOG(DEBUG_PMU_INFO, "%s:Charger Removed\n", __func__);
		if (bcm59035->pdata->pmu_event_cb)
			bcm59035->pdata->pmu_event_cb(PMU_EVENT_CHARGER_REMOVE, POWER_SUPPLY_TYPE_MAINS);
		bcm59035_stop_charging(bcm59035_power, true);
		break;

	case BCM59035_IRQID_INT2_USBRM:

		PMU_LOG(DEBUG_PMU_INFO, "%s:Charger Removed\n", __func__);
		/*stop charging if source is not battery */
		if (bcm59035_power->power_src != POWER_SUPPLY_TYPE_BATTERY) {
			if (bcm59035->pdata->pmu_event_cb)
				bcm59035->pdata->pmu_event_cb(PMU_EVENT_CHARGER_REMOVE,
						bcm59035_power->power_src);
			pr_info(KERN_INFO "BCM59035_IRQID_INT2_USBRM\n");
#if defined(CONFIG_HAS_WAKELOCK)
			if (bcm59035_power->power_src == POWER_SUPPLY_TYPE_USB)
				wake_unlock(&bcm59035_power->usb_charger_wl); /*unlock charger wl*/
#endif
			bcm59035_stop_charging(bcm59035_power, true);
		}
		pr_info("bcm59035_power->power_src = %d\n",
		 bcm59035_power->power_src);
		pr_info("bcm59035_power->charging_status = %d\n",
		 bcm59035_power->charging_status);
		break;

	case BCM59035_IRQID_INT2_MBCCHGERR:
		pr_info("%s:Charing error - BCM59035_IRQID_INT2_MBCCHGERR\n",
		 __func__);
		bcm59035_power->batt_health = POWER_SUPPLY_HEALTH_DEAD;
		bcm59035_stop_charging(bcm59035_power, false);
		break;

	case BCM59035_IRQID_INT2_CHGEOC:
		bcm59035_disable_irq(bcm59035_power->bcm59035,
				 BCM59035_IRQID_INT2_CHGEOC);
		bcm59035_disable_irq(bcm59035_power->bcm59035,
			 BCM59035_IRQID_INT2_MBCCHGERR);

		PMU_LOG(DEBUG_PMU_INFO, "%s:BCM59035_IRQID_INT2_CHGEOC\n",
			__func__);
		bcm59035_power->charging_status = POWER_SUPPLY_STATUS_FULL;
		bcm59035_power->batt_percentage = 100;
		power_supply_changed(&bcm59035_power->battery);

#ifdef BCM59035_LOG_CHARGING_TIME
		bcm59035_power->charging_end_time = ktime_get();
		bcm59035_power->charging_time =
		 ktime_sub(bcm59035_power->charging_end_time,
			 bcm59035_power->charging_start_time);
		PMU_LOG(DEBUG_PMU_INFO, "%s:Total Charging Time %lld us\n",
			__func__,
			ktime_to_us(bcm59035_power->charging_time));
		charging_time_sec = bcm59035_power->charging_time.tv.sec;
		charging_time_nsec = bcm59035_power->charging_time.tv.nsec;
#endif
		break;
	}
	mutex_unlock(&bcm59035_power->power_lock);
	PMU_LOG(DEBUG_PMU_INFO, "%s:exit\n", __func__);

}

void pmu_usb_driver_initialized(void)
{
	int ret;
	struct bcm59035_power *bcm59035_power;

	PMU_LOG(DEBUG_PMU_INFO, "Inside %s\n", __func__);
	if (usb_driver_init)
		return;
	usb_driver_init = true;

	if (power_device == NULL)
		return;
	bcm59035_power = platform_get_drvdata(power_device);
	if (bcm59035_power == NULL)
		return;
	ret = bcm59035_get_power_supply_type(bcm59035_power);
	if (bcm59035_power->power_src == POWER_SUPPLY_TYPE_BATTERY &&
		ret != POWER_SUPPLY_TYPE_BATTERY) {
		PMU_LOG(DEBUG_PMU_INFO, "%s: %s is connected\n", __func__,
			(ret == POWER_SUPPLY_TYPE_MAINS ? "Wall" : "USB"));
		if (ret == POWER_SUPPLY_TYPE_MAINS) {
			bcm59035_power_isr(BCM59035_IRQID_INT2_CHGINS,
					 bcm59035_power);
		} else {
			bcm59035_power_isr(BCM59035_IRQID_INT2_USBINS,
					 bcm59035_power);
		}
	}
}

static void bcm59035_init_charger(struct bcm59035_power *bcm59035_power)
{
	u8 regVal,mbcctrl2;
	struct bcm59035 *bcm59035 = bcm59035_power->bcm59035;
	struct bcm59035_power_pdata *pdata = bcm59035->pdata->power;

	bcm59035->read_dev(bcm59035, BCM59035_REG_MBCCTRL2, &mbcctrl2);
	mbcctrl2 &= ~BCM59035_MBCCTRL2_VCHGRRC; /* clear wac rapid charge enable bit */

	if((bcm59035->flags & BCM59035_USE_DEDICATED_USB_CHARGER) == 0)
	{
		/*set wac charging timer */
		bcm59035->read_dev(bcm59035, BCM59035_REG_MBCCTRL1, &regVal);
		regVal &= ~BCM59035_MBCCTRL1_WAC_ELAPSED_TIMER_MASK;
		regVal |= (pdata->wac.max_charging_time << BCM59035_MBCCTRL1_WAC_ELAPSED_TIMER_OFFSET);
		bcm59035->write_dev(bcm59035, BCM59035_REG_MBCCTRL1, regVal);

		/*set usb charging timer */
		mbcctrl2 &= ~BCM59035_MBCCTRL2_USB_ELAPSED_TIMER_MASK;
		mbcctrl2 |= (pdata->usb.max_charging_time << BCM59035_MBCCTRL2_USB_ELAPSED_TIMER_OFFSET);
	}
	bcm59035->write_dev(bcm59035, BCM59035_REG_MBCCTRL2, mbcctrl2);

	bcm59035->read_dev(bcm59035, BCM59035_REG_MBCCTRL8, &regVal);
	regVal &= ~(BCM59035_MBCCTRL8_VUBGRRC|BCM59035_MBCCTRL8_MBCFAULTCNT_MASK);	/*clear fault count value */
	regVal |= BCM59035_MBCCTRL8_MBCFAULTCNT;	/* set fault count & enable USB RC */
	bcm59035->write_dev(bcm59035, BCM59035_REG_MBCCTRL8, regVal);

	/* Set Rapid Charge and Trickle Charge threashold values */
	/* For USB, set TC2 and RC threshold values */
	bcm59035->write_dev(bcm59035, BCM59035_REG_MBCCTRL5,
			 ((pdata->usb.tc_thold << 4) | pdata->usb.rc_thold));
	/* For wall, set TC2 and RC threshold values */
	bcm59035->write_dev(bcm59035, BCM59035_REG_MBCCTRL3,
			 ((pdata->wac.tc_thold << 4) | pdata->wac.rc_thold));
	/*USB Pre-enum RC charging current */
	bcm59035->read_dev(bcm59035, BCM59035_REG_MBCCTRL6, &regVal);
	regVal &= ~BCM59035_MBCCTRL6_RC1_MASK;
	regVal |= USB_PRE_ENUM_CUR;
	bcm59035->write_dev(bcm59035, BCM59035_REG_MBCCTRL6, regVal);
	/*TC current selection is spread across two registers - MBCCTRL4 & MBCCTRL11
	 MBCCTRL4 also holds wall RC */
	regVal = pdata->wac.rc_current & BCM59035_MBCCTRL4_RC1_MASK;
	/* usb tc bit 4 */
	if (pdata->usb.tc_current & 0x10)
		regVal |= BCM59035_MBCCTRL4_MBCUTC1_4;
	/* wall tc bit 4 */
	if (pdata->wac.tc_current & 0x10)
		regVal |= BCM59035_MBCCTRL4_MBCWTC1_4;
	bcm59035->write_dev(bcm59035, BCM59035_REG_MBCCTRL4, regVal);

	regVal =
	 (pdata->usb.tc_current & 0xF) << 4 | (pdata->wac.tc_current & 0x0F);
	bcm59035->write_dev(bcm59035, BCM59035_REG_MBCCTRL11, regVal);

	/*set EOC current */
	bcm59035->read_dev(bcm59035, BCM59035_REG_MBCCTRL7, &regVal);
	regVal &= ~BCM59035_MBCCTRL7_EOCS_MASK;	/*clear EOC */
	regVal |=
	 (pdata->
	 eoc_current & BCM59035_MBCCTRL7_EOCS_MASK) |
	 BCM59035_MBCCTRL7_OV_DISABLE;
	bcm59035->write_dev(bcm59035, BCM59035_REG_MBCCTRL7, regVal);

	/*Enable low cost wall charger support */
	bcm59035->read_dev(bcm59035, BCM59035_REG_MBCCTRL9, &regVal);
	regVal |=
	 BCM59035_MBCCTRL9_LOWCOST_WAC_EN | BCM59035_MBCCTRL9_LOWCOST_USB_EN;
	if (bcm59035->flags & BCM59035_USE_DEDICATED_USB_CHARGER) {
		regVal |= BCM59035_MBCCTRL9_CON_TYP;
	} else {
		regVal &= ~BCM59035_MBCCTRL9_CON_TYP;
	}
	regVal |= BCM59035_MBCCTRL9_MAINTCHRG;	/* enable maintenance charging */
	bcm59035->write_dev(bcm59035, BCM59035_REG_MBCCTRL9, regVal);
	/*set Low bat & maintenance charge level */
	bcm59035->read_dev(bcm59035, BCM59035_REG_LOWBATCVS, &regVal);
	regVal &=
	 ~(BCM59035_LOWBATCVS_LOWBATCVS_MASK |
	 BCM59035_LOWBATCVS_MBMCVS_MASK);
	regVal |=
	 BCM59035_LOWBATCVS_LOWBATCVS_3_6V | (BCM59035_LOWBATCVS_MBMCVS_4_10V
						 <<
						 BCM59035_LOWBATCVS_MBMCVS_POS);
	bcm59035->write_dev(bcm59035, BCM59035_REG_LOWBATCVS, regVal);

	/* Turn ON/OFF NTC block as per board data */
	bcm59035->read_dev(bcm59035, BCM59035_REG_MBCCTRL10, &regVal);
	if (bcm59035->flags & BCM59035_ENABLE_NTC)
		regVal |= BCM59035_MBCCTRL10_NTCON;
	else
		regVal &= ~BCM59035_MBCCTRL10_NTCON;
	bcm59035->write_dev(bcm59035, BCM59035_REG_MBCCTRL10, regVal);

	/*Enable synchronous mode for fuel guage */
	bcm59035->read_dev(bcm59035, BCM59035_REG_FGCTRL3, &regVal);
	regVal |= (1 << 5);	/*enable sync mode */
	bcm59035->write_dev(bcm59035, BCM59035_REG_FGCTRL3, regVal);

	/*Freeze fuel guage in deep sleep */
	bcm59035->write_dev(bcm59035, BCM59035_REG_FGOPMODCTRL, 5);

	/*Register for interrupts */
	bcm59035_request_irq(bcm59035, BCM59035_IRQID_INT2_CHGEOC, false, bcm59035_power_isr, bcm59035_power);	/*EOC charge interrupt */
	bcm59035_request_irq(bcm59035, BCM59035_IRQID_INT2_CHGINS, true, bcm59035_power_isr, bcm59035_power);	/*WAC connected interrupt */
	bcm59035_request_irq(bcm59035, BCM59035_IRQID_INT2_USBINS, true, bcm59035_power_isr, bcm59035_power);	/*USB connected interrupt */
	bcm59035_request_irq(bcm59035, BCM59035_IRQID_INT2_CHGRM, true, bcm59035_power_isr, bcm59035_power);	/*WAC removed interrupt */
	bcm59035_request_irq(bcm59035, BCM59035_IRQID_INT2_USBRM, true, bcm59035_power_isr, bcm59035_power);	/*USB removed interrupt */

	bcm59035_request_irq(bcm59035, BCM59035_IRQID_INT2_MBCCHGERR, false, bcm59035_power_isr, bcm59035_power);	/*USB removed interrupt */
}

static int bcm59035_power_ioctl_handler(u32 cmd, u32 arg, void *pri_data)
{
	struct bcm59035_power *bcm59035_power = pri_data;
	int ret = -EINVAL;
	PMU_LOG(DEBUG_PMU_INFO, "Inside %s, IOCTL command %d\n", __func__,
		cmd);
	switch (cmd) {
	case BCM_PMU_IOCTL_START_CHARGING:
		{
			int pwr_spply_type;
			if (copy_from_user
			 (&pwr_spply_type, (int *)arg, sizeof(int)) != 0) {
				pr_info("Error copying data from user\n");
				return -EFAULT;
			}
			pr_info
			 ("bcm59035_power_ioctl_handler: bcm59035_power->power_src %d, pwr_spply_type %d\n",
			 bcm59035_power->power_src, pwr_spply_type);
			if (bcm59035_power->power_src != pwr_spply_type)
				return -EINVAL;
			if (bcm59035_power->charging_status !=
			 POWER_SUPPLY_STATUS_CHARGING) {
				bcm59035_start_charging(bcm59035_power,
							bcm59035_power->
							power_src);
				ret = SUCCESS;
			} else {
				pr_info
				 ("bcm59035_power: already in charging mode or charger is not connected\n");
				ret = -EPERM;
			}
			break;
		}
	case BCM_PMU_IOCTL_STOP_CHARGING:
		{
		    	if ((bcm59035_power->charging_status != POWER_SUPPLY_STATUS_DISCHARGING) &&
			   	(bcm59035_power->charging_status != POWER_SUPPLY_STATUS_NOT_CHARGING))
			{
				bcm59035_stop_charging(bcm59035_power, false);
					ret = SUCCESS;
			} else {
				pr_info
				 ("bcm59035_power: already not in charging mode\n");
				ret = -EPERM;
			}
			break;
		}
	case BCM_PMU_IOCTL_SET_CHARGING_CURRENT:
		{
			/* Not required for now */
			break;
		}
	case BCM_PMU_IOCTL_GET_CHARGING_CURRENT:
		{
			/* Not required for now */
			break;
		}
	}
	return ret;
}
#if defined(CONFIG_HAS_EARLYSUSPEND)

static void bcm59035_power_early_suspend(struct early_suspend *h)
{
	struct bcm59035_power *bcm59035_power =
		container_of(h, struct bcm59035_power, early_suspend_desc);
	pr_info("[%s]+++\n",__func__);
	cancel_delayed_work_sync(&bcm59035_power->batt_lvl_mon_wq);
	pr_info("[%s]+++\n",__func__);
}

static void bcm59035_power_late_resume(struct early_suspend *h)
{
	struct bcm59035_power *bcm59035_power =
		container_of(h, struct bcm59035_power, early_suspend_desc);
	pr_info("[%s]+++\n",__func__);

	schedule_delayed_work(&bcm59035_power->batt_lvl_mon_wq,
						msecs_to_jiffies(500));

	pr_info("[%s]+++\n",__func__);
}
#else
#ifdef CONFIG_PM
static int bcm59035_power_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct bcm59035_power *bcm59035_power = platform_get_drvdata(pdev);
	if (bcm59035_power) {
		cancel_delayed_work_sync(&bcm59035_power->batt_lvl_mon_wq);
	}
	return 0;
}

static int bcm59035_power_resume(struct platform_device *pdev)
{
	struct bcm59035_power *bcm59035_power = platform_get_drvdata(pdev);
	if (bcm59035_power) {
		schedule_delayed_work(&bcm59035_power->batt_lvl_mon_wq,
						msecs_to_jiffies(2000));
	}
	return 0;
}
#else
#define bcm59035_power_suspend NULL
#define bcm59035_power_resume NULL
#endif /* CONFIG_PM */
#endif /*CONFIG_HAS_EARLYSUSPEND*/

static int __devinit bcm59035_power_probe(struct platform_device *pdev)
{
	struct bcm59035_power *bcm59035_power;
	struct bcm59035 *bcm59035 = dev_get_drvdata(pdev->dev.parent);
	struct bcm59035_power_pdata *power_pdata;
	int ret,i;
	int adc_avg =0,adc, trial;
#if defined(CONFIG_BRCM_FUSE_RIL_2157SDB) || defined(CONFIG_BRCM_FUSE_RIL_2153SDB)
	unsigned long notify_id_list[] = { RIL_NOTIFY_DEVSPECIFIC_BATT_LEVEL };
#endif

	PMU_LOG(DEBUG_PMU_INFO, "Inside %s\n", __func__);

	if (unlikely(!bcm59035->pdata || !bcm59035->pdata->power)) {
		PMU_LOG(DEBUG_PMU_ERROR, "%s: invalid platform data !!!\n",
			__func__);
		return -EINVAL;
	}
	power_pdata = bcm59035->pdata->power;

	bcm59035_power = kzalloc(sizeof(struct bcm59035_power), GFP_KERNEL);

	if (unlikely(!bcm59035_power))
		return -ENOMEM;
	bcm59035_power->bcm59035 = bcm59035;

	if(power_pdata->batt_vol_adc_channel != -1 && bcm59035->pdata->pmu_event_cb)
	{
		for (i = 0; i < ADC_RUNNING_AVG_SIZE; i++)
		{
			trial = 0;
			do
			{
				adc = auxadc_access(power_pdata->batt_vol_adc_channel);
			}while(adc == ADC_ERROR_VALUE && ++trial < ADC_MAX_RETRY);
			adc_avg += adc;
		}
		adc_avg = bcm59035->pdata->pmu_event_cb(PMU_EVENT_BATT_ADC_TO_8BIT_ADC,adc_avg/ADC_RUNNING_AVG_SIZE);

		bcm59035_get_batt_capacity(adc_avg,
						&power_pdata->batt_adc_tbl, 0,&bcm59035_power->batt_percentage);

		PMU_LOG(DEBUG_PMU_INFO, "%s:Bat capacity at boot = %d\n", __func__,bcm59035_power->batt_percentage);
	}
	else
	{
		/** init batt capacity to 20% if PMU_EVENT_BATT_ADC_TO_8BIT_ADC is
		not supported*/
		bcm59035_power->batt_percentage = 20;
	}
	bcm59035_power->batt_voltage = power_pdata->batt_adc_tbl.bat_vol[0]*1000;
	bcm59035_power->batt_health = POWER_SUPPLY_HEALTH_GOOD;

	INIT_DELAYED_WORK(&bcm59035_power->batt_lvl_mon_wq,
			 bcm59035_batt_lvl_mon_wq);

	platform_set_drvdata(pdev, bcm59035_power);
	power_device = pdev;
	mutex_init(&bcm59035_power->power_lock);

#if defined(CONFIG_HAS_WAKELOCK)
	wake_lock_init(&bcm59035_power->usb_charger_wl, WAKE_LOCK_SUSPEND,
						__stringify(usb_charger_wl));
	wake_lock_init(&bcm59035_power->temp_adc_wl, WAKE_LOCK_IDLE,
		 __stringify(temp_adc_wl));
#endif

#ifdef BCM59035_LOG_CHARGING_TIME
	bcm59035_power->charging_start_time = ktime_set(0, 0);
	bcm59035_power->charging_end_time = ktime_set(0, 0);
	bcm59035_power->charging_time = ktime_set(0, 0);
#endif

	bcm59035_init_charger(bcm59035_power);

	bcm59035_power->charging_status = POWER_SUPPLY_STATUS_DISCHARGING;
	bcm59035_power->power_src = POWER_SUPPLY_TYPE_BATTERY;

	/*register power supplies */
	bcm59035_power->wall.name = "bcm59035-wall";
	bcm59035_power->wall.type = POWER_SUPPLY_TYPE_MAINS;
	bcm59035_power->wall.properties = wall_props;
	bcm59035_power->wall.num_properties = ARRAY_SIZE(wall_props);
	bcm59035_power->wall.get_property = bcm59035_wall_get_property;
	ret = power_supply_register(&pdev->dev, &bcm59035_power->wall);
	if (ret)
		goto wall_err;

	bcm59035_power->battery.name = "bcm59035-battery";
	bcm59035_power->battery.properties = battery_props;
	bcm59035_power->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	/* Temp property is kept as the last entry in battery_props array. Temp prop is registered only
		if a valid temp adc channel is specified in platform data
	*/
	bcm59035_power->battery.num_properties = (power_pdata->temp_adc_channel >= 0) ? ARRAY_SIZE(battery_props) : ARRAY_SIZE(battery_props) - 1;
	bcm59035_power->battery.get_property = bcm59035_battery_get_property;
	ret = power_supply_register(&pdev->dev, &bcm59035_power->battery);
	if (ret)
		goto batt_err;

	bcm59035_power->usb.name = "bcm59035-usb",
	bcm59035_power->usb.type = POWER_SUPPLY_TYPE_USB;
	bcm59035_power->usb.properties = usb_props;
	bcm59035_power->usb.num_properties = ARRAY_SIZE(usb_props);
	bcm59035_power->usb.get_property = bcm59035_usb_get_property;
	ret = power_supply_register(&pdev->dev, &bcm59035_power->usb);
	if (ret)
		goto usb_err;

#if defined(CONFIG_BRCM_FUSE_RIL_2157SDB) || defined(CONFIG_BRCM_FUSE_RIL_2153SDB)
	if (KRIL_Register
	 (BCM_POWER_CLIENT, NULL,
	 bcm59035_ril_adc_notify_cb, notify_id_list,
	 ARRAY_SIZE(notify_id_list)) == false)
	{
		PMU_LOG(DEBUG_PMU_ERROR, "%s:KRIL_Register failed\n",
			__func__);
	}
#else
	PMU_LOG(DEBUG_PMU_ERROR, "%s:KRIL_Register not defined\n",
		__func__);
#endif

	/* check if USB/WALL is connected while booting */
	ret = bcm59035_get_power_supply_type(bcm59035_power);
	if (ret != POWER_SUPPLY_TYPE_BATTERY && usb_driver_init) {
		PMU_LOG(DEBUG_PMU_INFO, "%s: %s is connected\n", __func__,
			(ret == POWER_SUPPLY_TYPE_MAINS ? "Wall" : "USB"));
		if (ret == POWER_SUPPLY_TYPE_MAINS) {
			bcm59035_power_isr(BCM59035_IRQID_INT2_CHGINS,
					 bcm59035_power);
		} else {
			bcm59035_power_isr(BCM59035_IRQID_INT2_USBINS,
					 bcm59035_power);
		}
	}

	bcm59035_register_ioctl_handler(bcm59035, BCM59035_SUBDEV_POWER,
					bcm59035_power_ioctl_handler,
					bcm59035_power);
	queue_delayed_work(bcm59035_power->bcm59035->pmu_workqueue,
			&bcm59035_power->batt_lvl_mon_wq, msecs_to_jiffies(500));
#if defined(CONFIG_HAS_EARLYSUSPEND)
	bcm59035_power->early_suspend_desc.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +1;
	bcm59035_power->early_suspend_desc.suspend = bcm59035_power_early_suspend;
	bcm59035_power->early_suspend_desc.resume = bcm59035_power_late_resume;
	register_early_suspend(&bcm59035_power->early_suspend_desc);
#endif /*CONFIG_HAS_EARLYSUSPEND*/

	if (KRIL_DevSpecific_Cmd(BCM_POWER_CLIENT,
		    RIL_DEVSPECIFICPARAM_BCM_PMU_GET_BATT_ADC, NULL, 0) == false)
	{
		PMU_LOG(DEBUG_PMU_ERROR, "%s: KRIL_DevSpecific_Cmd failed\n", __func__);
	}

	pr_info("BCM59035 Power : Probe success---------------\n");

	return 0;

usb_err:
	power_supply_unregister(&bcm59035_power->battery);
batt_err:
	power_supply_unregister(&bcm59035_power->wall);
wall_err:
	mutex_destroy(&bcm59035_power->power_lock);
	kfree(bcm59035_power);
	return ret;
}

static int __devexit bcm59035_power_remove(struct platform_device *pdev)
{
	struct bcm59035_power *bcm59035_power = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&bcm59035_power->batt_lvl_mon_wq);
#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&bcm59035_power->early_suspend_desc);
#endif
	power_supply_unregister(&bcm59035_power->battery);
	power_supply_unregister(&bcm59035_power->wall);
	power_supply_unregister(&bcm59035_power->usb);
#if defined(CONFIG_HAS_WAKELOCK)
	wake_lock_destroy(&bcm59035_power->usb_charger_wl);
	wake_lock_destroy(&bcm59035_power->temp_adc_wl);
#endif
	mutex_destroy(&bcm59035_power->power_lock);
	kfree(bcm59035_power);
	return 0;
}

static struct platform_driver bcm59035_power_driver = {
	.driver = {
		 .name = "bcm59035-power",
		 .owner = THIS_MODULE,
		 },
	.remove = __devexit_p(bcm59035_power_remove),
	.probe = bcm59035_power_probe,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= bcm59035_power_suspend,
	.resume		= bcm59035_power_resume,
#endif
};

static int __init bcm59035_power_init(void)
{
	return platform_driver_register(&bcm59035_power_driver);
}

late_initcall(bcm59035_power_init);

static void __exit bcm59035_power_exit(void)
{
	platform_driver_unregister(&bcm59035_power_driver);
}

module_exit(bcm59035_power_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Regulator Driver for Broadcom BCM59035 PMU");
MODULE_ALIAS("platform:bcm59035-power");
