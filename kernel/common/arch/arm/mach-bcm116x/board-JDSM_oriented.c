/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/board-brava.c
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
 * Brava board specific driver definitions
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/delay.h>
#include <linux/serial_8250.h>
#include <linux/broadcom/types.h>
#include <linux/broadcom/bcm_fuse_sysparm.h>
#include <linux/leds.h>

#if defined (CONFIG_ANDROID_PMEM)
#include <linux/android_pmem.h>
#include <linux/dma-mapping.h>
#define PMEM_ADSP_SIZE (SZ_4M)
#endif
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/hardware.h>
#include <mach/setup.h>
#include <mach/reg_sys.h>
#include <plat/syscfg.h>
#include <plat/bcm_i2c.h>

#ifdef CONFIG_BCM_GPIO_VIBRATOR
#include <linux/timed_gpio.h>
#endif

#if defined (CONFIG_INPUT_BMA150_SMB380)
#include <linux/bma150.h>
#endif

#if defined (CONFIG_SENSORS_AK8973)
#include <linux/akm8973.h>
#endif

#if defined  CONFIG_TOUCHSCREEN_TSC2017
#include <linux/i2c/tsc2017.h>
#endif

#if defined (CONFIG_BACKLIGHT_SN3226)
#include <linux/sn3226_bl.h>
#elif defined (CONFIG_BACKLIGHT_CAT3648)
#include <linux/cat3648_bl.h>
#endif

#ifdef CONFIG_MMC_BCM
#include <mach/reg_sdio.h>
#endif
#ifdef CONFIG_KEYBOARD_BCM
#include <plat/bcm_keypad.h>
#include <mach/bcm_keymap.h>
#endif
#if defined(CONFIG_MFD_BCM59035)
#include <mach/gpio.h>
#include <linux/mfd/bcm59035/bcm59035.h>
#include <linux/power_supply.h>
#endif

#ifdef CONFIG_BACKLIGHT_PWM
#include <linux/pwm_backlight.h>
#endif

#if defined(CONFIG_BACKLIGHT_BCM_PWM_KEYPAD)
#include <plat/bcm_pwm_keypad_bl.h>
#endif

#if defined(CONFIG_BCM_PWM)
#include <plat/bcm_pwm_block.h>
#endif

#if defined(CONFIG_BCM_AUXADC)
#include <plat/bcm_auxadc.h>
#endif

#include "device.h"

#include <plat/timer.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>

#if (defined(CONFIG_BCM_RFKILL) || defined(CONFIG_BCM_RFKILL_MODULE))
#include <linux/broadcom/bcmblt-rfkill.h>
#include <plat/bcm_rfkill.h>
#endif

#if defined (CONFIG_BRCM_HEADSET)  || defined (CONFIG_BRCM_HEADSET_MODULE)
#include <plat/brcm_headset_pd.h>
#endif

#ifdef CONFIG_SPI
#include <plat/bcm_spi.h>
#endif

#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android.h>
#endif

#define		BC_10_DETECTION_SUPPORT		1
#ifdef CONFIG_BCM_4319_DRIVER

extern void bcmsdhc_sdio_host_force_scan(struct platform_device *pdev);
void bcm_wlan_power_on(int val);
void bcm_wlan_power_off(int val);
int bcm_set_core_power(unsigned int bcm_core, unsigned int pow_on,
		       unsigned int reset);
#endif /* 4319 */

#if defined(CONFIG_MFD_BCM59035)
extern int bcm_gpio_pull_up(unsigned int gpio, bool up);
extern int bcm_gpio_pull_up_down_enable(unsigned int gpio, bool enable);
#endif

extern int bcm_gpio_set_db_val(unsigned int gpio, unsigned int db_val);

#ifdef CONFIG_USB_ANDROID
static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id = 0x18D1,
	.product_id = 0x0005,
	.adb_product_id = 0x0002,
	.version = 0x0100,
	.product_name = "Android Phone",
	.manufacturer_name = "TCL",
	.serial_number="TCL-Martini",
	.nluns = 1,
};

static struct platform_device android_usb_device = {
      .name = "android_usb",
      .id         = -1,
      .dev        = {
            .platform_data = &android_usb_pdata,
      },
};
#endif

#if defined(CONFIG_SERIAL_8250) || defined(CONFIG_SERIAL_8250_MODULE)
/*!
 * The serial port definition structure.
 */
static struct plat_serial8250_port serial_platform_data0[] = {
	{
	 .membase = (void __iomem *)HW_UART_A_BASE,
	 .mapbase = HW_IO_VIRT_TO_PHYS(HW_UART_A_BASE),
	 .irq = IRQ_UARTA,
	 .uartclk = 13000000,
	 .regshift = 2,
	 .iotype = UPIO_MEM,
	 .flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	 .type = PORT_16550A,
	 .private_data = (void __iomem *)(HW_UART_A_BASE + 0x100)
	 },
	{
	 .flags = 0,
	 },
};

static struct plat_serial8250_port serial_platform_data1[] = {
	{
	 .membase = (void __iomem *)HW_UART_B_BASE,
	 .mapbase = HW_IO_VIRT_TO_PHYS(HW_UART_B_BASE),
	 .irq = IRQ_UARTB,
	 .uartclk = 13000000,
	 .regshift = 2,
	 .iotype = UPIO_MEM,
	 .flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	 .type = PORT_16550A,
	 .private_data = (void __iomem *)(HW_UART_B_BASE + 0x100)
	 },
	{
	 .flags = 0,
	 },
};
#endif

#ifdef CONFIG_MMC_BCM
extern int bcmsdhc_cfg_card_detect(void __iomem *ioaddr, u8 ctrl_slot);
extern int bcmsdhc_external_reset(void __iomem *ioaddr, u8 ctrl_slot);
extern int bcmsdhc_enable_int(void __iomem *ioaddr, u8 ctrl_slot);

int bcmsdhc_ctrl_slot_is_invalid(u8 ctrl_slot)
{
	if (ctrl_slot > 2)
		return -EINVAL;

	return 0;
}

static struct bcmsdhc_platform_data bcm_sdhc_data1 = {
	.base_clk = 48000000,
	.card_in_state = 0,
	.cd_pullup_cfg = 0,
	.irq_cd = 0,
	.syscfg_interface = board_sysconfig,
	.cfg_card_detect = bcmsdhc_cfg_card_detect,
	.external_reset = bcmsdhc_external_reset,
	.enable_int = bcmsdhc_enable_int,
};

static struct bcmsdhc_platform_data bcm_sdhc_data2 = {
	.base_clk = 48000000,
	.card_in_state = 0,
	.cd_pullup_cfg = SDCD_UPDOWN_DISABLE,
	.irq_cd = 12,
	.syscfg_interface = board_sysconfig,
	.cfg_card_detect = bcmsdhc_cfg_card_detect,
	.external_reset = NULL,
	.enable_int = bcmsdhc_enable_int,
};

#endif

#ifdef CONFIG_BCM_4319_DRIVER

#define BCM4325_BT 1
#define BCM4325_WLAN 2
#define BCM4325_BT_RESET 27
#define BCM4325_WLAN_RESET 27
#define GPIO_WLAN_BT_REG_ON 27
int bcm_set_core_power(unsigned int bcm_core, unsigned int pow_on,
		       unsigned int reset)
{
	unsigned gpio_rst, gpio_rst_another;

	if ((reset != 2) && (reset != 1)) {
		pr_info("%s: Error!! BAD Argument. ", __FUNCTION__);
		return -EINVAL;
	}

	switch (bcm_core) {
	case BCM4325_BT:
		gpio_rst = BCM4325_BT_RESET;
		gpio_rst_another = BCM4325_WLAN_RESET;
		break;

	case BCM4325_WLAN:
		gpio_rst = BCM4325_WLAN_RESET;
		gpio_rst_another = BCM4325_BT_RESET;
		break;

	default:
		pr_err("bcm_power: Unknown bcm core!\n");
		return -1;
	}

/*    mutex_lock(&bcm4325_pwr_lock); */

	/* Since reg on is coupled, check whether the other core is ON before
	   touching it */
  if ((gpio_get_value(gpio_rst_another) == 0) && ((reset == 1)|| (reset == 2))) {	
		/* enable WLAN_BT_REG_ON */
		gpio_direction_output(GPIO_WLAN_BT_REG_ON, pow_on);
		pr_info
		    ("bcm_power: Set WLAN_BT_REG_ON %s because %s is OFF now.\n",
		     gpio_get_value(GPIO_WLAN_BT_REG_ON) ? "High" : "Low",
		     bcm_core ? "BT" : "WLAN");
		msleep(150);
	}
	/* enable specified core */
	gpio_direction_output(gpio_rst, pow_on);
	pr_info("bcm_power: Set %s %s\n",
		bcm_core ? "WLAN_RESET" : "BT_RESET",
		gpio_get_value(gpio_rst) ? "High [chip out of reset]" :
		"Low [put into reset]");

/*    mutex_unlock(&bcm4325_pwr_lock); */

	return 0;
}

#define BCM_RESET 2
#define BCM_POWER 1
#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif

/** @addtogroup BoardBravaAPIGroup
	@{
*/
/**
* @brief	Power ON the WLAN
*
* @param val	Start/Power ON
*/
void bcm_wlan_power_on(int val)
{

	int err = 0;
	pr_info("%s: Enter.\n ", __FUNCTION__);

	switch (val) {
	case BCM_POWER:
		pr_info("%s: WIFI POWER UP!!\n ", __FUNCTION__);
		err = bcm_set_core_power(BCM4325_WLAN, TRUE, val);
		if (err < 0)
			pr_info("%s: FAILED!!\n ", __FUNCTION__);
		bcmsdhc_sdio_host_force_scan(&bcm_sdhc_slot1);
		break;
	case BCM_RESET:
		pr_info("%s: WIFI DRIVER START!!\n ", __FUNCTION__);
		err = bcm_set_core_power(BCM4325_WLAN, TRUE, val);
		if (err < 0)
			pr_info("%s: FAILED!!\n ", __FUNCTION__);
		break;
	default:
		pr_info("%s: INVALID ARG!!\n ", __FUNCTION__);

	}

	/* Note: that the platform device struct has to be exported from where it is defined */
	/* The below function would induce a forced mmc_rescan to detect the newly */
	/* powered up card. */

}


/**
* @brief	Power OFF the WLAN
*
* @param val	Stop/Power OFF
*/
void bcm_wlan_power_off(int val)
{

	int err = 0;
	pr_info("%s: Enter.\n ", __FUNCTION__);

	switch (val) {
	case BCM_POWER:
		pr_info("%s: WIFI POWER DOWN!\n ", __FUNCTION__);
		err = bcm_set_core_power(BCM4325_WLAN, FALSE, val);
		if (err < 0)
			pr_info("%s: FAILED!!\n ", __FUNCTION__);
		bcmsdhc_sdio_host_force_scan(&bcm_sdhc_slot1);
		break;
	case BCM_RESET:
		pr_info("%s: WIFI DRIVER STOP!!\n ", __FUNCTION__);
		err = bcm_set_core_power(BCM4325_WLAN, FALSE, val);
		if (err < 0)
			pr_info("%s: FAILED!!\n ", __FUNCTION__);
		break;
	default:
		pr_info("%s: INVALID ARG!!\n ", __FUNCTION__);

	}

	/* Note: that the platform device struct has to be exported from where it is defined */
	/* The below function would induce a forced mmc_rescan to detect the newly */
	/* powered up card. */

}
/** @} */
EXPORT_SYMBOL(bcm_wlan_power_on);
EXPORT_SYMBOL(bcm_wlan_power_off);

#endif /* 4319 */

#ifdef CONFIG_BCM_GPIO_VIBRATOR
static struct timed_gpio timed_gpios[] = {
	{
	 .name = "vibrator",
	 .gpio = 16,
	 .max_timeout = 15000,
	 .active_low = 0,
	 },
};

static struct timed_gpio_platform_data timed_gpio_data = {
	.num_gpios = ARRAY_SIZE(timed_gpios),
	.gpios = timed_gpios,
};

static struct platform_device bcm_timed_gpio = {
	.name = "timed-gpio",
	.id = -1,
	.dev = {
		.platform_data = &timed_gpio_data,
		},
};
#endif

#if defined(CONFIG_BCM_AUXADC)
static struct bcm_plat_auxadc adcplat = {
	.readmask = 0x3FF,
	.croff = 0,
	.rdoff = 8,
	.ready = (0x1 << 15),
	.start = (0x1 << 3),
	.auxpm = 1,
	.regoff = (0x1 << 23),
	.bgoff = (0x1 << 22),
};
#endif

#if defined(CONFIG_MFD_BCM59035)


static struct regulator_consumer_supply sim_consumers[] = {
	{
	 .dev = NULL,
	 .supply = "sim_vcc",
	 },
};

static struct regulator_consumer_supply cam_consumers[] = {
	{
	 .dev = NULL,
	 .supply = "cam_vdd",
	 },
};

static struct regulator_consumer_supply hcldo1_consumers[] = {
	{
	 .dev = NULL,
	 .supply = "spk_vdd",
	},
};

static struct regulator_init_data sim_init_data = {
	.constraints = {
			.min_uV = 1800000,
			.max_uV = 3300000,
			.valid_ops_mask =
			REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.always_on = 0,
			.boot_on = 0,
			},
	.num_consumer_supplies = ARRAY_SIZE(sim_consumers),
	.consumer_supplies = sim_consumers,
};

static struct regulator_init_data cam_init_data = {
	.constraints = {
			.min_uV = 1200000,
			.max_uV = 1500000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.always_on = 0,
			.boot_on = 0,
			},
	.num_consumer_supplies = ARRAY_SIZE(cam_consumers),
	.consumer_supplies = cam_consumers,
};

static struct regulator_init_data hcldo1_init_data = {
	.constraints = {
			.min_uV = 3000000,
			.max_uV = 3000000,
			.valid_ops_mask =
			 REGULATOR_CHANGE_STATUS,
			.always_on = 0,
			.boot_on = 0,
			},
	.num_consumer_supplies = ARRAY_SIZE(hcldo1_consumers),
	.consumer_supplies = hcldo1_consumers,
};

static struct bcm59035_regl_init_data bcm2157_regulators[] = {
	{
	 .regl_id = BCM59035_REGL_SIMLDO,
	 .init_data = &sim_init_data,
	 .dsm_opmode = BCM59035_REGL_LPM_IN_DSM,
	},
	{
	 .regl_id = BCM59035_REGL_LVLDO2,
	 .init_data = &cam_init_data,
     .dsm_opmode = BCM59035_REGL_LPM_IN_DSM,
	},
	{
	 .regl_id = BCM59035_REGL_HCLDO1,
	 .init_data = &hcldo1_init_data,
	 .dsm_opmode = BCM59035_REGL_OFF_IN_DSM,
	}
};

static struct bcm59035_regl_pdata regl_pdata = {
	.num_regulators = ARRAY_SIZE(bcm2157_regulators),
	.regl_init = bcm2157_regulators,
	.regl_default_pmmode = {
				[BCM59035_REGL_ALDO1] = 0x11,
				[BCM59035_REGL_ALDO2] = 0x22,
				[BCM59035_REGL_RFLDO1] = 0x22,
				[BCM59035_REGL_RFLDO2] = 0x11,
				[BCM59035_REGL_LCLDO] = 0x22,
				[BCM59035_REGL_LVLDO1] = 0x11,
				[BCM59035_REGL_LVLDO2] = 0x11,
				[BCM59035_REGL_HCLDO1] = 0xAA,
				[BCM59035_REGL_HCLDO2] = 0x11,
				[BCM59035_REGL_IOLDO] 	= 0x11,
				[BCM59035_REGL_MSLDO1] = 0xAA,
				[BCM59035_REGL_MSLDO2] = 0x11,
				[BCM59035_REGL_AUXLDO1] = 0x11,
				[BCM59035_REGL_AUXLDO2] = 0x11,
				[BCM59035_REGL_SIMLDO] = 0x99,
				[BCM59035_REGL_CSR] = 0x11,
				[BCM59035_REGL_IOSR] = 0x11,
				},
};

static struct bcm59035_power_pdata power_pdata = {
	.usb = {
		.tc_thold = BCM59035_TC_RC_THOLD_3_60V,
		.rc_thold = BCM59035_TC_RC_THOLD_4_20V,
		.rc_current = BCM59035_CHARGING_CUR_460MA,
		.tc_current = BCM59035_CHARGING_CUR_460MA,
		},
	.wac = {
		.tc_thold = BCM59035_TC_RC_THOLD_3_20V,
		.rc_thold = BCM59035_TC_RC_THOLD_4_20V,
		.rc_current = BCM59035_CHARGING_CUR_960MA,
		.tc_current = BCM59035_CHARGING_CUR_960MA,
		},
	.eoc_current = BCM59035_EOC_140MA,

	.temp_adc_channel = 1,
    .batt_vol_adc_channel = 2,
	.temp_low_limit = 284,
	.temp_high_limit = 1017,
	.batt_technology = POWER_SUPPLY_TECHNOLOGY_LION,
};


#ifdef BC_10_DETECTION_SUPPORT

extern USB_Charger_Type_t Android_Perform_BC10_Dectection(void);
extern USB_Charger_Type_t Android_Get_USB_BC_Type(void);
#endif

#ifndef BC_10_DETECTION_SUPPORT
static struct completion usb_enum_complete;
#endif
static bool usb_enum_started;
/** @addtogroup BoardBravaAPIGroup
	@{
*/
/**
* @brief	Fucntion called from USB driver to indicare that USB device is
* detected and enumeration started.
*
* This method is called from USB driver on a USB reset interrupt. This
* indicates the plattform/power driver that the charger inserted is a USB
* charger. This is used for charger type detection.
*/
void pmu_usb_enum_started(void)
{
    pr_info("Inside %s\n", __func__);
    usb_enum_started = true;
#ifndef BC_10_DETECTION_SUPPORT
    complete_all(&usb_enum_complete);
#endif
}
/** @} */
EXPORT_SYMBOL(pmu_usb_enum_started);

#ifdef CONFIG_USB_DWC_OTG
extern void dwc_otg_pcd_StartUsb(int is_on);
#endif
#ifdef CONFIG_USB_ANDROID
extern void Android_PMU_USB_Start(void);
extern void Android_usb_cable_connection(bool is_connected);
#endif
static void PMU_USB_START(void)
{
	pr_info("BCM59035: PMU_USB_START\n");
#ifdef CONFIG_USB_ANDROID
	Android_usb_cable_connection(TRUE);
	Android_PMU_USB_Start();
#endif
}

static void PMU_USB_STOP(void)
{
	pr_info("BCM59035: PMU_USB_STOP\n");
#ifdef CONFIG_USB_ANDROID
	Android_usb_cable_connection(FALSE);
#endif
#ifdef CONFIG_USB_DWC_OTG
	dwc_otg_pcd_StartUsb(0);
#endif
}
#define PMU_IRQ_GPIO    (21)
#define USB_STATE_GPIO  (29)
static int pmu_event_callback(int event, int param)
{	
    int retVal = 0;
	static u16 bat_vol[] = {3400, 3500, 3648, 3700, 3728, 3748, 3780, 3828, 3900,3972,4060,4200};
	static u16 bat_adc[] = {0x16, 0x37, 0x5C, 0x69, 0x70, 0x75, 0x7D, 0x89, 0x9B,0xAD,0xC3,0xEC};
	const u32 adc_offset = 650; /* magic number */
	const u32 adc_5per = 700;
#ifdef BC_10_DETECTION_SUPPORT
	int bc_charger_type = 0;
#endif
	switch (event) {
    case PMU_EVENT_INIT_PLATFORM:
    pr_info("%s:PMU : init hardware ......\n", __func__);
    if(gpio_request(PMU_IRQ_GPIO, "pmu_irq") == 0)
	{
		gpio_direction_input(PMU_IRQ_GPIO);
		bcm_gpio_pull_up_down_enable(PMU_IRQ_GPIO, true);
		bcm_gpio_pull_up(PMU_IRQ_GPIO, true);
		set_irq_type(GPIO_TO_IRQ(PMU_IRQ_GPIO), IRQF_TRIGGER_FALLING);
    } else {
    	pr_info("%s:Failed to allocate GPIO for PMU IRQ\n", __func__);
    }
	/*By default set USB_STATE_GPIO to input with pull disabled */
	gpio_request(USB_STATE_GPIO, "USB_STATE_GPIO");
	gpio_direction_input(USB_STATE_GPIO);
	bcm_gpio_pull_up_down_enable(USB_STATE_GPIO, false);

	/* init batt params */
	power_pdata.batt_adc_tbl.num_entries = ARRAY_SIZE(bat_adc);
	power_pdata.batt_adc_tbl.bat_adc = bat_adc;
	power_pdata.batt_adc_tbl.bat_vol = bat_vol;

#ifndef BC_10_DETECTION_SUPPORT
	/*initialize the completion structure used for charger detection */
	init_completion(&usb_enum_complete);
#endif
	break;

    case PMU_EVENT_CHARGER_INSERT:
		pr_info("%s:PMU :PMU_EVENT_CHARGER_INSERT\n", __func__);
    break;
    case PMU_EVENT_CHARGER_REMOVE:
		pr_info("%s:PMU : PMU_EVENT_CHARGER_REMOVE\n", __func__);
		if (param == POWER_SUPPLY_TYPE_USB) {
			PMU_USB_STOP();
			usb_enum_started = false;
		}
	/* put back USB_STATE_GPIO to input with pull disabled */
	gpio_direction_input(USB_STATE_GPIO);
	bcm_gpio_pull_up_down_enable(USB_STATE_GPIO, false);

    break;
    case PMU_EVENT_GET_CHARGER_TYPE:

	retVal = POWER_SUPPLY_TYPE_BATTERY;
    pr_info("%s:PMU : PMU_EVENT_GET_CHARGER_TYPE\n", __func__);
	/* On charger insert, set USB_STATE_GPIO dir to output */
	gpio_direction_output(USB_STATE_GPIO,0);

#ifndef BC_10_DETECTION_SUPPORT
    init_completion(&usb_enum_complete);
    PMU_USB_START();
    wait_for_completion_timeout(&usb_enum_complete,	msecs_to_jiffies(5000));
    if (usb_enum_started == true) {
		pr_info("%s:USB charger detected\n", __func__);
        retVal = POWER_SUPPLY_TYPE_USB;
    }
	else {
        retVal = POWER_SUPPLY_TYPE_MAINS;
		PMU_USB_STOP();
		pr_info("%s:Wall charger detected\n", __func__);
    }
#else
#ifdef CONFIG_USB_DWC_OTG
    bc_charger_type = Android_Perform_BC10_Dectection();
    if (bc_charger_type == HOST_CHARGER) {
		pr_info("%s: USB charger detected. \n", __func__);
        retVal = POWER_SUPPLY_TYPE_USB;
		PMU_USB_START();
    } else if (bc_charger_type == DEDICATED_CHARGER) {
		pr_info("%s: wall charger detected.\n", __func__);
        retVal = POWER_SUPPLY_TYPE_MAINS;
    }
#endif
#endif
	/* On charger insert, make sure USB_STATE drive is low */
	gpio_set_value(USB_STATE_GPIO, 0);
    break;

	case PMU_EVENT_BATT_ADC_TO_8BIT_ADC:
	/*
		Power driver uses PMU_EVENT_BATT_ADC_TO_8BIT_ADC event to conver adc value read
		using adc_access function to 8-bit adc value. This event is used only during init to
		initialize battery capacity value. Later the battery capacity is accurately calcuated based
		on calibrated adc value received from CP

		PMU_EVENT_BATT_ADC_TO_8BIT_ADC is used only during inittilzation and a rough value would suffice as
		cp sends calibrated adc value later.

		Assumtion: if adc value is less than 700, battery capapcity is <= 5%
		otherwise 8-bit adc is calculated as adc - 652
	*/
		if(param <= adc_5per)
		{
			retVal = 30; /* 8-bit adc value for 5% battery level*/
		}
		else
			retVal = param - adc_offset;

		pr_info("%s:PMU_EVENT_BATT_ADC_TO_8BIT_ADC -  adc = %d, 8bit adc = %d \n", __func__,param,retVal);
		break;

    }
    return retVal;
}

/* indices into the bcm59035 pwm channel init data array */
#define BCM59035_PWM_LED_CHANNEL        0
#define BCM59035_PWM_LED_CHANNEL1       1

/*
 * Platform data for BCM59035 PWM channels. Valid values
 * for the duty_ns and period_ns fields are as follows:
 * -------------------------------------
 * min_ns            max_ns
 * -------------------------------------
 * 0 (0ns)           504000 (504us)
 * 32000 (32us)      2016000 (2.016ms)
 * 128000 (128us)    8064000 (8.064ms)
 * 1024064 (1.024ms) 64516032 (64.516ms)
 * -------------------------------------
 * NOTE:
 * The values selected for both duty_ns and period_ns should
 * be in the same range (from one of the ranges shown above)
 */
static struct bcm59035_pwm_channel_data bcm59035_pwm_channel_data[] = {
	{
		.id = BCM59035_PWM_CHANNEL0,
		.mode = MODE_PWM_OFF_LED_OFF,
		.duty_ns = 0,
		.period_ns = 65535,
		.init_state = MODE_PWM_OFF_LED_OFF,
		.pad_config = PAD_SEL_PWM1_PWM2,
	}, {
		.id = BCM59035_PWM_CHANNEL1,
		.mode = MODE_PWM_OFF_LED_OFF,
		.duty_ns = 0,
		.period_ns = 65535,
		.init_state = MODE_PWM_OFF_LED_OFF,
		.pad_config = PAD_SEL_PWM1_PWM2,
	},
};

static struct bcm59035_pwm_pdata bcm59035_pwm_pdata = {
	.start_id = BCM59035_PWM_CHANNEL0,
	.data = bcm59035_pwm_channel_data,
	.num_channels = ARRAY_SIZE(bcm59035_pwm_channel_data),
};

struct bcm59035_platform_data bcm59035_pdata = {
	.flags =
	    BCM59035_USE_REGULATORS | BCM59035_USE_RTC | BCM59035_USE_GPIO |
	    BCM59035_USE_POWER | BCM59035_USE_PONKEY |
	    BCM59035_REGISTER_POWER_OFF | BCM59035_ENABLE_DVS |
	    BCM59035_USE_DEDICATED_USB_CHARGER | BCM59035_USE_PWM,
	.regulators = &regl_pdata,
	.gpio_base = 64,
	.power = &power_pdata,
	.pmu_event_cb = pmu_event_callback,
	.csr_nm_volt = CSR_VOUT_1_28V,
	.csr_lpm_volt = CSR_VOUT_0_90V,
	.i2c_pdata = {.i2c_spd = I2C_SPD_100K,},
	.pwm = &bcm59035_pwm_pdata,
	.led = NULL,
};
#endif /*CONFIG_MFD_BCM59035 */

#if defined (CONFIG_INPUT_BMA150_SMB380)
int bma_gpio_init(struct device *dev)
{
	gpio_request(5, "bma150");
	gpio_direction_input(5);
	set_irq_type(GPIO_TO_IRQ(5), IRQF_TRIGGER_RISING);
	return 0;
}

static struct bma150_accl_platform_data bma_pdata = {
	.orientation = BMA_ROT_180,
	.invert = false,
        .init		= bma_gpio_init,
	.i2c_pdata      = { .i2c_spd = I2C_SPD_100K,},
};
#endif

#if defined (CONFIG_SENSORS_AK8973)
static int bcm_akm8973_gpio_setup(void)
{
	gpio_request(20, "akm8973");
	gpio_direction_input(20);
	set_irq_type(GPIO_TO_IRQ(20), IRQF_TRIGGER_RISING);

	return 0;
}

struct akm8973_platform_data akm8973_pdata = {
	.layout = AKM_BACK_ROT_90,
	.gpio_RST = 0,
	.gpio_INT = GPIO_TO_IRQ(20),
	.dac_offsetx = 0,
	.dac_offsety = 0,
	.dac_offsetz = 129,
	.pre_offsetx = 328,
	.pre_offsety = -248,
	.pre_offsetz = -48,
	.init = bcm_akm8973_gpio_setup,
	.i2c_pdata = {.i2c_spd = I2C_SPD_100K,},
};

#endif

#if defined(CONFIG_AMP_NCP2704)
struct i2c_slave_platform_data brava_ncp2704_pdata = {
	.i2c_spd = I2C_SPD_100K,
};
#elif defined(CONFIG_AMP_MAX9877) || defined(CONFIG_AMP_MAX9877_MODULE)
struct i2c_slave_platform_data brava_max9877_pdata = {
	.i2c_spd = I2C_SPD_100K,
};
#endif

#if defined ( CONFIG_TOUCHSCREEN_TSC2017 )
#define TSC_RST_N	(19)
#define TSC_PEN_IRQ	(13)


/*
 * This Function returns the pen down state of the penirq pin
 * parameters:
 * input:
 *  pin:irq number
 * output:
 *  irq line status
 */

int tsc2017_pen_down_state(void)
{
	return (gpio_get_value(TSC_PEN_IRQ)) ? 0 : 1;
}

/*  This Funtion detects the proper PENIRQ line.
 *  It return the GPIO line ID to which the PENIRQ is connected
 *  & sets the GPIO Line as input.
 *
 *  parameters:
 *  output:
 *  irq number of the correspoding penirq.
 */

void tsc2017_clear_penirq(void)
{
	return;
}

void tsc2017_reset(void)
{
	gpio_direction_output(TSC_RST_N, 0);
	msleep(150);
	gpio_direction_output(TSC_RST_N, 1);

	return;
}

int tsc2017_init_platform_hw(void)
{
	printk("tsc2017_init_platform_hw\n");
	gpio_direction_input(TSC_PEN_IRQ);
	bcm_gpio_set_db_val(TSC_PEN_IRQ, 0xa);// set debounce to 4ms
	set_irq_type(GPIO_TO_IRQ(TSC_PEN_IRQ), IRQF_TRIGGER_FALLING);
	tsc2017_reset();
	return 0;
}

void tsc2017_exit_platform_hw(void)
{
}

/* Computes the x value based on the screen orientation */
int tsc2017_get_x_value(struct tsc2017_platform_data *pdata, int x, u32 max_val)
{

	return ((x - pdata->validx_left) * max_val /
		(pdata->validx_right - pdata->validx_left));
}

/* Computes the y value based on the screen orientation */
int tsc2017_get_y_value(struct tsc2017_platform_data *pdata, int y, u32 max_val)
{
	return (((pdata->validy_down - pdata->validy_up) -
		 (y - pdata->validy_up)) * max_val /
		(pdata->validy_down - pdata->validy_up));
}

struct tsc2017_platform_data tsc2017_pdata = {
		.get_pendown_state = tsc2017_pen_down_state,
		.clear_penirq = tsc2017_clear_penirq,
		.reset = tsc2017_reset,
		.init_platform_hw = tsc2017_init_platform_hw,
		.exit_platform_hw = tsc2017_exit_platform_hw,
		.get_x_value = tsc2017_get_x_value,
		.get_y_value = tsc2017_get_y_value,
		.x_plate_ohms = 50,
		.validx_left = 340,
		.validy_up = 450,
		.validx_right = 3800,
		.validy_down = 3800,
};


#endif


#if defined ( CONFIG_BRCM_HAL_CAM_WITH_STILL_YUV )
#define I2C_CAM_SLAVE_ADDRESS		0x60

struct i2c_slave_platform_data OV2655_cam_pdata = {
	.i2c_spd = I2C_SPD_100K,
};

static struct i2c_board_info __initdata bcm2157_cam_i2c_board_info[] = {
	{I2C_BOARD_INFO("cami2c", (I2C_CAM_SLAVE_ADDRESS >> 1)),
	 .platform_data = (void *)&OV2655_cam_pdata,
	 .irq = IRQ_CAM,
	},
};
#endif

#if defined (CONFIG_I2C_BCM1160)
struct i2c_host_platform_data i2c1_host_pdata = {
	.retries = 3,
};
#endif

static struct i2c_board_info __initdata bcm2157_i2c_board_info[] = {
#if defined(CONFIG_MFD_BCM59035)
	{I2C_BOARD_INFO("bcm59035", 0x08),
	 .platform_data = (void *)&bcm59035_pdata,
	 .irq = GPIO_TO_IRQ(PMU_IRQ_GPIO),
	 },
#endif /*CONFIG_MFD_BCM59035 */
#if defined ( CONFIG_TOUCHSCREEN_TSC2017 )
	{I2C_BOARD_INFO("tsc2017", 0x48),
	 .platform_data = (void *)&tsc2017_pdata,
	 .irq = GPIO_TO_IRQ(TSC_PEN_IRQ),
	 },
#endif

#if defined(CONFIG_AMP_NCP2704)
	{
	 I2C_BOARD_INFO("ncp2704", 0x41),
	 .platform_data = (void *)&brava_ncp2704_pdata,
	 },
#elif defined(CONFIG_AMP_MAX9877) || defined(CONFIG_AMP_MAX9877_MODULE)
	{
	 I2C_BOARD_INFO("max9877", 0x4d),
	 .platform_data = (void *)&brava_max9877_pdata,
	 },
#endif

#if defined(CONFIG_INPUT_BMA150_SMB380)
	{I2C_BOARD_INFO("bma150_accl", 0x38),
	 .platform_data = (void *)&bma_pdata,
	 .irq = GPIO_TO_IRQ(5),
	 },
#endif

#if defined (CONFIG_SENSORS_AK8973)
	{I2C_BOARD_INFO("akm8973", 0x1C),
	 .platform_data = (void *)&akm8973_pdata,
	 .irq = GPIO_TO_IRQ(20),
	 }
#endif

};

#ifdef CONFIG_BACKLIGHT_BCM_PWM_KEYPAD
static struct keypad_bl_drv_pdata keypad_bl_drv_pdata = {
	.pwm_ctrl_gpio = -EINVAL,
	.bl_pdata = {
		/* kepad backlight */
		.pwm_id = 7,
		.max_brightness = 32,
		.dft_brightness = 0,
		.pwm_period_ns = 65535,
	},
};

static struct platform_device bcm_pwm_keypad_backlight_device = {
	.name = "pwm-keypad-backlight",
	.id = 0,
	.dev = {
		.platform_data = &keypad_bl_drv_pdata,
	},
};

static struct keypad_bl_drv_pdata keypad_bl1_drv_pdata = {
	.pwm_ctrl_gpio = -EINVAL,
	.bl_pdata = {
		/* kepad backlight */
		.pwm_id = 6,
		.max_brightness = 32,
		.dft_brightness = 0,
		.pwm_period_ns = 65535,
	},
};

static struct platform_device bcm_pwm_keypad_backlight1_device = {
	.name = "pwm-keypad-backlight",
	.id = 1,
	.dev = {
		.platform_data = &keypad_bl1_drv_pdata,
	},
};

#endif /* CONFIG_BACKLIGHT_BCM_PWM_KEYPAD */

#ifdef CONFIG_BCM_PWM
static struct pwm_platform_data pwm_dev = {
	.max_pwm_id = 6,

};

static struct resource bcm_pwm_resource[] = {
	{
	 .start = HW_PWM_BASE,
	 .end = HW_PWM_BASE + SZ_4K - 1,
	 .flags = IORESOURCE_MEM,
	 },
};

static struct platform_device bcm_pwm_device = {
	.name = "bcm_pwm",
	.id = -1,
	.resource = bcm_pwm_resource,
	.num_resources = ARRAY_SIZE(bcm_pwm_resource),
	.dev = {
		.platform_data = &pwm_dev,
		}
};

#ifdef CONFIG_BACKLIGHT_PWM
static struct platform_pwm_backlight_data bcm_backlight_data = {
	/* backlight */
	.pwm_id = 1,
	.max_brightness = 32,
	.dft_brightness = 32,
	.pwm_period_ns = 65535,
};

static struct platform_device bcm_backlight_devices = {
	.name = "pwm-backlight",
	.id = 0,
	.dev = {
		.platform_data = &bcm_backlight_data,
		},
};
#endif /* CONFIG_BACKLIGHT_PWM */
#endif /* CONFIG_BCM_PWM */

#if defined (CONFIG_FB_BCM)
static struct platform_device bcm_device_fb = {
	.name = "LCDfb",
	.id = -1,
	.dev = {
		.dma_mask = (u64 *) ~(0),
		.coherent_dma_mask = ~(u32) 0,
		},
	.num_resources = 0,
};
#endif

#if defined (CONFIG_BACKLIGHT_SN3226)
static struct platform_sn3226_backlight_data bcm_sn3226_backlight_data = {
	/* backlight */
	.max_brightness = 31,
	.dft_brightness = 31,
	.ctrl_pin = 17,
};

static struct platform_device bcm_backlight_devices = {
	.name = "sn3226-backlight",
	.id = -1,
	.dev = {
		.platform_data = &bcm_sn3226_backlight_data,
		},
};

#elif defined (CONFIG_BACKLIGHT_CAT3648)
static struct platform_cat3648_backlight_data bcm_cat3648_backlight_data = {
	/* backlight */
	.max_brightness = 31,
	.dft_brightness = 31,
	.ctrl_pin = 17,
};

static struct platform_device bcm_backlight_devices = {
	.name = "cat3648-backlight",
	.id = -1,
	.dev = {
		.platform_data = &bcm_cat3648_backlight_data,
		},
};

#endif

#if defined(CONFIG_KEYBOARD_BCM)
void bcm_keypad_config_iocr(int row, int col)
{
	row = (1 << row) - 1;
	col = (1 << col) - 1;
	/*
	 * Set lower "row" & "col" number of bits to 1 to indicate
	 * configuration of keypad
	 */
	REG_SYS_IOCR1 = row | (col << 8);
}

static struct bcm_keymap newKeymap[] = {
	{BCM_KEY_ROW_0, BCM_KEY_COL_0, "Answer Key", KEY_SEND},
	{BCM_KEY_ROW_0, BCM_KEY_COL_1, "unused", 0},
	{BCM_KEY_ROW_0, BCM_KEY_COL_2, "unused", 0},
	{BCM_KEY_ROW_0, BCM_KEY_COL_3, "unused", 0},
	{BCM_KEY_ROW_0, BCM_KEY_COL_4, "unused", 0},
	{BCM_KEY_ROW_0, BCM_KEY_COL_5, "unused", 0},
	{BCM_KEY_ROW_0, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_0, BCM_KEY_COL_7, "unused", 0},
	{BCM_KEY_ROW_1, BCM_KEY_COL_0, "Home key", KEY_HOME},
	{BCM_KEY_ROW_1, BCM_KEY_COL_1, "unused", 0},
	{BCM_KEY_ROW_1, BCM_KEY_COL_2, "unused", 0},
	{BCM_KEY_ROW_1, BCM_KEY_COL_3, "unused", 0},
	{BCM_KEY_ROW_1, BCM_KEY_COL_4, "unused", 0},
	{BCM_KEY_ROW_1, BCM_KEY_COL_5, "unused", 0},
	{BCM_KEY_ROW_1, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_1, BCM_KEY_COL_7, "unused", 0},
	{BCM_KEY_ROW_2, BCM_KEY_COL_0, "Volume Up", KEY_VOLUMEUP},
	{BCM_KEY_ROW_2, BCM_KEY_COL_1, "unused", 0},
	{BCM_KEY_ROW_2, BCM_KEY_COL_2, "unused", 0},
	{BCM_KEY_ROW_2, BCM_KEY_COL_3, "unused", 0},
	{BCM_KEY_ROW_2, BCM_KEY_COL_4, "unused", 0},
	{BCM_KEY_ROW_2, BCM_KEY_COL_5, "unused", 0},
	{BCM_KEY_ROW_2, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_2, BCM_KEY_COL_7, "unused", 0},
	{BCM_KEY_ROW_3, BCM_KEY_COL_0, "Volume Down", KEY_VOLUMEDOWN},
	{BCM_KEY_ROW_3, BCM_KEY_COL_1, "unused", 0},
	{BCM_KEY_ROW_3, BCM_KEY_COL_2, "unused", 0},
	{BCM_KEY_ROW_3, BCM_KEY_COL_3, "unused", 0},
	{BCM_KEY_ROW_3, BCM_KEY_COL_4, "unused", 0},
	{BCM_KEY_ROW_3, BCM_KEY_COL_5, "unused", 0},
	{BCM_KEY_ROW_3, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_3, BCM_KEY_COL_7, "unused", 0},
	{BCM_KEY_ROW_4, BCM_KEY_COL_0, "unused", 0},
	{BCM_KEY_ROW_4, BCM_KEY_COL_1, "unused", 0},
	{BCM_KEY_ROW_4, BCM_KEY_COL_2, "unused", 0},
	{BCM_KEY_ROW_4, BCM_KEY_COL_3, "unused", 0},
	{BCM_KEY_ROW_4, BCM_KEY_COL_4, "unused", 0},
	{BCM_KEY_ROW_4, BCM_KEY_COL_5, "unused", 0},
	{BCM_KEY_ROW_4, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_4, BCM_KEY_COL_7, "unused", 0},
	{BCM_KEY_ROW_5, BCM_KEY_COL_0, "unused", 0},
	{BCM_KEY_ROW_5, BCM_KEY_COL_1, "unused", 0},
	{BCM_KEY_ROW_5, BCM_KEY_COL_2, "unused", 0},
	{BCM_KEY_ROW_5, BCM_KEY_COL_3, "unused", 0},
	{BCM_KEY_ROW_5, BCM_KEY_COL_4, "unused", 0},
	{BCM_KEY_ROW_5, BCM_KEY_COL_5, "unused", 0},
	{BCM_KEY_ROW_5, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_5, BCM_KEY_COL_7, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_0, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_1, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_2, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_3, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_4, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_5, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_7, "unused", 0},
	{BCM_KEY_ROW_7, BCM_KEY_COL_0, "unused", 0},
	{BCM_KEY_ROW_7, BCM_KEY_COL_1, "unused", 0},
	{BCM_KEY_ROW_7, BCM_KEY_COL_2, "unused", 0},
	{BCM_KEY_ROW_7, BCM_KEY_COL_3, "unused", 0},
	{BCM_KEY_ROW_7, BCM_KEY_COL_4, "unused", 0},
	{BCM_KEY_ROW_7, BCM_KEY_COL_5, "unused", 0},
	{BCM_KEY_ROW_7, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_7, BCM_KEY_COL_7, "unused", 0},
};

static struct bcm_keypad_platform_info bcm2157_keypad_data = {
	.row_num = 4,
	.col_num = 1,
	.keymap = newKeymap,
	.iocr_cfg = bcm_keypad_config_iocr,
	.bcm_keypad_base = (void *__iomem)HW_KEYPAD_BASE,
};

static struct platform_device brava_kp_device = {
	.name = "bcm_keypad",
	.id = -1,
	.dev = {
		.platform_data = &bcm2157_keypad_data,
		},
};
#endif

#define HEADSET_DET_GPIO        4    /* HEADSET DET */

void check_hs_state (int *headset_state)
{
    *headset_state = (gpio_get_value(HEADSET_DET_GPIO)) ? 0 : 1;
}

#if defined (CONFIG_BRCM_HEADSET)  || defined (CONFIG_BRCM_HEADSET_MODULE)
static struct brcm_headset_pd headset_pd = {
	.hsirq = NULL,
	.hsbirq = NULL,
	.check_hs_state = check_hs_state,
	.hsgpio= HEADSET_DET_GPIO,
};
#endif

#ifdef CONFIG_SPI
static DEFINE_SPINLOCK(bcm_spi_lock);
static int bcm21xx_cs_control(struct driver_data *drv_data, u8 cs)
{
	unsigned long flags;

	if (!drv_data)
		return -EINVAL;

	spin_lock_irqsave(&bcm_spi_lock, flags);

	writeb(SPI_SPIFSSCR_FSSNEW(cs) | SPI_SPIFSSCR_FSSSELNEW,
	       drv_data->ioaddr + SPI_SPIFSSCR);

	spin_unlock_irqrestore(&bcm_spi_lock, flags);

	return 0;
}

static struct bcm21xx_spi_platform_data bcm21xx_spi_info = {
	.slot_id = 0,
	.enable_dma = 1,
	.cs_line = 1,
	.mode = 0,		/* Configure for master mode */
	.syscfg_inf = board_sysconfig,
	.cs_control = bcm21xx_cs_control,
};

/*
 * SPI board info for the slaves
 */
static struct spi_board_info spi_slave_board_info[] __initdata = {
	{
	 .modalias = "spidev",	/* use spidev generic driver */
	 .max_speed_hz = 60000000,	/* use max speed */
	 .bus_num = 0,		/* framework bus number */
	 .chip_select = 0,	/* for each slave */
	 .platform_data = NULL,	/* no spi_driver specific */
	 .controller_data = &bcm21xx_spi_info,
	 .irq = 0,		/* IRQ for this device */
	 .mode = SPI_LOOP,	/* SPI mode 0 */
	 },
	/* TODO: adding more slaves here */
};

#endif

#if (defined(CONFIG_BCM_RFKILL) || defined(CONFIG_BCM_RFKILL_MODULE))

#define BCMBLT_VREG_GPIO        28
#define BCMBLT_N_RESET_GPIO     26
#define BCMBLT_AUX0_GPIO        (-1)   /* clk32 */
#define BCMBLT_AUX1_GPIO        (-1)    /* UARTB_SEL */

static struct bcmblt_rfkill_platform_data board_bcmblt_rfkill_cfg = {
	.vreg_gpio = BCMBLT_VREG_GPIO,
	.n_reset_gpio = BCMBLT_N_RESET_GPIO,
	.aux0_gpio = BCMBLT_AUX0_GPIO,	/* CLK32 */
	.aux1_gpio = BCMBLT_AUX1_GPIO,	/* UARTB_SEL, probably not required */
};

static struct platform_device board_bcmblt_rfkill_device = {
	.name = "bcmblt-rfkill",
	.id = -1,
};
#endif

static void brava_add_platform_data(void)
{
#if defined(CONFIG_SERIAL_8250) || defined(CONFIG_SERIAL_8250_MODULE)
	bcm_serial_device0.dev.platform_data = &serial_platform_data0;
	bcm_serial_device1.dev.platform_data = &serial_platform_data1;
#endif
#ifdef CONFIG_MMC_BCM
	bcm_sdhc_slot1.dev.platform_data = &bcm_sdhc_data1;
	bcm_sdhc_slot2.dev.platform_data = &bcm_sdhc_data2;
#endif
#if defined (CONFIG_BCM_AUXADC)
	auxadc_device.dev.platform_data = &adcplat;
#endif
#ifdef CONFIG_SPI
	bcm21xx_device_spi.dev.platform_data = &bcm21xx_spi_info;
#endif
#if defined (CONFIG_I2C_BCM1160)
	bcm_device_i2c1.dev.platform_data = &i2c1_host_pdata;
#endif
#if (defined(CONFIG_BCM_RFKILL) || defined(CONFIG_BCM_RFKILL_MODULE))
	board_bcmblt_rfkill_device.dev.platform_data = &board_bcmblt_rfkill_cfg;
#endif
#if defined (CONFIG_BRCM_HEADSET)  || defined (CONFIG_BRCM_HEADSET_MODULE)
	bcm_headset_device.dev.platform_data = &headset_pd;
#endif
}

#if defined (CONFIG_MTD_NAND_BCM_UMI)
static struct resource nand_resource[] = {
	[0] = {
	       .start = HW_IO_VIRT_TO_PHYS(HW_NAND_BASE),
	       .end = HW_IO_VIRT_TO_PHYS(HW_NAND_BASE) + 0x1000 - 1,
	       .flags = IORESOURCE_MEM,
	       },
};

static struct platform_device brava_nand_device = {
	.name = "bcm-nand",
	.id = -1,
	.resource = nand_resource,
	.num_resources = ARRAY_SIZE(nand_resource),
};
#endif

#if defined (CONFIG_ANDROID_PMEM)
static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem_adsp",
	.no_allocator = 0,
	.cached = 0,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = {.platform_data = &android_pmem_pdata},
};
#endif
static void brava_add_i2c_slaves(void)
{
	i2c_register_board_info(I2C_BSC_ID0, bcm2157_i2c_board_info,
				ARRAY_SIZE(bcm2157_i2c_board_info));
#if defined (CONFIG_BRCM_HAL_CAM_WITH_STILL_YUV )
	i2c_register_board_info(I2C_BSC_ID1, bcm2157_cam_i2c_board_info,
				ARRAY_SIZE(bcm2157_cam_i2c_board_info));
#endif

}

/**
* @brief
*
* @param module
* @param op
*
* @return
*/
int board_sysconfig(uint32_t module, uint32_t op)
{
	static DEFINE_SPINLOCK(bcm_syscfg_lock);
	uint32_t val = 0, mask = 0, ret = 0;
	unsigned long flags;
	spin_lock_irqsave(&bcm_syscfg_lock, flags);
	switch (module) {
	case SYSCFG_LCD:
		REG_SYS_IOCR0 &= ~(3 << 28);
		break;
	case SYSCFG_LCDBACKLIGHT:
		if (op == SYSCFG_DISABLE)
			REG_SYS_IOCR0 &= ~REG_SYS_IOCR0_BKLIGHT;
		else if (op == SYSCFG_ENABLE)
			REG_SYS_IOCR0 |= REG_SYS_IOCR0_BKLIGHT;
		break;
	case SYSCFG_CAMERA:
		/* IOCR2 for Camera interface */
		mask = REG_SYS_IOCR2_I2C2_OEB_MASK
		    | REG_SYS_IOCR2_I2C2_SLEW_MASK;
		if (op == SYSCFG_DISABLE)
			val = 1 << REG_SYS_IOCR2_I2C2_OEB_SHIFT;
		else if (op == SYSCFG_ENABLE)
			val = 0 << REG_SYS_IOCR2_I2C2_OEB_SHIFT;
		val |= (0 << REG_SYS_IOCR2_I2C2_SLEW_SHIFT);
		REG_SYS_IOCR2 = (REG_SYS_IOCR2 & ~(mask)) | val;
		/* IOCR3 for Camera interface */
		mask = REG_SYS_IOCR3_CAMD_PD_MASK |
		    REG_SYS_IOCR3_CAMHVS_PD_MASK |
		    REG_SYS_IOCR3_CAMDCK_PD_MASK | REG_SYS_IOCR3_CAMCK_DIS_MASK;
		if (op == SYSCFG_DISABLE)
			val = (1 << REG_SYS_IOCR3_CAMD_PD_SHIFT) |
			    (1 << REG_SYS_IOCR3_CAMHVS_PD_SHIFT) |
			    (1 << REG_SYS_IOCR3_CAMDCK_PD_SHIFT) |
			    (1 << REG_SYS_IOCR3_CAMCK_DIS_SHIFT);
		else if (op == SYSCFG_ENABLE)
			val = (0 << REG_SYS_IOCR3_CAMD_PD_SHIFT) |
			    (0 << REG_SYS_IOCR3_CAMHVS_PD_SHIFT) |
			    (0 << REG_SYS_IOCR3_CAMDCK_PD_SHIFT) |
			    (0 << REG_SYS_IOCR3_CAMCK_DIS_SHIFT);
		REG_SYS_IOCR3 = (REG_SYS_IOCR3 & ~(mask)) | val;
		/* IOCR4 for Camera interface */
		mask = REG_SYS_IOCR4_DRIVE_MASK <<
		    REG_SYS_IOCR4_CAMERA_DRIVE_SHIFT;
		if (op == SYSCFG_DISABLE)
			val = REG_SYS_IOCR4_DRIVE_2mA <<
			    REG_SYS_IOCR4_CAMERA_DRIVE_SHIFT;
		else if (op == SYSCFG_ENABLE)
			val = REG_SYS_IOCR4_DRIVE_12mA <<
			    REG_SYS_IOCR4_CAMERA_DRIVE_SHIFT;
		REG_SYS_IOCR4 = (REG_SYS_IOCR4 & ~(mask)) | val;
		break;
	case SYSCFG_USB:
		if (op == SYSCFG_DISABLE) {
			REG_SYS_ANACR9 = (ANACR9_USB_IDLE_ENABLE |
					  ANACR9_USB_UTMI_STOP_DIGITAL_CLOCKS |
					  ANACR9_USB_UTMI_SOFT_RESET_DISABLE);
			REG_SYS_ANACR9 &= ~(ANACR9_PLL_SUSPEND_ENABLE |
					    ANACR9_USB_PLL_POWER_ON);
		} else if (op == SYSCFG_ENABLE) {
			REG_SYS_ANACR9 = (ANACR9_USB_RESERVED_CLK_DURING_SUSP |
					  ANACR9_USB_UTMI_STOP_DIGITAL_CLOCKS |
					  ANACR9_USB_UTMI_SOFT_RESET_DISABLE |
					  ANACR9_USB_PLL_POWER_ON |
					  ANACR9_USB_PLL_CAL_ENABLE |
					  ANACR9_USB_SELECT_OTG_MODE |
					  ANACR9_USB_SELECT_DEVICE_MODE |
					  ANACR9_USB_PPC_PWR_OFF_ANALOG_DRIVERS);

			/* Reset the PHY since this could
			 * be a mode change too */
			REG_SYS_ANACR9 = (ANACR9_USB_RESERVED_CLK_DURING_SUSP |
					  ANACR9_USB_UTMI_STOP_DIGITAL_CLOCKS |
					  ANACR9_USB_PLL_POWER_ON |
					  ANACR9_USB_PLL_CAL_ENABLE |
					  ANACR9_USB_SELECT_OTG_MODE |
					  ANACR9_USB_SELECT_DEVICE_MODE |
					  ANACR9_USB_PPC_PWR_OFF_ANALOG_DRIVERS);

			/* De-activate PHY reset */
			REG_SYS_ANACR9 = (ANACR9_USB_RESERVED_CLK_DURING_SUSP |
					  ANACR9_USB_UTMI_STOP_DIGITAL_CLOCKS |
					  ANACR9_USB_UTMI_SOFT_RESET_DISABLE |
					  ANACR9_USB_PLL_POWER_ON |
					  ANACR9_USB_PLL_CAL_ENABLE |
					  ANACR9_USB_SELECT_OTG_MODE |
					  ANACR9_USB_SELECT_DEVICE_MODE |
					  ANACR9_USB_PPC_PWR_OFF_ANALOG_DRIVERS
					  | ANACR9_PLL_SUSPEND_ENABLE);
		}
		break;
	case SYSCFG_HEADSET:
		/* power up analog block for headset */
		if (op == SYSCFG_INIT)
			REG_SYS_ANACR2 = 0x2;
		break;
	case SYSCFG_AUXMIC:
		if (op == SYSCFG_INIT) {
			*AUXMIC_PRB_CYC_REG |= 0x04;
			*AUXMIC_MSR_DLY_REG |= 0x01;
			*AUXMIC_MSR_INTVL_REG |= 0x00;
			*AUXMIC_CMC_REG |= 0x00;
			*AUXMIC_MIC_REG |= 0x01;
			*AUXMIC_AUXEN_REG |= 0x00;
			*AUXMIC_MICINTH_ADJ_REG |= 13;
			*AUXMIC_MICINENTH_ADJ_REG |= 9;
			*AUXMIC_MICONTH_ADJ_REG |= 0x00;
			*AUXMIC_MICONENTH_ADJ_REG |= 0x00;
			*AUXMIC_F_PWRDWN_REG |= 0x00;
		}
		break;
	case SYSCFG_I2S:
		if (op == SYSCFG_INIT || op == SYSCFG_DISABLE) {
			REG_SYS_IOCR3 |= REG_SYS_IOCR3_DA_DIS_MASK;
		} else if (op == SYSCFG_ENABLE) {
			REG_SYS_IOCR3 &= ~REG_SYS_IOCR3_DA_DIS_MASK;
		}
		break;
	case SYSCFG_SDHC1:
		if (op == SYSCFG_INIT) {
			/* SPI/SD/GPIO Mask */
			REG_SYS_IOCR0 &= ~(0x00000018);
			REG_SYS_IOCR5 &= ~(0x00000100);

			REG_SYS_IOCR10 &= ~(0xfc000000);
			/* SD1 Pull up enable */
			REG_SYS_IOCR10 |= 0x50000000;
			REG_SYS_IOCR10 |= 0x00E00000;
			REG_SYS_IOCR6 |= 0x00001C00;
		} else if (op == SYSCFG_ENABLE) {
			REG_SYS_IOCR10 &= ~(0xfc000000);
			/* SD1 Pull up enable */
			REG_SYS_IOCR10 |= 0x50000000;
			/* SD1 Data pull up */
			REG_SYS_IOCR10 |= 0x00E00000;
			/* SD1 Data pull up */
			REG_SYS_IOCR6 |= 0x00001C00;
		} else if (op == SYSCFG_DISABLE) {
			/* For power saving,
			 * clear SD1 state in system iocr10 */
			/* REG_SYS_IOCR10 &= ~(0xfc000000); */
			/* SD1 Pull down enable */
			/* REG_SYS_IOCR10 |= 0xa8000000; */
		}
		break;
	case SYSCFG_SDHC2:
		if (op == SYSCFG_INIT) {
			/* SD/GPIO mask */
			/* Bit 0 as SD[7:4] is not used */
			REG_SYS_IOCR0 &= ~(0x00000022);
			/* For power save
			   00 no pull
			   01 pull up
			   10 pull down
			   11 liegal setting
			   SD2 clock,CMD,data do not Pull up */			
			REG_SYS_IOCR4 &= ~(0x38000000);
			REG_SYS_IOCR4 |= (0x28000000);
			REG_SYS_IOCR10 |= (0x00000003);
		} else if (op == SYSCFG_ENABLE) {
			/* SD2 clock pull up */
			/* SD2 CMD pull up */
			/* SD2 Data pull up */
			REG_SYS_IOCR2 &= ~(0xfc000000);
                        REG_SYS_IOCR2 |= 0x50000000;
		} else if (op == SYSCFG_DISABLE) {
			/* For power saving,
			 * clear SD2 state in system iocr2 */
			REG_SYS_IOCR2 &= ~(0xfc180000);
			REG_SYS_IOCR2 |= (0xa8100000);
		}
		break;
	case SYSCFG_SPI1:
		if (op == SYSCFG_ENABLE) {
			/* Enable SPI */
			REG_SYS_IOCR3 &= ~(0x00004000);
		} else if (op == SYSCFG_DISABLE) {
			/* Disable SPI */
			REG_SYS_IOCR3 |= (0x00004000);
		}
		break;
	case SYSCFG_RESETREASON:
		if (op == SYSCFG_INIT) {
			ret = REG_SYS_PUMR;
		} else if (op == SYSCFG_DISABLE) {
			REG_SYS_IOCR0 &= ~((1 << 27) | (1 << 14)
						| (1 << 13));
			REG_SYS_IOCR8 &= ~(1 << 6);
			REG_SYS_PUMR = SOFT_RESET;
		} else if (op == SYSCFG_ENABLE) {
			/* Configure GPEN9 as GPIO 52 */
			REG_SYS_IOCR0 |= (1 << 27) | (1 << 14);
			REG_SYS_IOCR0 |= (1 << 13);
			REG_SYS_IOCR8 |= (1 << 6);
		}
		break;
	case SYSCFG_SYSTEMINIT:
		if (op == SYSCFG_INIT) {
			REG_SYS_IOCR0 &= ~0x0381e018;
			REG_SYS_IOCR0 |= 0x0381e018;//0x0381a218;//modified for power consumption
			REG_SYS_IOCR1 = 0x00001f1f;
			REG_SYS_IOCR2 = 0x00800000;
			REG_SYS_IOCR3 = 0x08200000;
			/* Disable all SPIPAD lines by default */
			/* Unset bit 14 in IOCR3 to disable SPI */
			REG_SYS_IOCR3 |= (0x00004000);
            /* Unset bit 6 in IOCR8 to configure GPEN8 to GPIO51 *///modified for power consumption
			REG_SYS_IOCR8 |= (0x40);
			/* Unset bit 30 in IOCR8 to enable SPI */
			REG_SYS_IOCR8 &= ~(0x40000000);
			/* For wlan power saving enable SD1 pull
			 * down by default in system iocr10 */
			REG_SYS_IOCR10 &= ~(0xfc000000);
			REG_SYS_IOCR10 |= 0xa8000000;
			/* For power saving enable SD2 pull down
			 * by default in system iocr2 */
			REG_SYS_IOCR2 &= ~(0xfc180000);
			REG_SYS_IOCR2 |= 0xa8100000;

			/* Muxing the USBOTG pins to GPIO 61 abd 62 and
			 * enable input pull down for the same */
			REG_SYS_IOCR2 |= 0x00000600;

			val = REG_SYS_ANACR10;
			val &= ~(1 << 10);	/* Disable PullUP on UACTSN */
			val |= (1 << 11);	/* Enable PullDN on UACTSN */
			REG_SYS_ANACR10 = val;
			/* Enable DIGMICDATA pull down */
			REG_SYS_IOCR3 |= (0x20000000);
		}
		break;
	default:
		pr_info("%s: inval arguments\n", __func__);
		spin_unlock_irqrestore(&bcm_syscfg_lock, flags);
		return -EINVAL;
	}
	spin_unlock_irqrestore(&bcm_syscfg_lock, flags);
	return ret;
}

EXPORT_SYMBOL(board_sysconfig);

void __init bcm2157_brava_init(void)
{
	board_sysconfig(SYSCFG_SYSTEMINIT, SYSCFG_INIT);

	__REG32(HW_GPIO_BASE + 0x00) = 0x94000000;
	__REG32(HW_GPIO_BASE + 0x04) = 0x26A22A4A;//0xAAA22A4A: GPIO31 input without interrupt
	__REG32(HW_GPIO_BASE + 0x08) = 0x00000AAA;
	__REG32(HW_GPIO_BASE + 0x0c) = 0x0002A000;//0x000AAA00; GPIO51,52,53,57 Input without interrupt
	__REG32(HW_GPIO_BASE + 0x10) = 0x25060000;
	__REG32(HW_GPIO_BASE + 0x14) = 0x02000000;

	__REG32(HW_GPIO_BASE + 0x18) = 0x040C3FC0;
	__REG32(HW_GPIO_BASE + 0x1c) = 0x00000000;
	__REG32(HW_GPIO_BASE + 0x20) = 0xFF7FFFDF;
	__REG32(HW_GPIO_BASE + 0x24) = 0xFFFFFFFF;
	__REG32(HW_GPIO_BASE + 0x28) = 0x800CBFDF;//0x000CBFDF; power
	__REG32(HW_GPIO_BASE + 0x2c) = 0x83BFFFFF;

	/*SPIPADPULL settings: Set pulldown for all SPI lines */
	writel(readl(HW_SPI_BASE + 0x1034) | (0x000000aa),
	       HW_SPI_BASE + 0x1034);

	/* initialize the configuration info for the bcm59035
	 * pwm channel connected to the LEDs. detect the boot
	 * reason. if boot reason is POWEROFF_CHARGING, then
	 * mark the bcm59035 pwm channel 0 as already ON, so
	 * that the pwm driver will not reprogram this channel.
	 */
	if (board_sysconfig(SYSCFG_RESETREASON, SYSCFG_INIT) ==
		POWEROFF_CHARGING) {
		bcm59035_pwm_channel_data[BCM59035_PWM_LED_CHANNEL].init_state
			= MODE_PWM_ON_LED_OFF;
        bcm59035_pwm_channel_data[BCM59035_PWM_LED_CHANNEL1].init_state
            = MODE_PWM_ON_LED_OFF;
 
	}

#if defined(CONFIG_MFD_BCM59035)
#ifndef BC_10_DETECTION_SUPPORT
	/*initialize the completion structure used for charger detection */
	init_completion(&usb_enum_complete);
#endif
#endif
}

static int __init gpio_late_init(void)
{
#if 0
	uint8_t *pGPIO54Setting;
	uint32_t regval;
	pGPIO54Setting = SYSPARM_GetGPIO_Default_Value(54);
	pr_info("pGPIO54Setting = %x\n", pGPIO54Setting[0]);
	/* Check GPIO 54 is enabled */
	if (pGPIO54Setting[0]) {
		/* --- GPIO 54 enabled, so this is not NXP */
		/* For NOT NXP, set  IOCR0[27,14] to [0,1]  */
		regval = REG_SYS_IOCR0;
		regval |= (1 << 14);	/* GPEN9_0 pin select */
		regval &= ~(1 << 27);	/* GPEN9_1 pin Select */
		REG_SYS_IOCR0 = regval;

		pr_info(" Not NXP: IOCR0 = %x\n", regval);
	} else {
		regval = REG_SYS_IOCR0;
		/* --- GPIO 54 disabled, so this is NXP */
		/* For NXP, set  IOCR0[27,14] to [1,0]  */
		regval &= ~(1 << 14);	/* GPEN9_0 pin select */
		regval |= (1 << 27);	/* GPEN9_1 pin Select */
		REG_SYS_IOCR0 = regval;
		pr_info(" NXP: IOCR0 = %x\n", regval);
	}
#endif
	board_sysconfig(SYSCFG_LCDBACKLIGHT, SYSCFG_INIT);
#if defined (CONFIG_ANDROID_PMEM)
	{
		static dma_addr_t dma_address;
		static void *alloc_mem;

		alloc_mem =
		    dma_alloc_coherent(NULL, PMEM_ADSP_SIZE, &dma_address,
				       GFP_ATOMIC | GFP_DMA);
		if (alloc_mem != NULL) {
			android_pmem_pdata.start = dma_address;
			android_pmem_pdata.size = PMEM_ADSP_SIZE;
			platform_device_register(&android_pmem_device);
			pr_info("%s:Success PMEM alloc 0x%x\n", __func__,
				dma_address);
		} else {
			pr_info("%s:Fail to alloc memory\n", __func__);
		}
	}
#endif
	return 0;
}

late_initcall(gpio_late_init);

static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_BCM_WATCHDOG)
	&bcm_watchdog_device,
#endif
#if defined(CONFIG_SERIAL_8250) || defined(CONFIG_SERIAL_8250_MODULE)
	&bcm_serial_device0,
	&bcm_serial_device1,
#endif
#if defined (CONFIG_I2C_BCM1160)
	&bcm_device_i2c1,
	&bcm_device_i2c2,
#endif
#if defined (CONFIG_BCM_AUXADC)
	&auxadc_device,
#endif
#if defined (CONFIG_BRCM_HEADSET) || defined (CONFIG_BRCM_HEADSET_MODULE)
	&bcm_headset_device,
#endif
#if defined (CONFIG_FB_BCM)
	&bcm_device_fb,
#endif
#if defined (CONFIG_MMC_BCM)
	&bcm_sdhc_slot1,
	&bcm_sdhc_slot2,
#endif
#ifdef CONFIG_BCM_I2SDAI
	&i2sdai_device,
#endif
#if defined (CONFIG_BACKLIGHT_SN3226) || defined (CONFIG_BACKLIGHT_CAT3648)
	&bcm_backlight_devices,
#endif

#if defined (CONFIG_KEYBOARD_BCM)
	&brava_kp_device,
#endif
#if defined (CONFIG_MTD_NAND_BCM_UMI)
	&brava_nand_device,
#endif
#ifdef CONFIG_BCM_PWM
	&bcm_pwm_device,
#ifdef CONFIG_BACKLIGHT_PWM
	&bcm_backlight_devices,
#endif
#ifdef CONFIG_BACKLIGHT_BCM_PWM_KEYPAD
	&bcm_pwm_keypad_backlight_device,
    &bcm_pwm_keypad_backlight1_device,
#endif
#endif
#ifdef CONFIG_BCM_GPIO_VIBRATOR
	&bcm_timed_gpio,
#endif
#ifdef CONFIG_SPI
	&bcm21xx_device_spi,
#endif
#if (defined(CONFIG_BCM_RFKILL) || defined(CONFIG_BCM_RFKILL_MODULE))
	&board_bcmblt_rfkill_device,
#endif
#if defined(CONFIG_USB_ANDROID)
	&android_usb_device,
#endif

};

static void __init bcm2157_timer_init(void)
{
	struct timer_config bcm2157_timer_config = {
		.cs_base = (void __iomem *)HW_GPTIMER_BASE,
		.ce_base = (void __iomem *)HW_GPTIMER_BASE,
		.cp_cs_base = (void __iomem *)HW_SMT_BASE,
		.cs_index = 0,
		.ce_index = 1,
		.cp_cs_index = 0,
		.irq = IRQ_GPTMR,
	};

	bcm_timer_init(&bcm2157_timer_config);
}

static struct sys_timer bcm2157_timer = {
	.init = bcm2157_timer_init,
};

static void __init bcm2157_init_machine(void)
{
	bcm215x_platform_init();
	bcm2157_brava_init();
	brava_add_i2c_slaves();
	brava_add_platform_data();
	platform_add_devices(devices, ARRAY_SIZE(devices));
#ifdef CONFIG_SPI
	/*Function to register SPI board info : required when spi device is
	   present */
	spi_register_board_info(spi_slave_board_info,
				ARRAY_SIZE(spi_slave_board_info));
#endif
}

MACHINE_START(BCM1160, "BCM2157 Brava platform")
	/* Maintainer: Broadcom Corporation */
	.phys_io        = IO_START,
	.io_pg_offst    = (IO_BASE >> 18) & 0xfffc,
#ifdef CONFIG_CRASH_DUMP
	.boot_params    = (PHYS_OFFSET + 0x1000),
#else
	.boot_params    = (PHYS_OFFSET + 0x100),
#endif
	.map_io         = bcm215x_map_io,
	.init_irq       = bcm215x_init_irq,
	.timer          = &bcm2157_timer,
	.init_machine   = bcm2157_init_machine,
MACHINE_END
