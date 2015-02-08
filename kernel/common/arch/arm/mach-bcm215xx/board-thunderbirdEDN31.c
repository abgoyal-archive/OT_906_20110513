/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm215xx/board-athenaray.c
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
 * AthenaRay board specific driver definitions
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/device.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>

#include <linux/serial_8250.h>
#include <mach/hardware.h>
#include <mach/reg_syscfg.h>
#include <asm/mach/map.h>
#include <mach/setup.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>

#if defined (CONFIG_ANDROID_PMEM)
#include <linux/android_pmem.h>
#include <linux/dma-mapping.h>
#define PMEM_ADSP_SIZE (SZ_2M)
#endif

#ifdef CONFIG_MMC_BCM
#include <mach/sdio.h>
#endif

#if defined(CONFIG_MFD_MAX8986)
#include <linux/power_supply.h>
#include <linux/mfd/max8986/max8986.h>
#include <linux/broadcom/max8986/max8986-audio.h>
#endif

#ifdef CONFIG_USB_DWC_OTG
#include <mach/usbctl.h>
#endif
#ifdef CONFIG_KEYBOARD_BCM
#include <mach/bcm_keymap.h>
#include <plat/bcm_keypad.h>
#endif
#ifdef CONFIG_BACKLIGHT_PWM
#include <linux/pwm_backlight.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_QT602240
#include <linux/i2c/qt602240_ts.h>
#endif

#if defined (CONFIG_BCM_AUXADC)
#include <plat/bcm_auxadc.h>
#endif
#include "device.h"
#ifdef CONFIG_BCM_GPIO_VIBRATOR
#include <linux/timed_gpio.h>
#endif

#if defined (CONFIG_INPUT_BMA150_SMB380)
#include <linux/bma150.h>
#endif

#if defined(CONFIG_BRCM_HEADSET)  || defined(CONFIG_BRCM_HEADSET_MODULE)
#include <plat/brcm_headset_pd.h>
#endif

#include <linux/spinlock.h>
#include <linux/spi/spi.h>

#ifdef CONFIG_SPI
#include <plat/bcm_spi.h>
#endif

#include <plat/syscfg.h>
#include <plat/timer.h>

#if defined (CONFIG_BCM_PWM)
#include <plat/bcm_pwm_block.h>
#endif

#include <plat/bcm_i2c.h>

#ifdef CONFIG_BCM215XX_DSS
#include <plat/bcm_lcdc.h>
#endif

extern int bcm_gpio_pull_up(unsigned int gpio, bool up);
extern int bcm_gpio_pull_up_down_enable(unsigned int gpio, bool enable);

#ifdef CONFIG_MMC_BCM
extern int bcmsdhc_cfg_card_detect(void __iomem *ioaddr, u8 ctrl_slot);
extern int bcmsdhc_external_reset(void __iomem *ioaddr, u8 ctrl_slot);
extern int bcmsdhc_enable_int(void __iomem *ioaddr, u8 ctrl_slot);

int bcmsdhc_set_sys_interface(u8 ctrl_slot);
int bcmsdhc_request_cfg_pullup(void __iomem *ioaddr, u8 ctrl_slot);
int bcmsdhc_finish_cfg_pullup(void __iomem *ioaddr, u8 ctrl_slot);

static struct bcmsdhc_platform_data bcm21553_sdhc_data1 = {
	.base_clk = 50000000,
	.card_in_state = 0,
	.cd_pullup_cfg = SDCD_PULLUP | SDCD_UPDOWN_ENABLE,
	.irq_cd = 32,
	.syscfg_interface = board_sysconfig,
	.cfg_card_detect = bcmsdhc_cfg_card_detect,
	.external_reset = bcmsdhc_external_reset,
	.enable_int = bcmsdhc_enable_int,
};

#if !defined(CONFIG_MTD_ONENAND)
/*
 * SDHC2 is used for eMMC and SDHC2 shares pin mux with FLASH(OneNAND)
 * So both OneNAND and SDHC2 cannot co-exist
 */

static struct bcmsdhc_platform_data bcm21553_sdhc_data2 = {
	.base_clk = 50000000,
	.card_in_state = 0,
	.cd_pullup_cfg = SDCD_PULLUP | SDCD_UPDOWN_ENABLE,
	.irq_cd = 16,
	.syscfg_interface = board_sysconfig,
	.cfg_card_detect = bcmsdhc_cfg_card_detect,
	.external_reset = bcmsdhc_external_reset,
	.enable_int = bcmsdhc_enable_int,
};
#endif /*CONFIG_MTD_ONENAND */

static struct bcmsdhc_platform_data bcm21553_sdhc_data3 = {
	.base_clk = 50000000,
	.card_in_state = 0,
	.cd_pullup_cfg = SDCD_PULLUP | SDCD_UPDOWN_ENABLE,
	.irq_cd = 24,
	.syscfg_interface = board_sysconfig,
	.cfg_card_detect = bcmsdhc_cfg_card_detect,
	.external_reset = bcmsdhc_external_reset,
	.enable_int = bcmsdhc_enable_int,
};
#endif /*CONFIG_MMC_BCM */

#ifdef CONFIG_KEYBOARD_BCM
static void bcm_keypad_config_iocr(int row, int col)
{
	row = (1 << row) - 1;
	col = (1 << col) - 1;
	/* Set lower "row" & "col" number of bits to 1 to indicate configuration of keypad */
	writel((row | (col << 8)), IO_ADDRESS(BCM21553_SYSCFG_BASE) + 0x04);
}

static struct bcm_keymap newKeymap[] = {
	{BCM_KEY_ROW_0, BCM_KEY_COL_0, "key y", KEY_Y},
	{BCM_KEY_ROW_0, BCM_KEY_COL_1, "key t", KEY_T},
	{BCM_KEY_ROW_0, BCM_KEY_COL_2, "key r", KEY_R},
	{BCM_KEY_ROW_0, BCM_KEY_COL_3, "key e", KEY_E},
	{BCM_KEY_ROW_0, BCM_KEY_COL_4, "key w", KEY_W},
	{BCM_KEY_ROW_0, BCM_KEY_COL_5, "key q", KEY_Q},
	{BCM_KEY_ROW_0, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_0, BCM_KEY_COL_7, "unused", 0},
	{BCM_KEY_ROW_1, BCM_KEY_COL_0, "key h", KEY_H},
	{BCM_KEY_ROW_1, BCM_KEY_COL_1, "key g", KEY_G},
	{BCM_KEY_ROW_1, BCM_KEY_COL_2, "key f", KEY_F},
	{BCM_KEY_ROW_1, BCM_KEY_COL_3, "key d", KEY_D},
	{BCM_KEY_ROW_1, BCM_KEY_COL_4, "key s", KEY_S},
	{BCM_KEY_ROW_1, BCM_KEY_COL_5, "key a", KEY_A},
	{BCM_KEY_ROW_1, BCM_KEY_COL_6, "Camera Key", KEY_CAMERA},
	{BCM_KEY_ROW_1, BCM_KEY_COL_7, "unused", 0},
	{BCM_KEY_ROW_2, BCM_KEY_COL_0, "key b", KEY_B},
	{BCM_KEY_ROW_2, BCM_KEY_COL_1, "key v", KEY_V},
	{BCM_KEY_ROW_2, BCM_KEY_COL_2, "key c", KEY_C},
	{BCM_KEY_ROW_2, BCM_KEY_COL_3, "key x", KEY_X},
	{BCM_KEY_ROW_2, BCM_KEY_COL_4, "key z", KEY_Z},
	{BCM_KEY_ROW_2, BCM_KEY_COL_5, "key lshift", KEY_LEFTSHIFT},
	{BCM_KEY_ROW_2, BCM_KEY_COL_6, "Volume Up", KEY_VOLUMEUP},
	{BCM_KEY_ROW_2, BCM_KEY_COL_7, "unused", 0},
	{BCM_KEY_ROW_3, BCM_KEY_COL_0, "unused", 0},
	{BCM_KEY_ROW_3, BCM_KEY_COL_1, "key space", KEY_SPACE},
	{BCM_KEY_ROW_3, BCM_KEY_COL_2, "key tab", KEY_TAB},
	{BCM_KEY_ROW_3, BCM_KEY_COL_3, "unused", 0},
	{BCM_KEY_ROW_3, BCM_KEY_COL_4, "Menu_key", KEY_MENU},
	{BCM_KEY_ROW_3, BCM_KEY_COL_5, "Home_key", KEY_HOME},
	{BCM_KEY_ROW_3, BCM_KEY_COL_6, "Volume Down", KEY_VOLUMEDOWN},
	{BCM_KEY_ROW_3, BCM_KEY_COL_7, "unused", 0},
	{BCM_KEY_ROW_4, BCM_KEY_COL_0, "key menu", KEY_BACK},
	{BCM_KEY_ROW_4, BCM_KEY_COL_1, "key dot", KEY_DOT},
	{BCM_KEY_ROW_4, BCM_KEY_COL_2, "key p", KEY_P},
	{BCM_KEY_ROW_4, BCM_KEY_COL_3, "key o", KEY_O},
	{BCM_KEY_ROW_4, BCM_KEY_COL_4, "key i", KEY_I},
	{BCM_KEY_ROW_4, BCM_KEY_COL_5, "key u", KEY_U},
	{BCM_KEY_ROW_4, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_4, BCM_KEY_COL_7, "unused", 0},
	{BCM_KEY_ROW_5, BCM_KEY_COL_0, "key send", KEY_SEND},
	{BCM_KEY_ROW_5, BCM_KEY_COL_1, "key semicol", KEY_SEMICOLON},
	{BCM_KEY_ROW_5, BCM_KEY_COL_2, "key bkspace", KEY_BACKSPACE},
	{BCM_KEY_ROW_5, BCM_KEY_COL_3, "key l", KEY_L},
	{BCM_KEY_ROW_5, BCM_KEY_COL_4, "key k", KEY_K},
	{BCM_KEY_ROW_5, BCM_KEY_COL_5, "key j", KEY_J},
	{BCM_KEY_ROW_5, BCM_KEY_COL_6, "unused", 0},
	{BCM_KEY_ROW_5, BCM_KEY_COL_7, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_0, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_1, "unused", 0},
	{BCM_KEY_ROW_6, BCM_KEY_COL_2, "key enter", KEY_ENTER},
	{BCM_KEY_ROW_6, BCM_KEY_COL_3, "key comma", KEY_COMMA},
	{BCM_KEY_ROW_6, BCM_KEY_COL_4, "key m", KEY_M},
	{BCM_KEY_ROW_6, BCM_KEY_COL_5, "key n", KEY_N},
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

static struct bcm_keypad_platform_info bcm215xx_keypad_data = {
	.row_num = 7,
	.col_num = 7,
	.keymap = newKeymap,
	.iocr_cfg = bcm_keypad_config_iocr,
	.bcm_keypad_base = (void *)__iomem IO_ADDRESS(BCM21553_KEYPAD_BASE),
};

#endif

#ifdef CONFIG_BACKLIGHT_PWM
struct platform_device bcm_backlight_devices = {
	.name = "pwm-backlight",
	.id = 0,
};

static struct platform_pwm_backlight_data bcm_backlight_data = {
	/* backlight */
	.pwm_id = 1,
	.max_brightness = 100,
	.dft_brightness = 100,
	.pwm_period_ns = 65535,
};
#endif /*CONFIG_BACKLIGHT_PWM */

#ifdef CONFIG_BCM_PWM
static struct pwm_platform_data pwm_dev = {
	.max_pwm_id = 6,
	.syscfg_inf = board_sysconfig,
};
#endif

#if defined (CONFIG_MTD_ONENAND_BCM_XBOARD)
static struct resource onenand_resources[] = {
	{
	 .start = IO_ADDRESS(BCM21553_ONENAND_BASE),
	 .end = IO_ADDRESS(BCM21553_ONENAND_BASE) + SZ_128K - 1,
	 .flags = IORESOURCE_MEM,
	 },
};

struct flash_platform_data onenand_data = {
	.parts = NULL,
	.nr_parts = 0,
};

struct platform_device athenaray_device_onenand = {
	.name = "onenand",
	.resource = onenand_resources,
	.num_resources = ARRAY_SIZE(onenand_resources),
	.dev = {
		.platform_data = &onenand_data,
		},
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

#if defined (CONFIG_BRCM_HAL_CAM)
struct i2c_slave_platform_data stv_cam_pdata = {
        .i2c_spd = I2C_SPD_400K,
};

static struct i2c_board_info __initdata bcm21553_cam_i2c_board_info[] = {
        {I2C_BOARD_INFO("cami2c", (0x20 >> 1)),
         .platform_data = (void *)&stv_cam_pdata,
         .irq = IRQ_CAM_CSI2,
         },
};
#endif

#if defined (CONFIG_INPUT_BMA150_SMB380)
int bma_gpio_init(struct device *dev)
{
	gpio_request(27, "bma150");
	gpio_direction_input(27);
	set_irq_type(GPIO_TO_IRQ(27), IRQF_TRIGGER_RISING);
	return 0;
}

static struct bma150_accl_platform_data bma_pdata = {
	.orientation = BMA_ROT_90,
	.invert = true,
	.init = bma_gpio_init,
	.i2c_pdata = {.i2c_spd = I2C_SPD_100K,},
};
#endif

#ifdef CONFIG_BCM_GPIO_VIBRATOR
#define HAPTIC_ENABLE 56
#define HAPTIC_PWM 16
static struct timed_gpio timed_gpios[] = {
	{
	   .name = "vibrator",
	   .gpio = HAPTIC_ENABLE,
	   .max_timeout = 15000,
	   .active_low = 0,
	},
};

struct timed_gpio_platform_data timed_gpio_data = {
	.num_gpios      = ARRAY_SIZE(timed_gpios),
	.gpios          = timed_gpios,
};

struct platform_device bcm_timed_gpio = {
	.name           = "timed-gpio",
	.id             = -1,
	.dev			= {
		.platform_data = &timed_gpio_data,
	},
};

static void vibrator_init(void)
{
	/* Initialise the vibrator settings */
	board_sysconfig(SYSCFG_VIBRATOR, SYSCFG_INIT);

	gpio_request(HAPTIC_ENABLE, "haptic_enable");
	gpio_direction_output(HAPTIC_ENABLE, 0);
	bcm_gpio_pull_up(HAPTIC_ENABLE, true);
	bcm_gpio_pull_up_down_enable(HAPTIC_ENABLE, true);

	gpio_request(HAPTIC_PWM, "haptic_pwm");
	gpio_direction_output(HAPTIC_PWM, 0);
	bcm_gpio_pull_up(HAPTIC_PWM, true);
	bcm_gpio_pull_up_down_enable(HAPTIC_PWM, true);
}
#endif

#if defined(CONFIG_MFD_MAX8986)

static struct regulator_consumer_supply dldo3_consumers[] = {
	{
		.dev = NULL,
		.supply = "cam_vdd",
	},
	{
		.dev = NULL,
		.supply = "haptic_vcc",
	},
};

static struct regulator_init_data dldo3_init_data = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 3000000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		.always_on = 0,
		.boot_on = 0,
	},
	.num_consumer_supplies = ARRAY_SIZE(dldo3_consumers),
	.consumer_supplies = dldo3_consumers,
};

static struct regulator_consumer_supply sim_consumers[] = {
	{
		.dev	= NULL,
		.supply	= "sim_vcc",
	},
};

static struct regulator_init_data sim_init_data = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		.always_on = 0,
		.boot_on = 0,
	},
	.num_consumer_supplies = ARRAY_SIZE(sim_consumers),
	.consumer_supplies = sim_consumers,
};

static struct max8986_regl_init_data bcm21553_regulators[] = {
	{
		.regl_id = MAX8986_REGL_SIMLDO,
		.dsm_opmode = MAX8986_REGL_LPM_IN_DSM,
		.init_data = &sim_init_data,
	},
	{
		.regl_id = MAX8986_REGL_DLDO3,
		.dsm_opmode = MAX8986_REGL_OFF_IN_DSM,
		.init_data = &dldo3_init_data,
	},
};

static struct max8986_regl_pdata regl_pdata = {
	.num_regulators = ARRAY_SIZE(bcm21553_regulators),
	.regl_init =  bcm21553_regulators,
	.regl_default_pmmode = {
		[MAX8986_REGL_ALDO1]	= 0x00,
		[MAX8986_REGL_ALDO2]	= 0x00,
		[MAX8986_REGL_ALDO3]	= 0x00,
		[MAX8986_REGL_ALDO4]	= 0x00,
		[MAX8986_REGL_ALDO5]	= 0x00,
		[MAX8986_REGL_ALDO6]	= 0x00,
		[MAX8986_REGL_ALDO7]	= 0x00,
		[MAX8986_REGL_ALDO8]	= 0x00,
		[MAX8986_REGL_ALDO9]	= 0x00,
		[MAX8986_REGL_DLDO1]	= 0x00,
		[MAX8986_REGL_DLDO1]	= 0x00,
		[MAX8986_REGL_DLDO2]	= 0x00,
		[MAX8986_REGL_DLDO3]	= 0xAA,
		[MAX8986_REGL_DLDO4]	= 0x00,
		[MAX8986_REGL_HCLDO1]	= 0x00,
		[MAX8986_REGL_HCLDO2]	= 0x00,
		[MAX8986_REGL_LVLDO]	= 0x00,
		[MAX8986_REGL_SIMLDO]	= 0xAA,
		[MAX8986_REGL_AUXLDO1]	= 0x00,
		[MAX8986_REGL_TSRLDO]	= 0x00,
		[MAX8986_REGL_CSR]	= 0x11,
		[MAX8986_REGL_IOSR]	= 0x11,
	},
};

/*Default settings for audio amp */
static struct max8986_audio_pdata audio_pdata = {
	.ina_def_mode = MAX8986_INPUT_MODE_SINGLE_ENDED,
	.inb_def_mode = MAX8986_INPUT_MODE_DIFFERENTIAL,
	.ina_def_preampgain = PREAMP_GAIN_6DB,
	.inb_def_preampgain = PREAMP_GAIN_6DB,

	.lhs_def_mixer_in = MAX8986_MIXER_INB2,
	.rhs_def_mixer_in = MAX8986_MIXER_INB1,
	.ihf_def_mixer_in = MAX8986_MIXER_INA1,
};
static struct max8986_power_pdata power_pdata = {
	.usb = {
		.charging_cv = MBCCV_4_20V,
		.charging_cc = MAX8986_CHARGING_CURR_450MA
	},
	.wac = {
		.charging_cv = MBCCV_4_20V,
		.charging_cc = MAX8986_CHARGING_CURR_550MA
	},
	.eoc_current = MAX8986_EOC_200MA,

	.temp_adc_channel = 1,
	.temp_low_limit = 31,
	.temp_high_limit = 178,

	.batt_min_volt = 3200,
	.batt_max_volt = 4200,
	.batt_technology = POWER_SUPPLY_TECHNOLOGY_LION,
};

/*****************************************************************************
 * platform specific usb interface routines for the power driver
 *****************************************************************************/
/* indicates whether the usb stack is ready or not */
static bool usb_driver_initialized;
/* indicates whether usb charger is detected or not */
static bool usb_charger_detected;

/* if both the power driver and the usb driver are ready, then start the
 * enumeration seqence.
 */
static void pmu_usb_start(void)
{
	pr_info("%s\n", __func__);

#ifdef CONFIG_USB_ANDROID
	if (usb_driver_initialized == true && usb_charger_detected == true)
		Android_PMU_USB_Start();
	else
		pr_info("%s: usb stack is not up\n", __func__);
#endif
}

static void pmu_usb_stop(void)
{
	pr_info("%s\n", __func__);
#ifdef CONFIG_USB_DWC_OTG
	dwc_otg_pcd_StartUsb(0);
#endif
}

/*****************************************************************************
 * USB driver call backs
 *****************************************************************************/

/* this function is called from the usb driver once the USB reset event is
 * detected.
 */
void pmu_usb_enum_started(void)
{
	pr_info("%s\n", __func__);
}
EXPORT_SYMBOL(pmu_usb_enum_started);

/* this function is called by the usb driver once the usb stack init is
 * completed.
 */
void pmu_usb_driver_initialized(void)
{
	pr_info("%s\n", __func__);

	/* start usb enumeration if this function is called for the first
	 * time
	 */
	if (usb_driver_initialized == false) {
		usb_driver_initialized = true;
		pmu_usb_start();
	}
}
EXPORT_SYMBOL(pmu_usb_driver_initialized);

#define PMU_IRQ_GPIO    (34)
static int pmu_event_callback(int event, int param)
{
	int ret = 0;

	switch (event) {

	case PMU_EVENT_INIT_PLATFORM:
	pr_info("%s: PMU: init hardware\n", __func__);

	if (gpio_request(PMU_IRQ_GPIO, "pmu_irq") == 0) {
		gpio_direction_input(PMU_IRQ_GPIO);
		bcm_gpio_pull_up_down_enable(PMU_IRQ_GPIO, true);
		bcm_gpio_pull_up(PMU_IRQ_GPIO, true);

		set_irq_type(GPIO_TO_IRQ(PMU_IRQ_GPIO), IRQF_TRIGGER_FALLING);
	} else {
		pr_err("%s: failed to allocate GPIO for PMU IRQ\n",
			__func__);
	}
	break;

	case PMU_EVENT_CHARGER_INSERT:
	pr_info("%s: PMU_EVENT_CHARGER_INSERT\n", __func__);

	/* try to start the usb enumeration process. this may not succeed if
	 * the usb stack is still not up.
	 */
	if (param == POWER_SUPPLY_TYPE_USB) {
		usb_charger_detected = true;
		pmu_usb_start();
	}
	break;

	case PMU_EVENT_CHARGER_REMOVE:
	pr_info("%s: PMU_EVENT_CHARGER_REMOVE\n", __func__);
	if (param == POWER_SUPPLY_TYPE_USB) {
		usb_charger_detected = false;
		pmu_usb_stop();
	}
	break;

	default:
	pr_err("%s: unrecognized event: %d\n", __func__, event);
	break;

	}
	return ret;
}

struct max8986_platform_data max8986_pdata = {
	.flags = MAX8986_USE_REGULATORS | MAX8986_REGISTER_POWER_OFF |
		MAX8986_ENABLE_AUDIO | MAX8986_USE_PONKEY |
		MAX8986_USE_RTC | MAX8986_USE_POWER | MAX8986_ENABLE_DVS,
	.pmu_event_cb = pmu_event_callback,
	.regulators = &regl_pdata,
	.audio_pdata = &audio_pdata,
	.power = &power_pdata,
	/* CSR Voltage data */
	.csr_nm_volt = CSR_VOUT_1_20V,
	.csr_nm2_volt = CSR_VOUT_1_20V,
	.csr_lpm_volt = CSR_VOUT_0_90V,
};
#endif /*CONFIG_MFD_MAX8986*/

#ifdef CONFIG_BCM_AUXADC
static struct bcm_plat_auxadc adcplat = {
	.readmask = 0x3FF,
	.croff = 0,
	.rdoff = 8,
	.ready = (0x1 << 15),
	.start = (0x1 << 3),
	.auxpm = 0,
	.regoff = (0x1 << 23),
	.bgoff = (0x1 << 22),
};
#endif

#ifdef CONFIG_TOUCHSCREEN_QT602240
#define PEN_IRQ_GPIO   30
static int qt602240_init_platform_hw(void)
{
	gpio_request(PEN_IRQ_GPIO, "qt602240");
	gpio_direction_input(PEN_IRQ_GPIO);
	bcm_gpio_pull_up(PEN_IRQ_GPIO, true);
	bcm_gpio_pull_up_down_enable(PEN_IRQ_GPIO, true);
	set_irq_type(GPIO_TO_IRQ(PEN_IRQ_GPIO), IRQF_TRIGGER_FALLING);
	return 0;
}

static struct qt602240_platform_data qt602240_platform_data = {
	.i2c_pdata = {.i2c_spd = I2C_SPD_400K,},
	.platform_init  = qt602240_init_platform_hw,
	.x_line         = 16,
	.y_line         = 11,
	.x_size         = 1024,
	.y_size         = 1024,
	.x_min		= 90,
	.y_min		= 90,
	.x_max		= 0x3ff,
	.y_max		= 0x3ff,
	.max_area	= 0xff,
	.blen           = 0x21,
	.threshold      = 0x28,
	.voltage        = 2800000,              /* 2.8V */
	.orient         = QT602240_DIAGONAL_COUNTER,
};

#endif

#if defined(CONFIG_BRCM_HEADSET)  || defined(CONFIG_BRCM_HEADSET_MODULE)
static struct brcm_headset_pd headset_pd = {
	.hsirq		= IRQ_MICIN,
	.hsbirq		= IRQ_MICON,
	.check_hs_state = NULL,
};
#endif

#if defined(CONFIG_I2C_BOARDINFO)
static struct i2c_board_info __initdata athenaray_i2c1_board_info[] = {
#ifdef CONFIG_TOUCHSCREEN_QT602240
{
	I2C_BOARD_INFO("qt602240_ts", 0x4b),
	.platform_data = &qt602240_platform_data,
	.irq = GPIO_TO_IRQ(PEN_IRQ_GPIO),
},
#endif
};

static struct i2c_board_info __initdata athenaray_i2c2_board_info[] = {
#if defined(CONFIG_INPUT_BMA150_SMB380)
	{I2C_BOARD_INFO("bma150_accl", 0x38),
	 .platform_data = (void *)&bma_pdata,
	 .irq = GPIO_TO_IRQ(27),
	 },
#endif
};

static struct i2c_board_info __initdata athenaray_i2c3_board_info[] = {
#if defined(CONFIG_MFD_MAX8986)
	{
		I2C_BOARD_INFO("max8986", 0x08),
		.platform_data = (void *)&max8986_pdata,
		.irq = GPIO_TO_IRQ(34),
	},
#endif /*CONFIG_MFD_MAX8986*/
};

#endif

static void athenaray_add_i2c_slaves(void)
{
	i2c_register_board_info(I2C_BSC_ID0, athenaray_i2c1_board_info,
				ARRAY_SIZE(athenaray_i2c1_board_info));
	i2c_register_board_info(I2C_BSC_ID1, athenaray_i2c2_board_info,
				ARRAY_SIZE(athenaray_i2c2_board_info));
	i2c_register_board_info(I2C_BSC_ID2, athenaray_i2c3_board_info,
				ARRAY_SIZE(athenaray_i2c3_board_info));
#if defined (CONFIG_BRCM_HAL_CAM)
        i2c_register_board_info(I2C_BSC_ID1, bcm21553_cam_i2c_board_info,
                                ARRAY_SIZE(bcm21553_cam_i2c_board_info));
#endif
}

#ifdef CONFIG_SPI
static DEFINE_SPINLOCK(bcm_spi_lock);
static int bcm21xx_cs_control(struct driver_data *drv_data, u8 cs)
{
	unsigned long flags;

	if (!drv_data)
		return -EINVAL;

	spin_lock_irqsave(&bcm_spi_lock, flags);

	writeb(SPI_SPIFSSCR_FSSNEW(cs) |
	       SPI_SPIFSSCR_FSSSELNEW, drv_data->ioaddr + SPI_SPIFSSCR);

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

#if defined(CONFIG_FB_BCM)
/*!
 *  * The LCD definition structure.
 *   */

struct platform_device bcm21553_device_fb = {
	.name   = "LCDfb",
	.id     = -1,
	.dev    = {
		.dma_mask               = (u64 *)~(u32)0,
		.coherent_dma_mask      = ~(u32)0,
	},
	.num_resources = 0,
};
#endif

#ifdef CONFIG_BCM215XX_DSS
static struct lcdc_platform_data_t lcdc_pdata = {
	.gpio = 45,
	.te_supported = true,
};
#endif

/* These are active devices on this board */
static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_SERIAL_8250) || defined(CONFIG_SERIAL_8250_MODULE)
	&bcm21553_serial_device0,
	&bcm21553_serial_device1,
	&bcm21553_serial_device2,
#endif
#if defined(CONFIG_DMADEVICES) && defined(CONFIG_OSDAL_SUPPORT)
	&bcm21xx_dma_device,
#endif
#ifdef CONFIG_MMC_BCM
	&bcm21553_sdhc_slot1,
#if !defined(CONFIG_MTD_ONENAND)
	&bcm21553_sdhc_slot2,
#endif
	&bcm21553_sdhc_slot3,
#endif

#if defined(CONFIG_BCM_WATCHDOG)
	&bcm_watchdog_device,
#endif

#ifdef CONFIG_KEYBOARD_BCM
	&bcm215xx_kp_device,
#endif
#ifdef CONFIG_BCM_AUXADC
	&auxadc_device,
#endif
#if defined(CONFIG_BRCM_HEADSET) || defined(CONFIG_BRCM_HEADSET_MODULE)
	&bcm_headset_device,
#endif
#ifdef CONFIG_BCM_PWM
	&bcm_pwm_device,
#ifdef CONFIG_BACKLIGHT_PWM
	&bcm_backlight_devices,
#endif
#endif
#ifdef CONFIG_MTD_ONENAND_BCM_XBOARD
	&athenaray_device_onenand,
#endif
#ifdef CONFIG_I2C
	&bcm21553_device_i2c1,
	&bcm21553_device_i2c2,
	&bcm21553_device_i2c3,
#endif
#ifdef CONFIG_SPI
	&bcm21xx_device_spi,
#endif
#ifdef CONFIG_FB_BCM
	&bcm21553_device_fb,
#endif
#ifdef CONFIG_BCM215XX_DSS
	&bcm215xx_lcdc_device,
#endif
#if defined (CONFIG_BCM_OTP)
	&bcm_otp_device,
#endif
#ifdef CONFIG_BCM_GPIO_VIBRATOR
	&bcm_timed_gpio,
#endif
#ifdef CONFIG_BCM_I2SDAI
    &i2sdai_device,
    &i2s_device,
#endif

};

#ifdef CONFIG_MMC_BCM
int bcmsdhc_ctrl_slot_is_invalid(u8 ctrl_slot)
{
	if (ctrl_slot > 3)
		return -EINVAL;

	return 0;
}
#endif /*CONFIG_MMC_BCM */

static void __init bcm21553_timer_init(void)
{
	struct timer_config bcm21553_timer_config = {
		.cs_base = (void __iomem *)io_p2v(BCM21553_GPTIMER_BASE),
		.ce_base = (void __iomem *)io_p2v(BCM21553_GPTIMER_BASE),
		.cp_cs_base = (void __iomem *)io_p2v(BCM21553_SLPTMR1_BASE),
		.cs_index = 0,
		.ce_index = 1,
		.cp_cs_index = 0,
		.irq = IRQ_GPT1,
	};
	bcm_timer_init(&bcm21553_timer_config);
}

static struct sys_timer bcm21553_timer = {
	.init = bcm21553_timer_init,
};

int board_sysconfig(uint32_t module, uint32_t op)
{
	static DEFINE_SPINLOCK(bcm_syscfg_lock);
	unsigned long flags, syscfg_base, auxmic_base;
	spin_lock_irqsave(&bcm_syscfg_lock, flags);

	syscfg_base = (unsigned long) io_p2v(BCM21553_SYSCFG_BASE);
	auxmic_base = (unsigned long) io_p2v(BCM21553_AUXMIC_BASE);
	switch (module) {
	case SYSCFG_LCD:
		/* Configure IOCR0[29].IOCR0[25] = 10 (GPIO[49:44])*/
		/* Configure IOCR0[29].IOCR0[25] = 00 (LCDCTRL,LCDD[0])*/
		writel((readl(syscfg_base + SYSCFG_IOCR0_OFF) & ~0x22000000),
			syscfg_base + SYSCFG_IOCR0_OFF);
		if (op == SYSCFG_INIT) {
			writel((readl(syscfg_base + SYSCFG_IOCR0_OFF) | 0x20000000),
			syscfg_base + SYSCFG_IOCR0_OFF);
		}
		break;
	case (SYSCFG_PWM0 + 1): /* PWM1 => LCD Backlight */
		if ((op == SYSCFG_INIT) || (op == SYSCFG_DISABLE))
			writel(readl(syscfg_base + 0x00) &
				~0x200, syscfg_base + 0x00);
		else if (op == SYSCFG_ENABLE)
			writel(readl(syscfg_base + 0x00) |
				0x200, syscfg_base + 0x00);

		break;
	 case SYSCFG_CAMERA:
                 if(op == SYSCFG_INIT){
                        u32 val;
                        /* IOCR 0 */
                        val = readl( io_p2v(BCM21553_SYSCFG_BASE) + 0x00) & ~(0x80000000);
                        writel(val, io_p2v(BCM21553_SYSCFG_BASE) + 0x00);
                        /* IOCR 3 */
                        val = readl(io_p2v(BCM21553_SYSCFG_BASE) + 0x1c);
                         /* Clear bits 6,5 and 4 */
                        val &= ~(0x70);
                        val |= 0x40;
                        writel(val, io_p2v(BCM21553_SYSCFG_BASE) + 0x1c);
                        /* IOCR 4*/
                        val = readl(io_p2v(BCM21553_SYSCFG_BASE) + 0x20);
                        /* Bits 14:12 */
                        val |= 0x7000;
                        writel(val, io_p2v(BCM21553_SYSCFG_BASE) + 0x20);
                  } else if(op == SYSCFG_ENABLE){
                        u32 val;
                         /* IOCR 0 */
                        val = readl( io_p2v(BCM21553_SYSCFG_BASE) + 0x00) & ~(0x80000000);
                        writel(val, io_p2v(BCM21553_SYSCFG_BASE) + 0x00);
                        /* IOCR 3 */
                        val = readl(io_p2v(BCM21553_SYSCFG_BASE) + 0x1c);
                        val &= ~(0x7F0);
                        val |= 0x40;
                        writel(val, io_p2v(BCM21553_SYSCFG_BASE) + 0x1c);
                        /* IOCR 5 for trace muxing */
                        val = readl(io_p2v(BCM21553_SYSCFG_BASE) + 0x24);
                        val &= ~(1 << 26);
                        writel(val, io_p2v(BCM21553_SYSCFG_BASE) + 0x24);
                        /* IOCR 4*/
                        val = readl(io_p2v(BCM21553_SYSCFG_BASE) + 0x20);
                        /* Bits 14:12 Cam drive strength */
                        val |= 0x7000;
                        writel(val, io_p2v(BCM21553_SYSCFG_BASE) + 0x20);
                        /* IOCR 6 for cam afe and mode */
                        val = readl(io_p2v(BCM21553_SYSCFG_BASE) + 0x28);
                        val &= ~(1 << 30);
                        val |= (1 << 30); /* CAM 1 interface for now */
                        val &= ~(0x3 << 27); /* Clear */
                        val |= (0x2 << 27);
                        writel(val, io_p2v(BCM21553_SYSCFG_BASE) + 0x28);
                        printk(KERN_INFO"Board sys Done enable 0x%x\n", readl(io_p2v(BCM21553_SYSCFG_BASE) + 0x28));
                  } else if(op == SYSCFG_DISABLE) {
			u32 val;
                          /* IOCR 3 */
                        val = readl(io_p2v(BCM21553_SYSCFG_BASE) + 0x1c);
                        val &= ~(0x10);
                        val |= 0x10; /* Disable CAM outputs */
                        writel(val, io_p2v(BCM21553_SYSCFG_BASE) + 0x1c);
                        /* IOCR 4*/
                        val = readl(io_p2v(BCM21553_SYSCFG_BASE) + 0x20);
                        /* Bits 14:12 Cam drive strength */
                        val &= ~(0x7 << 12);
                        /* Drive strength minimum */
                        val |= (0x1 << 12);
                        writel(val, io_p2v(BCM21553_SYSCFG_BASE) + 0x20);
                        /* IOCR 6 for cam afe and mode */
                        val = readl(io_p2v(BCM21553_SYSCFG_BASE) + 0x28);
                        val &= ~(0x3 << 27); /* Clear */
                        writel(val, io_p2v(BCM21553_SYSCFG_BASE) + 0x28);
                   }
                break;
	case SYSCFG_SDHC1:
		if (op == SYSCFG_INIT) {
			/* Offset for IOCR0 = 0x00 */
			/* SPI_GPIO_MASK = 0x18 */
			writel((readl(syscfg_base + 0x00) &
				~0x00000018),
			       syscfg_base + 0x00);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(syscfg_base + 0x0c) &
			       ~(SD1CMD_PULL_UP | SD1CMD_PULL_DOWN |
				 SD1DAT_PULL_UP | SD1DAT_PULL_DOWN),
			       syscfg_base + 0x0c);
		} else if (op == SYSCFG_ENABLE) {
			/* Offset for IOCR2 = 0x0c */
			writel(readl(syscfg_base + 0x0c)
			       & ~(SD1CMD_PULL_UP | SD1CMD_PULL_DOWN),
			       syscfg_base + 0x0c);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(syscfg_base + 0x0c)
			       | SD1CMD_PULL_UP,
			       syscfg_base + 0x0c);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(syscfg_base + 0x0c)
			       & ~(SD1DAT_PULL_UP | SD1DAT_PULL_DOWN),
			       syscfg_base + 0x0c);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(syscfg_base + 0x0c)
			       | SD1DAT_PULL_UP,
			       syscfg_base + 0x0c);
		} else if (op == SYSCFG_DISABLE) {
			/* Offset for IOCR2 = 0x0c */
			writel(readl(syscfg_base + 0x0c) &
			       ~(SD1CMD_PULL_UP | SD1CMD_PULL_DOWN |
				 SD1DAT_PULL_UP | SD1DAT_PULL_DOWN),
			       syscfg_base + 0x0c);
		}
		break;
	case SYSCFG_SDHC2:
		if (op == SYSCFG_INIT) {
			/* Offset for IOCR0 = 0x00 */
			writel((readl(syscfg_base + 0x00) |
				0x40000000),
			       syscfg_base + 0x00);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(syscfg_base + 0x0c) &
			       ~(SD2CMD_PULL_UP | SD2CMD_PULL_DOWN |
				 SD2DAT_PULL_UP | SD2DAT_PULL_DOWN),
			       syscfg_base + 0x0c);
		} else if (op == SYSCFG_ENABLE) {
			/* Offset for IOCR2 = 0x0c */
			writel(readl(syscfg_base + 0x0c)
			       & ~(SD2CMD_PULL_UP | SD2CMD_PULL_DOWN),
			       syscfg_base + 0x0c);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(syscfg_base + 0x0c)
			       | SD2CMD_PULL_UP,
			       syscfg_base + 0x0c);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(syscfg_base + 0x0c)
			       & ~(SD2DAT_PULL_UP | SD2DAT_PULL_DOWN),
			       syscfg_base + 0x0c);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(syscfg_base + 0x0c)
			       | SD2DAT_PULL_UP,
			       syscfg_base + 0x0c);
		} else if (op == SYSCFG_DISABLE) {
			/* Offset for IOCR2 = 0x0c */
			writel(readl(syscfg_base + 0x0c) &
			       ~(SD2CMD_PULL_UP | SD2CMD_PULL_DOWN |
				 SD2DAT_PULL_UP | SD2DAT_PULL_DOWN),
			       syscfg_base + 0x0c);
		}
		break;
	case SYSCFG_SDHC3:
		if (op == SYSCFG_INIT) {
			/* Offset for IOCR0 = 0x00 */
			/* MSPRO_SD3 = 0x3 */
			writel((readl(syscfg_base + 0x00) &
				~0x00000003),
			       syscfg_base + 0x00);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(syscfg_base + 0x0c) &
			       ~(SD3CMD_PULL_UP | SD3CMD_PULL_DOWN |
				 SD3DAT_PULL_UP | SD3DAT_PULL_DOWN),
			       syscfg_base + 0x0c);
		} else if (op == SYSCFG_ENABLE) {
			/* Offset for IOCR2 = 0x0c */
			writel(readl(syscfg_base + 0x0c)
			       & ~(SD3CMD_PULL_UP | SD3CMD_PULL_DOWN),
			       syscfg_base + 0x0c);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(syscfg_base + 0x0c)
			       | SD3CMD_PULL_UP,
			       syscfg_base + 0x0c);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(syscfg_base + 0x0c)
			       & ~(SD3DAT_PULL_UP | SD3DAT_PULL_DOWN),
			       syscfg_base + 0x0c);
			/* Offset for IOCR2 = 0x0c */
			writel(readl(syscfg_base + 0x0c)
			       | SD3DAT_PULL_UP,
			       syscfg_base + 0x0c);
		} else if (op == SYSCFG_DISABLE) {
			/* Offset for IOCR2 = 0x0c */
			writel(readl(syscfg_base + 0x0c) &
			       ~(SD3CMD_PULL_UP | SD3CMD_PULL_DOWN |
				 SD3DAT_PULL_UP | SD3DAT_PULL_DOWN),
			       syscfg_base + 0x0c);
		}
		break;
#ifdef CONFIG_USB_DWC_OTG
	case SYSCFG_USB:
		if (op == SYSCFG_DISABLE)
			{
				REG_SYS_ANACR9 = (ANACR9_USB_IDLE_ENABLE |
				ANACR9_USB_UTMI_STOP_DIGITAL_CLOCKS |
				ANACR9_USB_UTMI_SOFT_RESET_DISABLE);
				REG_SYS_ANACR9 &= ~(ANACR9_PLL_SUSPEND_ENABLE |
					ANACR9_USB_PLL_POWER_ON);

				REG_CLKPWR_CLK_USB_48_ENABLE &= ~1;
				REG_CLKPWR_CLK_ANALOG_PHASE_ENABLE |= 2; //bit 1
				REG_CLKPWR_CLK_USB_48_ENABLE &= ~1;
				REG_CLKPWR_USBPLL_ENABLE &= ~1;

			}
			else if (op == SYSCFG_ENABLE)
			{
				REG_CLKPWR_CLK_USB_48_ENABLE |= 1;
				REG_CLKPWR_CLK_ANALOG_PHASE_ENABLE &= ~2; //bit 1
				REG_CLKPWR_CLK_USB_48_ENABLE |= 1;
				REG_CLKPWR_USBPLL_ENABLE |= 1;

				REG_SYS_ANACR9 = (
				ANACR9_USB_RESERVED_CLK_DURING_SUSP |
				ANACR9_USB_UTMI_STOP_DIGITAL_CLOCKS |
				ANACR9_USB_UTMI_SOFT_RESET_DISABLE |
				ANACR9_USB_PLL_POWER_ON |
				ANACR9_USB_PLL_CAL_ENABLE |
				ANACR9_USB_SELECT_OTG_MODE |
				ANACR9_USB_SELECT_DEVICE_MODE |
				ANACR9_PLL_SUSPEND_ENABLE |
				ANACR9_USB_PPC_PWR_OFF_ANALOG_DRIVERS);

				/* Reset the PHY since this could
				 * be a mode change too */
				REG_SYS_ANACR9 = (
				ANACR9_USB_RESERVED_CLK_DURING_SUSP |
				ANACR9_USB_UTMI_STOP_DIGITAL_CLOCKS |
				ANACR9_USB_PLL_POWER_ON |
				ANACR9_USB_PLL_CAL_ENABLE |
				ANACR9_USB_SELECT_OTG_MODE |
				ANACR9_USB_SELECT_DEVICE_MODE |
				ANACR9_PLL_SUSPEND_ENABLE |
				ANACR9_USB_PPC_PWR_OFF_ANALOG_DRIVERS);

				/* De-activate PHY reset */
				REG_SYS_ANACR9 = (
				ANACR9_USB_RESERVED_CLK_DURING_SUSP |
				ANACR9_USB_UTMI_STOP_DIGITAL_CLOCKS |
				ANACR9_USB_UTMI_SOFT_RESET_DISABLE |
				ANACR9_USB_PLL_POWER_ON |
				ANACR9_USB_PLL_CAL_ENABLE |
				ANACR9_USB_SELECT_OTG_MODE |
				ANACR9_USB_SELECT_DEVICE_MODE |
				ANACR9_USB_PPC_PWR_OFF_ANALOG_DRIVERS |
				ANACR9_PLL_SUSPEND_ENABLE);
			}
		break;
#endif
	case SYSCFG_HEADSET:
		/* power up analog block for headset */
		if (op == SYSCFG_INIT)
			writel(0x02, syscfg_base + SYSCFG_IRDROP_MON5_OFF);
		break;
	case SYSCFG_AUXMIC:
		if (op == SYSCFG_INIT) {
			writel(0x04, auxmic_base + AUXMIC_PRB_CYC);
			writel(0x01, auxmic_base + AUXMIC_MSR_DLY);
			writel(0x00, auxmic_base + AUXMIC_MSR_INTVL);
			writel(0x00, auxmic_base + AUXMIC_CMC);
			writel(0x01, auxmic_base + AUXMIC_MIC);
			writel(0x00, auxmic_base + AUXMIC_AUXEN);
			writel(13, auxmic_base + AUXMIC_MICINTH_ADJ);
			writel(9, auxmic_base + AUXMIC_MICINENTH_ADJ);
			writel(0x00, auxmic_base + AUXMIC_MICONTH_ADJ);
			writel(0x00, auxmic_base + AUXMIC_MICONENTH_ADJ);
			writel(0x00, auxmic_base + AUXMIC_F_PWRDWN);
		}
		break;
	case SYSCFG_SPI1:
		/* Add system  related information here
		 * May need to configure IOCR here */
		break;
	case SYSCFG_SYSTEMINIT:
		if (op == SYSCFG_INIT) {
			writel(0x0981f800, syscfg_base + 0x00);
			/* SD1/SD2/SD3 DAT[0:3]/CMD lines pulled down
			 * UARTA enable internal pullup */
			writel(0xaa901400, syscfg_base + 0x0c);
			writel(0x88200000, syscfg_base + 0x1c);
			/* UARTB enable internal pullup */
			writel(readl(syscfg_base + 0x24) |
			    0x00004a00, syscfg_base + 0x24);
			/* SD2 DAT[7:4] pulled down */
			writel(readl(syscfg_base + 0x28) |
			    0x00200000, syscfg_base + 0x28);
		}
		break;
#ifdef CONFIG_BRCM_V3D
	case SYSCFG_V3D:
		if (op == SYSCFG_INIT) {
			writel(1, ADDR_SYSCFG_V3DRSTR);
			writel(0, ADDR_SYSCFG_V3DRSTR);
		}
		break;
#endif
	case SYSCFG_VIBRATOR:
		if (op == SYSCFG_INIT) {
			/* Selecting GPIO56 */
			writel(readl(syscfg_base + SYSCFG_IOCR0_OFF)
			| 0x00120000, syscfg_base + SYSCFG_IOCR0_OFF);
		}
		break;

	default:
		pr_info("%s: inval arguments\n", __func__);
		spin_unlock_irqrestore(&bcm_syscfg_lock, flags);
		return -EINVAL;
	}
	spin_unlock_irqrestore(&bcm_syscfg_lock, flags);
	return 0;
}

EXPORT_SYMBOL(board_sysconfig);

static void __init bcm21553_anthenaray_init(void)
{
	/* Configure the SYSCFG Registers */
	board_sysconfig(SYSCFG_SYSTEMINIT, SYSCFG_INIT);
}

static void athenaray_add_platform_data(void)
{
#if defined(CONFIG_SERIAL_8250) || defined(CONFIG_SERIAL_8250_MODULE)
	bcm21553_serial_device0.dev.platform_data = &serial_platform_data0;
	bcm21553_serial_device1.dev.platform_data = &serial_platform_data1;
	bcm21553_serial_device2.dev.platform_data = &serial_platform_data2;
#endif
#ifdef CONFIG_MMC
	bcm21553_sdhc_slot1.dev.platform_data = &bcm21553_sdhc_data1;
#if !defined(CONFIG_MTD_ONENAND)
	bcm21553_sdhc_slot2.dev.platform_data = &bcm21553_sdhc_data2;
#endif
	bcm21553_sdhc_slot3.dev.platform_data = &bcm21553_sdhc_data3;
#endif
#ifdef CONFIG_BCM_PWM
	bcm_pwm_device.dev.platform_data = &pwm_dev;
#ifdef CONFIG_BACKLIGHT_PWM
	bcm_backlight_devices.dev.platform_data = &bcm_backlight_data;
#endif
#endif
#ifdef CONFIG_KEYBOARD_BCM
	bcm215xx_kp_device.dev.platform_data = &bcm215xx_keypad_data;
#endif
#ifdef CONFIG_BCM_AUXADC
	auxadc_device.dev.platform_data = &adcplat;
#endif
#ifdef CONFIG_SPI
	bcm21xx_device_spi.dev.platform_data = &bcm21xx_spi_info;
#endif
#if defined(CONFIG_BRCM_HEADSET)  || defined(CONFIG_BRCM_HEADSET_MODULE)
	bcm_headset_device.dev.platform_data = &headset_pd;
#endif
#ifdef CONFIG_BCM215XX_DSS
	bcm215xx_lcdc_device.dev.platform_data = &lcdc_pdata;
#endif
}

static int __init arch_late_init(void)
{
	pr_info("arch_late_init\n");
#ifdef CONFIG_BCM_GPIO_VIBRATOR
	vibrator_init();
#endif
	return 0;
}
late_initcall(arch_late_init);

static void __init bcm21553_init_machine(void)
{
	bcm21553_platform_init();
	bcm21553_anthenaray_init();
	athenaray_add_i2c_slaves();
	athenaray_add_platform_data();
	platform_add_devices(devices, ARRAY_SIZE(devices));
#ifdef CONFIG_SPI
	/*Function to register SPI board info : required when spi device is
	   present */
	spi_register_board_info(spi_slave_board_info,
				ARRAY_SIZE(spi_slave_board_info));
#endif
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
}

/* TODO: Replace BCM1160 with BCM21553/AthenaRay once registered */
MACHINE_START(BCM1160, "BCM21553 ThunderbirdEDN31 platform")
	/* Maintainer: Broadcom Corporation */
	.phys_io = BCM21553_UART_A_BASE,
	.io_pg_offst = (IO_ADDRESS(BCM21553_UART_A_BASE) >> 18) & 0xfffc,
	.boot_params = (PHYS_OFFSET + 0x100),
	.map_io = bcm21553_map_io,
	.init_irq = bcm21553_init_irq,
	.timer = &bcm21553_timer,
	.init_machine = bcm21553_init_machine,
MACHINE_END

