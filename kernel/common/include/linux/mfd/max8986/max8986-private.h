/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	include/linux/mfd/max8986/max8986-private.h
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
*   @file   max8986-private.h
*
*   @brief  Contains definitions common to all Maxim PMU chips
*
****************************************************************************/

#ifndef __MAX8986_PRIVATE_H__
#define __MAX8986_PRIVATE_H__


#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/regulator/machine.h>
#include <plat/bcm_i2c.h>

/* MAX8986 Regulator IDs */
enum {
	MAX8986_REGL_ALDO1 = 0,
	MAX8986_REGL_ALDO2,
	MAX8986_REGL_ALDO3,
	MAX8986_REGL_ALDO4,
	MAX8986_REGL_ALDO5,
	MAX8986_REGL_ALDO6,
	MAX8986_REGL_ALDO7,
	MAX8986_REGL_ALDO8,
	MAX8986_REGL_ALDO9,

	MAX8986_REGL_DLDO1,
	MAX8986_REGL_DLDO2,
	MAX8986_REGL_DLDO3,
	MAX8986_REGL_DLDO4,

	MAX8986_REGL_HCLDO1,
	MAX8986_REGL_HCLDO2,

	MAX8986_REGL_LVLDO,

	MAX8986_REGL_SIMLDO,

	MAX8986_REGL_AUXLDO1,

	MAX8986_REGL_TSRLDO,

	MAX8986_REGL_CSR,
	MAX8986_REGL_IOSR,

	MAX8986_REGL_NUM_REGULATOR
};

/*ioctl cmds*/
#define BCM_PMU_MAGIC   'P'

#define BCM_PMU_CMD_FIRST               0x80

#define BCM_PMU_CMD_ENABLE_INTS         0x80
#define BCM_PMU_CMD_DISABLE_INTS        0x81
#define BCM_PMU_CMD_READ_REG            0x83
#define BCM_PMU_CMD_WRITE_REG           0x84
#define BCM_PMU_CMD_ACTIVATESIM         0x85
#define BCM_PMU_CMD_DEACTIVATESIM       0x86
#define BCM_PMU_CMD_GET_REGULATOR_STATE 0x87
#define BCM_PMU_CMD_SET_REGULATOR_STATE 0x88
#define BCM_PMU_CMD_SET_PWM_LED_CTRL    0x89
#define BCM_PMU_CMD_POWERONOFF          0x00
#define BCM_PMU_CMD_SET_PWM_HI_PER      0x8a
#define BCM_PMU_CMD_SET_PWM_LO_PER      0x8b
#define BCM_PMU_CMD_SET_PWM_PWR_CTRL    0x8c
#define BCM_PMU_CMD_GET_VOLTAGE         0x8d
#define BCM_PMU_CMD_SET_VOLTAGE         0x8e
#define BCM_PMU_CMD_START_CHARGING	0x8f
#define BCM_PMU_CMD_STOP_CHARGING	0x90
#define BCM_PMU_CMD_SET_CHARGING_CUR	0x91
#define BCM_PMU_CMD_GET_CHARGING_CUR	0x92

#define BCM_PMU_CMD_LAST                0x92

#define BCM_PMU_IOCTL_ENABLE_INTS   \
		_IO(BCM_PMU_MAGIC, BCM_PMU_CMD_ENABLE_INTS)
#define BCM_PMU_IOCTL_DISABLE_INTS  \
		_IO(BCM_PMU_MAGIC, BCM_PMU_CMD_DISABLE_INTS)
#define BCM_PMU_IOCTL_READ_REG      \
		_IOWR(BCM_PMU_MAGIC, BCM_PMU_CMD_READ_REG, pmu_reg)
#define BCM_PMU_IOCTL_WRITE_REG     \
		_IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_WRITE_REG, pmu_reg)
#define BCM_PMU_IOCTL_ACTIVATESIM   \
		_IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_ACTIVATESIM, pmu_sim_volt)
#define BCM_PMU_IOCTL_DEACTIVATESIM \
		_IO(BCM_PMU_MAGIC, BCM_PMU_CMD_DEACTIVATESIM)
#define BCM_PMU_IOCTL_GET_REGULATOR_STATE \
		_IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_GET_REGULATOR_STATE, pmu_regl)
#define BCM_PMU_IOCTL_SET_REGULATOR_STATE \
		_IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_REGULATOR_STATE, pmu_regl)
#define BCM_PMU_IOCTL_GET_VOLTAGE   \
		_IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_GET_VOLTAGE, pmu_regl_volt)
#define BCM_PMU_IOCTL_SET_VOLTAGE   \
		_IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_VOLTAGE, pmu_regl_volt)
#define BCM_PMU_IOCTL_SET_PWM_LED_CTRL \
		_IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_PWM_LED_CTRL, pmu_pwm_ctrl)
#define BCM_PMU_IOCTL_POWERONOFF    \
		_IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_POWERONOFF, pmu_reg)
#define BCM_PMU_IOCTL_SET_PWM_HI_PER \
		_IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_PWM_HI_PER, int)
#define BCM_PMU_IOCTL_SET_PWM_LO_PER \
		_IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_PWM_LO_PER, int)
#define BCM_PMU_IOCTL_SET_PWM_PWR_CTRL \
		_IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_PWM_PWR_CTRL, int)
#define BCM_PMU_IOCTL_GET_CHARGING_CURRENT \
		_IOR(BCM_PMU_MAGIC, BCM_PMU_CMD_GET_CHARGING_CUR, int)
#define BCM_PMU_IOCTL_SET_CHARGING_CURRENT \
		_IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_SET_CHARGING_CUR, int)
#define BCM_PMU_IOCTL_START_CHARGING \
		_IOW(BCM_PMU_MAGIC, BCM_PMU_CMD_START_CHARGING, int)
#define BCM_PMU_IOCTL_STOP_CHARGING \
		_IO(BCM_PMU_MAGIC, BCM_PMU_CMD_STOP_CHARGING)

enum muic_adc_output {
	PMU_MUIC_ADC_OUTPUT_GND,
	PMU_MUIC_ADC_OUTPUT_2K,
	PMU_MUIC_ADC_OUTPUT_2P6K,
	PMU_MUIC_ADC_OUTPUT_3K,
	PMU_MUIC_ADC_OUTPUT_4K,
	PMU_MUIC_ADC_OUTPUT_4P8K,
	PMU_MUIC_ADC_OUTPUT_6K,
	PMU_MUIC_ADC_OUTPUT_8K,
	PMU_MUIC_ADC_OUTPUT_10K,
	PMU_MUIC_ADC_OUTPUT_12K,
	PMU_MUIC_ADC_OUTPUT_14K,
	PMU_MUIC_ADC_OUTPUT_17K,
	PMU_MUIC_ADC_OUTPUT_20K,
	PMU_MUIC_ADC_OUTPUT_24K,
	PMU_MUIC_ADC_OUTPUT_28K,
	PMU_MUIC_ADC_OUTPUT_34K,
	PMU_MUIC_ADC_OUTPUT_40K,
	PMU_MUIC_ADC_OUTPUT_49K,
	PMU_MUIC_ADC_OUTPUT_64K,
	PMU_MUIC_ADC_OUTPUT_80K,
	PMU_MUIC_ADC_OUTPUT_102K,
	PMU_MUIC_ADC_OUTPUT_121K,
	PMU_MUIC_ADC_OUTPUT_150K,
	PMU_MUIC_ADC_OUTPUT_200K,
	PMU_MUIC_ADC_OUTPUT_255K,
	PMU_MUIC_ADC_OUTPUT_301K,
	PMU_MUIC_ADC_OUTPUT_365K,
	PMU_MUIC_ADC_OUTPUT_442K,
	PMU_MUIC_ADC_OUTPUT_523K,
	PMU_MUIC_ADC_OUTPUT_619K,
	PMU_MUIC_ADC_OUTPUT_1M,
	PMU_MUIC_ADC_OUTPUT_OPEN,

	PMU_MUIC_ADC_OUTPUT_INIT
};

enum muic_chgtyp {
	PMU_MUIC_CHGTYP_NONE,
	PMU_MUIC_CHGTYP_USB,
	PMU_MUIC_CHGTYP_DOWNSTREAM_PORT,
	PMU_MUIC_CHGTYP_DEDICATED_CHGR,
	PMU_MUIC_CHGTYP_SPL_500MA,
	PMU_MUIC_CHGTYP_SPL_1A,
	PMU_MUIC_CHGTYP_RESERVED,
	PMU_MUIC_CHGTYP_DEAD_BATT_CHG,

	PMU_MUIC_CHGTYP_INIT
};

/* used to communicate events between the mfd core, power driver and the
 * platform code
 */
enum {
	PMU_EVENT_INIT_PLATFORM,
	PMU_EVENT_CHARGER_INSERT,
	PMU_EVENT_CHARGER_REMOVE,
	PMU_EVENT_BATT_TEMP_BEYOND_LIMIT,
	/*used in case of dedicated charger support to get the charger type*/
	PMU_EVENT_GET_CHARGER_TYPE,
};

typedef int (*pmu_platform_callback)(int event, int param);

typedef struct {
	u32  reg;
	u8  val;
} pmu_reg;

typedef enum {
	SIM_3POINT0VOLT = 0,
	SIM_2POINT5VOLT,
	SIM_3POINT1VOLT,
	SIM_1POINT8VOLT,
	SIM_MAX_VOLTAGE
} pmu_sim_volt;

typedef enum {
	PMU_REGL_ON = 0x0,
	PMU_REGL_ECO, /*0x01 */
	PMU_REGL_OFF, /*0x02 */
	PMU_REGL_MASK
} pmu_regl_state;

typedef struct {
	int regl_id;
	pmu_regl_state state;
} pmu_regl;

typedef struct {
	u32 regoffset;
	u32 pwmled_ctrl ;
	u32 pwmdiv ; /* divider value. fsys/x value. */
} pmu_pwm_ctrl;

typedef struct {
	int regl_id;
	int voltage;
	int min;
	int max;
	int step;
} pmu_regl_volt;

typedef int(*pmu_subdev_ioctl_handler)(u32 cmd, u32 arg, void *pri_data);


enum {
	DEBUG_PMU_INFO = 1U << 0,
	DEBUG_PMU_WARNING = 1U << 1,
	DEBUG_PMU_ERROR = 1U << 2,
};

enum {
	MAX8986_USE_REGULATORS   			=  (1 << 0),
	MAX8986_USE_RTC          			=  (1 << 1),
	MAX8986_USE_POWER        			=  (1 << 2),
	MAX8986_USE_PONKEY 				=  (1 << 3),
	MAX8986_ENABLE_DVS       			=  (1 << 4),
	MAX8986_REGISTER_POWER_OFF			=  (1 << 5),
	MAX8986_ENABLE_AUDIO				=  (1 << 6),
	MAX8986_USE_DEDICATED_USB_CHARGER		=  (1 << 7),
};

/*regualtor DSM settings */
enum {
	MAX8986_REGL_LPM_IN_DSM,   /*if enabled, LPM in DSM (PC1 = 0)*/
	MAX8986_REGL_OFF_IN_DSM, /*if enabled, off in DSM (PC1 = 0)*/
	MAX8986_REGL_ON_IN_DSM ,   /*if enabled, ON in DSM (PC1 = 0)*/
};


/*MAX8986 mfd sub device ids*/
enum {
	MAX8986_SUBDEV_POWER,
	MAX8986_SUBDEV_REGULATOR,
	MAX8986_SUBDEV_MAX
};

/* MAX8986 slave IDs */
enum {
	MAX8986_SLAVE_CORE = 0,
	MAX8986_SLAVE_RTC,
	MAX8986_SLAVE_MUIC,
	MAX8986_SLAVE_AUDIO,
	MAX8986_NUM_SLAVES
};

struct max_pmu_irq {
	struct list_head node;
	void (*handler)(int, void *);
	void *data;
	int irq;
	bool irq_enabled;
};

struct max8986_regl_init_data {
	int regl_id;
	u8 dsm_opmode;
	struct regulator_init_data *init_data;

};

struct max8986_regl_pdata {
	int num_regulators;
	struct max8986_regl_init_data *regl_init;
	/*Regulator PM mode*/
	u8 regl_default_pmmode[MAX8986_REGL_NUM_REGULATOR];
};

struct charger_info {
	u8 charging_cc;
	u8 charging_cv;
};

struct pmu_ioctl_handler {
	pmu_subdev_ioctl_handler handler;
	void *pri_data;
};


struct max8986_power_pdata {
	struct charger_info usb;
	struct charger_info wac;
	u8 eoc_current;

	u8 temp_adc_channel;

	u16 temp_low_limit;
	u16 temp_high_limit;

	u16 batt_min_volt;
	u16 batt_max_volt;

	u16 batt_adc_max;
	u16 batt_adc_min;

	u8 batt_technology;
};

struct max8986_audio_pdata
{
	u8 ina_def_mode;
	u8 inb_def_mode;
	u8 ina_def_preampgain;
	u8 inb_def_preampgain;

	u8 lhs_def_mixer_in;
	u8 rhs_def_mixer_in;
	u8 ihf_def_mixer_in;
};

struct max8986_client {
	struct i2c_client *client;
	u16 addr;
};

struct max8986_platform_data {
	struct i2c_slave_platform_data i2c_pdata;
	int flags;
	pmu_platform_callback pmu_event_cb;
	/* Regulator specific data */
	struct max8986_regl_pdata *regulators;

	/*audio default settings*/
	struct max8986_audio_pdata* audio_pdata;

	/*power data*/
	struct max8986_power_pdata *power;

	/* CSR Voltage data */
	u8 csr_nm_volt;
	u8 csr_nm2_volt;
	u8 csr_lpm_volt;

};

struct max8986 {
	int revision;
	int flags;
	struct max8986_client max8986_cl[MAX8986_NUM_SLAVES];
	/* Single byte write */
	int (*write_dev) (struct max8986 *max8986, u32 command, u8 val);
	/* Single byte read */
	int (*read_dev) (struct max8986 *max8986, u32 command, u8 *regVal);

	int (*write_mul_dev) (struct max8986 *max8986, u32 command,
						u32 length, u8 *val);
	int (*read_mul_dev) (struct max8986 *max8986, u32 command,
						u32 length, u8 *val);

	struct mutex list_lock;
	struct mutex i2c_rw_lock;
	struct mutex muic_int_lock;
	int muic_int_enable;

	int irq;
	struct list_head irq_handlers;
	struct work_struct work;
	struct workqueue_struct *pmu_workqueue;
	struct pmu_ioctl_handler ioctl_handler[MAX8986_SUBDEV_MAX];

	struct max8986_platform_data *pdata;
};

#endif /* __MAX8986_PRIVATE_H__ */
