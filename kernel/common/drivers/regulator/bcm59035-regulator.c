/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*	@file	drivers/regulator/bcm59035-regulator.c
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
 *   @file   bcm59035-regulator.c
 *
 *   @brief  Regulator Driver for Broadcom BCM59035 PMU
 *
 ****************************************************************************/

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/mfd/bcm59035/bcm59035.h>
#include <asm/io.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>

#define MICRO_VOLT(x)	((x)*1000L*1000L)
#define _DEFINE_REGL_VOLT(vol, regVal)	{ MICRO_VOLT(vol), regVal }

#define BCM59035_DEFINE_REGL(_name, _pm_reg, _ctrl_reg, _vol_list)\
	[BCM59035_REGL_##_name] = {\
		.desc = {\
			.name = "BCM59035_REGL_"#_name, \
			.ops = &bcm59035_regulator_ops, \
			.type = REGULATOR_VOLTAGE, \
			.id = BCM59035_REGL_##_name, \
			.owner = THIS_MODULE, \
		}, \
		.ctrl_reg = _ctrl_reg, \
		.pm_reg = _pm_reg, \
		.num_vol = ARRAY_SIZE(_vol_list), \
		.vol_list = _vol_list, \
	}

enum {
	PC2PC1_00 = 0,
	PC2PC1_01 = 2,
	PC2PC1_10 = 4,
	PC2PC1_11 = 6
};

struct bcm59035_regl {
	struct regulator_desc desc;
	u8 ctrl_reg;
	u8 pm_reg;
	u8 num_vol;
	u8 dsm_opmode;
	int const (*vol_list)[2];
};

struct bcm59035_regl_priv {
	struct bcm59035 *bcm59035;
	int num_regl;
	struct regulator_dev *regl[];
};

static const int aldo_vol[][2] = {
	_DEFINE_REGL_VOLT(2.5, 0),	/* 000: 2.5V */
	_DEFINE_REGL_VOLT(2.6, 1),	/* 001: 2.6V */
	_DEFINE_REGL_VOLT(2.7, 2),	/* 010: 2.7V */
	_DEFINE_REGL_VOLT(2.8, 3),	/* 011: 2.8V */
	_DEFINE_REGL_VOLT(2.9, 4),	/* 100: 2.9V */
	_DEFINE_REGL_VOLT(3.2, 5),	/* 101: 3.2V */
	_DEFINE_REGL_VOLT(3.1, 6),	/* 110: 3.1V */
	_DEFINE_REGL_VOLT(3.0, 7)	/* 111: 3.0V */
};

static const int rfldo1_vol[][2] = {
	_DEFINE_REGL_VOLT(2.5, 0),	/* 000: 2.5V */
	_DEFINE_REGL_VOLT(2.6, 1),	/* 001: 2.6V */
	_DEFINE_REGL_VOLT(2.8, 2),	/* 010: 2.8V */
	_DEFINE_REGL_VOLT(2.65, 3),	/* 011: 2.65V */
	_DEFINE_REGL_VOLT(2.9, 4),	/* 100: 2.9V */
	_DEFINE_REGL_VOLT(3.0, 5),	/* 101: 3.0V */
	_DEFINE_REGL_VOLT(3.1, 6),	/* 110: 3.1V */
	_DEFINE_REGL_VOLT(2.7, 7),	/* 111: 2.7V */
};

static const int rfldo2_vol[][2] = {
	_DEFINE_REGL_VOLT(3.2, 0),	/* 000: 3.2 */
	_DEFINE_REGL_VOLT(2.6, 1),	/* 001: 2.6V */
	_DEFINE_REGL_VOLT(2.7, 2),	/* 010: 2.7V */
	_DEFINE_REGL_VOLT(2.8, 3),	/* 011: 2.8V */
	_DEFINE_REGL_VOLT(2.9, 4),	/* 100: 2.9V */
	_DEFINE_REGL_VOLT(3.0, 5),	/* 101: 3.0V */
	_DEFINE_REGL_VOLT(3.1, 6),	/* 110: 3.1V */
	_DEFINE_REGL_VOLT(2.5, 7),	/* 111: 2.5V */
};

static const int lcldo_vol[][2] = {
	_DEFINE_REGL_VOLT(1.3, 1),	/* 00 001: 1.3V */
	_DEFINE_REGL_VOLT(1.4, 2),	/* 00 010: 1.4V */
	_DEFINE_REGL_VOLT(1.5, 3),	/* 00 011: 1.5V */
	_DEFINE_REGL_VOLT(1.6, 4),	/* 00 100: 1.6V */
	_DEFINE_REGL_VOLT(1.7, 5),	/* 00 101: 1.7V */
	_DEFINE_REGL_VOLT(1.8, 6),	/* 00 110: 1.8V */
	_DEFINE_REGL_VOLT(1.9, 7),	/* 00 111: 1.9V */
	_DEFINE_REGL_VOLT(2, 8),	/* 01 000: 2.0V */
	_DEFINE_REGL_VOLT(2.1, 9),	/* 01 001: 2.1V */
	_DEFINE_REGL_VOLT(2.2, 0xA),	/* 01 010: 2.2V */
	_DEFINE_REGL_VOLT(2.3, 0xB),	/* 01 011: 2.3V */
	_DEFINE_REGL_VOLT(2.4, 0xC),	/* 01 100: 2.4V */
	_DEFINE_REGL_VOLT(2.5, 0xD),	/* 01 101: 2.5V */
	_DEFINE_REGL_VOLT(2.6, 0xE),	/* 01 110: 2.6V */
	_DEFINE_REGL_VOLT(2.8, 0x10),	/* 10 000: 2.8V */
	_DEFINE_REGL_VOLT(2.9, 0x11),	/* 10 001: 2.9V */
	_DEFINE_REGL_VOLT(3, 0x12),	/* 10 010: 3.0V */
	_DEFINE_REGL_VOLT(3.1, 0x13),	/* 10 011: 3.1V */
	_DEFINE_REGL_VOLT(1.25, 0x14),	/* 10 100: 1.25V */
	_DEFINE_REGL_VOLT(2.7, 0x1F),	/* 11 111: 2.7V */
};

static const int lvldo1_vol[][2] = {
	_DEFINE_REGL_VOLT(1.2, 0),	/* 0000: 1.20V */
	_DEFINE_REGL_VOLT(1.225, 1),	/* 0001: 1.225V */
	_DEFINE_REGL_VOLT(1.25, 2),	/* 0010: 1.250V */
	_DEFINE_REGL_VOLT(1.275, 3),	/* 0011: 1.275V */
	_DEFINE_REGL_VOLT(1.3, 4),	/* 0100: 1.30V */
	_DEFINE_REGL_VOLT(1.325, 5),	/* 0101: 1.325V */
	_DEFINE_REGL_VOLT(1.35, 6),	/* 0110: 1.35V */
	_DEFINE_REGL_VOLT(1.375, 7),	/* 0111: 1.375 */
	_DEFINE_REGL_VOLT(1.4, 8),	/* 1000: 1.40V */
	_DEFINE_REGL_VOLT(1.5, 0xF),	/* 1111: 1.50V */
};

static const int lvldo2_vol[][2] = {
	_DEFINE_REGL_VOLT(1.2, 0),	/* 0000: 1.2V */
	_DEFINE_REGL_VOLT(1.225, 1),	/* 0001: 1.225V */
	_DEFINE_REGL_VOLT(1.25, 2),	/* 0010: 1.250V */
	_DEFINE_REGL_VOLT(1.275, 3),	/* 0011: 1.275V */
	_DEFINE_REGL_VOLT(1.325, 4),	/* 0100: 1.325V */
	_DEFINE_REGL_VOLT(1.35, 5),	/* 0101: 1.350V */
	_DEFINE_REGL_VOLT(1.375, 6),	/* 0110: 1.375V */
	_DEFINE_REGL_VOLT(1.4, 7),	/* 0111: 1.40V */
	_DEFINE_REGL_VOLT(1.5, 8),	/* 1000: 1.50V */
	_DEFINE_REGL_VOLT(1.3, 0xF),	/* 1111: 1.30V */
};

/* hcldo & aldo have same voltage values */
#define hcldo_vol aldo_vol

static const int ioldo_vol[][2] = {
	_DEFINE_REGL_VOLT(1.3, 1),	/* 00 001: 1.3V */
	_DEFINE_REGL_VOLT(1.4, 2),	/* 00 010: 1.4V */
	_DEFINE_REGL_VOLT(1.5, 3),	/* 00 011: 1.5V */
	_DEFINE_REGL_VOLT(1.6, 4),	/* 00 100: 1.6V */
	_DEFINE_REGL_VOLT(1.7, 5),	/* 00 101: 1.7V */
	_DEFINE_REGL_VOLT(1.8, 6),	/* 00 110: 1.8V */
	_DEFINE_REGL_VOLT(1.9, 7),	/* 00 111: 1.9V */
	_DEFINE_REGL_VOLT(2, 8),	/* 01 000: 2.0V */
	_DEFINE_REGL_VOLT(2.1, 9),	/* 01 001: 2.1V */
	_DEFINE_REGL_VOLT(2.2, 0xA),	/* 01 010: 2.2V */
	_DEFINE_REGL_VOLT(2.3, 0xB),	/* 01 011: 2.3V */
	_DEFINE_REGL_VOLT(2.4, 0xC),	/* 01 100: 2.4V */
	_DEFINE_REGL_VOLT(2.5, 0xD),	/* 01 101: 2.5V */
	_DEFINE_REGL_VOLT(2.6, 0xE),	/* 01 110: 2.6V */
	_DEFINE_REGL_VOLT(2.7, 0xF),	/* 01 111: 2.7V */
	_DEFINE_REGL_VOLT(2.8, 0x10),	/* 10 000: 2.8V */
	_DEFINE_REGL_VOLT(2.9, 0x11),	/* 10 001: 2.9V */
	_DEFINE_REGL_VOLT(3.1, 0x13),	/* 10 011: 3.1V */
	_DEFINE_REGL_VOLT(3.2, 0x14),	/* 10 100: 3.2V */
	_DEFINE_REGL_VOLT(3, 0x1F)	/* 11 111: 3.0V */
};

static const int msldo1_vol[][2] = {
	_DEFINE_REGL_VOLT(1.3, 1),	/* 00 001: 1.3V */
	_DEFINE_REGL_VOLT(1.4, 2),	/* 00 010: 1.4V */
	_DEFINE_REGL_VOLT(1.5, 3),	/* 00 011: 1.5V */
	_DEFINE_REGL_VOLT(1.6, 4),	/* 00 100: 1.6V */
	_DEFINE_REGL_VOLT(1.7, 5),	/* 00 101: 1.7V */
	_DEFINE_REGL_VOLT(1.8, 6),	/* 00 110: 1.8V */
	_DEFINE_REGL_VOLT(1.9, 7),	/* 00 111: 1.9V */
	_DEFINE_REGL_VOLT(2.0, 8),	/* 01 000: 2.0V */
	_DEFINE_REGL_VOLT(2.1, 9),	/* 01 001: 2.1V */
	_DEFINE_REGL_VOLT(2.3, 0xA),	/* 01 010: 2.2V */
	_DEFINE_REGL_VOLT(2.3, 0xB),	/* 01 011: 2.3V */
	_DEFINE_REGL_VOLT(2.4, 0xC),	/* 01 100: 2.4V */
	_DEFINE_REGL_VOLT(2.5, 0xD),	/* 01 101: 2.5V */
	_DEFINE_REGL_VOLT(2.6, 0xE),	/* 01 110: 2.6V */
	_DEFINE_REGL_VOLT(2.7, 0xF),	/* 01 111: 2.7V */
	_DEFINE_REGL_VOLT(2.9, 0x11),	/* 10 001: 2.9V */
	_DEFINE_REGL_VOLT(3.0, 0x12),	/* 10 010: 3.0V */
	_DEFINE_REGL_VOLT(3.1, 0x13),	/* 10 011: 3.1V */
	_DEFINE_REGL_VOLT(3.2, 0x14),	/* 10 100: 3.2V */
	_DEFINE_REGL_VOLT(2.8, 0x1F),	/* 11 111: 2.8V */
};

static const int msldo2_vol[][2] = {
	_DEFINE_REGL_VOLT(2.8, 0),	/* 000: 2.8V */
	_DEFINE_REGL_VOLT(2.9, 1),	/* 001: 2.9V */
	_DEFINE_REGL_VOLT(3.0, 2),	/* 010: 3.0V */
	_DEFINE_REGL_VOLT(3.1, 3),	/* 011: 3.1V */
	_DEFINE_REGL_VOLT(3.2, 4),	/* 100: 3.2V */
	_DEFINE_REGL_VOLT(3.3, 5),	/* 111: 3.3V */
};

/* msldo1 & auxldo1 have same voltage list */
#define auxldo1_vol msldo1_vol

static const int auxldo2_vol[][2] = {
	_DEFINE_REGL_VOLT(1.3, 1),	/* 00 001: 1.3V */
	_DEFINE_REGL_VOLT(1.4, 2),	/* 00 010: 1.4V */
	_DEFINE_REGL_VOLT(1.5, 3),	/* 00 011: 1.5V */
	_DEFINE_REGL_VOLT(1.6, 4),	/* 00 100: 1.6V */
	_DEFINE_REGL_VOLT(1.7, 5),	/* 00 101: 1.7V */
	_DEFINE_REGL_VOLT(1.9, 7),	/* 00 111: 1.9V */
	_DEFINE_REGL_VOLT(2.0, 8),	/* 01 000: 2.0V */
	_DEFINE_REGL_VOLT(2.1, 9),	/* 01 001: 2.1V */
	_DEFINE_REGL_VOLT(2.2, 0xA),	/* 01 010: 2.2V */
	_DEFINE_REGL_VOLT(2.3, 0xB),	/* 01 011: 2.3V */
	_DEFINE_REGL_VOLT(2.4, 0xC),	/* 01 100: 2.4V */
	_DEFINE_REGL_VOLT(2.5, 0xD),	/* 01 101: 2.5V */
	_DEFINE_REGL_VOLT(2.6, 0xE),	/* 01 110: 2.6V */
	_DEFINE_REGL_VOLT(2.7, 0xF),	/* 01 111: 2.7V */
	_DEFINE_REGL_VOLT(2.8, 0x10),	/* 10 000: 2.8V */
	_DEFINE_REGL_VOLT(2.9, 0x11),	/* 10 001: 2.9V */
	_DEFINE_REGL_VOLT(3.0, 0x12),	/* 10 010: 3.0V */
	_DEFINE_REGL_VOLT(3.1, 0x13),	/* 10 011: 3.1V */
	_DEFINE_REGL_VOLT(3.2, 0x14),	/* 10 100: 3.2V */
	_DEFINE_REGL_VOLT(1.8, 0x1F),	/* 11 111: 1.8V */
};

static const int simldo_vol[][2] = {
	_DEFINE_REGL_VOLT(3.0, 0),	/* 00: 3.0V */
	_DEFINE_REGL_VOLT(2.5, 1),	/* 01: 2.5V */
	_DEFINE_REGL_VOLT(3.1, 2),	/* 10: 3.1V */
	_DEFINE_REGL_VOLT(1.8, 3),	/* 11: 1.8V */
};

static const int csr_dvs_vol[][2] = {
	_DEFINE_REGL_VOLT(1.50, 0),
	_DEFINE_REGL_VOLT(1.48, 1),
	_DEFINE_REGL_VOLT(1.46, 2),
	_DEFINE_REGL_VOLT(1.44, 3),
	_DEFINE_REGL_VOLT(1.42, 4),
	_DEFINE_REGL_VOLT(1.40, 5),
	_DEFINE_REGL_VOLT(1.38, 6),
	_DEFINE_REGL_VOLT(1.36, 7),
	_DEFINE_REGL_VOLT(1.34, 8),
	_DEFINE_REGL_VOLT(1.32, 9),
	_DEFINE_REGL_VOLT(1.30, 0xA),
	_DEFINE_REGL_VOLT(1.28, 0xB),
	_DEFINE_REGL_VOLT(1.26, 0xC),
	_DEFINE_REGL_VOLT(1.24, 0xD),
	_DEFINE_REGL_VOLT(1.22, 0xE),
	_DEFINE_REGL_VOLT(1.20, 0xF),
	_DEFINE_REGL_VOLT(1.18, 0x10),
	_DEFINE_REGL_VOLT(1.16, 0x11),
	_DEFINE_REGL_VOLT(1.14, 0x12),
	_DEFINE_REGL_VOLT(1.12, 0x13),
	_DEFINE_REGL_VOLT(1.10, 0x14),
	_DEFINE_REGL_VOLT(1.08, 0x15),
	_DEFINE_REGL_VOLT(1.06, 0x16),
	_DEFINE_REGL_VOLT(1.04, 0x17),
	_DEFINE_REGL_VOLT(1.02, 0x18),
	_DEFINE_REGL_VOLT(1.00, 0x19),
	_DEFINE_REGL_VOLT(0.98, 0x1A),
	_DEFINE_REGL_VOLT(0.96, 0x1B),
	_DEFINE_REGL_VOLT(0.94, 0x1C),
	_DEFINE_REGL_VOLT(0.92, 0x1D),
	_DEFINE_REGL_VOLT(0.90, 0x1E)
};

#define csr_no_dvs_vol iosr_vol	/*CSR no dvs & IOSR has same set of voltage values */

static const int iosr_vol[][2] = {
	_DEFINE_REGL_VOLT(0.9, 0x10),
	_DEFINE_REGL_VOLT(1.0, 0xF),
	_DEFINE_REGL_VOLT(1.1, 0xE),
	_DEFINE_REGL_VOLT(1.2, 0xD),
	_DEFINE_REGL_VOLT(1.3, 0xC),
	_DEFINE_REGL_VOLT(1.4, 0xB),
	_DEFINE_REGL_VOLT(1.5, 0xA),
	_DEFINE_REGL_VOLT(1.6, 0x9),
	_DEFINE_REGL_VOLT(1.7, 0x8),
	_DEFINE_REGL_VOLT(1.8, 0x7),
	_DEFINE_REGL_VOLT(1.9, 0x6),
	_DEFINE_REGL_VOLT(2.0, 0x5),
	_DEFINE_REGL_VOLT(2.1, 0x4),
	_DEFINE_REGL_VOLT(2.2, 0x3),
	_DEFINE_REGL_VOLT(2.3, 0x2),
	_DEFINE_REGL_VOLT(2.4, 0x1),
	_DEFINE_REGL_VOLT(2.5, 0x0)
};

static struct regulator_ops bcm59035_regulator_ops;

static struct bcm59035_regl bcm59035_regls[] = {

	BCM59035_DEFINE_REGL(ALDO1, BCM59035_REG_A1OPMODCTRL,
			     BCM59035_REG_ALDOCTRL, aldo_vol),
	BCM59035_DEFINE_REGL(ALDO2, BCM59035_REG_A2OPMODCTRL,
			     BCM59035_REG_ALDOCTRL, aldo_vol),

	BCM59035_DEFINE_REGL(RFLDO1, BCM59035_REG_R1OPMODCTRL,
			     BCM59035_REG_RFLDOCTRL, rfldo1_vol),
	BCM59035_DEFINE_REGL(RFLDO2, BCM59035_REG_R2OPMODCTRL,
			     BCM59035_REG_RFLDOCTRL, rfldo2_vol),

	BCM59035_DEFINE_REGL(LCLDO, BCM59035_REG_LOPMODCTRL,
			     BCM59035_REG_LCSIMDOCTRL, lcldo_vol),

	BCM59035_DEFINE_REGL(LVLDO1, BCM59035_REG_LV1OPMODCTRL,
			     BCM59035_REG_LVLDOCTRL, lvldo1_vol),
	BCM59035_DEFINE_REGL(LVLDO2, BCM59035_REG_LV2OPMODCTRL,
			     BCM59035_REG_LVLDOCTRL, lvldo2_vol),

	BCM59035_DEFINE_REGL(HCLDO1, BCM59035_REG_H1OPMODCTRL,
			     BCM59035_REG_HCLDOCTRL, hcldo_vol),
	BCM59035_DEFINE_REGL(HCLDO2, BCM59035_REG_H2OPMODCTRL,
			     BCM59035_REG_HCLDOCTRL, hcldo_vol),

	BCM59035_DEFINE_REGL(IOLDO, BCM59035_REG_IOPMODCTRL,
			     BCM59035_REG_IOLDOCTRL, ioldo_vol),

	BCM59035_DEFINE_REGL(MSLDO1, BCM59035_REG_M1OPMODCTRL,
			     BCM59035_REG_MSLDOCTRL, msldo1_vol),
	BCM59035_DEFINE_REGL(MSLDO2, BCM59035_REG_M2OPMODCTRL,
			     BCM59035_REG_MSLDOCTRL, msldo2_vol),

	BCM59035_DEFINE_REGL(AUXLDO1, BCM59035_REG_AX1OPMODCTRL,
			     BCM59035_REG_AX1LDOCTRL, auxldo1_vol),
	BCM59035_DEFINE_REGL(AUXLDO2, BCM59035_REG_AX2OPMODCTRL,
			     BCM59035_REG_AX2LDOCTRL, auxldo2_vol),

	BCM59035_DEFINE_REGL(SIMLDO, BCM59035_REG_SOPMODCTRL,
			     BCM59035_REG_LCSIMDOCTRL, simldo_vol),

	BCM59035_DEFINE_REGL(CSR, BCM59035_REG_CSROPMODCTRL, BCM59035_REG_CSRCTRL10, csr_dvs_vol),	/* set DVS en values by default */
	BCM59035_DEFINE_REGL(IOSR, BCM59035_REG_IOSROPMODCTRL,
			     BCM59035_REG_IOSRCTRL2, iosr_vol),
};

static u8 bcm59035_regulator_ctrl_reg_mask(int regl_id, int *bitPos)
{
	u8 mask = 0;
	switch (regl_id) {
	case BCM59035_REGL_RFLDO1:
	case BCM59035_REGL_ALDO1:
	case BCM59035_REGL_HCLDO1:
		*bitPos = 0;
		mask = 0x7;
		break;

	case BCM59035_REGL_RFLDO2:
	case BCM59035_REGL_ALDO2:
	case BCM59035_REGL_HCLDO2:
		*bitPos = 4;
		mask = 0x7;
		break;

	case BCM59035_REGL_AUXLDO1:
	case BCM59035_REGL_AUXLDO2:
	case BCM59035_REGL_LCLDO:
	case BCM59035_REGL_MSLDO1:
	case BCM59035_REGL_IOLDO:
		*bitPos = 0;
		mask = 0x1F;
		break;

	case BCM59035_REGL_MSLDO2:
		*bitPos = 5;
		mask = 0x7;
		break;

	case BCM59035_REGL_SIMLDO:
		*bitPos = 6;
		mask = 0x3;
		break;

	case BCM59035_REGL_LVLDO1:
		*bitPos = 0;
		mask = 0xF;
		break;

	case BCM59035_REGL_LVLDO2:
		*bitPos = 4;
		mask = 0xF;
		break;

	case BCM59035_REGL_CSR:
		mask = 0x1F;
		if (bcm59035_regls[BCM59035_REGL_CSR].vol_list == csr_no_dvs_vol)	/*No DVS ..uses CSRSTRL2 reg */
			*bitPos = 2;
		else
			*bitPos = 0;
		break;

	case BCM59035_REGL_IOSR:
		*bitPos = 2;
		mask = 0x1F;
		break;

	default:
		return 0;

	}
	mask <<= *bitPos;
	return mask;
}

static int _bcm59035_regulator_enable(struct bcm59035_regl_priv *regl_priv,
				      int id)
{
	struct bcm59035 *bcm59035 = regl_priv->bcm59035;
	int ret;
	u8 regVal;
	ret = bcm59035->read_dev(bcm59035, bcm59035_regls[id].pm_reg, &regVal);
	/*00: ON (normal) 01: Low power mode 10: OFF */
	/*Normal mode : PC1 = 1 */
	regVal &= ~((PMU_REGL_MASK << PC2PC1_01) | (PMU_REGL_MASK << PC2PC1_11)); /* clear normal mode bits*/
	switch(bcm59035_regls[id].dsm_opmode)
	{
		case BCM59035_REGL_OFF_IN_DSM:
			regVal &= ~((PMU_REGL_MASK << PC2PC1_00) | (PMU_REGL_MASK << PC2PC1_10)); /* clear LPM bits */
			regVal |= ((PMU_REGL_OFF << PC2PC1_00) | (PMU_REGL_OFF << PC2PC1_10)); /* set to off in DSM */
			break;

		case BCM59035_REGL_LPM_IN_DSM:
			regVal &= ~((PMU_REGL_MASK << PC2PC1_00) | (PMU_REGL_MASK << PC2PC1_10)); /* clear LPM bits */
			regVal |= ((PMU_REGL_ECO << PC2PC1_00) | (PMU_REGL_ECO << PC2PC1_10)); /* set to LPM in DSM */
			break;

		case BCM59035_REGL_ON_IN_DSM:
			regVal &= ~((PMU_REGL_MASK << PC2PC1_00) | (PMU_REGL_MASK << PC2PC1_10)); /* clear LPM bits - on in DSM */
			break;
	}

	ret = bcm59035->write_dev(bcm59035, bcm59035_regls[id].pm_reg, regVal);
	return ret;
}

static int bcm59035_regulator_enable(struct regulator_dev *rdev)
{
	struct bcm59035_regl_priv *regl_priv = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	return _bcm59035_regulator_enable(regl_priv, id);
}

static int _bcm59035_regulator_disable(struct bcm59035_regl_priv *regl_priv,
				       int id)
{
	struct bcm59035 *bcm59035 = regl_priv->bcm59035;
	int ret;
	u8 regVal;
	/*00: ON (normal) 01: Low power mode 10: OFF */
	/*Normal mode : PC1 = 1 */
	regVal = ((PMU_REGL_OFF << PC2PC1_00) | (PMU_REGL_OFF << PC2PC1_10) |
			  (PMU_REGL_OFF << PC2PC1_01) | (PMU_REGL_OFF << PC2PC1_11)); /* set to off in all modes */
	ret = bcm59035->write_dev(bcm59035, bcm59035_regls[id].pm_reg, regVal);
	return ret;
}


static int bcm59035_regulator_disable(struct regulator_dev *rdev)
{
	struct bcm59035_regl_priv *regl_priv = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);

	return _bcm59035_regulator_disable(regl_priv, id);
}

static int _bcm59035_regulator_is_enabled(struct bcm59035_regl_priv *regl_priv,
					  int id)
{
	struct bcm59035 *bcm59035 = regl_priv->bcm59035;
	int ret;
	u8 regVal;

	ret = bcm59035->read_dev(bcm59035, bcm59035_regls[id].pm_reg, &regVal);

	if (ret)
		return ret;
	/*00: ON (normal) 01: Low power mode 10: OFF */
	return (regVal & (PMU_REGL_MASK << PC2PC1_01)) == 0;
}

static int bcm59035_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct bcm59035_regl_priv *regl_priv = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	return _bcm59035_regulator_is_enabled(regl_priv, id);
}

static int bcm59035_get_best_voltage_inx(int reg_id, int min_uV, int max_uV)
{
	/* int reg_id = rdev_get_id(rdev); */
	int i;
	int bestmatch;
	int bestindex;

	/*
	 * Locate the minimum voltage fitting the criteria on
	 * this regulator. The switchable voltages are not
	 * in strict falling order so we need to check them
	 * all for the best match.
	 */
	if ((reg_id < 0) || (reg_id > BCM59035_REGL_NUM_REGULATOR))
		return -EINVAL;

	bestmatch = INT_MAX;
	bestindex = -1;
	for (i = 0; i < bcm59035_regls[reg_id].num_vol; i++) {
		if (bcm59035_regls[reg_id].vol_list[i][0] >= min_uV &&
		    bcm59035_regls[reg_id].vol_list[i][0] < bestmatch) {
			bestmatch = bcm59035_regls[reg_id].vol_list[i][0];
			bestindex = i;
		}
	}

	if (bestindex < 0 || bestmatch > max_uV) {
		PMU_LOG(DEBUG_PMU_WARNING,
			"%s: no possible values for min_uV = %d & max_uV = %d\n",
			__FUNCTION__, min_uV, max_uV);
		return -EINVAL;
	}
	return bestindex;
}

static int _bcm59035_regulator_set_voltage(struct bcm59035_regl_priv *pri_dev,
					   int id, int volt)
{
	struct bcm59035_regl_priv *regl_priv = pri_dev;
	struct bcm59035 *bcm59035 = regl_priv->bcm59035;
	int bitPos, ret;
	u8 mask, regVal;

	if ((id < 0) || (id >= BCM59035_REGL_NUM_REGULATOR))
		return -EINVAL;
	mask = bcm59035_regulator_ctrl_reg_mask(id, &bitPos);
	if (!mask)
		return -EINVAL;
	ret =
	    bcm59035->read_dev(bcm59035, bcm59035_regls[id].ctrl_reg, &regVal);
	regVal &= ~(mask);
	volt <<= bitPos;
	regVal |= (volt & mask);
	ret =
	    bcm59035->write_dev(bcm59035, bcm59035_regls[id].ctrl_reg, regVal);
	return ret;
}

static int bcm59035_regulator_set_voltage(struct regulator_dev *rdev,
					  int min_uV, int max_uV)
{
	struct bcm59035_regl_priv *regl_priv = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int inx;
	u8 value;

	/* Find the best index */
	inx = bcm59035_get_best_voltage_inx(id, min_uV, max_uV);

	if (inx < 0)
		return inx;
	value = bcm59035_regls[id].vol_list[inx][1];

	return _bcm59035_regulator_set_voltage(regl_priv, id, value);
}

static int _bcm59035_regulator_get_voltage(struct bcm59035_regl_priv *pri_dev,
					   int id)
{
	int ret, val;
	u8 mask, regVal;
	int bitPos, i;
	struct bcm59035_regl_priv *regl_priv = pri_dev;
	struct bcm59035 *bcm59035 = regl_priv->bcm59035;

	if ((id < 0) || (id >= BCM59035_REGL_NUM_REGULATOR))
		return -EINVAL;

	ret =
	    bcm59035->read_dev(bcm59035, bcm59035_regls[id].ctrl_reg, &regVal);
	if (ret)
		return ret;
	mask = bcm59035_regulator_ctrl_reg_mask(id, &bitPos);
	val = (regVal & mask) >> bitPos;

	for (i = 0; i < bcm59035_regls[id].num_vol; i++) {
		if (val == bcm59035_regls[id].vol_list[i][1]) {
			PMU_LOG(DEBUG_PMU_INFO, "%s id: %d val: %d\n", __func__,
				id, val);
			return bcm59035_regls[id].vol_list[i][0];
		}
	}
	return -EINVAL;
}

static int bcm59035_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct bcm59035_regl_priv *regl_priv = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);

	return _bcm59035_regulator_get_voltage(regl_priv, id);
}

static int _bcm59035_regulator_set_mode(struct bcm59035_regl_priv *pri_dev,
					int id, pmu_regl_state mode)
{
	struct bcm59035_regl_priv *regl_priv = pri_dev;
	struct bcm59035 *bcm59035 = regl_priv->bcm59035;
	u8 regVal;
	int ret;

	if ((id < 0) || (id >= BCM59035_REGL_NUM_REGULATOR))
		return -EINVAL;
	if (mode < 0 || mode >= PMU_REGL_MASK)
		return -EINVAL;
	ret = bcm59035->read_dev(bcm59035, bcm59035_regls[id].pm_reg, &regVal);
	if (ret)
		return ret;
	/* Clear it off and then set the passed mode. */
	regVal &=
	    ~((PMU_REGL_MASK << PC2PC1_01) | (PMU_REGL_MASK << PC2PC1_11));
	regVal |= ((mode << PC2PC1_01) | (mode << PC2PC1_11));
	ret = bcm59035->write_dev(bcm59035, bcm59035_regls[id].pm_reg, regVal);
	return ret;
}

static int bcm59035_regulator_set_mode(struct regulator_dev *rdev,
				       unsigned int mode)
{
	struct bcm59035_regl_priv *regl_priv = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);

	return _bcm59035_regulator_set_mode(regl_priv, id, mode);

}

static int _bcm59035_regulator_get_mode(struct bcm59035_regl_priv *pri_dev,
					int id)
{
	struct bcm59035_regl_priv *regl_priv = pri_dev;
	struct bcm59035 *bcm59035 = regl_priv->bcm59035;

	u8 regVal = 0;
	int ret;

	if ((id < 0) || (id >= BCM59035_REGL_NUM_REGULATOR))
		return -EINVAL;

	ret = bcm59035->read_dev(bcm59035, bcm59035_regls[id].pm_reg, &regVal);
	pr_info("%s : regVal = 0x%x\n", __FUNCTION__, regVal);
	if (ret)
		return ret;
	regVal &= (PMU_REGL_MASK << PC2PC1_01);
	regVal >>= PC2PC1_01;
	return regVal;
}

static unsigned int bcm59035_regulator_get_mode(struct regulator_dev *rdev)
{
	struct bcm59035_regl_priv *regl_priv = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);

	if ((id < 0) || (id >= BCM59035_REGL_NUM_REGULATOR))
		return -EINVAL;
	return _bcm59035_regulator_get_mode(regl_priv, id);
}

static struct regulator_ops bcm59035_regulator_ops = {
	.enable = bcm59035_regulator_enable,
	.disable = bcm59035_regulator_disable,
	.is_enabled = bcm59035_regulator_is_enabled,
	.set_voltage = bcm59035_regulator_set_voltage,
	.get_voltage = bcm59035_regulator_get_voltage,
	.set_mode = bcm59035_regulator_set_mode,
	.get_mode = bcm59035_regulator_get_mode,
};

static int bcm59035_regulator_ioctl_handler(u32 cmd, u32 arg, void *pri_data)
{
	struct bcm59035_regl_priv *regl_priv =
	    (struct bcm59035_regl_priv *)pri_data;
	int ret = -EINVAL;
	PMU_LOG(DEBUG_PMU_INFO, "Inside %s, IOCTL command is %d\n",
		__FUNCTION__, cmd);
	switch (cmd) {
	case BCM_PMU_IOCTL_SET_VOLTAGE:
		{
			pmu_regl_volt regulator;
			int inx;
			u8 value;
			if (copy_from_user
			    (&regulator, (pmu_regl_volt *) arg,
			     sizeof(pmu_regl_volt)) != 0) {
				return -EFAULT;
			}
			/* Validate the input voltage */
			inx =
			    bcm59035_get_best_voltage_inx(regulator.regl_id,
							  regulator.voltage,
							  regulator.voltage);
			if (inx < 0)
				return inx;
			value =
			    bcm59035_regls[regulator.regl_id].vol_list[inx][1];
			ret =
			    _bcm59035_regulator_set_voltage(regl_priv,
							    regulator.regl_id,
							    value);
			break;
		}
	case BCM_PMU_IOCTL_GET_VOLTAGE:
		{
			pmu_regl_volt regulator;
			int volt;
			if (copy_from_user
			    (&regulator, (pmu_regl_volt *) arg,
			     sizeof(pmu_regl_volt)) != 0) {
				return -EFAULT;
			}
			volt =
			    _bcm59035_regulator_get_voltage(regl_priv,
							    regulator.regl_id);
			if (volt > 0) {
				regulator.voltage = volt;
				ret =
				    copy_to_user((pmu_regl_volt *) arg,
						 &regulator, sizeof(regulator));
			} else
				ret = volt;
			break;
		}
	case BCM_PMU_IOCTL_GET_REGULATOR_STATE:
		{
			pmu_regl rmode;
			if (copy_from_user
			    (&rmode, (pmu_regl *) arg, sizeof(pmu_regl)) != 0) {
				return -EFAULT;
			}
			rmode.state =
			    _bcm59035_regulator_get_mode(regl_priv,
							 rmode.regl_id);
			if (rmode.state > 0)
				ret =
				    copy_to_user((pmu_regl *) arg, &rmode,
						 sizeof(rmode));
			else
				ret = rmode.state;
			break;
		}
	case BCM_PMU_IOCTL_SET_REGULATOR_STATE:
		{
			pmu_regl rmode;
			if (copy_from_user
			    (&rmode, (pmu_regl *) arg, sizeof(pmu_regl)) != 0) {
				return -EFAULT;
			}
			ret =
			    _bcm59035_regulator_set_mode(regl_priv,
							 rmode.regl_id,
							 rmode.state);
			break;
		}
	case BCM_PMU_IOCTL_ACTIVATESIM:
		{
			int id = BCM59035_REGL_SIMLDO;
			pmu_sim_volt sim_volt;
			u8 value;
			if (copy_from_user(&sim_volt, (int *)arg, sizeof(int))
			    != 0) {
				return -EFAULT;
			}
			/*check the status of SIMLDO */
			ret = _bcm59035_regulator_is_enabled(regl_priv, id);
			if (ret) {
				pr_info("SIMLDO is activated already\n");
				return -EPERM;
			}
			/* Put SIMLDO in ON State */
			ret = _bcm59035_regulator_enable(regl_priv, id);
			if (ret)
				return ret;
			/* Set SIMLDO voltage */
			value = bcm59035_regls[id].vol_list[sim_volt][1];
			ret =
			    _bcm59035_regulator_set_voltage(regl_priv, id,
							    value);
			break;
		}
	case BCM_PMU_IOCTL_DEACTIVATESIM:
		{
			int id = BCM59035_REGL_SIMLDO;
			/*check the status of SIMLDO */
			ret = _bcm59035_regulator_is_enabled(regl_priv, id);
			if (!ret) {
				pr_info("SIMLDFO is already disabled\n");
				return -EPERM;
			}
			ret = _bcm59035_regulator_disable(regl_priv, id);
			if (ret)
				return ret;
			break;
		}
	}
	return ret;
}

static int __devinit bcm59035_regulator_probe(struct platform_device *pdev)
{
	struct bcm59035_regl_priv *regl_priv;
	struct bcm59035 *bcm59035 = dev_get_drvdata(pdev->dev.parent);
	struct bcm59035_regl_pdata *regulators;
	int i, ret;

	PMU_LOG(DEBUG_PMU_INFO, "Inside %s\n", __FUNCTION__);

	if (unlikely(!bcm59035->pdata || !bcm59035->pdata->regulators)) {
		PMU_LOG(DEBUG_PMU_ERROR, "%s: invalid platform data !!!\n",
			__FUNCTION__);
		return -EINVAL;
	}
	regulators = bcm59035->pdata->regulators;

	regl_priv = kzalloc((sizeof(struct bcm59035_regl_priv) +
			     regulators->num_regulators *
			     sizeof(struct regulator_dev *)), GFP_KERNEL);
	if (unlikely(!regl_priv))
		return -ENOMEM;

	regl_priv->bcm59035 = bcm59035;

	/*Change CSR regulator voltage table if DVS is not enabled */
	if ((bcm59035->flags & BCM59035_ENABLE_DVS) == 0) {
		bcm59035_regls[BCM59035_REGL_CSR].ctrl_reg =
		    BCM59035_REG_CSRCTRL2;
		bcm59035_regls[BCM59035_REGL_CSR].num_vol =
		    ARRAY_SIZE(csr_no_dvs_vol);
		bcm59035_regls[BCM59035_REGL_CSR].vol_list = csr_no_dvs_vol;
	}

	for (i = 0; i < regulators->num_regulators; i++) {

		/* copy flags */
		bcm59035_regls[regulators->regl_init[i].regl_id].dsm_opmode = regulators->regl_init[i].dsm_opmode;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 30))
		pdev->dev.platform_data = regulators->regl_init[i].init_data;

		regl_priv->regl[i] =
		    regulator_register(&bcm59035_regls
				       [regulators->regl_init[i].regl_id].desc,
				       &pdev->dev, regl_priv);
#else
		regl_priv->regl[i] =
		    regulator_register(&bcm59035_regls
				       [regulators->regl_init[i].regl_id].desc,
				       &pdev->dev,
				       regulators->regl_init[i].init_data,
				       regl_priv);

#endif

		if (IS_ERR(regl_priv->regl[i])) {
			PMU_LOG(DEBUG_PMU_ERROR,
				"%s: regulator_register Error !!!\n",
				__FUNCTION__);
			ret = PTR_ERR(regl_priv->regl[i]);
			goto err;
		}
	}
	regl_priv->num_regl = regulators->num_regulators;

	/* Set default Regualtor PM mode values */
	bcm59035->write_dev(bcm59035, BCM59035_REG_A1OPMODCTRL,
			    regulators->
			    regl_default_pmmode[BCM59035_REGL_ALDO1]);
	bcm59035->write_dev(bcm59035, BCM59035_REG_A2OPMODCTRL,
			    regulators->
			    regl_default_pmmode[BCM59035_REGL_ALDO2]);
	bcm59035->write_dev(bcm59035, BCM59035_REG_R1OPMODCTRL,
			    regulators->
			    regl_default_pmmode[BCM59035_REGL_RFLDO1]);
	bcm59035->write_dev(bcm59035, BCM59035_REG_R2OPMODCTRL,
			    regulators->
			    regl_default_pmmode[BCM59035_REGL_RFLDO2]);
	bcm59035->write_dev(bcm59035, BCM59035_REG_H1OPMODCTRL,
			    regulators->
			    regl_default_pmmode[BCM59035_REGL_HCLDO1]);
	bcm59035->write_dev(bcm59035, BCM59035_REG_H2OPMODCTRL,
			    regulators->
			    regl_default_pmmode[BCM59035_REGL_HCLDO2]);
	bcm59035->write_dev(bcm59035, BCM59035_REG_IOPMODCTRL,
			    regulators->
			    regl_default_pmmode[BCM59035_REGL_IOLDO]);
	bcm59035->write_dev(bcm59035, BCM59035_REG_M1OPMODCTRL,
			    regulators->
			    regl_default_pmmode[BCM59035_REGL_MSLDO1]);
	bcm59035->write_dev(bcm59035, BCM59035_REG_LOPMODCTRL,
			    regulators->
			    regl_default_pmmode[BCM59035_REGL_LCLDO]);
	bcm59035->write_dev(bcm59035, BCM59035_REG_LV1OPMODCTRL,
			    regulators->
			    regl_default_pmmode[BCM59035_REGL_LVLDO1]);
	bcm59035->write_dev(bcm59035, BCM59035_REG_LV2OPMODCTRL,
			    regulators->
			    regl_default_pmmode[BCM59035_REGL_LVLDO2]);
	bcm59035->write_dev(bcm59035, BCM59035_REG_M2OPMODCTRL,
			    regulators->
			    regl_default_pmmode[BCM59035_REGL_MSLDO2]);
	bcm59035->write_dev(bcm59035, BCM59035_REG_AX1OPMODCTRL,
			    regulators->
			    regl_default_pmmode[BCM59035_REGL_AUXLDO1]);
	bcm59035->write_dev(bcm59035, BCM59035_REG_AX2OPMODCTRL,
			    regulators->
			    regl_default_pmmode[BCM59035_REGL_AUXLDO2]);
	bcm59035->write_dev(bcm59035, BCM59035_REG_CSROPMODCTRL,
			    regulators->regl_default_pmmode[BCM59035_REGL_CSR]);
	bcm59035->write_dev(bcm59035, BCM59035_REG_IOSROPMODCTRL,
			    regulators->
			    regl_default_pmmode[BCM59035_REGL_IOSR]);
	bcm59035->write_dev(bcm59035, BCM59035_REG_SOPMODCTRL,
			    regulators->
			    regl_default_pmmode[BCM59035_REGL_SIMLDO]);

	platform_set_drvdata(pdev, regl_priv);

	bcm59035_register_ioctl_handler(bcm59035, BCM59035_SUBDEV_REGULATOR,
					bcm59035_regulator_ioctl_handler,
					regl_priv);

	return 0;

err:
	while (--i >= 0)
		regulator_unregister(regl_priv->regl[i]);
	kfree(regl_priv);
	return ret;
}

static int __devexit bcm59035_regulator_remove(struct platform_device *pdev)
{
	struct bcm59035_regl_priv *regl_priv = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < regl_priv->num_regl; i++)
		regulator_unregister(regl_priv->regl[i]);
	kfree(regl_priv);
	return 0;
}

static struct platform_driver bcm59035_regulator_driver = {
	.driver = {
		   .name = "bcm59035-regulator",
		   .owner = THIS_MODULE,
		   },
	.remove = __devexit_p(bcm59035_regulator_remove),
	.probe = bcm59035_regulator_probe,
};

static int __init bcm59035_regulator_init(void)
{
	return platform_driver_register(&bcm59035_regulator_driver);
}

subsys_initcall(bcm59035_regulator_init);

static void __exit bcm59035_regulator_exit(void)
{
	platform_driver_unregister(&bcm59035_regulator_driver);
}

module_exit(bcm59035_regulator_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Regulator Driver for Broadcom BCM59035 PMU");
MODULE_ALIAS("platform:bcm59035-regulator");
