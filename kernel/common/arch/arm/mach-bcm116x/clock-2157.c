/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/clock-2157.c
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
*   @file   clock-2157.c
*   @brief   Module to register and intialize the clocks in BCM2157
*
*****************************************************************************/



/**
*   @defgroup   Clock2157APIGroup   clock 2157 API's
*   @brief      This group defines the clock 2157 API's
*
*****************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <plat/clock.h>
#include <asm/clkdev.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>

/*#include "reg_clkpwr.h" */
#include <mach/reg_clkpwr.h>
/*#include "reg_syscfg.h" */
#include <mach/reg_sys.h>
#include <mach/clkmgr.h>

#define MAX_UART_N_VAL 0x1FF
#define MAX_UART_M_VAL 0x1FF
#define MULTIPLICATION_FACTOR 10

#define BCM_CLK_NO_DISABLE_ON_INIT (1 << 15)

#define BRCM_CLK_NAME(_name) clk_##_name
#define DEFINE_BRCM_CLK(_name, _id, _parent, _flgs, _enable_reg, _enable_bit_mask) \
	static struct clk BRCM_CLK_NAME(_name) = \
	{\
								.id = BCM2157_CLK_##_id,\
								.flags = _flgs,\
								.enable = bcm2157_##_name##_enable,\
								.disable = bcm2157_##_name##_disable,\
								.set_rate = bcm2157_##_name##_set_rate,\
								.get_rate = bcm2157_##_name##_get_rate,\
								.round_rate = bcm2157_##_name##_round_rate,\
								.parent = &BRCM_CLK_NAME(_parent),\
								.enable_bit_mask = _enable_bit_mask,\
								.enable_reg = (void __iomem *)_enable_reg,\
						 }

#define DEFINE_BRCM_CLK_SIMPLE(_name, _id, _parent, _flgs, _enable_reg, _enable_bit_mask) \
	static struct clk BRCM_CLK_NAME(_name) = \
	{\
								.id = BCM2157_CLK_##_id,\
								.flags = _flgs,\
								.parent = &BRCM_CLK_NAME(_parent),\
								.enable_bit_mask = _enable_bit_mask,\
								.enable_reg = (void __iomem *)_enable_reg,\
						 }

#define BRCM_REGISTER_CLK(con, dev, clock)	\
						{\
							.con_id = con,\
							.dev_id = dev,\
							.clk = &BRCM_CLK_NAME(clock),\
						}

#define FREQ_MHZ(mhz)		((mhz)*1000UL*1000UL)
#define FREQ_KHZ(khz)		((khz)*1000UL)

/* Clock Ids */
enum {
	BCM2157_CLK_MAIN_PLL = 1,
	BCM2157_CLK_APPS_PLL,
	BCM2157_CLK_ARM11,
	BCM2157_CLK_I2S_INT,
	BCM2157_CLK_I2S_EXT,
	BCM2157_CLK_DAMCK,
	BCM2157_CLK_PDPCK,
	BCM2157_CLK_SDIO1,
	BCM2157_CLK_SDIO2,
	BCM2157_CLK_SPI,
	BCM2157_CLK_UARTA,
	BCM2157_CLK_UARTB,
	BCM2157_CLK_GP,
	BCM2157_CLK_I2C1,
	BCM2157_CLK_I2C2,
	BCM2157_CLK_CMI,
	BCM2157_CLK_PWM,
	BCM2157_CLK_CAMERA,
	BCM2157_CLK_USB,
	BCM2157_CLK_GE,
	BCM2157_CLK_VCODEC,
	BCM2157_CLK_DMAC,
	BCM2157_CLK_LCD,
	BCM2157_CLK_RNG,
	BCM2157_CLK_MPHI,
	BCM2157_CLK_PEDESTAL_CTRL
};

static struct proc_dir_entry *brcm_proc_file;

/*ARM11 clock */
#define bcm2157_arm11_enable 		NULL
#define bcm2157_arm11_disable 		NULL
#define bcm2157_arm11_set_parent 	NULL
static unsigned long bcm2157_arm11_get_rate(struct clk *clk);
static int bcm2157_arm11_set_rate(struct clk *clk, unsigned long val);
static long bcm2157_arm11_round_rate(struct clk *clk,
				     unsigned long desired_val);

/* CAM clock */
static void bcm2157_cam_disable(struct clk *clk);
static int bcm2157_cam_enable(struct clk *clk);
static unsigned long bcm2157_cam_get_rate(struct clk *clk);
static int bcm2157_cam_set_rate(struct clk *clk, unsigned long val);
static long bcm2157_cam_round_rate(struct clk *clk, unsigned long desired_val);

/*I2S Int clk */
#define bcm2157_i2s_int_enable 		NULL
#define bcm2157_i2s_int_disable 		NULL
static unsigned long bcm2157_i2s_int_get_rate(struct clk *clk);
static int bcm2157_i2s_int_set_rate(struct clk *clk, unsigned long val);
static long bcm2157_i2s_int_round_rate(struct clk *clk,
				       unsigned long desired_val);

/*DAM clk */
#define bcm2157_damck_get_rate 	NULL
#define bcm2157_damck_set_rate		NULL
#define bcm2157_damck_round_rate	NULL
static int bcm2157_damck_enable(struct clk *clk);
static void bcm2157_damck_disable(struct clk *clk);

/*PDP clk */
#define bcm2157_pdpck_enable 		NULL
#define bcm2157_pdpck_disable 		NULL
static unsigned long bcm2157_pdpck_get_rate(struct clk *clk);
static int bcm2157_pdpck_set_rate(struct clk *clk, unsigned long val);
static long bcm2157_pdpck_round_rate(struct clk *clk,
				     unsigned long desired_val);

/*SDIO1 clk*/
#define bcm2157_sdio1_enable 	bcm2157_sdio_enable
#define bcm2157_sdio1_disable  bcm2157_sdio_disable
#define bcm2157_sdio1_get_rate	bcm2157_sdio_get_rate
#define bcm2157_sdio1_set_rate bcm2157_sdio_set_rate
#define bcm2157_sdio1_round_rate bcm2157_sdio_round_rate

/*SDIO2 clk*/
#define bcm2157_sdio2_enable 	bcm2157_sdio_enable
#define bcm2157_sdio2_disable  bcm2157_sdio_disable
#define bcm2157_sdio2_get_rate	bcm2157_sdio_get_rate
#define bcm2157_sdio2_set_rate bcm2157_sdio_set_rate
#define bcm2157_sdio2_round_rate bcm2157_sdio_round_rate

static int bcm2157_sdio_enable(struct clk *clk);
static void bcm2157_sdio_disable(struct clk *clk);
static unsigned long bcm2157_sdio_get_rate(struct clk *clk);
static int bcm2157_sdio_set_rate(struct clk *clk, unsigned long val);
static long bcm2157_sdio_round_rate(struct clk *clk, unsigned long desired_val);

/*SPI clk */
#define bcm2157_spi_enable 			NULL
#define bcm2157_spi_disable  		NULL
static unsigned long bcm2157_spi_get_rate(struct clk *clk);
static int bcm2157_spi_set_rate(struct clk *clk, unsigned long val);
static long bcm2157_spi_round_rate(struct clk *clk, unsigned long desired_val);

/* UARTA clk */
#define bcm2157_uarta_enable 	bcm2157_uart_enable
#define bcm2157_uarta_disable  bcm2157_uart_disable
#define bcm2157_uarta_get_rate	bcm2157_uart_get_rate
#define bcm2157_uarta_set_rate bcm2157_uart_set_rate
#define bcm2157_uarta_round_rate bcm2157_uart_round_rate

/* UARTB clk */
#define bcm2157_uartb_enable 		bcm2157_uart_enable
#define bcm2157_uartb_disable  	bcm2157_uart_disable
#define bcm2157_uartb_get_rate	bcm2157_uart_get_rate
#define bcm2157_uartb_set_rate bcm2157_uart_set_rate
#define bcm2157_uartb_round_rate bcm2157_uart_round_rate

static int bcm2157_uart_enable(struct clk *clk);
static void bcm2157_uart_disable(struct clk *clk);
static unsigned long bcm2157_uart_get_rate(struct clk *clk);
static int bcm2157_uart_set_rate(struct clk *clk, unsigned long val);
static long bcm2157_uart_round_rate(struct clk *clk, unsigned long desired_val);

/*GP clk*/
#define bcm2157_gp_enable 	NULL
#define bcm2157_gp_disable  NULL
static unsigned long bcm2157_gp_get_rate(struct clk *clk);
static int bcm2157_gp_set_rate(struct clk *clk, unsigned long val);
static long bcm2157_gp_round_rate(struct clk *clk, unsigned long desired_val);

/*USB Clk*/
#define bcm2157_usb_get_rate		NULL
#define bcm2157_usb_set_rate		NULL
#define bcm2157_usb_round_rate		NULL
static void bcm2157_usb_disable(struct clk *clk);
static int bcm2157_usb_enable(struct clk *clk);

/* I2C1 clk */
#define bcm2157_i2c1_enable 	NULL
#define bcm2157_i2c1_disable  	NULL
#define bcm2157_i2c1_get_rate	bcm2157_i2c_get_rate
#define bcm2157_i2c1_set_rate 	NULL
#define bcm2157_i2c1_round_rate NULL

/* I2C2 clk */
#define bcm2157_i2c2_enable 	NULL
#define bcm2157_i2c2_disable  	NULL
#define bcm2157_i2c2_get_rate	bcm2157_i2c_get_rate
#define bcm2157_i2c2_set_rate 	NULL
#define bcm2157_i2c2_round_rate NULL

static unsigned long bcm2157_i2c_get_rate(struct clk *clk);

 /*CMI*/
#define bcm2157_cmi_enable 	NULL
#define bcm2157_cmi_disable  NULL
static unsigned long bcm2157_cmi_get_rate(struct clk *clk);
static int bcm2157_cmi_set_rate(struct clk *clk, unsigned long val);
static long bcm2157_cmi_round_rate(struct clk *clk, unsigned long desired_val);

static u32 bcm2157_generic_round_rate(u32 desired_val,
				      const u32 *supportedFreqList, u8 count);
static long bcm2157_uart_find_m_n(unsigned long freq_val, long *m_set,
				  long *n_set);

/*Pedestal mode control flag */
static u32 pedestal_ctrl_flag;
/* Define clocks */
static struct clk clk_mainpll = {
	.id = BCM2157_CLK_MAIN_PLL, .flags = BCM_CLK_ALWAYS_ENABLED,	/* can't be disabled thro' s/w */
};

DEFINE_BRCM_CLK_SIMPLE(pedestal_ctrl, PEDESTAL_CTRL, mainpll, 0,
		       &pedestal_ctrl_flag, 0x01);
DEFINE_BRCM_CLK(arm11, ARM11, mainpll, BCM_CLK_ALWAYS_ENABLED, 0, 0);
DEFINE_BRCM_CLK(cam, CAMERA, pedestal_ctrl, 0, ADDR_CLKPWR_CLK_CAMCK_ENABLE,
		0x01);
DEFINE_BRCM_CLK(i2s_int, I2S_INT, pedestal_ctrl, 0,
		ADDR_CLKPWR_CLK_I2S_INT_ENABLE, 0x01);
DEFINE_BRCM_CLK_SIMPLE(i2s_ext, I2S_EXT, pedestal_ctrl, 0,
		       ADDR_CLKPWR_CLK_I2S_EXT_ENABLE, 0x01);
DEFINE_BRCM_CLK(damck, DAMCK, pedestal_ctrl, 0, ADDR_CLKPWR_CLK_DAMCK_ENABLE,
		0x01);
DEFINE_BRCM_CLK(pdpck, PDPCK, pedestal_ctrl, 0, ADDR_CLKPWR_CLK_PDPDCK_ENABLE,
		0x01);
DEFINE_BRCM_CLK(sdio1, SDIO1, pedestal_ctrl, 0, ADDR_CLKPWR_CLK_SDIO1_ENABLE,
		0x01);
DEFINE_BRCM_CLK(sdio2, SDIO2, pedestal_ctrl, 0, ADDR_CLKPWR_CLK_SDIO2_ENABLE,
		0x01);
DEFINE_BRCM_CLK(spi, SPI, pedestal_ctrl, 0, ADDR_CLKPWR_CLK_SPI_ENABLE, 0x01);
#if defined(CONFIG_DEBUG_LL)
DEFINE_BRCM_CLK(uarta, UARTA, mainpll, BCM_CLK_NO_DISABLE_ON_INIT,
		ADDR_CLKPWR_CLK_UARTA_ENABLE, 0x01);
#else
DEFINE_BRCM_CLK(uarta, UARTA, mainpll, 0, ADDR_CLKPWR_CLK_UARTA_ENABLE, 0x01);
#endif
DEFINE_BRCM_CLK(uartb, UARTB, mainpll, 0, ADDR_CLKPWR_CLK_UARTB_ENABLE, 0x01);
DEFINE_BRCM_CLK(gp, GP, pedestal_ctrl, 0, ADDR_CLKPWR_CLK_GPCK_ENABLE, 0x01);
DEFINE_BRCM_CLK(usb, USB, pedestal_ctrl, BCM_CLK_NO_DISABLE_ON_INIT, 0, 0);
DEFINE_BRCM_CLK(i2c1, I2C1, pedestal_ctrl, 0, (HW_I2C_BASE + 0x2C), 0x01);
DEFINE_BRCM_CLK(i2c2, I2C2, pedestal_ctrl, 0, (HW_I2C2_BASE + 0x2C), 0x01);
DEFINE_BRCM_CLK(cmi, CMI, pedestal_ctrl, 0, ADDR_CLKPWR_CLK_CMI_ENABLE, 0x01);
DEFINE_BRCM_CLK_SIMPLE(pwm, PWM, mainpll, 0, ADDR_CLKPWR_CLK_PWM_ENABLE, 0x01);
DEFINE_BRCM_CLK_SIMPLE(ge, GE, pedestal_ctrl, 0, ADDR_SYSCFG_GE_AHB_CLK_EN,
		       0x01);
DEFINE_BRCM_CLK_SIMPLE(vcodec, VCODEC, pedestal_ctrl, 0,
		       ADDR_SYSCFG_VIDEO_CODEC_AHB_CLK_EN, 0x01);
DEFINE_BRCM_CLK_SIMPLE(dmac, DMAC, mainpll, 0, ADDR_SYSCFG_DMAC_AHB_CLK_EN,
		       0x01);
DEFINE_BRCM_CLK_SIMPLE(lcd, LCD, mainpll, BCM_CLK_NO_DISABLE_ON_INIT, ADDR_SYSCFG_LCD_AHB_CLK_EN, 0x01);
DEFINE_BRCM_CLK_SIMPLE(rng, RNG, pedestal_ctrl, 0, ADDR_SYSCFG_RNG_AHB_CLK_EN,
		       0x01);
DEFINE_BRCM_CLK_SIMPLE(mphi, MPHI, pedestal_ctrl, 0,
		       ADDR_SYSCFG_MPHI_AHB_CLK_EN, 0x01);

static u32 cam_freq_list[] = { FREQ_MHZ(12), FREQ_MHZ(13), FREQ_MHZ(24),
	FREQ_MHZ(26), FREQ_MHZ(48)
};

static u32 i2s_int_freq_list[] = { FREQ_MHZ(12), FREQ_MHZ(2.4), FREQ_MHZ(2.048), FREQ_MHZ(1.536),
	FREQ_KHZ(512), FREQ_KHZ(256), FREQ_KHZ(128)
};

static u32 pdpck_freq_list[] = { FREQ_MHZ(78), FREQ_MHZ(39), FREQ_MHZ(26), FREQ_MHZ(19.5),
	FREQ_MHZ(15.6), FREQ_MHZ(13), FREQ_MHZ(11.142857),
	FREQ_MHZ(9.75)
};

static u32 spi_freq_list[] = { FREQ_MHZ(104), FREQ_MHZ(78), FREQ_MHZ(39), FREQ_MHZ(26),
	FREQ_MHZ(19.5), FREQ_MHZ(15.6), FREQ_MHZ(13), FREQ_MHZ(11.142857),
	FREQ_MHZ(9.75)
};

static u32 gp_freq_list[] = { FREQ_MHZ(78), FREQ_MHZ(39), FREQ_MHZ(26), FREQ_MHZ(19.5),
	FREQ_MHZ(15.6), FREQ_MHZ(13), FREQ_MHZ(11.142857),
	FREQ_MHZ(9.75)
};

static u32 cmi_freq_list[] = { FREQ_MHZ(12), FREQ_MHZ(13), FREQ_MHZ(24), FREQ_MHZ(26),
	FREQ_MHZ(48), FREQ_MHZ(52), FREQ_MHZ(78)
};

static u8 clk_armahb_arm11_divider[] = { 12, 8, 6, 6, 4, 16, 4, 3, 2, 12, 12, 12, 12, 3, 2, 2 };

/** @addtogroup Clock2157APIGroup
	@{
*/
/**
* @brief Check if the SOC is allowed to enter pedestal mode
*
* @return returns true pedestal mode is allowed,
*
* Method to check if the SOC is allowed to enter pedestal mode. If any of the
* clocks which prevent pedestal are active, this method returns false.
*
*/
bool brcm_clk_is_pedestal_allowed(void)
{
	return (pedestal_ctrl_flag == 0);
}
/** @} */
EXPORT_SYMBOL(brcm_clk_is_pedestal_allowed);

static u32 bcm2157_get_apps_pll_freq(void)
{
	u32 regVal;
	u8 p1, p2;
	u8 nInt;
	u8 nFrac;
	u32 apps_pll_freq;

	regVal = readl(ADDR_CLKPWR_CLK_APPSPLL_DIVIDERS);
	/*P1 - While reading back, information is available on bits [0:3] */
	p1 = regVal & 0xF;
	/*P2 - While reading back, information is available on bits [4:7]  */
	p2 = (regVal >> 4) & 0xF;
	/*nInt - While reading back, information is available on bits [8:15] */
	nInt = (regVal >> 8) & 0xFF;

	regVal = readl(ADDR_CLKPWR_CLK_APPSPLL_FRAC_DIVIDERS);
	nFrac = regVal & 0xFF;	/*Alt nFrac */

	/*
	   Fout = P2/P1* (Nint + (nFrac/2^24))*26Mhz
	 */

	apps_pll_freq = ((0x1000000 * nInt + nFrac) / 0x1000000) * 26;

	apps_pll_freq = (apps_pll_freq * p2) / p1;

	return apps_pll_freq;
}

unsigned long bcm2157_arm11_get_rate(struct clk *clk)
{
	u32 div = 0;
	u32 regVal;
	u32 appsFreq;

	div = readl(ADDR_CLKPWR_CLK_ARMAHB_MODE) & 0xF;
	regVal = readl(ADDR_CLKPWR_CLK_SEL_MODE);

	if ((regVal & 0x3) == 0) {	/*main pll ? */
		return FREQ_MHZ(624) / clk_armahb_arm11_divider[div];
	}
	appsFreq = bcm2157_get_apps_pll_freq();
	pr_info("Apps pll freq = %dMhz\n", appsFreq);
	return FREQ_MHZ(appsFreq) / clk_armahb_arm11_divider[div];
}

int bcm2157_arm11_set_rate(struct clk *clk, unsigned long val)
{
	pr_info("%s:Not implemented\n", __func__);
	return -EINVAL;
}

long bcm2157_arm11_round_rate(struct clk *clk, unsigned long desired_val)
{
	pr_info("%s:Not implemented\n", __func__);
	return 0;
}

/* Camera clk - interface functions */
void bcm2157_cam_disable(struct clk *clk)
{
	u32 regVal;
	/* Enable CAM clk */
	regVal = readl(clk->enable_reg);
	regVal &= ~(clk->enable_bit_mask);
	writel(regVal, clk->enable_reg);

	writel(0x0, ADDR_SYSCFG_CAMERA_INTERFACE_AHB_CLK_EN);	/* disable AHB clock */

}

int bcm2157_cam_enable(struct clk *clk)
{
	u32 regVal;

	writel(0x01, ADDR_SYSCFG_CAMERA_INTERFACE_AHB_CLK_EN);	/* Enable AHB clock */

	/* Enable CAM clk */
	regVal = readl(clk->enable_reg);
	regVal |= clk->enable_bit_mask;
	writel(regVal, clk->enable_reg);
	return 0;
}

unsigned long bcm2157_cam_get_rate(struct clk *clk)
{
	return cam_freq_list[(readl(ADDR_CLKPWR_CLK_CAMCK_MODE) & 0x07)];
}

int bcm2157_cam_set_rate(struct clk *clk, unsigned long val)
{
	int inx;
	int count = ARRAY_SIZE(cam_freq_list);

	for (inx = 0; inx < count; inx++) {
		if (cam_freq_list[inx] == val) {
			writel(inx & 0x07, ADDR_CLKPWR_CLK_CAMCK_MODE);
			return 0;
		}
	}

	return -EINVAL;
}

long bcm2157_cam_round_rate(struct clk *clk, unsigned long desired_val)
{
	int count = ARRAY_SIZE(cam_freq_list);

	return (long)bcm2157_generic_round_rate(desired_val, cam_freq_list,
						count);
}

/*I2S_INT - interface functions*/
unsigned long bcm2157_i2s_int_get_rate(struct clk *clk)
{
	u32 regVal;
	int i;
	const u32 i2s_int_mode[] = { 0x00C, 0x040, 0x04B, 0x064, 0x12F, 0x260, 0x4C1 };
	int count = ARRAY_SIZE(i2s_int_mode);

	regVal = readl(ADDR_CLKPWR_CLK_I2S_INT_MODE) & 0xFFF;

	for (i = 0; i < count; i++) {
		if (i2s_int_mode[i] == regVal) {
			return i2s_int_freq_list[i];
		}
	}
	return 0;
}

int bcm2157_i2s_int_set_rate(struct clk *clk, unsigned long val)
{
	u32 regVal;
	int i;
	const u32 i2s_int_mode[] = { 0x00C, 0x040, 0x04B, 0x064, 0x12F, 0x260, 0x4C1 };
	const u32 i2s_int_frac[] = { 0x0, 0x00, 0x0B, 0x24, 0x2C, 0x18, 0x30 };
	int count = ARRAY_SIZE(i2s_int_mode);

	for (i = 0; i < count; i++) {
		if (i2s_int_freq_list[i] == val) {
			regVal = i2s_int_mode[i] & 0xFFF;
			writel(regVal, ADDR_CLKPWR_CLK_I2S_INT_MODE);

			regVal = i2s_int_frac[i] & 0x3F;
			writel(regVal, ADDR_CLKPWR_CLK_I2S_FRAC_MODE);

			return 0;
		}
	}

	return -EINVAL;

}

long bcm2157_i2s_int_round_rate(struct clk *clk, unsigned long desired_val)
{
	u32 count = ARRAY_SIZE(i2s_int_freq_list);

	return (long)bcm2157_generic_round_rate(desired_val, i2s_int_freq_list,
						count);
}

/* PDP clk - interface functions */
unsigned long bcm2157_pdpck_get_rate(struct clk *clk)
{
	return pdpck_freq_list[(readl(ADDR_CLKPWR_CLK_PDPDCK_DIV) & 0x7)];

}

int bcm2157_pdpck_set_rate(struct clk *clk, unsigned long val)
{
	u32 count = ARRAY_SIZE(pdpck_freq_list);
	u32 inx;

	for (inx = 0; inx < count; inx++) {
		if (pdpck_freq_list[inx] == val) {
			writel(inx & 0x07, ADDR_CLKPWR_CLK_PDPDCK_DIV);
			return 0;
		}
	}

	return -EINVAL;

}

long bcm2157_pdpck_round_rate(struct clk *clk, unsigned long desired_val)
{
	u32 count = ARRAY_SIZE(pdpck_freq_list);

	return (long)bcm2157_generic_round_rate(desired_val, pdpck_freq_list,
						count);
}

/* SDIO clk - interface functions */
int bcm2157_sdio_enable(struct clk *clk)
{
	u32 regVal;
	u32 ahbReg;

	/* Enable AHB clock */
	if (clk->id == BCM2157_CLK_SDIO1) {
		ahbReg = ADDR_SYSCFG_SDIO1_AHB_CLK_EN;
	} else if (clk->id == BCM2157_CLK_SDIO2) {
		ahbReg = ADDR_SYSCFG_SDIO2_AHB_CLK_EN;
	} else {
		pr_info("Invalid clk !!!!\n");
		return -EINVAL;
	}
	writel(0x01, ahbReg);	/* Enable AHB clock */

	/* Enable SDIO clk */
	regVal = readl(clk->enable_reg);
	regVal |= clk->enable_bit_mask;
	writel(regVal, clk->enable_reg);

	return 0;
}

void bcm2157_sdio_disable(struct clk *clk)
{
	u32 regVal;
	u32 ahbReg;

	/*  AHB clock enable reg address */
	if (clk->id == BCM2157_CLK_SDIO1) {
		ahbReg = ADDR_SYSCFG_SDIO1_AHB_CLK_EN;
	} else if (clk->id == BCM2157_CLK_SDIO2) {
		ahbReg = ADDR_SYSCFG_SDIO2_AHB_CLK_EN;
	} else {
		pr_info("Invalid clk !!!!\n");
		return;
	}
	/* disable SDIO clk */
	regVal = readl(clk->enable_reg);
	regVal &= ~(clk->enable_bit_mask);
	writel(regVal, clk->enable_reg);

	writel(0x00, ahbReg);	/* Disable AHB clock */

}

unsigned long bcm2157_sdio_get_rate(struct clk *clk)
{
	unsigned int div = 0;
	volatile u32 regVal;
	u32 regAddr;
	u32 rate = 0;

	if (clk->id == BCM2157_CLK_SDIO1) {
		regAddr = ADDR_CLKPWR_CLK_SDIO1_DIV;
	} else if (clk->id == BCM2157_CLK_SDIO2) {
		regAddr = ADDR_CLKPWR_CLK_SDIO2_DIV;
	} else {
		pr_info("Invalid clk !!!!\n");
		return 0;
	}

	regVal = readl(regAddr);

	/* See if 24/48 Mhz select bit is set */
	if (regVal & CLK_SDIO_DIV_48_24_SEL) {
		if ((regVal & (CLK_SDIO_DIV_24_SEL | CLK_SDIO_DIV_48_24_EN)) ==
		    (CLK_SDIO_DIV_24_SEL | CLK_SDIO_DIV_48_24_EN)) {
			rate = FREQ_MHZ(24);
		} else if (regVal & CLK_SDIO_DIV_48_24_EN) {
			rate = FREQ_MHZ(48);
		}
	} else {
		div = regVal & CLK_SDIO_DIV_DIVIDER_MASK;

		/* The clk_sdio1 fequency = 104 / n */
		/* where n is equal to (clk_sdio_div_divider+1)*2 */
		/* For example, when you program clk_sdio1_div = 1, */
		/* then clk_sdio1 = 104 / ((1+1)*2)) = 26 Mhz. */

		rate = ((FREQ_MHZ(104) / ((div + 1) * 2)));
	}
	return rate;
}

int bcm2157_sdio_set_rate(struct clk *clk, unsigned long val)
{
	long div = 0;
	volatile u32 regVal;
	u32 regAddr;

	/* Freq = 104MHz / ( ( div + 1 ) *2 ) */
	if (val > FREQ_MHZ(52)) {
		return -EINVAL;
	}

	if (clk->id == BCM2157_CLK_SDIO1) {
		regAddr = ADDR_CLKPWR_CLK_SDIO1_DIV;
	} else if (clk->id == BCM2157_CLK_SDIO2) {
		regAddr = ADDR_CLKPWR_CLK_SDIO2_DIV;
	} else {
		pr_info("Invalid clk !!!!\n");
		return -EINVAL;
	}

	if (val == FREQ_MHZ(24)) {
		writel(CLK_SDIO_DIV_48_24_SEL, regAddr);
		writel((CLK_SDIO_DIV_48_24_SEL | CLK_SDIO_DIV_48_24_EN),
		       regAddr);
		writel(CLK_SDIO_DIV_48_24_SEL | CLK_SDIO_DIV_48_24_EN |
		       CLK_SDIO_DIV_24_SEL, regAddr);

	} else if (val == FREQ_MHZ(48)) {

		writel(CLK_SDIO_DIV_48_24_SEL, regAddr);
		writel((CLK_SDIO_DIV_48_24_SEL | CLK_SDIO_DIV_48_24_EN),
		       regAddr);
	} else {

		div = FREQ_MHZ(52) / val;	/* (div + 1 ) = 52MHZ / val. */
		div = div - 1;
		regVal = div & CLK_SDIO_DIV_DIVIDER_MASK;
		writel(regVal, regAddr);
	}

	return 0;
}

long bcm2157_sdio_round_rate(struct clk *clk, unsigned long desired_val)
{
	long temp = 0;

	/* Freq = 104MHz / ( ( div + 1 ) *2 ) */
	if (desired_val > FREQ_MHZ(52)) {
		return -EINVAL;
	}
	if (desired_val == FREQ_MHZ(24) || desired_val == FREQ_MHZ(48))
		return desired_val;

	temp = FREQ_MHZ(52) / desired_val;	/* (div + 1 ) = 52MHZ / val. */

	temp = temp - 1;

	/* Now return the actual possible value. */
	return FREQ_MHZ(104) / ((temp + 1) * 2);
}

/*SPI clk - interface functions */
unsigned long bcm2157_spi_get_rate(struct clk *clk)
{
	u32 regVal;
	regVal = readl(ADDR_CLKPWR_CLK_SPI_DIV);

	if (regVal & CLK_SPI_DIV_104_EN)
		return FREQ_MHZ(104);
	else {
		regVal &= 0x7;
		return spi_freq_list[regVal + 1];
	}

}

int bcm2157_spi_set_rate(struct clk *clk, unsigned long val)
{
	int i;
	u32 count = ARRAY_SIZE(spi_freq_list);

	if (val == FREQ_MHZ(104)) {
		writel(CLK_SPI_DIV_104_EN, ADDR_CLKPWR_CLK_SPI_DIV);
		return 0;
	} else {
		for (i = 1; i < count; i++) {
			if (val == spi_freq_list[i]) {
				writel(i - 1, ADDR_CLKPWR_CLK_SPI_DIV);
				return 0;
			}
		}
	}
	return -EINVAL;
}

long bcm2157_spi_round_rate(struct clk *clk, unsigned long desired_val)
{
	u32 count = ARRAY_SIZE(spi_freq_list);

	return (long)bcm2157_generic_round_rate(desired_val, spi_freq_list,
						count);
}

/*UART clk - interface clocks*/
int bcm2157_uart_enable(struct clk *clk)
{
	u32 regVal;
	u32 ahbReg;

	/* Enable AHB clock */
	if (clk->id == BCM2157_CLK_UARTA) {
		ahbReg = ADDR_SYSCFG_UARTA_AHB_CLK_EN;
	} else if (clk->id == BCM2157_CLK_UARTB) {
		ahbReg = ADDR_SYSCFG_UARTB_AHB_CLK_EN;
	} else {
		pr_info("Invalid clk !!!!\n");
		return -EINVAL;
	}
	writel(0x01, ahbReg);	/* Enable AHB clock */

	/* Enable UART clk */
	regVal = readl(clk->enable_reg);
	regVal |= clk->enable_bit_mask;
	writel(regVal, clk->enable_reg);

	return 0;
}

void bcm2157_uart_disable(struct clk *clk)
{
	u32 regVal;
	u32 ahbReg;
	/*  AHB clock enable reg address */
	if (clk->id == BCM2157_CLK_UARTA) {
		ahbReg = ADDR_SYSCFG_UARTA_AHB_CLK_EN;
	} else if (clk->id == BCM2157_CLK_UARTB) {
		ahbReg = ADDR_SYSCFG_UARTB_AHB_CLK_EN;
	} else {
		pr_info("Invalid clk !!!!\n");
		return;
	}
	/* disable SDIO clk */
	regVal = readl(clk->enable_reg);
	regVal &= ~(clk->enable_bit_mask);
	writel(regVal, clk->enable_reg);

	writel(0x00, ahbReg);	/* Disable AHB clock */

}

unsigned long bcm2157_uart_get_rate(struct clk *clk)
{
	volatile unsigned int rr_n = 0;
	volatile unsigned int rr_m = 0;

	if (clk->id == BCM2157_CLK_UARTA) {
		rr_n = ((readl(ADDR_CLKPWR_CLK_UARTA_N)) & 0x1FF);
		rr_m = ((readl(ADDR_CLKPWR_CLK_UARTA_M)) & 0x1FF);

	} else if (clk->id == BCM2157_CLK_UARTB) {
		rr_n = ((readl(ADDR_CLKPWR_CLK_UARTB_N)) & 0x1FF);
		rr_m = ((readl(ADDR_CLKPWR_CLK_UARTB_M)) & 0x1FF);
	} else {
		pr_info("Invalid clk !!!!\n");
		return -EINVAL;
	}
	/* The clk_uart fequency = 156 * n / m */
	return (FREQ_MHZ(156) / rr_m) * rr_n;
}

int bcm2157_uart_set_rate(struct clk *clk, unsigned long val)
{
	long ret_val = 0;
	long m_set = 0;
	long n_set = 0;
	u32 regVal;

	ret_val = bcm2157_uart_find_m_n(val, &m_set, &n_set);
	if (ret_val > 0) {

		if (clk->id == BCM2157_CLK_UARTA) {
			regVal = (u32) n_set & 0x1FF;
			writel(regVal, ADDR_CLKPWR_CLK_UARTA_N);

			regVal = (u32) m_set & 0x1FF;
			writel(regVal, ADDR_CLKPWR_CLK_UARTA_M);
		}

		else if (clk->id == BCM2157_CLK_UARTB) {
			regVal = (u32) n_set & 0x1FF;
			writel(regVal, ADDR_CLKPWR_CLK_UARTB_N);

			regVal = (u32) m_set & 0x1FF;
			writel(regVal, ADDR_CLKPWR_CLK_UARTB_M);

		} else {
			pr_info("Invalid clk !!!!\n");
			return -EINVAL;
		}
		return 0;
	}

	return ret_val;

}

long bcm2157_uart_round_rate(struct clk *clk, unsigned long desired_val)
{
	long m_set = 0;
	long n_set = 0;
	return bcm2157_uart_find_m_n(desired_val, &m_set, &n_set);
}

/*GP - interface function */
unsigned long bcm2157_gp_get_rate(struct clk *clk)
{
	return gp_freq_list[(readl(ADDR_CLKPWR_CLK_GPCK_DIV) & 0x7)];
}

int bcm2157_gp_set_rate(struct clk *clk, unsigned long val)
{
	u32 count = ARRAY_SIZE(gp_freq_list);
	u32 inx;
	u32 regVal;

	for (inx = 0; inx < count; inx++) {
		if (gp_freq_list[inx] == val) {
			regVal = inx & 0x07;
			writel(regVal, ADDR_CLKPWR_CLK_GPCK_DIV);
			return 0;
		}
	}

	return -EINVAL;
}

long bcm2157_gp_round_rate(struct clk *clk, unsigned long desired_val)
{
	u32 count = ARRAY_SIZE(gp_freq_list);

	return (long)bcm2157_generic_round_rate(desired_val, gp_freq_list,
						count);
}

/*I2C clk - interface functions */
unsigned long bcm2157_i2c_get_rate(struct clk *clk)
{
	return FREQ_MHZ(13);	/* BSC internally set the dividers to generate the baud clk */
}

/*CMI clk - interface functions*/
unsigned long bcm2157_cmi_get_rate(struct clk *clk)
{
	return cmi_freq_list[(readl(ADDR_CLKPWR_CLK_CMI_MODE) & 0x7)];
}

int bcm2157_cmi_set_rate(struct clk *clk, unsigned long val)
{
	u32 count = ARRAY_SIZE(cmi_freq_list);
	u32 inx;
	u32 regVal;

	for (inx = 0; inx < count; inx++) {
		if (cmi_freq_list[inx] == val) {
			regVal = inx & 0x07;
			writel(regVal, ADDR_CLKPWR_CLK_CMI_MODE);
			return 0;
		}
	}

	return -EINVAL;
}

long bcm2157_cmi_round_rate(struct clk *clk, unsigned long desired_val)
{
	u32 count = ARRAY_SIZE(cmi_freq_list);

	return (long)bcm2157_generic_round_rate(desired_val, cmi_freq_list,
						count);
}

int bcm2157_damck_enable(struct clk *clk)
{
	u32 regVal;

	writel(0x01, ADDR_SYSCFG_DA_AHB_CLK_EN);	/* Enable AHB clock */

	/* Enable DAM clk */
	regVal = readl(clk->enable_reg);
	regVal |= clk->enable_bit_mask;
	writel(regVal, clk->enable_reg);
	return 0;
}

void bcm2157_damck_disable(struct clk *clk)
{
	u32 regVal;
	/* Enable DAM clk */
	regVal = readl(clk->enable_reg);
	regVal &= ~(clk->enable_bit_mask);
	writel(regVal, clk->enable_reg);

	writel(0x0, ADDR_SYSCFG_DA_AHB_CLK_EN);	/* disable AHB clock */

}

/*USB clk - interface functions */
void bcm2157_usb_disable(struct clk *clk)
{
	writel(0, ADDR_CLKPWR_USBPLL_ENABLE);
	writel(0, ADDR_CLKPWR_USBPLL_OEN);
	/* AHB clock disable */
	writel(0, ADDR_SYSCFG_USB_AHB_CLK_EN);

}

int bcm2157_usb_enable(struct clk *clk)
{
	/* AHB clock enable */
	writel(1, ADDR_SYSCFG_USB_AHB_CLK_EN);

	writel(1, ADDR_CLKPWR_USBPLL_ENABLE);
	writel(1, ADDR_CLKPWR_USBPLL_OEN);
	return 0;
}

static struct clk_lookup lookups[] = {
	BRCM_REGISTER_CLK(BCM_CLK_MAIN_PLL_STR_ID, NULL, mainpll),
	BRCM_REGISTER_CLK(BCM_CLK_ARM11_STR_ID, NULL, arm11),
	BRCM_REGISTER_CLK(BCM_CLK_I2S_INT_STR_ID, NULL, i2s_int),
	BRCM_REGISTER_CLK(BCM_CLK_I2S_EXT_STR_ID, NULL, i2s_ext),
	BRCM_REGISTER_CLK(BCM_CLK_DAM_STR_ID, NULL, damck),
	BRCM_REGISTER_CLK(BCM_CLK_PDP_STR_ID, NULL, pdpck),
	BRCM_REGISTER_CLK(BCM_CLK_SDIO1_STR_ID, NULL, sdio1),
	BRCM_REGISTER_CLK(BCM_CLK_SDIO2_STR_ID, NULL, sdio2),
	BRCM_REGISTER_CLK(BCM_CLK_SPI0_STR_ID, NULL, spi),
	BRCM_REGISTER_CLK(BCM_CLK_UARTA_STR_ID, NULL, uarta),
	BRCM_REGISTER_CLK(BCM_CLK_UARTB_STR_ID, NULL, uartb),
	BRCM_REGISTER_CLK(BCM_CLK_GP_STR_ID, NULL, gp),
	BRCM_REGISTER_CLK(BCM_CLK_I2C1_STR_ID, NULL, i2c1),
	BRCM_REGISTER_CLK(BCM_CLK_I2C2_STR_ID, NULL, i2c2),
	BRCM_REGISTER_CLK(BCM_CLK_CAMERA_STR_ID, NULL, cam),
	BRCM_REGISTER_CLK(BCM_CLK_USB_STR_ID, NULL, usb),
	BRCM_REGISTER_CLK(BCM_CLK_CMI_STR_ID, NULL, cmi),
	BRCM_REGISTER_CLK(BCM_CLK_PWM_STR_ID, NULL, pwm),
	BRCM_REGISTER_CLK(BCM_CLK_GE_STR_ID, NULL, ge),
	BRCM_REGISTER_CLK(BCM_CLK_VCODEC_STR_ID, NULL, vcodec),
	BRCM_REGISTER_CLK(BCM_CLK_DMAC_STR_ID, NULL, dmac),
	BRCM_REGISTER_CLK(BCM_CLK_RNG_STR_ID, NULL, rng),
	BRCM_REGISTER_CLK(BCM_CLK_LCD_STR_ID, NULL, lcd),
	BRCM_REGISTER_CLK(BCM_CLK_MPHI_STR_ID, NULL, mphi)

};

u32 bcm2157_generic_round_rate(u32 desired_val, const u32 *supportedFreqList,
			       u8 count)
{
	u32 i = 0;
	const u32 *ppossible_freqs = supportedFreqList;
	u32 closest_freq = 0xFFFFFFFF;	/* Set it to some highest value. */
	u32 greatest_freq_in_array = 0;

	while (i < count) {
		if (desired_val == *ppossible_freqs) {
			return desired_val;
		} else {
			if ((*ppossible_freqs > desired_val)
			    && (*ppossible_freqs <= closest_freq)) {
				closest_freq = *ppossible_freqs;
			}
			if (*ppossible_freqs > greatest_freq_in_array) {
				greatest_freq_in_array = *ppossible_freqs;
			}
		}
		i++;
		ppossible_freqs++;
	}

	/* This means that desired_val is greater than the maximum possible values */
	if (closest_freq == 0xFFFFFFFF) {
		/* This means the desired_val is greater than the greatest element */
		/* So lets return with the greatest freq in array. */
		return greatest_freq_in_array;
	} else {
		return closest_freq;
	}
}

long bcm2157_uart_find_m_n(unsigned long freq_val, long *m_set, long *n_set)
{
	long factor = 0;	/* = m/n */
	long eq_2_val = MAX_UART_M_VAL + MAX_UART_N_VAL;
	long n_val = 0;
	long long n_iter = 0;
	long long m_iter = 0;
	long long left = 0;
	long long right = 0;
	long long diff = 0;
	long long least_diff = 0;
	long long least_diff_m_iter = 1;
	long long least_diff_n_iter = 1;
	long long val = 0;
	long ret_val;

	/* Formula : Freq = ( 156MHz * n ) / m. */

	if (freq_val > FREQ_MHZ(156)) {	/* Can't have freq greater than 156MHz. */
		return -EINVAL;
	} else if (freq_val == FREQ_MHZ(156)) {
		*m_set = *n_set = 1;
		return FREQ_MHZ(156);
	}

	*m_set = 0;
	*n_set = 0;

	factor = FREQ_MHZ(156) / freq_val;

	/* range of m :  0 <= m <= 0x1ff */
	/* range of n :  0 <= n <= 0x1ff */
	/* Equation 1 : factor = m /n. */
	/*           so m = ( factor * n). */
	/* Equation 2 : m + n <= 0x1ff + 0x1ff. */
	/*              m + n <= ( 2 * 0x1ff ) ; Let's call ( 2 * 0x1ff ) as eq_2_val. */
	/*          so  m + n <= eq_2_val. */
	/* Substiture eq1 in eq2. */
	/* ( factor * n) + n <= eq_2_val. */
	/* taking n common : */
	/* ( factor + 1) * n <= eq_2_val. */
	/* so n <= eq_2_val / ( factor + 1). <==== This is a very important conclusion. */
		/* Now the logic is to iterate(iter) till n, and find a value of m, where m = 156MHz * iter / val. */
	n_val = eq_2_val / (factor + 1);

	pr_info("n_val = %ld, eq_2_val = %ld, factor = %ld \n", n_val, eq_2_val,
		factor);

	n_iter = 0;
	diff = 1;
	least_diff = 0xffffffff;
	val = freq_val;

	while ((n_iter <= n_val) && (diff != 0)) {
		n_iter++;
		left = FREQ_MHZ(156) * n_iter;

		m_iter = 0;
		while ((m_iter <= MAX_UART_M_VAL) && (diff != 0)) {
			m_iter++;
			right = val * m_iter;

			/* Always left side must be greater than right. So diff has to be positive. */
			diff = left - right;

			if ((diff > 0) && (diff < least_diff)) {
				least_diff = diff;
				least_diff_m_iter = m_iter;
				least_diff_n_iter = n_iter;
			}
			/* printk("n_iter = %lld, m_iter = %lld, left = %lld, right = %lld, diff = %lld, least_diff = %lld, least_diff_m_iter = %lld, least_diff_n_iter = %lld \n", */
			/*                n_iter, m_iter, left , right , diff , least_diff , least_diff_m_iter , least_diff_n_iter  ) ; */
		}
	}

	/* if diff == 0, it means we found a perfect, m, and n value. */
	if (diff == 0) {
		/* printk("We found the correct, m, n values. m = %lld, n= %lld \n", m_iter, n_iter ) ; */
		*m_set = m_iter;
		*n_set = n_iter;
		return val;
	} else {
		/* This means we didn't find a good m,n value. */
		*m_set = least_diff_m_iter;
		*n_set = least_diff_n_iter;
		ret_val = (((FREQ_MHZ(156) / *m_set)) * *n_set);
		/* printk("Didn't find a perfect m, n value, but will give the nearest possible value. m = %lld, n = %lld, freq = %ld \n", least_diff_m_iter, least_diff_n_iter, ret_val ) ; */
		return ret_val;
	}
}

static int
brcm_clk_parse_string(const char *inputStr, char *clkName, u32 *opCode,
		      u32 *arg)
{
	int numArg;
	char tempStr[50];
	int ret = 2;

	numArg = sscanf(inputStr, "%s%s%u", clkName, tempStr, arg);

	if (numArg < 2) {
		return -1;
	}

	if (strcmp(tempStr, "enable") == 0) {
		*opCode = 1;
	}

	else if (strcmp(tempStr, "disable") == 0) {
		*opCode = 2;
	} else if (strcmp(tempStr, "getrate") == 0) {
		*opCode = 3;
	} else if (strcmp(tempStr, "setrate") == 0) {
		*opCode = 4;
		if (numArg < 3)
			return -1;
		ret = 3;
	}

	return ret;

}

static int
brcm_clk_proc_read(char *page, char **start,
		   off_t off, int count, int *eof, void *data)
{
	int i;
	struct clk *clk;
	int len = 0;
	char *pg = page;

	pg += sprintf(pg, "clk-name\tUsageCount\tCurrentRate\n");

	/* traverse through the clk list */
	for (i = 0; i < ARRAY_SIZE(lookups); i++) {
		clk = clk_get(NULL, lookups[i].con_id);

		if (!clk) {
			pr_info("clk_get() error !!! for %s\n",
				lookups[i].con_id);
		}
		pg +=
		    sprintf(pg, "%s\t\t%u\t\t%uKHz\n", lookups[i].con_id,
			    clk->cnt, (unsigned int)clk_get_rate(clk) / 1000);
	}

	*start = page;

	len = pg - page;
	if (len > off)
		len -= off;
	else
		len = 0;

	return len < count ? len : count;
}

static int
brcm_clk_proc_write(struct file *file,
		    const char *buffer, unsigned long count, void *data)
{
	int len;
	char inputStr[100];
	char clkName[10];
	u32 opCode = 0;
	u32 arg;
	int ret;
	int numArg;
	struct clk *clk;

	if (count > 100)
		len = 100;
	else
		len = count;

	if (copy_from_user(inputStr, buffer, len))
		return -EFAULT;

	numArg = brcm_clk_parse_string(inputStr, clkName, &opCode, &arg);

	clk = clk_get(NULL, clkName);
	if ((numArg <= 0) || (clk == NULL)) {
		pr_info("Invalid arguments !!!\n");
		return len;
	}
	switch (opCode) {
	case 1:
		ret = clk_enable(clk);
		pr_info("clk_enable() : retun value %i\n", ret);

		if (clk->enable_reg)
			pr_info(" %s clk enable reg (%x) = %x\n",
				clkName,
				(u32) io_v2p((u32) clk->enable_reg),
				readl(clk->enable_reg));
		break;

	case 2:
		clk_disable(clk);
		pr_info("clk_disable()\n");
		if (clk->enable_reg)
			pr_info(" %s clk enable reg (%x) = %x\n",
				clkName,
				(u32) io_v2p((u32) clk->enable_reg),
				readl(clk->enable_reg));
		break;

	case 3:
		pr_info("clk_get_rate() : %s rate = %u\n", clkName,
			(unsigned int)clk_get_rate(clk));
		break;

	case 4:
		ret = clk_set_rate(clk, arg);
		pr_info("clk_set_rate() : return value %i\n", ret);

		if (!ret) {
			pr_info("%s new rate = %u\n", clkName,
				(unsigned int)clk_get_rate(clk));
		}
		break;
	}
	return len;
}

static int __init clk_init(void)
{
	int i;
	u32 regVal;
	struct clk *clk;
	/* register the clock lookups */
	for (i = 0; i < ARRAY_SIZE(lookups); i++) {
		/*Make sure that all clocks are disabled by default */
		clk = lookups[i].clk;
		pr_info("%s:%s\n", __func__, lookups[i].con_id);
		if ((clk->
		     flags & (BCM_CLK_ALWAYS_ENABLED |
			      BCM_CLK_NO_DISABLE_ON_INIT)) == 0) {
			if (clk->disable)
				clk->disable(clk);
			else if (clk->enable_reg) {
				regVal = readl(clk->enable_reg);
				if (clk->flags & BCM_CLK_INVERT_ENABLE) {
					regVal |= clk->enable_bit_mask;
					writel(regVal, clk->enable_reg);
				} else {
					regVal &= ~(clk->enable_bit_mask);
					writel(regVal, clk->enable_reg);
				}
			}
		}

		clkdev_add(&lookups[i]);
	}

	brcm_proc_file = create_proc_entry("bcm2157_clks", 0644, NULL);

	if (brcm_proc_file) {
		brcm_proc_file->data = NULL;
		brcm_proc_file->read_proc = brcm_clk_proc_read;
		brcm_proc_file->write_proc = brcm_clk_proc_write;
		return 0;
	}
	return -ENODEV;
}

static void __exit clk_exit(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(lookups); i++)
		clkdev_drop(&lookups[i]);
	remove_proc_entry("bcm2157_clks", NULL);
}

arch_initcall(clk_init);
module_exit(clk_exit);
