/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/device.c
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
 * device specific definitions
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <mach/reg_sdio.h>
#include <mach/hardware.h>
#include <mach/setup.h>
#include <linux/io.h>
#include <linux/delay.h>

#ifdef CONFIG_USB_GADGET_DWC_OTG
#include <mach/lm.h>
#include <mach/reg_usb.h>
#include <mach/reg_clkpwr.h>
#endif

#ifdef CONFIG_BCM_I2SDAI
static struct resource bcm_i2sdai_resource[] = {
	{
	 .start = HW_I2S_BASE,
	 .end = HW_I2S_BASE + SZ_4K - 1,
	 .flags = IORESOURCE_MEM,
	 },
};

struct platform_device i2sdai_device = {
	.name = "bcm_i2sdai",
	.id = -1,
	.resource = bcm_i2sdai_resource,
	.num_resources = ARRAY_SIZE(bcm_i2sdai_resource),
};
#endif

#if defined (CONFIG_BCM_WATCHDOG)
#include <plat/bcm_watchdog.h>
#endif

#if defined(CONFIG_BRCM_HEADSET) || defined(CONFIG_BRCM_HEADSET_MODULE)

struct platform_device bcm_headset_device = {
	.name = "bcmheadset",
	.id = -1,
};

#endif

#if defined(CONFIG_SERIAL_8250) || defined(CONFIG_SERIAL_8250_MODULE)
struct platform_device bcm_serial_device0 = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
};

struct platform_device bcm_serial_device1 = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM1,
};
#endif /*CONFIG_SERIAL_8250 */

#if defined (CONFIG_MMC_BCM)

extern int bcmsdhc_ctrl_slot_is_invalid(u8 ctrl_slot);
int bcmsdhc_cfg_card_detect(void __iomem *ioaddr, u8 ctrl_slot)
{
	u32 value = 0;

	if (bcmsdhc_ctrl_slot_is_invalid(ctrl_slot))
		return -EINVAL;

	if (ctrl_slot == 1) {
		/* slot 1 is for Wifi ,Disable WP and CD
		   Disable WP and CD */
		value &= ~(REG_SDIO_CORESTAT_CD | REG_SDIO_CORESTAT_WP);
		writel(value, ioaddr + SDHOST_CORESTAT_SHIFT);
	} else if (ctrl_slot == 2) {
		/* For  SD/MMC card
		   Disable WP,Enable CD */
		value = REG_SDIO_CORESTAT_CD;
		writel(value, ioaddr + SDHOST_CORESTAT_SHIFT);
	}

	return 0;
}

/* *************************************************************************************************** */
/* Function Name: bcmsdhc_external_reset */
/* Description: external reset SD host */
/* *************************************************************************************************** */

int bcmsdhc_external_reset(void __iomem *ioaddr, u8 ctrl_slot)
{
	u32 result = 0;

	if (bcmsdhc_ctrl_slot_is_invalid(ctrl_slot))
		return -EINVAL;

	writel(REG_SDIO_CORECTRL_RESET, ioaddr + SDHOST_CORECTRL_SHIFT);
	mdelay(5);
	writel(result, ioaddr + SDHOST_CORECTRL_SHIFT);

	return 0;
}

int bcmsdhc_enable_int(void __iomem *ioaddr, u8 ctrl_slot)
{
	if (bcmsdhc_ctrl_slot_is_invalid(ctrl_slot))
		return -EINVAL;

	writel(REG_SDIO_COREIMR_IP, ioaddr + SDHOST_COREIMR_SHIFT);

	return 0;
}

static struct resource sdhc1_resources[] = {
	{
	 .start = HW_SDIO0_BASE,
	 .end = HW_SDIO0_BASE + SZ_64K - 1,
	 .flags = IORESOURCE_MEM,
	 .name = "sdhc1 phy address",
	 },
	{
	 .start = IRQ_SDIO1,
	 .flags = IORESOURCE_IRQ,
	 .name = "sdhc1 IRQ",
	 },
};

struct platform_device bcm_sdhc_slot1 = {
	.name = "bcm_sdhc",
	.id = 1,
	.num_resources = ARRAY_SIZE(sdhc1_resources),
	.resource = sdhc1_resources,
};

EXPORT_SYMBOL(bcm_sdhc_slot1);
static struct resource sdhc2_resources[] = {
	{
	 .start = HW_SDIO1_BASE,
	 .end = HW_SDIO1_BASE + SZ_64K - 1,
	 .flags = IORESOURCE_MEM,
	 .name = "sdhc2 phy address",
	 },
	{
	 .start = IRQ_SDIO2,
	 .flags = IORESOURCE_IRQ,
	 .name = "sdhc2 IRQ",
	 },
};

struct platform_device bcm_sdhc_slot2 = {
	.name = "bcm_sdhc",
	.id = 2,
	.num_resources = ARRAY_SIZE(sdhc2_resources),
	.resource = sdhc2_resources,
};
#endif
#if defined(CONFIG_BCM_WATCHDOG)
static struct resource bcm_wdt_res[] = {
	{
	 .start = HW_WDT_BASE1,
	 .end = HW_WDT_BASE1 + SZ_16 - 1,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = IRQ_WDOG2,
	 .end = IRQ_WDOG2,
	 .flags = IORESOURCE_IRQ,
	 },
};

struct platform_device bcm_watchdog_device = {
	.name = "watchdog",
	.id = 0,
	.resource = bcm_wdt_res,
	.num_resources = ARRAY_SIZE(bcm_wdt_res),
};
#endif

#if defined (CONFIG_I2C_BCM1160)
static struct resource bcm_i2c1_resources[] = {
	{
	 .start = HW_I2C_BASE,
	 .end = HW_I2C_BASE + SZ_4K - 1,
	 .flags = IORESOURCE_MEM,
	 }, {
	     .start = IRQ_I2C,
	     .end = IRQ_I2C,
	     .flags = IORESOURCE_IRQ,
	     },
};

struct platform_device bcm_device_i2c1 = {
	.name = "i2cbcm",
	.id = 0,
	.resource = bcm_i2c1_resources,
	.num_resources = ARRAY_SIZE(bcm_i2c1_resources),
};

static struct resource bcm_i2c2_resources[] = {
	{
	 .start = HW_I2C2_BASE,
	 .end = HW_I2C2_BASE + SZ_4K - 1,
	 .flags = IORESOURCE_MEM,
	 }, {
	     .start = IRQ_UARTC,
	     .end = IRQ_UARTC,
	     .flags = IORESOURCE_IRQ,
	     },
};

struct platform_device bcm_device_i2c2 = {
	.name = "i2cbcm",
	.id = 1,
	.resource = bcm_i2c2_resources,
	.num_resources = ARRAY_SIZE(bcm_i2c2_resources),
};
#endif
#if defined (CONFIG_BCM_AUXADC)
static struct resource bcm_auxadc_resource[] = {
	{
	 .start = HW_AUXADC_BASE,
	 .end = HW_AUXADC_BASE + SZ_16 - 1,
	 .flags = IORESOURCE_MEM,
	 },
};

struct platform_device auxadc_device = {
	.name = "bcm_auxadc",
	.id = -1,
	.resource = bcm_auxadc_resource,
	.num_resources = ARRAY_SIZE(bcm_auxadc_resource),
};
#endif
#if defined (CONFIG_BCM_OTP)
static struct resource bcm_otp[] = {
	{
	 .start = HW_OTP_BASE,
	 .end = HW_OTP_BASE + SZ_4K - 1,
	 .flags = IORESOURCE_MEM,
	 },
};

struct platform_device bcm_otp_device = {
	.name = "otp",
	.id = -1,
	.resource = bcm_otp,
	.num_resources = ARRAY_SIZE(bcm_otp),
};
#endif

#ifdef CONFIG_USB_GADGET_DWC_OTG
int __init usb_devices_init(void)
{
	struct lm_device *lmdev;
	int rc = -1;

	/* Power Up the 48MHz PLL */
	REG_CLKPWR_USBPLL_ENABLE |= 1;

	/* Turn on the output gate of the 48MHz PLL */
	REG_CLKPWR_USBPLL_OEN |= 1;

	/* Turn on the HS USB in the usb control register */
	REG_USB_CONTROL_REG |= REG_USB_CONTROL_USB_ON;

	lmdev = kmalloc(sizeof(struct lm_device), GFP_KERNEL);
	if (lmdev) {
		memset(lmdev, 0, sizeof(struct lm_device));

		lmdev->resource.start = HW_USBOTG_BASE;
		lmdev->resource.end = lmdev->resource.start + SZ_256K - 1;
		lmdev->resource.flags = IORESOURCE_MEM;

		lmdev->irq = IRQ_USB;
		lmdev->id = -2;

		rc = lm_device_register(lmdev);
	}

	return rc;
}

subsys_initcall(usb_devices_init);

#endif /* CONFIG_USB_GADGET_DWC_OTG */
#ifdef CONFIG_SPI
static struct resource bcm21xx_spi_resources[] = {
	{
	 .start = HW_SPI_BASE,
	 .end = HW_SPI_BASE + SZ_4K - 1,
	 .flags = IORESOURCE_MEM,
	 }, {
	     .start = IRQ_SPI,
	     .end = IRQ_SPI,
	     .flags = IORESOURCE_IRQ,
	     },
};

struct platform_device bcm21xx_device_spi = {
	.name = "bcm_spi",
	.id = 0,
	.resource = bcm21xx_spi_resources,
	.num_resources = ARRAY_SIZE(bcm21xx_spi_resources),
};
#endif
