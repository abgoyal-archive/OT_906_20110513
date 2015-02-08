/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*	@file	arch/arm/plat-bcmap/sysfs.c
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
 * SYSFS infrastructure specific Broadcom SoCs
 */
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <plat/syscfg.h>
#include <asm/cacheflush.h>
#include <linux/sched.h>

struct kobject *bcm_kobj;

static ssize_t
mem_store(struct device *dev, struct device_attribute *attr,
	  const char *buf, size_t n)
{
	uint32_t addr, val, count = 0, loop = 0;
	void __iomem *vaddr;
	char rw;

	if (sscanf(buf, "%c %x %x", &rw, &addr, &val) == 3) {
		pr_info("\n");
		vaddr = ioremap(addr, PAGE_SIZE);
		if (rw == 'W' || rw == 'w') {
			writel(val, vaddr);
			count = 4;
		} else if (rw == 'R' || rw == 'r') {
			count = val;	/* count read in val for simplicity */
			if (count & 0x3)	/* Align to 4 */
				count += (4 - (count & 0x3));
		}
		for (; loop < count; loop += 4) {
			val = readl(vaddr + loop);
			pr_info("[0x%08x] = 0x%08x\n", addr + loop, val);
		}
		iounmap(vaddr);
		return n;
	}
	pr_info("\nUsage: echo <R - Read/W - write> <Physical Address>"
		"<Value(Write)/Count(Read) > /sys/bcm/mem\n"
		"E.g. echo R 0x88CE000 0x40 > /sys/bcm/mem\n"
		"     echo w 0x88CE000 0xDEADBEAF > /sys/bcm/mem\n");
	return -EINVAL;
}

static char *str_reset_reason[] = {
	"power_on_reset",
	"soft_reset",
	"charging",
	"unknown"
};

static ssize_t
reset_reason_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int index, reset_reason;

	reset_reason = board_sysconfig(SYSCFG_RESETREASON, SYSCFG_INIT);
	switch (reset_reason) {
	case POWER_ON_RESET:
		index = 0;
		break;
	case SOFT_RESET:
		index = 1;
		break;
	case POWEROFF_CHARGING:
		index = 2;
		board_sysconfig(SYSCFG_RESETREASON, SYSCFG_ENABLE);
		break;
	default:
		index = 3;
		break;
	}
	sprintf(buf, "%s\n", str_reset_reason[index]);
	return strlen(str_reset_reason[index]) + 1;
}

#define BC_10_DETECTION_SUPPORT
#ifdef BC_10_DETECTION_SUPPORT
/* USB charger type */
typedef enum
{
	NO_CHARGER = 0,
	HOST_CHARGER,
	DEDICATED_CHARGER,
	UNKNOWN_CHARGER
} USB_Charger_Type_t;
extern USB_Charger_Type_t Android_Get_USB_BC_Type(void);
#else
inline int Android_Get_USB_BC_Type(void) { return 0; }
#endif

static ssize_t
charger_type_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int charger_type;
	static char *str_charger_type[] = {
		"no_charger",
		"host_charger",
		"dedicated_charger",
		"unknown_charger"
	};

	charger_type = Android_Get_USB_BC_Type();
	sprintf(buf, "%s\n", str_charger_type[charger_type]);
	return strlen(str_charger_type[charger_type]) + 1;
}

#define CACHE_LINE_SIZE	32
void flush_caches(void)
{

	/* Clean and Invalidate D cache */
	asm ("mcr p15, 0, %0, c7, c14, 0":: "r" (0));
}

void invalidate_caches(void)
{
	/* Invalidate D cache */
	asm ("mcr p15, 0, %0, c7, c6, 0":: "r" (0));
}

static ssize_t
cache_ops_store(struct device *dev, struct device_attribute *attr,
	  const char *buf, size_t n)
{
	uint32_t	start;
	ssize_t		size;
	char ops;

	if (sscanf(buf, "%c %x %d", &ops, &start, &size) == 3) {
		if (ops == 'I' || ops == 'i') {
#ifdef CONFIG_OUTER_CACHE
			outer_inv_range(virt_to_phys(start),
					 virt_to_phys(start + size));
#endif
			invalidate_caches();
		}
		if (ops == 'F' || ops == 'f') {
#ifdef CONFIG_OUTER_CACHE
			outer_flush_range(virt_to_phys(start),
					 virt_to_phys(start + size));
#endif
			flush_caches();
		}
		return n;
	}
	pr_info("\nUsage: echo <I - Invalidate/F - Flush/C - Clean>"
		"<start> <end> > /sys/bcm/cache\n"
		"E.g. echo I 0x88CE000 256 > /sys/bcm/cache\n"
		"     echo F 0x88CE000 256 > /sys/bcm/cache\n"
		"     echo C 0x88CE000 256 > /sys/bcm/cache\n");
	return -EINVAL;
}
static DEVICE_ATTR(mem, 0644, NULL, mem_store);
static DEVICE_ATTR(reset_reason, 0444, reset_reason_show, NULL);
static DEVICE_ATTR(charger_type, 0444, charger_type_show, NULL);
static DEVICE_ATTR(cache, 0666, NULL, cache_ops_store);

static struct attribute *bcm_attrs[] = {
	&dev_attr_mem.attr,
	&dev_attr_reset_reason.attr,
	&dev_attr_cache.attr,
	&dev_attr_charger_type.attr,
	NULL,
};

static struct attribute_group bcm_attr_group = {
	.attrs = bcm_attrs,
};

static int __init bcm_sysfs_init(void)
{
	bcm_kobj = kobject_create_and_add("bcm", NULL);
	if (!bcm_kobj)
		return -ENOMEM;
	return sysfs_create_group(bcm_kobj, &bcm_attr_group);
}

static void __exit bcm_sysfs_exit(void)
{
	sysfs_remove_group(bcm_kobj, &bcm_attr_group);
}

module_init(bcm_sysfs_init);
module_exit(bcm_sysfs_exit);
