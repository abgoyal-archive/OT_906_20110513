/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/mfd/bcm59035-core.c
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
 *   @file   bcm59035-core.c
 *
 *   @brief  Core/Protocol driver for Broadcom BCM59035 PMU Chip
 *
 ****************************************************************************/

/**
*   @defgroup   BCM59035CoreAPIGroup   PMUCORE API's
*   @brief      This group defines the PMUCORE API's
*
*****************************************************************************/

#include <linux/mfd/bcm59035/bcm59035.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/pm.h>

#define PMU_LOG_ENABLE 1

#define BCM59035_INT_BASE       BCM59035_REG_INT1
#define BCM59035_INT_MASK_BASE  BCM59035_REG_INT1M

#define IRQ_TO_REG_INX(irq)  ((irq)/8)
#define IRQ_TO_REG_BIT(irq)  ((irq) % 8)

/* Debug mask */
const u32 bcm59035_debug_mask = 0xFF;

static struct bcm59035 *bcm59035_info;

static struct bcm_pmu_irq *bcm59035_find_irq_handler(struct bcm59035 *bcm59035,
						     int irq)
{
	struct bcm_pmu_irq *p;
	struct bcm_pmu_irq *match = NULL;

/*      mutex_lock(&bcm59035->list_lock); */
	list_for_each_entry(p, &bcm59035->irq_handlers, node) {
		if (p->irq == irq) {
			match = p;
			break;
		}
	}

/*      mutex_unlock(&bcm59035->list_lock); */
	return match;
}
/** @addtogroup BCM59035CoreAPIGroup
	@{
*/
/**
* @brief PMU IRQ request to the core form the PMU sub devices
*
* @param bcm59035 PMU core driver private data structure
* @param irq irq number requested
* @param enable_irq boolean to indicate if the irq has to be enabled on request
* @param handler irq handler method
* @param data Any arguments to be passed to the handler.
*
* @return returns 0 on success and negative error value on failure.
*
* PMU core manages the interrupts from PMU and handles over the interrupts to
* respective sub device drivers. This method can be used by sub device drivers
* to request an irq.
*/
int bcm59035_request_irq(struct bcm59035 *bcm59035, int irq, bool enable_irq,
			 void (*handler) (int, void *), void *data)
{
	struct bcm_pmu_irq *irq_info;
	PMU_LOG(DEBUG_PMU_INFO, " Inside %s\n", __FUNCTION__);
	if (irq < 0 || irq >= BCM59035_TOTAL_IRQ || !handler)
		return -EINVAL;
	if (WARN_ON(bcm59035_find_irq_handler(bcm59035, irq)))
		return -EBUSY;

	irq_info = kzalloc(sizeof(struct bcm_pmu_irq), GFP_KERNEL);
	if (!irq_info)
		return -ENOMEM;

	irq_info->handler = handler;
	irq_info->data = data;
	irq_info->irq_enabled = enable_irq;
	irq_info->irq = irq;

	mutex_lock(&bcm59035->list_lock);
	list_add(&irq_info->node, &bcm59035->irq_handlers);
	mutex_unlock(&bcm59035->list_lock);

	enable_irq ? bcm59035_enable_irq(bcm59035,
					 irq) : bcm59035_disable_irq(bcm59035,
								     irq);
	return 0;
}


/**
* @brief Request PMU core to free the particular irq
*
* @param bcm59035 PMU core driver private data structure
* @param irq irq number to be freed.
*
* @return returns 0 on success and negative error value on failure
*
* PMU core manages the interrupts from PMU and handles over the interrupts to
* respective sub device drivers. This methos can be used by sub device driver
* to free a particular irq.
*/
int bcm59035_free_irq(struct bcm59035 *bcm59035, int irq)
{
	struct bcm_pmu_irq *irq_info;
	PMU_LOG(DEBUG_PMU_INFO, " Inside %s\n", __FUNCTION__);
	if (irq < 0 || irq > BCM59035_TOTAL_IRQ)
		return -EINVAL;
	irq_info = bcm59035_find_irq_handler(bcm59035, irq);
	if (irq_info) {
		mutex_lock(&bcm59035->list_lock);
		list_del(&irq_info->node);
		mutex_unlock(&bcm59035->list_lock);
		kfree(irq_info);
	}

	/* disalbe IRQ as there is no handler */
	bcm59035_disable_irq(bcm59035, irq);
	return 0;
}


/**
* @brief request the PMU core to disable a particular IRQ
*
* @param bcm59035 PMU core driver private data structure
* @param irq irq number to be disabled
*
* @return returns 0 on success and negative error value on failure
*
* PMU core manages the interrupts from PMU and handles over the interrupts to
* respective sub device drivers. This method can be used by the sub device
* drivers to disable a particular irq.
*/
int bcm59035_disable_irq(struct bcm59035 *bcm59035, int irq)
{
	int regInx;
	int st;
	u8 regVal;
	struct bcm_pmu_irq *handler;

	PMU_LOG(DEBUG_PMU_INFO, " Inside %s\n", __FUNCTION__);

	if (irq < 0 || irq > BCM59035_TOTAL_IRQ)
		return -EINVAL;

	regInx = IRQ_TO_REG_INX(irq);

	st = bcm59035->read_dev(bcm59035, regInx + BCM59035_INT_MASK_BASE,
				&regVal);
	if (st < 0) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"bcm59035_disable_irq : PMU reg read error !!!\n");
		return st;
	}
	regVal |= (1 << IRQ_TO_REG_BIT(irq));
	st = bcm59035->write_dev(bcm59035, regInx + BCM59035_INT_MASK_BASE,
				 regVal);
	if (st < 0) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"bcm59035_disable_irq : PMU reg write error !!!\n");
		return st;
	}

	handler = bcm59035_find_irq_handler(bcm59035, irq);
	if (handler) {
		handler->irq_enabled = false;
	}
	return 0;
}


/**
* @brief request the PMU core to enable a particular IRQ
*
* @param bcm59035 PMU core driver private data structure
* @param irq irq number to be enabled
*
* @return returns 0 on success and negative error value on failure
*
* PMU core manages the interrupts from PMU and handles over the interrupts to
* respective sub device drivers. This method can be used by sub device drivers
* to enable a particular irq. Only when this is enabled PMU core will deliver
* the interrupt to sub device driver.
*/
int bcm59035_enable_irq(struct bcm59035 *bcm59035, int irq)
{
	int regInx;
	int st;
	u8 regVal;
	struct bcm_pmu_irq *handler;

	PMU_LOG(DEBUG_PMU_INFO, " Inside %s\n", __FUNCTION__);

	if (irq < 0 || irq > BCM59035_TOTAL_IRQ)
		return -EINVAL;

	regInx = IRQ_TO_REG_INX(irq);

	st = bcm59035->read_dev(bcm59035, regInx + BCM59035_INT_MASK_BASE,
				&regVal);
	if (st < 0) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"bcm59035_enable_irq : PMU reg read error !!!\n");
		return st;
	}
	regVal &= ~(1 << IRQ_TO_REG_BIT(irq));
	st = bcm59035->write_dev(bcm59035, regInx + BCM59035_INT_MASK_BASE,
				 regVal);
	if (st < 0) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"bcm59035_enable_irq : PMU reg write error !!!\n");
		return st;
	}

	handler = bcm59035_find_irq_handler(bcm59035, irq);
	if (!handler) {
		PMU_LOG(DEBUG_PMU_WARNING,
			"bcm59035_enable_irq : Enabling PMU irq without registering handler!!!\n");
	} else {
		handler->irq_enabled = true;
	}
	return 0;
}


/**
* @brief Request the PMU core to register IOCTL handler for a PMU sub device
*
* @param bcm59035 PMU core driver private data structure
* @param sub_dev_id PMU sub device ID
* @param handler IOCTL handler of the sub device
* @param pri_data Private data, if any, to be passed to the IOCTL handler on invoking.
*
* @return returns 0 on success and negative error value on failure.
*
* PMU core manages the ioctls to the sub device drivers. This  method allows
* the sub device drivers to register handlers with PMU core.
*/
int bcm59035_register_ioctl_handler(struct bcm59035 *bcm59035, u8 sub_dev_id,
				    pmu_subdev_ioctl_handler handler,
				    void *pri_data)
{
	PMU_LOG(DEBUG_PMU_INFO, " Inside %s\n", __FUNCTION__);

	if (sub_dev_id >= BCM59035_SUBDEV_MAX || !handler)
		return -EINVAL;

	if (bcm59035->ioctl_handler[sub_dev_id].handler) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"%s: Handler already registered - id = %u\n",
			__FUNCTION__, sub_dev_id);
		return -EINVAL;
	}

	bcm59035->ioctl_handler[sub_dev_id].handler = handler;
	bcm59035->ioctl_handler[sub_dev_id].pri_data = pri_data;

	return 0;
}


/**
* @brief Turn off the BCM59035 PMU. This will inturn turns off the Host(SOC).
*
* @param bcm59035 PMU core driver private data structure
*
* This method turns off the PMU. So all the power supplies from PMU will be
* off and hence the host also will shutdown.
*/
void bcm59035_shutdown(struct bcm59035 *bcm59035)
{
	int ret = 0;
	u8 hostact;
	PMU_LOG(DEBUG_PMU_INFO, "inside %s\n", __func__);

	ret = bcm59035->read_dev(bcm59035, BCM59035_REG_HOSTACT, &hostact);
	if (!ret) {
		hostact |= BCM59035_HOSTACT_HOSTDICOFF;
		bcm59035->write_dev(bcm59035, BCM59035_REG_HOSTACT, hostact);
	}
}
/** @} */
EXPORT_SYMBOL_GPL(bcm59035_request_irq);
EXPORT_SYMBOL_GPL(bcm59035_free_irq);
EXPORT_SYMBOL_GPL(bcm59035_disable_irq);
EXPORT_SYMBOL_GPL(bcm59035_enable_irq);
EXPORT_SYMBOL_GPL(bcm59035_register_ioctl_handler);
EXPORT_SYMBOL(bcm59035_shutdown);

static void bcm59035_lock(struct bcm59035 *bcm59035)
{
	if (!mutex_trylock(&bcm59035->lock)) {
		mutex_lock(&bcm59035->lock);
	}
}

static void bcm59035_unlock(struct bcm59035 *bcm59035)
{
	mutex_unlock(&bcm59035->lock);
}

static void bcm59035_power_off(void)
{
	PMU_LOG(DEBUG_PMU_INFO, " Inside %s\n", __FUNCTION__);
	if (bcm59035_info)
		bcm59035_shutdown(bcm59035_info);
}

static void bcm59035_irq_workq(struct work_struct *work)
{
	struct bcm59035 *bcm59035 = container_of(work, struct bcm59035, work);
	int i;
	u8 intStatus[BCM59035_NUM_INT_REG];
	struct bcm_pmu_irq *handler;

	PMU_LOG(DEBUG_PMU_INFO, " Inside %s\n", __FUNCTION__);
	/* Read all interrupt status registers. All interrupt status registers are R&C */

	for (i = 0; i < BCM59035_NUM_INT_REG; i++) {
		if (bcm59035->
		    read_dev(bcm59035, BCM59035_INT_BASE + i, &intStatus[i])) {
			PMU_LOG(DEBUG_PMU_ERROR,
				"bcm59035_irq_workq : PMU reg read error !!!\n");
			return;
		}
	}

	mutex_lock(&bcm59035->list_lock);
	list_for_each_entry(handler, &bcm59035->irq_handlers, node) {
		if (handler->irq_enabled &&
		    (intStatus[IRQ_TO_REG_INX(handler->irq)] &
		     (1 << IRQ_TO_REG_BIT(handler->irq)))) {
			handler->handler(handler->irq, handler->data);
		}
	}
	mutex_unlock(&bcm59035->list_lock);
}

static irqreturn_t bcm59035_irq(int irq, void *dev_id)
{
	struct bcm59035 *bcm59035 = dev_id;
	PMU_LOG(DEBUG_PMU_INFO, " Inside %s\n", __func__);
	if (queue_work(bcm59035->pmu_workqueue,  &bcm59035->work) == 0) {
		printk(KERN_INFO "Work previously queued \n");
	}
	return IRQ_HANDLED;
}

static int bcm59035_client_dev_register(struct bcm59035 *bcm59035,
					const char *name)
{
	struct mfd_cell cell = { };

	cell.name = name;
	return mfd_add_devices(&bcm59035->client->dev, -1, &cell, 1, NULL, 0);
}

static int bcm59035_read_pmu_register(struct bcm59035 *bcm59035, u32 reg,
				      u8 *regVal)
{
	s32 data;
	bcm59035_lock(bcm59035);
	data = i2c_smbus_read_byte_data(bcm59035->client, reg);
	bcm59035_unlock(bcm59035);
	if (data < 0) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"bcm59035_read_pmu_register : PMU read error  !!!");
		return data;
	}
	*regVal = (u8) data;
	return 0;
}

static int bcm59035_write_pmu_register(struct bcm59035 *bcm59035, u32 reg,
				       u8 val)
{
	int ret;
	bcm59035_lock(bcm59035);
	ret = i2c_smbus_write_byte_data(bcm59035->client, reg, val);
	bcm59035_unlock(bcm59035);
	return ret;
}

static void bcm59035_init_chip(struct bcm59035 *bcm59035)
{
	int i;
	u8 regVal, reg;

	bcm59035->read_dev(bcm59035, BCM59035_REG_CSRCTRL1, &regVal);
	if (BCM59035_ENABLE_DVS & bcm59035->flags) {
		regVal &= ~BCM59035_CSRCTRL1_CSR_MASK;
		regVal |=
		    (BCM59035_CSRCTRL1_DVS_EN | bcm59035->pdata->csr_lpm_volt);
		bcm59035->write_dev(bcm59035, BCM59035_REG_CSRCTRL10,
				    bcm59035->pdata->csr_nm_volt);

	} else {
		regVal &= ~BCM59035_CSRCTRL1_DVS_EN;
		bcm59035->read_dev(bcm59035, BCM59035_REG_CSRCTRL2, &reg);
		reg &= ~BCM59035_CSRCTRL2_CSR_MASK;
		reg |= (bcm59035->pdata->csr_nm_volt << 0x2);
		bcm59035->write_dev(bcm59035, BCM59035_REG_CSRCTRL2, reg);
	}
	bcm59035->write_dev(bcm59035, BCM59035_REG_CSRCTRL1, regVal);

	/*Read & clear all interrupts */
	for (i = 0; i < BCM59035_NUM_INT_REG; i++) {
		bcm59035->read_dev(bcm59035, BCM59035_REG_INT1 + i, &regVal);
	}
	/*mask all interrupts */
	for (i = BCM59035_REG_INT1M; i <= BCM59035_REG_INT10M; i++)
		bcm59035->write_dev(bcm59035, i, 0xFF);
}

static int bcm59035_open(struct inode *inode, struct file *file)
{
	PMU_LOG(DEBUG_PMU_INFO, " Inside %s \n", __FUNCTION__);
	file->private_data = PDE(inode)->data;
	return 0;
}

int bcm59035_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static int bcm59035_ioctl(struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	struct bcm59035 *bcm59035 = file->private_data;
	pmu_reg reg;
	pmu_pwm_ctrl pwm_ctrl;
	int pwrctrl;
	int ret = 0, i;
	u8 val = 0;
	u8 regVal;

	PMU_LOG(DEBUG_PMU_INFO, " Inside %s \n", __FUNCTION__);

	if (!bcm59035)
		return -ENOTTY;
	switch (cmd) {
	case BCM_PMU_IOCTL_ENABLE_INTS:
		/* Clear all latched interrupts if any */
		for (i = 0; i < BCM59035_NUM_INT_REG; i++) {
			if (bcm59035->
			    read_dev(bcm59035, BCM59035_INT_BASE + i,
				     &regVal)) {
				pr_info
				    ("bcm59035_irq_workq : PMU reg read error !!!\n");
				return -EFAULT;
			}
			pr_info("Int Register 0x%x = 0x%x\n",
			       (BCM59035_INT_BASE + i), regVal);
		}
		enable_irq(bcm59035->irq);
		break;
	case BCM_PMU_IOCTL_DISABLE_INTS:
		disable_irq_nosync(bcm59035->irq);
		break;

	case BCM_PMU_IOCTL_READ_REG:
		if (copy_from_user(&reg, (pmu_reg *) arg, sizeof(pmu_reg)) != 0) {
			return -EFAULT;
		}
		ret = bcm59035->read_dev(bcm59035, reg.reg, &reg.val);

		if (ret < 0)
			return -EFAULT;

		if (copy_to_user((pmu_reg *) arg, &reg, sizeof(pmu_reg)) != 0) {
			return -EFAULT;
		}
		break;

	case BCM_PMU_IOCTL_WRITE_REG:

		if (copy_from_user(&reg, (pmu_reg *) arg, sizeof(pmu_reg)) != 0) {
			return -EFAULT;
		}
		ret = bcm59035->write_dev(bcm59035, reg.reg, reg.val);
		break;

	case BCM_PMU_IOCTL_SET_VOLTAGE:
	case BCM_PMU_IOCTL_GET_VOLTAGE:
	case BCM_PMU_IOCTL_GET_REGULATOR_STATE:
	case BCM_PMU_IOCTL_SET_REGULATOR_STATE:
	case BCM_PMU_IOCTL_ACTIVATESIM:
	case BCM_PMU_IOCTL_DEACTIVATESIM:
		if (bcm59035->ioctl_handler[BCM59035_SUBDEV_REGULATOR].handler)
			return bcm59035->
			    ioctl_handler[BCM59035_SUBDEV_REGULATOR].
			    handler(cmd, arg,
				    bcm59035->
				    ioctl_handler[BCM59035_SUBDEV_REGULATOR].
				    pri_data);
		else
			return -ENOTTY;

	case BCM_PMU_IOCTL_SET_PWM_HI_PER:
	case BCM_PMU_IOCTL_SET_PWM_LO_PER:

		return -ENOTTY;

	case BCM_PMU_IOCTL_START_CHARGING:
	case BCM_PMU_IOCTL_STOP_CHARGING:
	case BCM_PMU_IOCTL_SET_CHARGING_CURRENT:
	case BCM_PMU_IOCTL_GET_CHARGING_CURRENT:
		if (bcm59035->ioctl_handler[BCM59035_SUBDEV_POWER].handler)
			return bcm59035->ioctl_handler[BCM59035_SUBDEV_POWER].
			    handler(cmd, arg,
				    bcm59035->
				    ioctl_handler[BCM59035_SUBDEV_POWER].
				    pri_data);
		else
			return -ENOTTY;

	case BCM_PMU_IOCTL_SET_PWM_LED_CTRL:

		if (copy_from_user
		    (&pwm_ctrl, (pmu_pwm_ctrl *) arg,
		     sizeof(pmu_pwm_ctrl)) != 0) {
			return -EFAULT;
		}
		PMU_LOG(DEBUG_PMU_INFO,
			" %s : BCM_PMU_IOCTL_SET_PWM_LED_CTRL : reg = %x \n",
			__FUNCTION__, pwm_ctrl.regoffset);
		if ((pwm_ctrl.pwmled_ctrl > BCM59035_LEDON_MASK)
		    || (pwm_ctrl.pwmdiv > BCM59035_LEDON_MASK)
		    || ((pwm_ctrl.regoffset != BCM59035_REG_PWMLEDCTRL1)
			&& (pwm_ctrl.regoffset != BCM59035_REG_PWMLEDCTRL6))) {
			return -EINVAL;
		}
		/* set divider and pwmled control. */
		val = ((val | (pwm_ctrl.pwmdiv << 2)) | (pwm_ctrl.pwmled_ctrl));

		/* update register */
		ret = bcm59035->write_dev(bcm59035, pwm_ctrl.regoffset, val);
		break;

	case BCM_PMU_IOCTL_POWERONOFF:
		bcm59035_shutdown(bcm59035);
		break;

	case BCM_PMU_IOCTL_SET_PWM_PWR_CTRL:
		if (copy_from_user(&pwrctrl, (int *)arg, sizeof(int)) != 0) {
			return -EFAULT;
		}
		if (pwrctrl > 1) {
			PMU_LOG(DEBUG_PMU_ERROR,
				"Power contrl value can be either 0 or 1, given val = 0x%x\n",
				pwrctrl);
			return -EINVAL;
		}
		/* read current settings */
		ret =
		    bcm59035->read_dev(bcm59035, BCM59035_REG_PWMLEDCTRL5,
				       &regVal);
		if (ret < 0) {
			PMU_LOG(DEBUG_PMU_ERROR,
				"error reading pwm control register.\n");
			return ret;
		}

		if (pwrctrl == 0) {
			val = regVal & (~(BCM59035_PWMLED_PDN));	/* Disable bit 6, set to 0. */
		} else {
			val = regVal | (BCM59035_PWMLED_PDN);	/* Enable bit 6, set to 1. */
		}

		/* update register */
		ret =
		    bcm59035->write_dev(bcm59035, BCM59035_REG_PWMLEDCTRL5,
					val);

		break;

	default:
		PMU_LOG(DEBUG_PMU_ERROR, "bcm59035_ioctl: UNSUPPORTED CMD\n");
		ret = -ENOTTY;
	}
	return ret;
}

#define MAX_USER_INPUT_LEN      100
#define MAX_REGS_READ_WRITE     10

enum pmu_debug_ops {
	PMUDBG_READ_REG = 0UL,
	PMUDBG_WRITE_REG,
};

struct pmu_debug {
	int read_write;
	int len;
	int addr;
	u8 val[MAX_REGS_READ_WRITE];
};

static void pmu_dbg_usage(void)
{
	printk(KERN_INFO "Usage:\n");
	printk(KERN_INFO "Read a register: echo 0x0800 > /proc/pmu0\n");
	printk(KERN_INFO
		"Read multiple regs: echo 0x0800 -c 10 > /proc/pmu0\n");
	printk(KERN_INFO
		"Write multiple regs: echo 0x0800 0xFF 0xFF > /proc/pmu0\n");
	printk(KERN_INFO
		"Write single reg: echo 0x0800 0xFF > /proc/pmu0\n");
	printk(KERN_INFO "Max number of regs in single write is :%d\n",
		MAX_REGS_READ_WRITE);
	printk(KERN_INFO "Register address is encoded as follows:\n");
	printk(KERN_INFO "0xSSRR, SS: i2c slave addr, RR: register addr\n");
}

static int pmu_dbg_parse_args(char *cmd, struct pmu_debug *dbg)
{
	char *tok;                 /* used to separate tokens             */
	const char ct[] = " \t";   /* space or tab delimits the tokens    */
	bool count_flag = false;   /* whether -c option is present or not */
	int tok_count = 0;         /* total number of tokens parsed       */
	int i = 0;

	dbg->len = 0;

	/* parse the input string */
	while ((tok = strsep(&cmd, ct)) != NULL) {
		pr_debug("token: %s\n", tok);

		/* first token is always address */
		if (tok_count == 0) {
			sscanf(tok, "%x", &dbg->addr);
		} else if (strnicmp(tok, "-c", 2) == 0) {
			/* the next token will be number of regs to read */
			tok = strsep(&cmd, ct);
			if (tok == NULL)
				return -EINVAL;

			tok_count++;
			sscanf(tok, "%d", &dbg->len);
			count_flag = true;
			break;
		} else {
			int val;

			/* this is a value to be written to the pmu register */
			sscanf(tok, "%x", &val);
			if (i < MAX_REGS_READ_WRITE) {
				dbg->val[i] = val;
				i++;
			}
		}

		tok_count++;
	}

	/* decide whether it is a read or write operation based on the
	 * value of tok_count and count_flag.
	 * tok_count = 0: no inputs, invalid case.
	 * tok_count = 1: only reg address is given, so do a read.
	 * tok_count > 1, count_flag = false: reg address and atleast one
	 *     value is present, so do a write operation.
	 * tok_count > 1, count_flag = true: to a multiple reg read operation.
	 */
	switch (tok_count) {
	case 0:
		return -EINVAL;
	case 1:
		dbg->read_write = PMUDBG_READ_REG;
		dbg->len = 1;
		break;
	default:
		if (count_flag == true) {
			dbg->read_write = PMUDBG_READ_REG;
		} else {
			dbg->read_write = PMUDBG_WRITE_REG;
			dbg->len = i;
		}
	}

	return 0;
}

static ssize_t bcm59035_write(struct file *file, const char __user *buffer,
			      size_t len, loff_t *offset)
{
	struct bcm59035 *bcm59035 = file->private_data;
	struct pmu_debug dbg;
	char cmd[MAX_USER_INPUT_LEN];
	int ret, i;

	pr_debug("%s\n", __func__);

	if (!bcm59035) {
		pr_err("%s: driver not initialized\n", __func__);
		return -EINVAL;
	}

	if (len > MAX_USER_INPUT_LEN)
		len = MAX_USER_INPUT_LEN;

	if (copy_from_user(cmd, buffer, len)) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return -EFAULT;
	}

	/* chop of '\n' introduced by echo at the end of the input */
	if (cmd[len - 1] == '\n')
		cmd[len - 1] = '\0';

	if (pmu_dbg_parse_args(cmd, &dbg) < 0) {
		pmu_dbg_usage();
		return -EINVAL;
	}

	pr_debug("operation: %s\n", (dbg.read_write == PMUDBG_READ_REG) ?
		"read" : "write");
	pr_debug("address  : 0x%x\n", dbg.addr);
	pr_debug("length   : %d\n", dbg.len);

	if (dbg.read_write == PMUDBG_READ_REG) {
		ret = i2c_smbus_read_i2c_block_data(
			bcm59035->client, dbg.addr, dbg.len, dbg.val);
		if (ret < dbg.len) {
			pr_err("%s: pmu reg read failed\n", __func__);
			return -EFAULT;
		}

		for (i = 0; i < dbg.len; i++, dbg.addr++)
			printk(KERN_INFO "[%x] = 0x%02x\n", dbg.addr,
				dbg.val[i]);
	} else {
		ret = i2c_smbus_write_i2c_block_data(
			bcm59035->client, dbg.addr, dbg.len, dbg.val);
		if (ret < 0) {
			pr_err("%s: pmu reg write failed\n", __func__);
			return -EFAULT;
		}
	}

	*offset += len;

	return len;
}

static const struct file_operations bcm59035_pmu_ops = {
	.open = bcm59035_open,
	.ioctl = bcm59035_ioctl,
	.write = bcm59035_write,
	.release = bcm59035_release,
	.owner = THIS_MODULE,
};

static int __devinit bcm59035_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	int ret;
	struct bcm59035 *bcm59035;
	struct bcm59035_platform_data *pdata = client->dev.platform_data;

	PMU_LOG(DEBUG_PMU_INFO, " Inside %s : Client name = %s\n", __FUNCTION__,
		client->name);
	/*  if(!pdata || pdata->pmu_id != 0x20)
	   {
	   PMU_LOG(DEBUG_PMU_ERROR, "Invalid platform data !!\n");
	   return -EINVAL;
	   } */

	bcm59035 = kzalloc(sizeof(struct bcm59035), GFP_KERNEL);
	if (!bcm59035)
		return -ENOMEM;
	i2c_set_clientdata(client, bcm59035);
	bcm59035->client = client;
	bcm59035->pdata = pdata;
	bcm59035->irq = client->irq;
	bcm59035->flags = pdata->flags;
	INIT_LIST_HEAD(&bcm59035->irq_handlers);
	INIT_WORK(&bcm59035->work, bcm59035_irq_workq);
	bcm59035->pmu_workqueue = create_workqueue("pmu_events");
	if (!bcm59035->pmu_workqueue) {
	    printk(KERN_ERR "failed to create work queue");
	    return -ESRCH;
	}
	mutex_init(&bcm59035->list_lock);
	mutex_init(&bcm59035->lock);
	bcm59035->read_dev = bcm59035_read_pmu_register;
	bcm59035->write_dev = bcm59035_write_pmu_register;

	bcm59035_info = bcm59035;

	bcm59035_init_chip(bcm59035);
	/*Do platform specific init */
	if (pdata->pmu_event_cb)
		pdata->pmu_event_cb(PMU_EVENT_INIT_PLATFORM, 0);

	/* Register battery charging device     */
	if (bcm59035->flags & BCM59035_USE_POWER)
		bcm59035_client_dev_register(bcm59035, "bcm59035-power");
	/* Register gpio device */
	if (bcm59035->flags & BCM59035_USE_GPIO)
		bcm59035_client_dev_register(bcm59035, "bcm59035-gpio");
	/* Register regulator device */
	if (bcm59035->flags & BCM59035_USE_REGULATORS)
		bcm59035_client_dev_register(bcm59035, "bcm59035-regulator");
	/* Register RTC device */
	if (bcm59035->flags & BCM59035_USE_RTC)
		bcm59035_client_dev_register(bcm59035, "bcm59035-rtc");
	/* Register PONKEY device */
	if (bcm59035->flags & BCM59035_USE_PONKEY)
		bcm59035_client_dev_register(bcm59035, "bcm59035-ponkey");
	/* Register PWM device */
	if (bcm59035->flags & BCM59035_USE_PWM)
		bcm59035_client_dev_register(bcm59035, "bcm59035-pwm");
	/* Register LED device */
	if (bcm59035->flags & BCM59035_USE_LED)
		bcm59035_client_dev_register(bcm59035, "bcm59035-led");

	if (bcm59035->flags & BCM59035_REGISTER_POWER_OFF) {
		PMU_LOG(DEBUG_PMU_INFO, "Registering pm_power_off function\n");
		pm_power_off = bcm59035_power_off;
	}

	ret = request_irq(client->irq, bcm59035_irq,
		(IRQF_DISABLED | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND),
		"bcm59035_irq", bcm59035);
	if (ret < 0) {
		PMU_LOG(DEBUG_PMU_ERROR,
			"Failed to register BCM59035 PMU IRQ !!!- ret = %d\n",
			ret);
		kfree(bcm59035);
		return ret;
	}
	disable_irq(client->irq);

	proc_create_data("pmu0", S_IRWXUGO, NULL, &bcm59035_pmu_ops, bcm59035);
	enable_irq(client->irq);

	return 0;
}

static int __devexit bcm59035_remove(struct i2c_client *client)
{
	struct bcm59035 *bcm59035;
	bcm59035 = i2c_get_clientdata(client);
	mfd_remove_devices(&client->dev);
	mutex_destroy(bcm59035->list_lock);
	mutex_destroy(bcm59035->lock);
	remove_proc_entry("pmu0", NULL);
	kfree(bcm59035);
	return 0;
}

static struct i2c_device_id bcm59035_idtable[] = {
	{"bcm59035", 0}
	,
	{}
};

MODULE_DEVICE_TABLE(i2c, pmu_idtable);
static struct i2c_driver bcm59035_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "bcm59035"
	},
	.id_table = bcm59035_idtable,
	.probe = bcm59035_probe,
	.remove = __devexit_p(bcm59035_remove),
};

/*
=======================================================================================================================
=======================================================================================================================
*/

static int __init bcm59035_init(void)
{
	PMU_LOG(DEBUG_PMU_INFO, "inside bcm59035_init\n");
	return i2c_add_driver(&bcm59035_driver);
}

static void __exit bcm59035_exit(void)
{
	i2c_del_driver(&bcm59035_driver);
}

subsys_initcall(bcm59035_init);
module_exit(bcm59035_exit);

MODULE_DESCRIPTION("Core/Protocol driver for Broadcom PMU Chip");
MODULE_LICENSE("GPL");
