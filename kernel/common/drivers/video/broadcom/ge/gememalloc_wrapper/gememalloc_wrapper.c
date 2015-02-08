/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*	@file	drivers/video/.../gememalloc_wrapper/gememalloc_wrapper.c
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

#include <linux/kernel.h>
#include <linux/module.h>
/* needed for __init,__exit directives */
#include <linux/init.h>
/* needed for remap_page_range */
#include <linux/mm.h>
/* obviously, for kmalloc */
#include <linux/slab.h>
/* for struct file_operations, register_chrdev() */
#include <linux/fs.h>
/* standard error codes */
#include <linux/errno.h>
/* this header files wraps some common module-space operations ...
   here we use mem_map_reserve() macro */
#include <linux/dma-mapping.h>
#include <linux/ioport.h>
#include <linux/list.h>
/* for current pid */
#include <linux/sched.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/bcm_gememalloc_wrapper.h>
#include <linux/broadcom/bcm_gememalloc_ioctl.h>

/* module description */
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Braodcom");

#define DRV_NAME "gememalloc"
#define DEV_NAME "gememalloc"

#define gememalloc_BASIC 0
#define gememalloc_MAX_OUTPUT 1
#define gememalloc_SIZE  (1024*1024*2)
//(SZ_4M)

static dma_addr_t dma_cohr_start_addr;
static void *alloc_mem;


static int gememalloc_major = BCM_GEMEMALLOC_MAJOR;

static struct list_head heap_list;

static DEFINE_SPINLOCK(mem_lock);

static struct gememalloc_logic logic;

static int gememalloc_wrapper_ioctl(struct inode *inode, struct file *filp,
				  unsigned int cmd, unsigned long arg)
{
	int result = -1;

	pr_debug(KERN_DEBUG "gememalloc_wrapper: ioctl cmd 0x%08x\n", cmd);

	if (inode == NULL || filp == NULL || arg == 0)
		return -EFAULT;
	/*
	 * extract the type and number bitfields, and don't decode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if (_IOC_TYPE(cmd) != GEMEMALLOC_WRAP_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > GEMEMALLOC_WRAP_MAXNR)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		result = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		result = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
	if (result)
		return -EFAULT;

	switch (cmd) {

	case GEMEMALLOC_WRAP_ACQUIRE_BUFFER:
		{
			GEMemallocwrapParams memparams;

			pr_debug(KERN_DEBUG "gememalloc_wrapper: ALLOC BUFFER\n");
			spin_lock(&mem_lock);

			__copy_from_user(&memparams, (const void *)arg,
					 sizeof(memparams));

			result =
			    logic.AllocMemory(filp->private_data, &memparams.busAddress,
					       memparams.size);

			__copy_to_user((void *)arg, &memparams,
				       sizeof(memparams));

			spin_unlock(&mem_lock);
		}
		break;
	case GEMEMALLOC_WRAP_RELEASE_BUFFER:
		{

			unsigned long busaddr;
			pr_debug(KERN_DEBUG
				 "gememalloc_wrapper: RELEASE BUFFER\n");
			spin_lock(&mem_lock);
			__get_user(busaddr, (unsigned long *)arg);
			result = logic.FreeMemory(filp->private_data, &busaddr);

			spin_unlock(&mem_lock);
		}
		break;
	}

	return result;
}

static int gememalloc_wrapper_open(struct inode *inode, struct file *filp)
{
	int r;

	spin_lock(&mem_lock);
	r = logic.open(&filp->private_data);
	spin_unlock(&mem_lock);

	pr_debug(KERN_DEBUG "gememalloc_wrapper_open\n");

	return r;
}

static int gememalloc_wrapper_release(struct inode *inode, struct file *filp)
{

	int r;

	spin_lock(&mem_lock);
	r = logic.release(filp->private_data);
	filp->private_data = NULL;
	spin_unlock(&mem_lock);

	pr_debug(KERN_DEBUG "gememalloc_wrapper_release\n");

	return r;
}
#define pgprot_cached(prot) \
__pgprot((pgprot_val(prot) & ~L_PTE_MT_MASK) | L_PTE_MT_WRITETHROUGH)

static int gememalloc_wrapper_mmap(struct file *file, struct vm_area_struct *vma)
{
	int r;
	unsigned long size = vma->vm_end - vma->vm_start;

	r = logic.mmap(size, vma->vm_pgoff);

//	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_page_prot = pgprot_cached(vma->vm_page_prot);
	/* Remap-pfn-range will mark the range VM_IO and VM_RESERVED */
	if (remap_pfn_range(vma,
			    vma->vm_start,
			    vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		pr_err("%s(): remap_pfn_range() failed\n", __FUNCTION__);
		return -1;
	}

	return r;
}

/* VFS methods */
static const struct file_operations gememalloc_wrapper_fops = {
      .open	= gememalloc_wrapper_open,
      .release	= gememalloc_wrapper_release,
      .ioctl	= gememalloc_wrapper_ioctl,
      .mmap	= gememalloc_wrapper_mmap,
};

int register_gememalloc_wrapper(struct gememalloc_logic *gememalloc_fops)
{
	pr_debug(KERN_DEBUG "register_gememalloc_wrapper\n");
	memcpy(&logic, gememalloc_fops, sizeof(logic));

	return logic.init(gememalloc_SIZE, dma_cohr_start_addr);
}
EXPORT_SYMBOL(register_gememalloc_wrapper);

void deregister_gememalloc_wrapper(void)
{
	if (logic.cleanup)
		logic.cleanup();
}
EXPORT_SYMBOL(deregister_gememalloc_wrapper);

int __init gememalloc_wrapper_init(void)
{
	int result;

	pr_debug(KERN_DEBUG "gememalloc_init\n");

	result =
	    register_chrdev(gememalloc_major, "gememalloc", &gememalloc_wrapper_fops);
	if (result < 0) {
		pr_debug(KERN_ERR "gememalloc_wrapper: unable to get major %d\n",
			 gememalloc_major);
		return result;
	} else if (result != 0) {	/* this is for dynamic major */
		gememalloc_major = result;
	}
	pr_info("gememalloc_wrapper: major =%d\n",gememalloc_major);

	alloc_mem = dma_alloc_coherent(NULL, gememalloc_SIZE,
				       &dma_cohr_start_addr,
				       GFP_ATOMIC | GFP_DMA);

	if (alloc_mem == NULL) {
		result = -ENOMEM;
		pr_err("gememalloc_wrapper: DMA Allocation failed\n");
		goto err;
	}
	pr_info("gememalloc_wrapper: module inserted. Major = %d\n",gememalloc_major);

	return 0;

err:
	if (alloc_mem != NULL)
		dma_free_coherent(NULL, gememalloc_SIZE, alloc_mem,
				  dma_cohr_start_addr);
	return result;
}

void __exit gememalloc_wrapper_cleanup(void)
{

	pr_debug(KERN_DEBUG "gememalloc_wrapper_cleanup\n");

	if (alloc_mem != NULL)
		dma_free_coherent(NULL, gememalloc_SIZE, alloc_mem,
				  dma_cohr_start_addr);
	unregister_chrdev(gememalloc_major, "gememalloc");

	pr_notice("gememalloc_wrapper: module removed\n");
	return;
}

module_init(gememalloc_wrapper_init);
module_exit(gememalloc_wrapper_cleanup);
