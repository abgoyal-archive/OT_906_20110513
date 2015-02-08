/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*       @file   arch/arm/mach-bcm215xx/bcm215xx_pm.h
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

#ifndef __BCM215XX_PM_H__
#define __BCM215XX_PM_H__

#include <asm/sizes.h>

/*
 * Indices into the dormant context save buffer. Any update to this list
 * must reflect in the following declarations:
 *
 * 1. typedef struct bcm_pm_sleep (in this header file)
 * 2. static const char *bcm_sleep_buf_elem_names[] (in bcm215xx_pm.c)
 */
#define PM_DEEPSLEEP_ALLOWED          (0  << 2)
#define PM_DORMANT_ALLOWED            (1  << 2)
#define PM_SLEEP_MODE_COUNT           (2  << 2)
#define PM_DORMANT_MODE_COUNT	      (3  << 2)
#define PM_SLEEP_MODE_TIME_LOW        (4  << 2)
#define PM_SLEEP_MODE_TIME_HIGH       (5  << 2)
#define PM_PEDESTAL_MODE_COUNT        (6  << 2)
#define PM_PEDESTAL_MODE_TIME_LOW     (7  << 2)
#define PM_PEDESTAL_MODE_TIME_HIGH    (8  << 2)
#define PM_DORM_VERIFY                (9  << 2)
#define PM_CHECKSUM_ENTER             (10 << 2)
#define PM_CHECKSUM_EXIT              (11 << 2)
#define PM_CHECKSUM_COMPARE           (12 << 2)
#define PM_PRE_DORMANT_CPSR           (13 << 2)
#define PM_RST_VECTOR                 (14 << 2)
#define PM_UND_VECTOR                 (15 << 2)
#define PM_SWI_VECTOR                 (16 << 2)
#define PM_PBT_VECTOR                 (17 << 2)
#define PM_ABT_VECTOR                 (18 << 2)
#define PM_IRQ_VECTOR                 (19 << 2)
#define PM_FIQ_VECTOR                 (20 << 2)
#define PM_DOR_SAVE_STAT              (21 << 2)
#define PM_DOR_EXIT_STAT              (22 << 2)
#define PM_DOR_SAVE_CNT               (23 << 2)
#define PM_DOR_EXIT_CNT               (24 << 2)
#define PM_DOR_INT_CNT                (25 << 2)
#define PM_DOR_FAIL_CNT               (26 << 2)
#define PM_DOR_FAIL_STAT              (27 << 2)
#define PM_CONTEXT_BUF_PHYS           (28 << 2)
#define PM_DORM_SEQUENCE_DBG          (29 << 2)
#define PM_DORM_STORE_END             (30 << 2)
/* This must be the last element of the above list */
#define PM_NUM_SLEEP_ELEMENTS         (31)

#ifndef __ASSEMBLY__

/*
 * Dormant and sleep mode context buffer. This buffer also contains
 * fields to trace the sequence (for debugging).
 */
typedef struct bcm_pm_sleep {
	/*
	 * Sleep state control
	 */
	u32 deepsleep_allowed;
	u32 dormant_allowed;
	/*
	 * Counters to debug sleep mode states
	 */
	u32 sleep_mode_count;
	u32 dormant_mode_count;
	u32 sleep_mode_time_low;
	u32 sleep_mode_time_high;
	u32 pedestal_mode_count;
	u32 pedestal_mode_time_low;
	u32 pedestal_mode_time_high;
	/*
	 * Checksum verification status
	 */
	u32 verify;
	u32 chksum_enter;
	u32 chksum_exit;
	u32 chksum_compare;
	/*
	 * Save CPSR of current state. Dormant state context save happens
	 * in SYSTEM mode. Hence the previous mode CPSR is saved here
	 * before the dormant mode sequence begins.
	 */
	u32 pre_dormant_cpsr;
	/*
	 * These locations are used to take a backup of the current
	 * exception vector table contents. Dormant entry sequence
	 * replaces these three vector jump addresses with dormant wake
	 * sequence code.
	 */
	u32 rst_vector;
	u32 und_vector;
	u32 swi_vector;
	u32 pbt_vector;
	u32 dbt_vector;
	u32 irq_vector;
	u32 fiq_vector;
	/*
	 * These variables are used to trace the dormant mode flow. Examine
	 * these locations once control comes out of dormant to get an idea
	 * of how the dormant flow executed.
	 */
	u32 save_stat;
	u32 exit_stat;
	u32 save_cnt;
	u32 exit_cnt;
	u32 int_cnt;
	u32 fail_cnt;
	u32 fail_stat;
	/*
	 * Physical address of this structure. Dormant wakeup sequence
	 * code runs with MMU off. Hence it needs the physical address
	 * of this structure in a position-independent way to restore
	 * the ARM system context. Used by the assembly code.
	 */
	u32 context_buf_phys_addr;
	/*
	 * Set to 'true' to enable dormant mode save-restore sequence.
	 * If true, then WFI instruction is bypassed and control is
	 * transferred to the dormant restore sequence.
	 */
	u32 dorm_sequence_dbg;
	/*
	 * Stores the offset of the end of dormant context save buffer after
	 * the save sequence is completed. This offset is added in a position
	 * independent way during the resume sequence to restore the context.
	 */
	u32 dormant_store_end_offset;
	u32 dormant_store[SZ_1K - (PM_NUM_SLEEP_ELEMENTS * sizeof(u32))];

} bcm_pm_sleep;

/***************************************************************************
 * Assembly routines to move apps processor to sleep states.
 ****************************************************************************/
extern u32 *bcm215xx_dorm_wake_handler(void);
extern void bcm215xx_sleep(struct bcm_pm_sleep *pm_sleep);
extern void bcm215xx_dorm_exception_handler(void);
extern void bcm215xx_pabt_handler(void);
extern void bcm215xx_dabt_handler(void);

#endif /* !__ASSEMBLY__ */

#endif /* __BCM215XX_PM_H__ */
