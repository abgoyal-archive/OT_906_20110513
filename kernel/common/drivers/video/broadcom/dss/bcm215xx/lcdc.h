/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/video/broadcom/dss/bcm215xx/lcdc.h
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

#ifndef __BCM_LCDC_H
#define __BCM_LCDC_H

#include <linux/broadcom/lcd.h>
#include <cfg_global.h>
/* #include <hw_cfg.h> */

typedef void (*lcd_DMAEND_CBF) (void);

#ifndef LCD_WTR_HOLD
#define LCD_WTR_HOLD	0x00010000
#endif
#ifndef LCD_WTR_PULSE
#define LCD_WTR_PULSE	0x00000800	/* 100ns @ MSP_CLK 78MHZ */
#endif
#ifndef LCD_WTR_SETUP
#define LCD_WTR_SETUP	0x00000001
#endif

#define LCD_Write_Timing 	(LCD_WTR_HOLD | LCD_WTR_PULSE | LCD_WTR_SETUP)

#ifdef LCD_RTR_HOLD
#define LCD_Read_Timing	(LCD_RTR_HOLD | LCD_RTR_PULSE | LCD_RTR_SETUP)
#else
#define LCD_Read_Timing	LCD_Write_Timing
#endif

/* RGB - 565 */
#define RED_MASK	0xF800
#define GREEN_MASK	0x07C0
#define BLUE_MASK	0x001F

#define RED_SHIFT	11
#define GREEN_SHIFT	5
#define BLUE_SHIFT	0

#define RGB_VAL(r, g, b)	(((r) << RED_SHIFT) | ((g) << GREEN_SHIFT) | ((b) << BLUE_SHIFT))

#ifdef USE_DMA
/* desired DMA channel: 1 is second highest priority */
#define DESIRED_DMA_CHANNEL	1

#define DMA_CONTROL	\
	(REG_DMA_CHAN_CTL_SRC_INCR		\
	| REG_DMA_CHAN_CTL_DEST_WIDTH_32	\
	| REG_DMA_CHAN_CTL_SRC_WIDTH_32		\
	| REG_DMA_CHAN_CTL_DEST_BURST_SIZE_4	\
	| REG_DMA_CHAN_CTL_SRC_BURST_SIZE_32)

#define DMA_DIVIDE_WIDTH	DMA_DIVIDE_WIDTH_32	/* 32-bit transfers */

#define DMA_CONFIG	\
	(REG_DMA_CHAN_CFG_TC_INT_ENABLE		\
	| REG_DMA_CHAN_CFG_ERROR_INT_ENABLE		\
	| REG_DMA_CHAN_CFG_FLOW_CTL_MEM_TO_PRF_DMA	\
	| (REG_DMA_PERIPHERAL_LCD << REG_DMA_CHAN_CFG_DEST_PERIPHERAL_SHIFT)	\
	| REG_DMA_CHAN_CFG_ENABLE)

/* definition of DMA Linked List Item (LLI) */
typedef struct {
	u32 source;		/* source address */
	u32 dest;		/* dest address */
	u32 link;		/* link to next LLI */
	u32 control;		/* control word */
} DMA_LLI_t;

/* structure for restoring overwritten LLI information */
typedef struct {
	int valid;		/* flag to show data is valid */
	int row;		/* row to store */
	u32 link;		/* link register */
	u32 control;		/* control register */
	DMA_LLI_t *list;	/* pointer to start of list (main vs sub) */
} DMA_Restore_t;

/* swap from big endian to little endian for 32 bits */
#define DMA_SWAP(val)		cpu_to_le32(val)
#define DMA_BYTES_PER_LLI	16	/* bytes per Linked List Item (LLI) */
#define DMA_DIVIDE_WIDTH_32	4	/* 4 bytes for 32-bit-wide transfer */
#define DMA_DIVIDE_WIDTH_16	2	/* 2 bytes for 16-bit-wide transfer */
#define DMA_DIVIDE_WIDTH_8 	1	/* 1 byte  for 8-bit-wide transfer */

#endif /* USE_DMA */

/* any panel-specific stuff */
extern LCD_Intf_t LCD_Intf;
extern LCD_Bus_t LCD_Bus;
extern const char *LCD_panel_name;
extern int LCD_num_panels;
extern LCD_dev_info_t LCD_device[];
void lcd_init_panels(void);
void lcd_poweroff_panels(void);
void lcd_ResetStartAddr(LCD_dev_info_t *dev);
void lcd_setup_for_data(LCD_dev_info_t *dev);

#endif /* __BCM_LCDC_H */
