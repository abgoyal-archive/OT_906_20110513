/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
* 
* 	@file	drivers/video/broadcom/displays/lcd_NT35582.c
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

/****************************************************************************
*
*  lcd_NT35582.c
*
*  PURPOSE:
*    This is the LCD-specific code for a Novatek NT35582 module.
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/types.h>
#include <linux/delay.h>

#if defined(CONFIG_BCM_IDLE_PROFILER_SUPPORT)
#include <linux/broadcom/idle_prof.h>
#endif
#include <linux/broadcom/cpu_sleep.h>

#include <linux/broadcom/hw.h>
#include <linux/broadcom/lcd.h>
#include <linux/broadcom/PowerManager.h>
#include <linux/broadcom/lcd_backlight.h>

#include <cfg_global.h>

#include "lcd.h"
#include "lcd_NT35582.h"

/*
 * Attributes of the LCD
 */
#ifdef CONFIG_HVGA_HACK_FOR_ATHENA_B0
#define LCD_HEIGHT		480
#define LCD_WIDTH		320
#else
#define LCD_HEIGHT		800
#define LCD_WIDTH		480
#endif
#define LCD_BITS_PER_PIXEL      16

/* ---- Public Variables ------------------------------------------------- */

/* globals for lcd_common.c to use */
int LCD_num_panels = 1;
LCD_dev_info_t LCD_device[1] = {
	{
	 .panel		= LCD_main_panel,
	 .height	= LCD_HEIGHT,
	 .width		= LCD_WIDTH,
	 .bits_per_pixel= LCD_BITS_PER_PIXEL,
	 .te_supported	= true,
	 }
};


const char *LCD_panel_name = "Novatek NT35582 LCD";

LCD_Intf_t LCD_Intf = LCD_Z80;
LCD_Bus_t LCD_Bus = LCD_18BIT;

/* ---- Private Variables ------------------------------------------------ */

typedef enum
{
	WR_CMND,
	WR_CMND_DATA,
	SLEEP_MS,
	CTRL_END,
}ctrl_t;

typedef struct lcd_init_t
{
    ctrl_t type;
    uint16_t cmd;
    uint16_t data;

} Lcd_init_t;

static Lcd_init_t g_init[] =
{
	{WR_CMND, NT35582_SLEEP_OUT, 0},
	{SLEEP_MS, 0, 200},
	{WR_CMND_DATA, NT35582_PWCTR1_0, 0x86},
	{WR_CMND_DATA, NT35582_PWCTR1_1, 0x00},
	{WR_CMND_DATA, NT35582_PWCTR1_2, 0x86},
	{WR_CMND_DATA, NT35582_PWCTR1_3, 0x00},
	{WR_CMND_DATA, NT35582_PWCTR2_0, 0x45},
	{WR_CMND_DATA, NT35582_PWCTR3_0, 0x21},
	{WR_CMND_DATA, NT35582_PWCTR3_2, 0x02},
	{WR_CMND_DATA, NT35582_SD_OP_SET_0, 0x30},
	{WR_CMND_DATA, NT35582_SD_OP_SET_2, 0x30},
	{WR_CMND_DATA, NT35582_VCOM, 0x8F},

#ifdef __WVGA_888U_TO_888P__
	{WR_CMND_DATA, NT35582_SET_PIXEL_FORMAT,PIXEL_FORMAT_RGB888},
#else
	{WR_CMND_DATA, NT35582_SET_PIXEL_FORMAT,PIXEL_FORMAT_RGB666},
#endif

	{WR_CMND_DATA, NT35582_GMACTRL_1_00, 0x0E},
	{WR_CMND_DATA, NT35582_GMACTRL_1_01, 0x14},
	{WR_CMND_DATA, NT35582_GMACTRL_1_02, 0x29},
	{WR_CMND_DATA, NT35582_GMACTRL_1_03, 0x3A},
	{WR_CMND_DATA, NT35582_GMACTRL_1_04, 0x1D},
	{WR_CMND_DATA, NT35582_GMACTRL_1_05, 0x30},
	{WR_CMND_DATA, NT35582_GMACTRL_1_06, 0x61},
	{WR_CMND_DATA, NT35582_GMACTRL_1_07, 0x3D},
	{WR_CMND_DATA, NT35582_GMACTRL_1_08, 0x22},
	{WR_CMND_DATA, NT35582_GMACTRL_1_09, 0x2A},
	{WR_CMND_DATA, NT35582_GMACTRL_1_0A, 0x87},
	{WR_CMND_DATA, NT35582_GMACTRL_1_0B, 0x16},
	{WR_CMND_DATA, NT35582_GMACTRL_1_0C, 0x3B},
	{WR_CMND_DATA, NT35582_GMACTRL_1_0D, 0x4C},
	{WR_CMND_DATA, NT35582_GMACTRL_1_0E, 0x78},
	{WR_CMND_DATA, NT35582_GMACTRL_1_0F, 0x96},
	{WR_CMND_DATA, NT35582_GMACTRL_1_10, 0x4A},
	{WR_CMND_DATA, NT35582_GMACTRL_1_11, 0x4D},
	{WR_CMND_DATA, NT35582_GMACTRL_2_00, 0x0E},
	{WR_CMND_DATA, NT35582_GMACTRL_2_01, 0x14},
	{WR_CMND_DATA, NT35582_GMACTRL_2_02, 0x29},
	{WR_CMND_DATA, NT35582_GMACTRL_2_03, 0x3A},
	{WR_CMND_DATA, NT35582_GMACTRL_2_04, 0x1D},
	{WR_CMND_DATA, NT35582_GMACTRL_2_05, 0x30},
	{WR_CMND_DATA, NT35582_GMACTRL_2_06, 0x61},
	{WR_CMND_DATA, NT35582_GMACTRL_2_07, 0x3F},
	{WR_CMND_DATA, NT35582_GMACTRL_2_08, 0x20},
	{WR_CMND_DATA, NT35582_GMACTRL_2_09, 0x26},
	{WR_CMND_DATA, NT35582_GMACTRL_2_0A, 0x83},
	{WR_CMND_DATA, NT35582_GMACTRL_2_0B, 0x16},
	{WR_CMND_DATA, NT35582_GMACTRL_2_0C, 0x3B},
	{WR_CMND_DATA, NT35582_GMACTRL_2_0D, 0x4C},
	{WR_CMND_DATA, NT35582_GMACTRL_2_0E, 0x78},
	{WR_CMND_DATA, NT35582_GMACTRL_2_0F, 0x96},
	{WR_CMND_DATA, NT35582_GMACTRL_2_10, 0x4A},
	{WR_CMND_DATA, NT35582_GMACTRL_2_11, 0x4D},
	{WR_CMND_DATA, NT35582_GMACTRL_3_00, 0x0E},
	{WR_CMND_DATA, NT35582_GMACTRL_3_01, 0x14},
	{WR_CMND_DATA, NT35582_GMACTRL_3_02, 0x29},
	{WR_CMND_DATA, NT35582_GMACTRL_3_03, 0x3A},
	{WR_CMND_DATA, NT35582_GMACTRL_3_04, 0x1D},
	{WR_CMND_DATA, NT35582_GMACTRL_3_05, 0x30},
	{WR_CMND_DATA, NT35582_GMACTRL_3_06, 0x61},
	{WR_CMND_DATA, NT35582_GMACTRL_3_07, 0x3D},
	{WR_CMND_DATA, NT35582_GMACTRL_3_08, 0x22},
	{WR_CMND_DATA, NT35582_GMACTRL_3_09, 0x2A},
	{WR_CMND_DATA, NT35582_GMACTRL_3_0A, 0x87},
	{WR_CMND_DATA, NT35582_GMACTRL_3_0B, 0x16},
	{WR_CMND_DATA, NT35582_GMACTRL_3_0C, 0x3B},
	{WR_CMND_DATA, NT35582_GMACTRL_3_0D, 0x4C},
	{WR_CMND_DATA, NT35582_GMACTRL_3_0E, 0x78},
	{WR_CMND_DATA, NT35582_GMACTRL_3_0F, 0x96},
	{WR_CMND_DATA, NT35582_GMACTRL_3_10, 0x4A},
	{WR_CMND_DATA, NT35582_GMACTRL_3_11, 0x4D},
	{WR_CMND_DATA, NT35582_GMACTRL_4_00, 0x0E},
	{WR_CMND_DATA, NT35582_GMACTRL_4_01, 0x14},
	{WR_CMND_DATA, NT35582_GMACTRL_4_02, 0x29},
	{WR_CMND_DATA, NT35582_GMACTRL_4_03, 0x3A},
	{WR_CMND_DATA, NT35582_GMACTRL_4_04, 0x1D},
	{WR_CMND_DATA, NT35582_GMACTRL_4_05, 0x30},
	{WR_CMND_DATA, NT35582_GMACTRL_4_06, 0x61},
	{WR_CMND_DATA, NT35582_GMACTRL_4_07, 0x3F},
	{WR_CMND_DATA, NT35582_GMACTRL_4_08, 0x20},
	{WR_CMND_DATA, NT35582_GMACTRL_4_09, 0x26},
	{WR_CMND_DATA, NT35582_GMACTRL_4_0A, 0x83},
	{WR_CMND_DATA, NT35582_GMACTRL_4_0B, 0x16},
	{WR_CMND_DATA, NT35582_GMACTRL_4_0C, 0x3B},
	{WR_CMND_DATA, NT35582_GMACTRL_4_0D, 0x4C},
	{WR_CMND_DATA, NT35582_GMACTRL_4_0E, 0x78},
	{WR_CMND_DATA, NT35582_GMACTRL_4_0F, 0x96},
	{WR_CMND_DATA, NT35582_GMACTRL_4_10, 0x4A},
	{WR_CMND_DATA, NT35582_GMACTRL_4_11, 0x4D},
	{WR_CMND_DATA, NT35582_GMACTRL_5_00, 0x0E},
	{WR_CMND_DATA, NT35582_GMACTRL_5_01, 0x14},
	{WR_CMND_DATA, NT35582_GMACTRL_5_02, 0x29},
	{WR_CMND_DATA, NT35582_GMACTRL_5_03, 0x3A},
	{WR_CMND_DATA, NT35582_GMACTRL_5_04, 0x1D},
	{WR_CMND_DATA, NT35582_GMACTRL_5_05, 0x30},
	{WR_CMND_DATA, NT35582_GMACTRL_5_06, 0x61},
	{WR_CMND_DATA, NT35582_GMACTRL_5_07, 0x3D},
	{WR_CMND_DATA, NT35582_GMACTRL_5_08, 0x22},
	{WR_CMND_DATA, NT35582_GMACTRL_5_09, 0x2A},
	{WR_CMND_DATA, NT35582_GMACTRL_5_0A, 0x87},
	{WR_CMND_DATA, NT35582_GMACTRL_5_0B, 0x16},
	{WR_CMND_DATA, NT35582_GMACTRL_5_0C, 0x3B},
	{WR_CMND_DATA, NT35582_GMACTRL_5_0D, 0x4C},
	{WR_CMND_DATA, NT35582_GMACTRL_5_0E, 0x78},
	{WR_CMND_DATA, NT35582_GMACTRL_5_0F, 0x96},
	{WR_CMND_DATA, NT35582_GMACTRL_5_10, 0x4A},
	{WR_CMND_DATA, NT35582_GMACTRL_5_11, 0x4D},
	{WR_CMND_DATA, NT35582_GMACTRL_6_00, 0x0E},
	{WR_CMND_DATA, NT35582_GMACTRL_6_01, 0x14},
	{WR_CMND_DATA, NT35582_GMACTRL_6_02, 0x29},
	{WR_CMND_DATA, NT35582_GMACTRL_6_03, 0x3A},
	{WR_CMND_DATA, NT35582_GMACTRL_6_04, 0x1D},
	{WR_CMND_DATA, NT35582_GMACTRL_6_05, 0x30},
	{WR_CMND_DATA, NT35582_GMACTRL_6_06, 0x61},
	{WR_CMND_DATA, NT35582_GMACTRL_6_07, 0x3F},
	{WR_CMND_DATA, NT35582_GMACTRL_6_08, 0x20},
	{WR_CMND_DATA, NT35582_GMACTRL_6_09, 0x26},
	{WR_CMND_DATA, NT35582_GMACTRL_6_0A, 0x83},
	{WR_CMND_DATA, NT35582_GMACTRL_6_0B, 0x16},
	{WR_CMND_DATA, NT35582_GMACTRL_6_0C, 0x3B},
	{WR_CMND_DATA, NT35582_GMACTRL_6_0D, 0x4C},
	{WR_CMND_DATA, NT35582_GMACTRL_6_0E, 0x78},
	{WR_CMND_DATA, NT35582_GMACTRL_6_0F, 0x96},
	{WR_CMND_DATA, NT35582_GMACTRL_6_10, 0x4A},
	{WR_CMND_DATA, NT35582_GMACTRL_6_11, 0x4D},
	{WR_CMND, NT35582_DISPLAY_ON, 0},
	{SLEEP_MS, 0, 200},
	{WR_CMND, NT35582_SET_TEAR_ON, 0},

	{CTRL_END, 0, 0}
};

static Lcd_init_t g_poweroff[] =
{
	{WR_CMND, NT35582_SLEEP_IN, 0},
	{SLEEP_MS, 0, 120},
	{CTRL_END, 0, 0}
};

/* ---- Functions -------------------------------------------------------- */

static int lcd_send_cmd_sequence(Lcd_init_t *init)
{
    int i;

    for (i = 0; init[i].type != CTRL_END; i++)
    {
	switch(init[i].type)
	{
		case WR_CMND:
			WRITE_LCD_CMD(init[i].cmd);
			break;
		case WR_CMND_DATA:
			WRITE_LCD_CMD(init[i].cmd);
			WRITE_LCD_PARAM(init[i].data);
			break;
		case SLEEP_MS:
			mdelay(init[i].data);
			break;
		default:break;
	}
    }
    return 0;
} // lcd_send_cmd_sequence


void lcd_ResetStartAddr(LCD_dev_info_t *dev)
{

#ifdef CONFIG_HVGA_HACK_FOR_ATHENA_B0
    /*Display the HVGA content at the center of the WVGA panel*/
    /*Update region will be (80, 160) - (399, 639)*/
    dev->row_start += 160;
    dev->row_end += 160;
    dev->col_start += 80;
    dev->col_end += 80;
#endif
	Lcd_init_t resetSeq[] = {
		//Horizontal start
		{WR_CMND_DATA, NT35582_SET_HOR_ADDR_S_MSB, (dev->col_start) >> 8},
		{WR_CMND_DATA, NT35582_SET_HOR_ADDR_S_LSB, dev->col_start & 0xFF},

		//Horizontal end
		{WR_CMND_DATA, NT35582_SET_HOR_ADDR_E_MSB, (dev->col_end) >> 8},
		{WR_CMND_DATA, NT35582_SET_HOR_ADDR_E_LSB, dev->col_end & 0xFF},

		//Vertical start
		{WR_CMND_DATA, NT35582_SET_VER_ADDR_S_MSB, (dev->row_start) >> 8},
		{WR_CMND_DATA, NT35582_SET_VER_ADDR_S_LSB, dev->row_start & 0xFF},

		//Vertical end
		{WR_CMND_DATA, NT35582_SET_VER_ADDR_E_MSB, (dev->row_end) >> 8},
		{WR_CMND_DATA, NT35582_SET_VER_ADDR_E_LSB, dev->row_end & 0xFF},

		//RAM Horizontal start address
		{WR_CMND_DATA, NT35582_SET_RAM_ADDR_X_MSB, (dev->row_start) >> 8},
		{WR_CMND_DATA, NT35582_SET_RAM_ADDR_X_LSB, dev->row_start & 0xFF},

		//RAM vertical start address
		{WR_CMND_DATA, NT35582_SET_RAM_ADDR_Y_MSB, (dev->col_start) >> 8},
		{WR_CMND_DATA, NT35582_SET_RAM_ADDR_Y_LSB, dev->col_start & 0xFF},
		{CTRL_END, 0, 0}
	};
#ifdef CONFIG_HVGA_HACK_FOR_ATHENA_B0
    /* Roll back to (0,0) - (319, 479)*/
    dev->row_start -= 160;
    dev->row_end -= 160;
    dev->col_start -= 80;
    dev->col_end -= 80;
#endif

	lcd_send_cmd_sequence(resetSeq);
}


void lcd_setup_for_data(LCD_dev_info_t *dev)
{
    lcd_ResetStartAddr(dev);

    WRITE_LCD_CMD(NT35582_WR_MEM_START);

}

void lcd_init_panels(void)
{
    /*LCD_PUTS("enter");*/

    lcd_send_cmd_sequence(g_init);

    /*LCD_PUTS("done");*/
}

void lcd_poweroff_panels(void)
{
    /*LCD_PUTS("enter");*/

    lcd_send_cmd_sequence(g_poweroff);

    /*LCD_PUTS("done");*/
}

