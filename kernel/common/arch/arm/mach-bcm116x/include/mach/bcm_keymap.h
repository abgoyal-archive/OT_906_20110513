/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/mach-bcm116x/include/mach/bcm_keymap.h
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

#include<linux/input.h>

/* These key codes are for Android */
#define KEY_STARS    227
#define KEY_POUNDS   228
#define KEY_MENUS    229
#define KEY_ACTIONS  232
#define KEY_FOCUS  211

#define BCM_KEY_ROW_0  0
#define BCM_KEY_ROW_1  1
#define BCM_KEY_ROW_2  2
#define BCM_KEY_ROW_3  3
#define BCM_KEY_ROW_4  4
#define BCM_KEY_ROW_5  5
#define BCM_KEY_ROW_6  6
#define BCM_KEY_ROW_7  7

#define BCM_KEY_COL_0  0
#define BCM_KEY_COL_1  1
#define BCM_KEY_COL_2  2
#define BCM_KEY_COL_3  3
#define BCM_KEY_COL_4  4
#define BCM_KEY_COL_5  5
#define BCM_KEY_COL_6  6
#define BCM_KEY_COL_7  7

/* The following define is casted to BCM_KEYMAP specified in /drivers/input/keyboard/bcm_keypad.c
   The last Defines for each gpio is actually fed to Linux input sybsystem when key is pressed.
*/
#define bcm2153_keymap \
{ \
   { BCM_KEY_ROW_0,    BCM_KEY_COL_0,  "Dpad down",      KEY_DOWN  }, \
   { BCM_KEY_ROW_0,    BCM_KEY_COL_1,  "Home key",       KEY_HOME  }, \
   { BCM_KEY_ROW_0,    BCM_KEY_COL_2,  "Dpad up",        KEY_UP    }, \
  /* KEY_END has been taken care by PONKEY so make it unused */ \
   { BCM_KEY_ROW_0,    BCM_KEY_COL_3,  "unused",                0  }, \
   { BCM_KEY_ROW_0,    BCM_KEY_COL_4,  "Menu",           KEY_MENUS }, \
   { BCM_KEY_ROW_0,    BCM_KEY_COL_5,  "unused",                0  }, \
   { BCM_KEY_ROW_0,    BCM_KEY_COL_6,  "unused",                0  }, \
   { BCM_KEY_ROW_0,    BCM_KEY_COL_7,  "unused",                0  }, \
   { BCM_KEY_ROW_1,    BCM_KEY_COL_0,  "Dpad Left",      KEY_LEFT  }, \
   { BCM_KEY_ROW_1,    BCM_KEY_COL_1,  "Num 1 ",         KEY_1     }, \
   { BCM_KEY_ROW_1,    BCM_KEY_COL_2,  "Num 2 ",         KEY_2     }, \
   { BCM_KEY_ROW_1,    BCM_KEY_COL_3,  "Num 3 ",         KEY_3     }, \
   { BCM_KEY_ROW_1,    BCM_KEY_COL_4,  "CAMERA",         KEY_CAMERA}, \
   { BCM_KEY_ROW_1,    BCM_KEY_COL_5,  "unused",                 0 }, \
   { BCM_KEY_ROW_1,    BCM_KEY_COL_6,  "unused",                 0 }, \
   { BCM_KEY_ROW_1,    BCM_KEY_COL_7,  "unused",                 0 }, \
   { BCM_KEY_ROW_2,    BCM_KEY_COL_0,  "Dpad right",     KEY_RIGHT }, \
   { BCM_KEY_ROW_2,    BCM_KEY_COL_1,  "Num 4 ",         KEY_4     }, \
   { BCM_KEY_ROW_2,    BCM_KEY_COL_2,  "Num 5 ",         KEY_5     }, \
   { BCM_KEY_ROW_2,    BCM_KEY_COL_3,  "Num 6 ",         KEY_6     }, \
   { BCM_KEY_ROW_2,    BCM_KEY_COL_4,  "Volue down",     KEY_VOLUMEDOWN}, \
   { BCM_KEY_ROW_2,    BCM_KEY_COL_5,  "unused",                 0 }, \
   { BCM_KEY_ROW_2,    BCM_KEY_COL_6,  "unused",                 0 }, \
   { BCM_KEY_ROW_2,    BCM_KEY_COL_7,  "unused",                 0 }, \
   { BCM_KEY_ROW_3,    BCM_KEY_COL_0,  "Answer",         KEY_SEND  }, \
   { BCM_KEY_ROW_3,    BCM_KEY_COL_1,  "num 7 ",         KEY_7     }, \
   { BCM_KEY_ROW_3,    BCM_KEY_COL_2,  "num 8 ",         KEY_8     }, \
   { BCM_KEY_ROW_3,    BCM_KEY_COL_3,  "num 9 ",         KEY_9     }, \
   { BCM_KEY_ROW_3,    BCM_KEY_COL_4,  "Volue up",       KEY_VOLUMEUP}, \
   { BCM_KEY_ROW_3,    BCM_KEY_COL_5,  "unused",                 0 }, \
   { BCM_KEY_ROW_3,    BCM_KEY_COL_6,  "unused",                 0 }, \
   { BCM_KEY_ROW_3,    BCM_KEY_COL_7,  "unused",                 0 }, \
   { BCM_KEY_ROW_4,    BCM_KEY_COL_0,  "Action key",     KEY_ACTIONS}, \
   { BCM_KEY_ROW_4,    BCM_KEY_COL_1,  "star key",       KEY_STARS }, \
   { BCM_KEY_ROW_4,    BCM_KEY_COL_2,  "Num 0 ",         KEY_0     }, \
   { BCM_KEY_ROW_4,    BCM_KEY_COL_3,  "Pound key",      KEY_POUNDS}, \
   { BCM_KEY_ROW_4,    BCM_KEY_COL_4,  "unused",         KEY_BACK  }, \
   { BCM_KEY_ROW_4,    BCM_KEY_COL_5,  "unused",                 0 }, \
   { BCM_KEY_ROW_4,    BCM_KEY_COL_6,  "unused",                 0 }, \
   { BCM_KEY_ROW_4,    BCM_KEY_COL_7,  "unused",                 0 }, \
   { BCM_KEY_ROW_5,    BCM_KEY_COL_0,  "unused",                 0 }, \
   { BCM_KEY_ROW_5,    BCM_KEY_COL_1,  "unused",                 0 }, \
   { BCM_KEY_ROW_5,    BCM_KEY_COL_2,  "unused",                 0 }, \
   { BCM_KEY_ROW_5,    BCM_KEY_COL_3,  "unused",                 0 }, \
   { BCM_KEY_ROW_5,    BCM_KEY_COL_4,  "unused",                 0 }, \
   { BCM_KEY_ROW_5,    BCM_KEY_COL_5,  "unused",                 0 }, \
   { BCM_KEY_ROW_5,    BCM_KEY_COL_6,  "unused",                 0 }, \
   { BCM_KEY_ROW_5,    BCM_KEY_COL_7,  "unused",                 0 }, \
   { BCM_KEY_ROW_6,    BCM_KEY_COL_0,  "unused",                 0 }, \
   { BCM_KEY_ROW_6,    BCM_KEY_COL_1,  "unused",                 0 }, \
   { BCM_KEY_ROW_6,    BCM_KEY_COL_2,  "unused",                 0 }, \
   { BCM_KEY_ROW_6,    BCM_KEY_COL_3,  "unused",                 0 }, \
   { BCM_KEY_ROW_6,    BCM_KEY_COL_4,  "unused",                 0 }, \
   { BCM_KEY_ROW_6,    BCM_KEY_COL_5,  "unused",                 0 }, \
   { BCM_KEY_ROW_6,    BCM_KEY_COL_6,  "unused",                 0 }, \
   { BCM_KEY_ROW_6,    BCM_KEY_COL_7,  "unused",                 0 }, \
   { BCM_KEY_ROW_7,    BCM_KEY_COL_0,  "unused",                 0 }, \
   { BCM_KEY_ROW_7,    BCM_KEY_COL_1,  "unused",                 0 }, \
   { BCM_KEY_ROW_7,    BCM_KEY_COL_2,  "unused",                 0 }, \
   { BCM_KEY_ROW_7,    BCM_KEY_COL_3,  "unused",                 0 }, \
   { BCM_KEY_ROW_7,    BCM_KEY_COL_4,  "unused",                 0 }, \
   { BCM_KEY_ROW_7,    BCM_KEY_COL_5,  "unused",                 0 }, \
   { BCM_KEY_ROW_7,    BCM_KEY_COL_6,  "unused",                 0 }, \
   { BCM_KEY_ROW_7,    BCM_KEY_COL_7,  "unused",                 0 }, \
}
