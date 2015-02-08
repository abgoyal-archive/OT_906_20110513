/*****************************************************************************
*
*    (c) 2001-2009 Broadcom Corporation
*
* This program is the proprietary software of Broadcom Corporation and/or
* its licensors, and may only be used, duplicated, modified or distributed
* pursuant to the terms and conditions of a separate, written license
* agreement executed between you and Broadcom (an "Authorized License").
* Except as set forth in an Authorized License, Broadcom grants no license
* (express or implied), right to use, or waiver of any kind with respect to
* the Software, and Broadcom expressly reserves all rights in and to the
* Software and all intellectual property rights therein.
* IF YOU HAVE NO AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO USE THIS
* SOFTWARE IN ANY WAY, AND SHOULD IMMEDIATELY NOTIFY BROADCOM AND DISCONTINUE
* ALL USE OF THE SOFTWARE.  
*
* Except as expressly set forth in the Authorized License,
*
* 1. This program, including its structure, sequence and organization,
*    constitutes the valuable trade secrets of Broadcom, and you shall use all
*    reasonable efforts to protect the confidentiality thereof, and to use
*    this information only in connection with your use of Broadcom integrated
*    circuit products.
*
* 2. TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED "AS IS"
*    AND WITH ALL FAULTS AND BROADCOM MAKES NO PROMISES, REPRESENTATIONS OR
*    WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR OTHERWISE, WITH
*    RESPECT TO THE SOFTWARE.  BROADCOM SPECIFICALLY DISCLAIMS ANY AND ALL
*    IMPLIED WARRANTIES OF TITLE, MERCHANTABILITY, NONINFRINGEMENT, FITNESS
*    FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR COMPLETENESS,
*    QUIET ENJOYMENT, QUIET POSSESSION OR CORRESPONDENCE TO DESCRIPTION. YOU
*    ASSUME THE ENTIRE RISK ARISING OUT OF USE OR PERFORMANCE OF THE SOFTWARE.
*
* 3. TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT SHALL BROADCOM OR ITS
*    LICENSORS BE LIABLE FOR (i) CONSEQUENTIAL, INCIDENTAL, SPECIAL, INDIRECT,
*    OR EXEMPLARY DAMAGES WHATSOEVER ARISING OUT OF OR IN ANY WAY RELATING TO
*    YOUR USE OF OR INABILITY TO USE THE SOFTWARE EVEN IF BROADCOM HAS BEEN
*    ADVISED OF THE POSSIBILITY OF SUCH DAMAGES; OR (ii) ANY AMOUNT IN EXCESS
*    OF THE AMOUNT ACTUALLY PAID FOR THE SOFTWARE ITSELF OR U.S. $1, WHICHEVER
*    IS GREATER. THESE LIMITATIONS SHALL APPLY NOTWITHSTANDING ANY FAILURE OF
*    ESSENTIAL PURPOSE OF ANY LIMITED REMEDY.
*
*****************************************************************************/
/**
*
*   @file   clk_drv.h
*   @brief  System clock configuration driver 
*
****************************************************************************/
/**
*
* @defgroup CLKGroup Broadcom Clock Controller
* @brief This group defines the APIs for Clock interfaces.
*
* @ingroup CSLGroup

Click here to navigate back to the Chip Support Library Overview page: \ref CSLOverview
*****************************************************************************/

#ifndef __CLOCK_DRV_H
#define __CLOCK_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
Global types
*****************************************************************************/
#include <plat/osdal_os_service.h>

#define CLK_AUDIO_RX_ADC_CLOCK  OSDAL_CLK_AUDIO_RX_ADC_CLOCK
#define CLK_AUDIO_TX_DAC_CLOCK  OSDAL_CLK_AUDIO_TX_DAC_CLOCK


#define CLKDRV_Open()  NULL


#if 1    
// Stop clock

#define CLKDRV_Stop_Clock(clk_handle,id)    { \
                                                OSDAL_CLK_HANDLE hClk; \
                                                hClk = OSDAL_CLK_Open(id); \
                                                OSDAL_CLK_Stop(hClk); \
                                                OSDAL_CLK_Close(hClk); \
                                            }


// Start Clock
#define CLKDRV_Start_Clock(clk_handle,id)    { \
                                                OSDAL_CLK_HANDLE hClk; \
                                                hClk = OSDAL_CLK_Open(id); \
                                                OSDAL_CLK_Start(hClk); \
                                                OSDAL_CLK_Close(hClk); \
                                            }

#else

#define CLKDRV_Stop_Clock(clk_handle,id)    


// Start Clock
#define CLKDRV_Start_Clock(clk_handle,id)    

#endif

#endif // __SYSCFG_H
