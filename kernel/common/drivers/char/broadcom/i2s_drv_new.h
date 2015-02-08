/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/char/broadcom/i2s_drv_new.h
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
* @file     i2s.h

* @brief    Header file for I2S driver for BCM21xx.

****************************************************************************/

/* Requires the following header files before its inclusion in a c file
#include "mobcom_types.h"
*/

#ifndef _I2S_H_
#define _I2S_H_

#ifdef __cplusplus
extern "C" {
#endif
#if 0
	typedef enum {
		I2S_SAMPLERATE_8000HZ,	/* /< 8000HZ */
		I2S_SAMPLERATE_11030HZ,	/* /< 11030HZ */
		I2S_SAMPLERATE_12000HZ,	/* /< 12000HZ */
		I2S_SAMPLERATE_16000HZ,	/* /< 16000HZ */
		I2S_SAMPLERATE_22060HZ,	/* /< 22060HZ */
		I2S_SAMPLERATE_24000HZ,	/* /< 24000HZ */
		I2S_SAMPLERATE_32000HZ,	/* /< 32000HZ */
		I2S_SAMPLERATE_48000HZ,	/* /< 48000HZ */
		I2S_SAMPLERATE_44100HZ,	/* /< 44100HZ */
		I2S_SAMPLERATE_96000HZ	/* /< 96000HZ */
	} I2S_SAMPLE_RATE_t;
#endif
	typedef enum {
		I2S_RX_LEFT_CHAN,	/* !<  recording from left channel in mono mode */
		I2S_RX_RIGHT_CHAN,	/* !<  recording from right channel in mono mode */
		I2S_RX_AEVERAGE	/* !<  recording with (L + R)/2 in mono mode */
	} I2S_RX_CHAN_t;

/* ! return status */
#define I2S_WRONG_STATE                (-1)
#define I2S_SAMPLE_RATE_NOT_SUPPORT    (-2)
#define I2S_BAD_HANDLE                 (-3)
#define I2S_NO_CB_DEFINED              (-4)
#define I2S_DMA_SETUP_ERROR            (-5)

	typedef enum {
		I2STST_NOT_INIT,	/* !< Default state */
		I2STST_READY,	/* !< Driver running */
		I2STST_STREAMING,	/* !< Playing audio */
		I2STST_RUNNING_NO_DMA,	/* !< I2S direct pass */
		I2STST_PAUSING,	/* !< Attempting to pause audio stream */
		I2STST_ERROR	/* !< error */
	} I2S_STATE;

	typedef enum {
		I2S_BLK1_ID = 0,
#if defined (_BCM21551_) || defined (_ATHENA_)
		I2S_BLK2_ID = 1,
#endif
		NUM_I2S_BLOCK
	} I2S_BLK_ID_t;

	typedef struct {
		unsigned int startAddr;
		unsigned int bufferSize;
		unsigned int bufferNum;
	} I2S_Cir_t;

	typedef bool(*I2sEndCb) (bool dma_ok);
	typedef void *I2S_HANDLE;

	I2S_HANDLE I2SDRV_Init(I2S_BLK_ID_t id);
	unsigned char I2SDRV_Cleanup(I2S_HANDLE handle);
	unsigned char I2SDRV_Set_mode(I2S_HANDLE handle, bool master);

	I2S_STATE I2SDRV_Get_tx_state(I2S_HANDLE handle);
	I2S_STATE I2SDRV_Get_rx_state(I2S_HANDLE handle);

	unsigned char I2SDRV_Register_tx_Cb(I2S_HANDLE handle, I2sEndCb endCb);
	unsigned char I2SDRV_Register_rx_Cb(I2S_HANDLE handle, I2sEndCb endCb);

	unsigned char I2SDRV_Set_tx_format(I2S_HANDLE handle, int sampleRate,
					   bool stereo);
	unsigned char I2SDRV_Set_rx_format(I2S_HANDLE handle, int sampleRate,
					   bool stereo,
					   I2S_RX_CHAN_t mono_from);

	unsigned char I2SDRV_Queue_tx_buf(I2S_HANDLE handle,
					  unsigned short *buf,
					  unsigned int len);
	unsigned char I2SDRV_Queue_rx_buf(I2S_HANDLE handle,
					  unsigned short *buf,
					  unsigned int len);

	unsigned char I2SDRV_Start_tx(I2S_HANDLE handle);
	unsigned char I2SDRV_Start_rx(I2S_HANDLE handle);

	unsigned char I2SDRV_Pause_tx(I2S_HANDLE handle);
	unsigned char I2SDRV_Pause_rx(I2S_HANDLE handle);

	unsigned char I2SDRV_Resume_tx(I2S_HANDLE handle);
	unsigned char I2SDRV_Resume_rx(I2S_HANDLE handle);

	unsigned char I2SDRV_Stop_tx(I2S_HANDLE handle);
	unsigned char I2SDRV_Stop_rx(I2S_HANDLE handle);

	unsigned char I2SDRV_Enable_Loopback(I2S_HANDLE handle, bool enable);
	unsigned short I2SDRV_Get_Data_Count(I2S_HANDLE handle,
					     unsigned char which);
	unsigned int I2SDRV_Get_Data_Port(I2S_HANDLE handle);

	unsigned char I2SDRV_Start_rx_noDMA(I2S_HANDLE handle);
	unsigned char I2SDRV_Stop_rx_noDMA(I2S_HANDLE handle);
#ifdef __cplusplus
}
#endif
#endif				/* _I2S_H_ */
