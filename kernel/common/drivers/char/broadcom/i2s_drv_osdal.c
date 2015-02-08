/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/char/broadcom/i2s_drv_osdal.c
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
*   @file   i2s_drv_osdal.c
*   @brief  This file defines the I2S Driver API using New DMA driver
*
*****************************************************************************/

/**
*   @defgroup   I2SDRVOSDALGroup   I2S Driver API using New DMA driver
*   @brief      This group defines the I2S Driver API
*
*****************************************************************************/

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/stddef.h>
#include <asm/sizes.h>
#include <mach/hardware.h>
#include <mach/memory.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <plat/bcm_i2sdai.h>
#include <plat/cpu.h>
#include "i2s_drv_new.h"
#include <plat/dma.h>
#include <linux/dma-mapping.h>

#include <plat/mobcom_types.h>
#include <plat/dma_drv.h>

/**
 * I2S Device structure
 */
struct i2s_dev {
	I2sEndCb endCallback;
	bool callback_status;
	I2S_STATE state;
	int sampleRate;
	bool stereo;
	I2S_RX_CHAN_t mono_from;	/* only needed for RX */
	UInt32 dmaChanNum;
	int bufCnt;
	unsigned int src_addr, len, dest_addr;
	int tx_buf_needs_unmap, rx_buf_needs_unmap;
};

/** 
 * I2S block is an I2S controller which has a tx device and a rx device 
 */
struct i2s_block {
	struct i2s_dev dev[2];
	bool initialized;
	I2S_BLK_ID_t id;
	bool master;

	bool loopback;
	struct clk *dam_clk;
	/* magic number is the last member of the object */
	unsigned int magic_number;
	struct i2s_device *i2s_handle;
};
static struct i2s_block i2s_blk[NUM_I2S_BLOCK];
static long unsigned int i2s_base_addr;

static Dma_Chan_Info dma_info_tx, dma_info_rx;
static Dma_Buffer_List dma_buf_list_tx, dma_buf_list_rx;
static Dma_Data dma_data_tx, dma_data_rx;
static struct device *pdevice;
struct i2s_block *tx_handle, *rx_handle;

#define TX_INDEX    0
#define RX_INDEX    1
#define I2S_BLOCK_MAGIC_NUMBER   'I2S'

/** @addtogroup I2SDRVOSDALGroup
    @{
*/


/**
* @brief	DMA Interrupt handler for I2S TX DMA channel 
*
* @param Err	DMA driver callback status
*/
static void i2s_dma_handler_tx(DMADRV_CALLBACK_STATUS_t Err)
{
	/* Unmap the DMA TX buffer */
	if (tx_handle->dev[TX_INDEX].tx_buf_needs_unmap)
		dma_unmap_single(pdevice, tx_handle->dev[TX_INDEX].src_addr,
				 tx_handle->dev[TX_INDEX].len, DMA_TO_DEVICE);

	tx_handle->dev[TX_INDEX].tx_buf_needs_unmap = 0;
	return;
}

/**
* @brief 	DMA Interrupt handler for I2S RX DMA channel
*
* @param Err	DMA driver callback status
*/
static void i2s_dma_handler_rx(DMADRV_CALLBACK_STATUS_t Err)
{
	if (rx_handle->dev[RX_INDEX].rx_buf_needs_unmap)
		dma_unmap_single(pdevice, rx_handle->dev[RX_INDEX].dest_addr,
				 rx_handle->dev[RX_INDEX].len, DMA_FROM_DEVICE);

	rx_handle->dev[RX_INDEX].rx_buf_needs_unmap = 0;
	return;
}


/**
* @brief    Initialize the I2S Driver, should be the first function to call
*
* @param id     I2S block enum that is to be initialized
*
* @return       I2S handle, NULL if error
*/
I2S_HANDLE I2SDRV_Init(I2S_BLK_ID_t id)
{
	struct i2s_block *block;
	I2S_HANDLE handle;

	if (id >= NUM_I2S_BLOCK) {
		pr_info("I2SDRV_Init: bad block id\n");
		goto err_return;
	}
	block = &i2s_blk[id];
	handle = (I2S_HANDLE) block;
	if (block->initialized) {

		/* force READY states */
		pr_debug
		    ("I2SDRV_Init: already init'ed (%lu for TX, %lu for RX)\r\n",
		     block->dev[TX_INDEX].dmaChanNum,
		     block->dev[RX_INDEX].dmaChanNum);
		block->dev[TX_INDEX].state = I2STST_READY;
		block->dev[RX_INDEX].state = I2STST_READY;
		goto err_return;
	}

	block->dam_clk = clk_get(NULL, "DAM");
	clk_enable(block->dam_clk);
	block->master = true;
	block->id = id;
	block->magic_number = (int)"I2S_BLOCK_MAGIC_NUMBER" + id;
	block->initialized = true;
	block->dev[RX_INDEX].bufCnt = 0;
	block->dev[TX_INDEX].bufCnt = 0;

	block->i2s_handle = use_i2s_device();

	pr_debug("I2SDRV_Init: dma channel %lu for TX, and %lu for RX\r\n",
		 block->dev[TX_INDEX].dmaChanNum,
		 block->dev[RX_INDEX].dmaChanNum);
	block->dev[TX_INDEX].state = I2STST_READY;
	block->dev[RX_INDEX].state = I2STST_READY;

	return handle;

err_return:
	return NULL;
}

/**
* @brief    De-initialze the I2S Driver, should be the last function to call
*
* @param handle     I2S handle
*
* @return           0 for success, i2s error status on error
*/
unsigned char I2SDRV_Cleanup(I2S_HANDLE handle)
{
	struct i2s_block *block = (struct i2s_block *)handle;
	if (!handle)
		return 1;
	if (block->dev[TX_INDEX].state != I2STST_READY ||
	    block->dev[RX_INDEX].state != I2STST_READY) {
		pr_debug("I2SDRV_cleanup: error! tx_state=%d, rx_state=%d\r\n",
			 block->dev[TX_INDEX].state,
			 block->dev[RX_INDEX].state);
		return I2S_WRONG_STATE;
	}

	DMADRV_Release_Channel(block->dev[RX_INDEX].dmaChanNum);
	DMADRV_Release_Channel(block->dev[TX_INDEX].dmaChanNum);

	release_i2s_device(handle);
	block->dev[TX_INDEX].state = I2STST_NOT_INIT;
	block->dev[RX_INDEX].state = I2STST_NOT_INIT;

	block->initialized = false;
	clk_disable(block->dam_clk);

	return 0;
}


/**
* @brief    Configure I2S Master/Slave Mode
*
* @param handle     I2S handle
* @param master     Master/Slave mode
*
* @return           0 for success, i2s error status on error
*/
unsigned char I2SDRV_Set_mode(I2S_HANDLE handle, bool master)
{
	struct i2s_block *block = (struct i2s_block *)handle;
	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_set_mode: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}
	if (bcm_i2s_interface_mode(block->i2s_handle, master, 0)) {
		pr_debug
		    ("I2SDRV_set_mode: bcm_i2s_interface_mode failed! \r\n");
	}
	if (bcm_i2s_config_dma(block->i2s_handle, PLAYBACK, TXRXDMASIZE_16))
		pr_debug("I2SDRV_set_mode:bcm_i2s_config_dma \
			for PLAYBACK failed! \r\n");
	if (bcm_i2s_config_dma(block->i2s_handle, CAPTURE, TXRXDMASIZE_16))
		pr_debug("I2SDRV_set_mode: bcm_i2s_config_dma \
			for CAPTURE failed! \r\n");
	bcm_i2s_txrx_fifo_enable(block->i2s_handle, 1);
	return 0;

}


/**
* @brief    Configure I2S transmit (TX) format, for Playback
*
* @param handle         I2S handle
* @param sampleRate     Sampling rate
* @param stereo         stereo/mono selection
*
* @return           0 for success, i2s error status on error
*/
unsigned char I2SDRV_Set_tx_format(I2S_HANDLE handle, int sampleRate,
				   bool stereo)
{
	struct i2s_block *block = (struct i2s_block *)handle;
	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_set_mode: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}
	if (block->dev[TX_INDEX].state != I2STST_READY) {
		pr_debug("I2SDRV_Set_tx_format: wrong state tx_state=%d\r\n",
			 block->dev[TX_INDEX].state);
		return I2S_WRONG_STATE;
	}
	if (bcm_i2s_sample_rate_div(block->i2s_handle, sampleRate, PLAYBACK)) {
		pr_debug
		    ("I2SDRV_set_mode:bcm_i2s_sample_rate_div failed! \r\n");
	}
	if (bcm_i2s_interface_mode(block->i2s_handle, 1, stereo)) {
		pr_debug("I2SDRV_set_mode:bcm_i2s_interface_mode failed! \r\n");
	}
	return 0;

}

/**
* @brief    Configure I2S Receive (RX) format, for Capture
*
* @param handle         I2S handle
* @param sampleRate     Sampling rate
* @param stereo         stereo/mono selection
* @param mono_from      where mono data is from. only used for mono mode
*
* @return               0 for success, i2s error status on error
*/
unsigned char I2SDRV_Set_rx_format(I2S_HANDLE handle, int sampleRate,
				   bool stereo, I2S_RX_CHAN_t mono_from)
{
	struct i2s_block *block = (struct i2s_block *)handle;
	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_set_mode: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}
	if (block->dev[RX_INDEX].state != I2STST_READY) {
		pr_debug("I2SDRV_Set_rx_format: wrong state! rx_state=%d\r\n",
			 block->dev[RX_INDEX].state);
		return I2S_WRONG_STATE;
	}
	if (bcm_i2s_sample_rate_div(block->i2s_handle, sampleRate, CAPTURE))
		pr_debug("I2SDRV_set_mode:bcm_i2s_sample_rate_div \
			failed!\r\n");
	if (bcm_i2s_interface_mode(block->i2s_handle, 0, stereo))
		pr_debug("I2SDRV_set_mode:bcm_i2s_interface_mode failed!\r\n");
	if (bcm_i2s_setup_mono_record(block->i2s_handle, mono_from, stereo, 0))
		pr_debug("I2SDRV_set_mode:bcm_i2s_setup_mono_record \
			failed! \r\n");
	return 0;

}

/**
* @brief     Queue a DMA buffer for Transmit (TX)
*
* @param handle     I2S handle
* @param buf        DMA buffer
* @param len        Length of buffer in 16-bit I2S words
*
* @return           0 for success, i2s error status on error
*/
unsigned char I2SDRV_Queue_tx_buf(I2S_HANDLE handle, unsigned short *buf,
				  unsigned int len)
{
	struct i2s_block *block = (struct i2s_block *)handle;
	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_set_mode: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}
	pr_debug("I2SDRV_queue_tx_buf, bufCnt=%d, state=%d\r\n",
		 block->dev[TX_INDEX].bufCnt, block->dev[TX_INDEX].state);
	if (block->dev[TX_INDEX].state == I2STST_NOT_INIT)
		return I2S_WRONG_STATE;

	block->dev[TX_INDEX].src_addr = dma_map_single(pdevice,
						       buf, len, DMA_TO_DEVICE);
	if (dma_mapping_error(pdevice, block->dev[TX_INDEX].src_addr)) {
		pr_info("%s(): Failed to map DMA buffer\n", __func__);
		return -1;
	}

	/* Make the memory consistent, invalidate/writeback the cache */
	dma_cache_maint(buf, len, DMA_TO_DEVICE);
	block->dev[TX_INDEX].tx_buf_needs_unmap = 1;
	tx_handle = block;

	block->dev[TX_INDEX].len = len;
	block->dev[TX_INDEX].bufCnt++;

	return 0;
}

/**
* @brief     Queue a DMA buffer for Receive (Rx)
*
* @param handle     I2S handle
* @param buf        DMA buffer
* @param len        Length of buffer in 16-bit I2S words
*
* @return           0 for success, i2s error status on error
*/
unsigned char I2SDRV_Queue_rx_buf(I2S_HANDLE handle, unsigned short *buf,
				  unsigned int len)
{
	struct i2s_block *block = (struct i2s_block *)handle;
	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_set_mode: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}

	pr_debug("I2SDRV_queue_rx_buf, bufCnt=%d, state=%d\r\n",
		 block->dev[RX_INDEX].bufCnt, block->dev[RX_INDEX].state);

	if (block->dev[RX_INDEX].state == I2STST_NOT_INIT)
		return I2S_WRONG_STATE;
	block->dev[RX_INDEX].dest_addr = dma_map_single(pdevice,
							buf, len,
							DMA_FROM_DEVICE);
	if (dma_mapping_error(pdevice, block->dev[RX_INDEX].dest_addr)) {
		pr_info("%s(): Failed to map DMA buffer\n", __func__);
		return -1;
	}

	/* Make the memory consistent, invalidate/writeback the cache */
	dma_cache_maint(buf, len, DMA_FROM_DEVICE);
	block->dev[RX_INDEX].rx_buf_needs_unmap = 1;
	rx_handle = block;

	block->dev[RX_INDEX].len = len;
	block->dev[RX_INDEX].bufCnt++;
	return 0;
}

/**
* @brief    Start I2S Transmit (TX) in DMA mode
*
* @param handle     I2S handle
*
* @return           0 for success, i2s error status on error
*/
unsigned char I2SDRV_Start_tx(I2S_HANDLE handle)
{
	struct i2s_block *block = (struct i2s_block *)handle;

	/*Request DMA channel for I2S_TX */
	if (DMADRV_Obtain_Channel(DMA_CLIENT_MEMORY, DMA_CLIENT_I2S_TX,
				  (DMA_CHANNEL *) & block->dev[TX_INDEX].
				  dmaChanNum) != DMADRV_STATUS_OK) {
		pr_info("DMADRV_Obtain_Channel failed for I2S TX channel\n");
		goto tx_error1;
	}

	dma_info_tx.srcID = DMA_CLIENT_MEMORY;
	dma_info_tx.dstID = DMA_CLIENT_I2S_TX;
	dma_info_tx.type = DMA_FCTRL_MEM_TO_PERI;
	dma_info_tx.alignment = DMA_ALIGNMENT_32;
	dma_info_tx.srcBstSize = DMA_BURST_SIZE_16;
	dma_info_tx.dstBstSize = DMA_BURST_SIZE_16;
	dma_info_tx.srcDataWidth = DMA_DATA_SIZE_16BIT;
	dma_info_tx.dstDataWidth = DMA_DATA_SIZE_16BIT;
	dma_info_tx.incMode = DMA_INC_MODE_SRC;
	dma_info_tx.xferCompleteCb = (DmaDrv_Callback) i2s_dma_handler_tx;
	dma_info_tx.freeChan = TRUE;
	dma_info_tx.priority = 0;
	dma_info_tx.bCircular = FALSE;

	if (DMADRV_Config_Channel(block->dev[TX_INDEX].dmaChanNum, &dma_info_tx)
	    != DMADRV_STATUS_OK) {
		pr_info("DMADRV_Config_Channel Failed for I2S TX channel\n");
		goto tx_error2;
	}

	bcm_i2s_enable_clk(block->i2s_handle, I2S_EXT_CLK, 0);

	bcm_i2s_txrx_fifo_enable(block->i2s_handle, 0);

	bcm_i2s_interface_enable(block->i2s_handle, true, false, 0);

	pr_debug("I2SDRV_Start_tx, bufCnt=%d, state=%d\r\n",
		 block->dev[RX_INDEX].bufCnt, block->dev[RX_INDEX].state);

	dma_buf_list_tx.buffers[0].srcAddr = block->dev[TX_INDEX].src_addr;
	dma_buf_list_tx.buffers[0].length = block->dev[TX_INDEX].len;
	dma_buf_list_tx.buffers[0].destAddr =
	    (int)(io_v2p(i2s_base_addr + 0x0008));
	dma_buf_list_tx.buffers[0].bRepeat = 0;
	dma_buf_list_tx.buffers[0].interrupt = 1;
	dma_data_tx.numBuffer = 1;
	dma_data_tx.pBufList = (Dma_Buffer_List *) & dma_buf_list_tx;

	if (DMADRV_Bind_Data(block->dev[TX_INDEX].dmaChanNum,
			     &dma_data_tx) != DMADRV_STATUS_OK) {
		pr_info("DMADRV_Bind_Data Failed for I2S TX channel\n");
		goto tx_error2;
	}

	if (DMADRV_Start_Transfer(block->dev[TX_INDEX].dmaChanNum) !=
	    DMADRV_STATUS_OK) {
		pr_info("DMADRV_Start_Transfer Failed for I2S TX channel\n");
		goto tx_error2;
	}
	return 0;

tx_error2:
	DMADRV_Release_Channel(block->dev[TX_INDEX].dmaChanNum);
tx_error1:
	return I2S_DMA_SETUP_ERROR;
}

/**
* @brief    Start I2S Receive (RX) in DMA mode
*
* @param handle     I2S handle
*
* @return           0 for success, i2s error status on error
*/
unsigned char I2SDRV_Start_rx(I2S_HANDLE handle)
{
	struct i2s_block *block = (struct i2s_block *)handle;

	/*Request the  DMA channel for I2S_RX */
	if (DMADRV_Obtain_Channel(DMA_CLIENT_I2S_RX, DMA_CLIENT_MEMORY,
				  (DMA_CHANNEL *) & block->dev[RX_INDEX].
				  dmaChanNum) != DMADRV_STATUS_OK) {
		pr_info("DMADRV_Obtain_Channel failed for I2S RX channel\n");
		goto rx_error1;
	}

	dma_info_rx.srcID = DMA_CLIENT_I2S_RX;
	dma_info_rx.dstID = DMA_CLIENT_MEMORY;
	dma_info_rx.type = DMA_FCTRL_PERI_TO_MEM;
	dma_info_rx.alignment = DMA_ALIGNMENT_32;
	dma_info_rx.srcBstSize = DMA_BURST_SIZE_16;
	dma_info_rx.dstBstSize = DMA_BURST_SIZE_16;
	dma_info_rx.srcDataWidth = DMA_DATA_SIZE_16BIT;
	dma_info_rx.dstDataWidth = DMA_DATA_SIZE_16BIT;
	dma_info_rx.incMode = DMA_INC_MODE_DST;
	dma_info_rx.xferCompleteCb = (DmaDrv_Callback) i2s_dma_handler_rx;
	dma_info_rx.freeChan = TRUE;
	dma_info_rx.priority = 0;
	dma_info_rx.bCircular = FALSE;

	if (DMADRV_Config_Channel(block->dev[RX_INDEX].dmaChanNum, &dma_info_rx)
	    != DMADRV_STATUS_OK) {
		pr_info("DMADRV_Config_Channel Failed for I2S RX channel\n");
		goto rx_error2;
	}

	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_set_mode: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}

	bcm_i2s_txrx_fifo_enable(block->i2s_handle, 1);

	bcm_i2s_enable_clk(block->i2s_handle, I2S_EXT_CLK, 0);

	bcm_i2s_interface_enable(block->i2s_handle, false, true, 0);

	pr_debug("I2SDRV_Start_rx, bufCnt=%d, state=%d\r\n",
		 block->dev[RX_INDEX].bufCnt, block->dev[RX_INDEX].state);

	dma_buf_list_rx.buffers[0].srcAddr =
	    (int)(io_v2p(i2s_base_addr + 0x0008));
	dma_buf_list_rx.buffers[0].length = block->dev[RX_INDEX].len;
	dma_buf_list_rx.buffers[0].destAddr = block->dev[RX_INDEX].dest_addr;
	dma_buf_list_rx.buffers[0].bRepeat = 0;
	dma_buf_list_rx.buffers[0].interrupt = 1;
	dma_data_rx.numBuffer = 1;
	dma_data_rx.pBufList = (Dma_Buffer_List *) & dma_buf_list_rx;

	if (DMADRV_Bind_Data(block->dev[RX_INDEX].dmaChanNum, &dma_data_rx) !=
	    DMADRV_STATUS_OK) {
		pr_info("DMADRV_Bind_Data Failed for I2S RX channel\n");
		goto rx_error2;
	}

	if (DMADRV_Start_Transfer(block->dev[RX_INDEX].dmaChanNum) !=
	    DMADRV_STATUS_OK) {
		pr_info("DMADRV_Start_Transfer Failed for I2S RX channel\n");
		goto rx_error2;
	}
	return 0;

      rx_error2:
	DMADRV_Release_Channel(block->dev[RX_INDEX].dmaChanNum);
      rx_error1:
	return I2S_DMA_SETUP_ERROR;
}

/**
* @brief    Start I2S Receive (RX) in non DMA mode
*
* @param handle     I2S handle
*
* @return           0 for success, i2s error status on error
*/
unsigned char I2SDRV_Start_rx_noDMA(I2S_HANDLE handle)
{
	struct i2s_block *block = (struct i2s_block *)handle;
	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_Start_rx_noDMA: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}
	bcm_i2s_enable_clk(block->i2s_handle, I2S_EXT_CLK, 0);

	pr_debug("I2SDRV_Start_rx_noDMA, bufCnt=%d, state=%d\r\n",
		 block->dev[RX_INDEX].bufCnt, block->dev[RX_INDEX].state);
	if (bcm_i2s_txrx_fifo_enable(block->i2s_handle, CAPTURE))
		pr_debug("I2SDRV_Start_rx_noDMA:bcm_i2s_txrx_fifo_enable \
			failed! \r\n");
	if (bcm_i2s_interface_enable(block->i2s_handle, false, true, 0))
		pr_debug("I2SDRV_Start_rx_noDMA:bcm_i2s_interface_enable \
			failed! \r\n");
	return 0;

}

/**
* @brief Stop I2S Receive (TX) in non DMA mode
*
* @param handle     I2S handle
*
* @return           0 for success, i2s error status on error
*/
unsigned char I2SDRV_Stop_rx_noDMA(I2S_HANDLE handle)
{
	struct i2s_block *block = (struct i2s_block *)handle;
	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_set_mode: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}
	pr_debug("I2SDRV_Start_tx_noDMA, bufCnt=%d, state=%d\r\n",
		 block->dev[RX_INDEX].bufCnt, block->dev[RX_INDEX].state);
	if (bcm_i2s_interface_disable(block->i2s_handle, false, true))
		pr_debug("I2SDRV_Stop_rx_noDMA:bcm_i2s_interface_disable \
			failed! \r\n");
	if (bcm_i2s_disable_clk(block->i2s_handle, I2S_INT_CLK))
		pr_debug("I2SDRV_Stop_rx_noDMA :bcm_i2s_disable_clk \
			failed! \r\n");
	block->dev[RX_INDEX].state = I2STST_READY;
	return 0;
}

/**
* @brief    Stop I2S Transmit (TX)
*
* @param handle     I2S handle
*
* @return           0 for success, i2s error status on error
*/
unsigned char I2SDRV_Stop_tx(I2S_HANDLE handle)
{
	struct i2s_block *block = (struct i2s_block *)handle;

	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_set_mode: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}
	pr_debug("I2SDRV_Stop_tx, bufCnt=%d, state=%d\r\n",
		 block->dev[RX_INDEX].bufCnt, block->dev[RX_INDEX].state);
	if (block->dev[TX_INDEX].bufCnt != 0) {
		/* stopping */
		if (bcm_i2s_interface_disable(block->i2s_handle, true, false)) {
			pr_debug
			    ("I2SDRV_Stop_tx:bcm_i2s_interface_disable  failed! \r\n");
		}
		if (bcm_i2s_disable_clk(block->i2s_handle, I2S_INT_CLK)) {
			pr_debug
			    ("I2SDRV_Stop_tx:bcm_i2s_disable_clk  failed! \r\n");
		}
		block->dev[TX_INDEX].bufCnt = 0;
	}
	block->dev[TX_INDEX].state = I2STST_READY;
	return 0;

}

/**
* @brief    Stop I2S Receive (RX)
*
* @param handle     I2S handle
*
* @return           0 for success, i2s error status on error
*/
unsigned char I2SDRV_Stop_rx(I2S_HANDLE handle)
{
	struct i2s_block *block = (struct i2s_block *)handle;

	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_set_mode: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}
	pr_debug("I2SDRV_Stop_tx, bufCnt=%d, state=%d\r\n",
		 block->dev[RX_INDEX].bufCnt, block->dev[RX_INDEX].state);
	/* abort any any on-going DMA, make sure bufCnt is 0 */
	if (block->dev[RX_INDEX].bufCnt != 0) {
		/* stopping */

		block->dev[RX_INDEX].bufCnt = 0;
		bcm_i2s_interface_disable(block->i2s_handle, false, true);
		bcm_i2s_disable_clk(block->i2s_handle, I2S_INT_CLK);
		block->dev[RX_INDEX].state = I2STST_READY;
	}
	return 0;

}


/**
* @brief    Register an end-of-transfer callback function for Transmit (TX)
*
* @param handle     I2S handle
* @param endCb      Pointer to callback function
*
* @return           0 for success, i2s error status on error
*/
unsigned char I2SDRV_Register_tx_Cb(I2S_HANDLE handle, I2sEndCb endCb)
{
	struct i2s_block *block = (struct i2s_block *)handle;

	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_set_mode: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}
	pr_debug("I2SDRV_register_tx_Cb, old=%x, new=%x\r\n",
		 (unsigned int)block->dev[TX_INDEX].endCallback,
		 (unsigned int)endCb);

	/* Register API's End-Call-back function */
	if (block->dev[TX_INDEX].endCallback != endCb) {
		block->dev[TX_INDEX].endCallback = endCb;
	}
	return 0;
}


/**
* @brief    Register an end-of-transfer callback function for Receive (RX)
*
* @param handle     I2S handle
* @param endCb      Pointer to callback function
*
* @return           0 for success, i2s error status on error
*/
unsigned char I2SDRV_Register_rx_Cb(I2S_HANDLE handle, I2sEndCb endCb)
{
	struct i2s_block *block = (struct i2s_block *)handle;

	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_set_mode: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}
	pr_debug("I2SDRV_register_rx_Cb, old=%x, new=%x\r\n",
		 (unsigned int)block->dev[RX_INDEX].endCallback,
		 (unsigned int)endCb);
	/* Register API's End-Call-back function */
	if (block->dev[RX_INDEX].endCallback != endCb) {
		block->dev[RX_INDEX].endCallback = endCb;
	}
	return 0;
}

/**
* @brief    Get the I2S device Transmit (TX) state
*
* @param handle     I2S handle
*
* @return           0 for success, i2s error status on error
*/
I2S_STATE I2SDRV_Get_tx_state(I2S_HANDLE handle)
{
	struct i2s_block *block = (struct i2s_block *)handle;
	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_set_mode: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}
	return block->dev[TX_INDEX].state;

}

/**
* @brief    Get the I2S device Receive (RX) state
*
* @param handle     I2S handle
*
* @return           0 for success, i2s error status on error
*/
I2S_STATE I2SDRV_Get_rx_state(I2S_HANDLE handle)
{
	struct i2s_block *block = (struct i2s_block *)handle;
	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_set_mode: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}
	return block->dev[RX_INDEX].state;
}

/**
* @brief    Pause I2S Transmission (TX)
*
* @param handle     I2S handle
*
* @return           0 for success, i2s error status on error
*/
unsigned char I2SDRV_Pause_tx(I2S_HANDLE handle)
{
	struct i2s_block *block = (struct i2s_block *)handle;

	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_set_mode: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}

	if (block->dev[TX_INDEX].state != I2STST_STREAMING) {
		pr_debug("I2SDRV:i2s_pause_tx: error! can't pause at %d\r\n",
			 block->dev[TX_INDEX].state);
		return (unsigned char)I2S_WRONG_STATE;
	}
	bcm_i2s_interface_disable(block->i2s_handle, true, false);
	bcm_i2s_txrx_fifo_disable(block->i2s_handle, PLAYBACK);
	block->dev[TX_INDEX].state = I2STST_PAUSING;
	pr_debug("I2SDRV_Pause_tx : Pausing TX.. \r\n");
	return 0;

}

/**
* @brief    Pause I2S Reception (RX)
*
* @param handle     I2S handle
*
* @return           0 for success, i2s error status on error
*/
unsigned char I2SDRV_Pause_rx(I2S_HANDLE handle)
{
	struct i2s_block *block = (struct i2s_block *)handle;

	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_set_mode: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}

	if (block->dev[RX_INDEX].state != I2STST_STREAMING) {
		pr_debug("I2SDRV:i2s_pause_rx: error! can't pause at %d\r\n",
			 block->dev[RX_INDEX].state);
		return (unsigned char)I2S_WRONG_STATE;
	}
	bcm_i2s_interface_disable(block->i2s_handle, false, true);
	bcm_i2s_txrx_fifo_disable(block->i2s_handle, CAPTURE);
	block->dev[RX_INDEX].state = I2STST_PAUSING;
	pr_debug("I2SDRV_Pause_tx : Pausing RX.. \r\n");
	return 0;
}

/**
* @brief    Resume previously 'paused' I2S Transmission (TX)
*
* @param handle     I2S handle
*
* @return           0 for success, i2s error status on error
*/
unsigned char I2SDRV_Resume_tx(I2S_HANDLE handle)
{
	struct i2s_block *block = (struct i2s_block *)handle;
	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_set_mode: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}

	if (block->dev[TX_INDEX].state != I2STST_PAUSING) {
		pr_debug("I2SDRV_resume_tx: error! can't resume %d\r\n",
			 block->dev[TX_INDEX].state);
		return (unsigned char)I2S_WRONG_STATE;
	}

	bcm_i2s_interface_enable(block->i2s_handle, true, false, 0);
	bcm_i2s_txrx_fifo_enable(block->i2s_handle, PLAYBACK);
	block->dev[TX_INDEX].state = I2STST_STREAMING;
	pr_debug("I2SDRV_Resume_tx : Resuming TX.. \r\n");
	return 0;
}

/**
* @brief    Resume previously 'paused' I2S Reception (RX)
*
* @param handle     I2S handle
*
* @return           0 for success, i2s error status on error
*/
unsigned char I2SDRV_Resume_rx(I2S_HANDLE handle)
{
	struct i2s_block *block = (struct i2s_block *)handle;

	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_set_mode: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}

	if (block->dev[RX_INDEX].state != I2STST_PAUSING) {
		pr_debug("I2SDRV_resume_rx: error! can't resume %d\r\n",
			 block->dev[RX_INDEX].state);
		return (unsigned char)I2S_WRONG_STATE;
	}

	bcm_i2s_interface_enable(block->i2s_handle, false, true, 0);
	bcm_i2s_txrx_fifo_enable(block->i2s_handle, CAPTURE);
	block->dev[RX_INDEX].state = I2STST_STREAMING;
	pr_debug("I2SDRV_Resume_rx : Resuming RX.. \r\n");
	return 0;
}

/**
* @brief    Enable/Disable I2S internal loopback for testing
*
* @param handle     I2S handle
* @param enable     enable (true/false)
*
* @return           0 for success, i2s error status on error
*/
unsigned char I2SDRV_Enable_Loopback(I2S_HANDLE handle, bool enable)
{
	struct i2s_block *block = (struct i2s_block *)handle;
	if (block->magic_number != (int)"I2S_BLOCK_MAGIC_NUMBER" + block->id) {
		pr_debug("I2SDRV_set_mode: bad handle\r\n");
		return I2S_BAD_HANDLE;
	}
	bcm_i2s_loopback_cfg(block->i2s_handle, enable);
	return 0;

}

/*
 * Probe function for I2S Driver
 */
static int __devinit i2s_probe(struct platform_device *pdev)
{
	int ret = -1;
	struct resource *res;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"can't get platform resource -  bcm_i2s_\n");
		ret = -ENOMEM;
		goto err;
	}
	i2s_base_addr = (long unsigned int)res->start;
	pdevice = &pdev->dev;
	return 0;
err:
	return -1;
}

/**
 * Remove function for I2S Driver
 */
static int __devexit i2s_remove(struct platform_device *dev)
{
	return 0;
}

/**
 * I2S Platform Driver struct
 */
static struct platform_driver i2s_driver = {
	.probe = i2s_probe,
	.remove = i2s_remove,
	.driver = {
		   .name = "i2sdai",
		   },
};


/**
 * I2S Driver module Init function
 */
static int __init i2s_init(void)
{
	pr_info("i2s module loaded successfully\n");
	return platform_driver_register(&i2s_driver);
}

/**
 * I2S Driver module exit function
 */
static void __exit i2s_exit(void)
{
	platform_driver_unregister(&i2s_driver);
	pr_info("dma_i2s module unloaded successfully\n");
}
/** @} */

EXPORT_SYMBOL(I2SDRV_Init);
EXPORT_SYMBOL(I2SDRV_Set_tx_format);
EXPORT_SYMBOL(I2SDRV_Set_mode);
EXPORT_SYMBOL(I2SDRV_Cleanup);
EXPORT_SYMBOL(I2SDRV_Set_rx_format);
EXPORT_SYMBOL(I2SDRV_Queue_tx_buf);
EXPORT_SYMBOL(I2SDRV_Queue_rx_buf);
EXPORT_SYMBOL(I2SDRV_Start_tx);
EXPORT_SYMBOL(I2SDRV_Start_rx);
EXPORT_SYMBOL(I2SDRV_Start_rx_noDMA);
EXPORT_SYMBOL(I2SDRV_Stop_rx_noDMA);
EXPORT_SYMBOL(I2SDRV_Stop_tx);
EXPORT_SYMBOL(I2SDRV_Stop_rx);
EXPORT_SYMBOL(I2SDRV_Register_tx_Cb);
EXPORT_SYMBOL(I2SDRV_Register_rx_Cb);
EXPORT_SYMBOL(I2SDRV_Get_tx_state);
EXPORT_SYMBOL(I2SDRV_Get_rx_state);
EXPORT_SYMBOL(I2SDRV_Pause_tx);
EXPORT_SYMBOL(I2SDRV_Pause_rx);
EXPORT_SYMBOL(I2SDRV_Resume_tx);
EXPORT_SYMBOL(I2SDRV_Resume_rx);
EXPORT_SYMBOL(I2SDRV_Enable_Loopback);

module_init(i2s_init);
module_exit(i2s_exit);
