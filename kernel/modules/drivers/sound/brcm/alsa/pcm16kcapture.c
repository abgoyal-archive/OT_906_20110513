/*
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
/*******************************************************************************************
Copyright 2010 Broadcom Corporation.  All rights reserved.

Unless you and Broadcom execute a separate written software license agreement 
governing use of this software, this software is licensed to you under the 
terms of the GNU General Public License version 2, available at 
http://www.gnu.org/copyleft/gpl.html (the "GPL"). 

Notwithstanding the above, under no circumstances may you combine this software 
in any way with any other Broadcom software provided under a license other than 
the GPL, without Broadcom's express prior written consent.
*******************************************************************************************/

#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>

#include <linux/broadcom/hw_cfg.h>

#include "brcm_alsa.h"
#include "audvoc_drv.h"
#include "audvoc_consts.h"

#include "audio_ipc_consts.h"
#include "hal_audio.h"
#include "hal_audio_access.h"
#include <mach/reg_sys.h>

#include "vpu.h"

long GetCtrlValue(int ctrlIndex);


#define	FLAG_PCM16_OPENED			0x00000001
#define	FLAG_PCM16_DSPENABLED		0x00000002
#define	FLAG_PCM16_DSPLOADED		0x00000004


typedef	struct	_TAMRWBCapture
{
	brcm_alsa_chip_t 	*mpChip;
	spinlock_t 			mspinlock_pcm16kcap;
	UInt32		   		mu32Flags;
	SharedMem_t 		*pg0_sm;
	PAGE5_SharedMem_t 	*mpPg5_sm;
	Unpaged_SharedMem_t *mpUnpage_sm;

	
	UInt32				u32TotalFrames;
	VPUDumpFramesCB_t 	mpfDumpFrames;

}TAMRWBCapture, *PTAMRWBCapture;

static	TAMRWBCapture	gPcm16kCapture;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//  Function Name: PCM16kCaptureStart
//
//  Description: Start PCM 16k capture
//
//-----------------------------------------------------
int 	PCM16kCaptureStart(struct snd_pcm_substream * substream, VPUDumpFramesCB_t pfDumpFrames)
{
	brcm_alsa_chip_t *pChip = snd_pcm_substream_chip(substream);
	PTAMRWBCapture pPcmCap;
	Shared_poly_audio_t *pg4_sm;

	pPcmCap = &gPcm16kCapture;
	pPcmCap->mpfDumpFrames = pfDumpFrames;
	pPcmCap->mspinlock_pcm16kcap = SPIN_LOCK_UNLOCKED;
	pPcmCap->mpChip = pChip;
	pPcmCap->mpPg5_sm = SHAREDMEM_GetPage5SharedMemPtr();
	pPcmCap->mpUnpage_sm = SHAREDMEM_GetUnpagedSharedMemPtr();

	
	pPcmCap->mu32Flags = 0; //clear
	pPcmCap->u32TotalFrames = 0;

	//1. Open
	pg4_sm = (Shared_poly_audio_t *) SHAREDMEM_GetPage4SharedMemPtr();
	
	pg4_sm->shared_Inbuf_LOW_Sts_TH 	= (0xc00)>> 1;			// means half of the total input buffer
	pg4_sm->shared_Outbuf_LOW_Sts_TH	= 0x280;				// means 2 times of the Speech frame for the 16K ISR
	pg4_sm->shared_Outbuf_Freq_Sts_TH	= 1;					// means frame delivery for 20ms from the encoder
	
	pPcmCap->pg0_sm = SHAREDMEM_GetSharedMemPtr();

	//WB AMR has a variable for SETUP State that it reflects static or global mode of operation. This is shared_WB_AMR_Ctrl_state. The bit#7 is considered for CAPTURE/ENCODE mode for the record. If is set, DSP performs CAPTURE/ENCODE mode.
	pPcmCap->pg0_sm->shared_WB_AMR_Ctrl_state = 0x80;		//MIME, 6.60 kbit/s		
	pPcmCap->pg0_sm->shared_16ISR_state = 1;				
	
	//2. Load encoder image to DSP if not loaded
//always do	if( !(pPcmCap->mu32Flags & FLAG_PCM16_DSPLOADED) )
	{
		CmdQ_t	msg;
		pPcmCap->mpUnpage_sm->shared_MM_wb_amr_mode[3] = 0x0;	  // capture only
		pPcmCap->mpUnpage_sm->shared_MM_wb_amr_mode[4] = 0x1;
		pPcmCap->mpUnpage_sm->shared_MM_wb_amr_mode[2]= WB_AMR_RECORD;//WB_AMR_RECORD;
		audio_control_dsp(AP_MM_WB_ENCODE, 0, 0, 0);


		msg.cmd = COMMAND_NOISE_SUPPRESSION;
		msg.arg0 = 1; 	
		msg.arg1 = 0;	
		msg.arg2 = 0;
		
		post_msg(msg.cmd, msg.arg0, msg.arg1, msg.arg2);
		pPcmCap->mpUnpage_sm->shared_PRAM_code_loaded.pram_mm_wb_amr = 0;					// To make sure the pram is loaded everytime
	
		msg.cmd = COMMAND_MM_ENABLE;
		msg.arg0 = pPcmCap->mpUnpage_sm->shared_MM_wb_amr_mode[2]; 	// Type of MM function: Playback or Record
		msg.arg1 = 0;					// Initial value for Status message of 0xaaaa, 0xbbbb
		msg.arg2 = 0;

		pPcmCap->mu32Flags |= FLAG_PCM16_DSPENABLED;
	
		post_msg(msg.cmd, msg.arg0, msg.arg1, msg.arg2);

	}
		
	//3. Config
	

	//4. Enable path
	{
		HAL_AUDIO_Control_Params_un_t audioControlParam;
		HAL_AUDIO_Get_Param_t getParam;
						
		//get present Audio channel and volume levels from HAL
		getParam.paramType = GET_AUDIO_CHNL;
		HAL_AUDIO_Ctrl( ACTION_AUD_GetParam, &getParam, NULL);
		audioControlParam.param_pathCfg.outputChnl= (AUDIO_CHANNEL_t) *(getParam.paramPtr);
		
//		getParam.paramType = GET_AUDIO_VOLUMELEVEL;
//		HAL_AUDIO_Ctrl( ACTION_AUD_GetParam, &getParam, NULL);
		audioControlParam.param_pathCfg.volumeLevel = 40; //(UInt16)*(getParam.paramPtr); 
		
		if(substream->number == 2) //AUX, wired headset
			audioControlParam.param_pathCfg.outputChnl = AUDIO_CHNL_HEADPHONE;
		audioControlParam.param_pathCfg.audioID = AUDIO_ID_AMRWB_RECORD;
		HAL_AUDIO_Ctrl( ACTION_AUD_EnablePath, &audioControlParam, NULL);
		DEBUG("\n %lx:Enable path outputChnl=%d vol=%d \n",jiffies, audioControlParam.param_pathCfg.outputChnl, audioControlParam.param_pathCfg.volumeLevel );
		
		HAL_AUDIO_Ctrl( ACTION_AUD_SetVolumeWithPath, &audioControlParam, NULL);
	}

	//5. Start
	{
		post_msg(COMMAND_MM_ENABLE, WB_AMR_RECORD, WB_AMR_RECORD, 0);
		pPcmCap->mpUnpage_sm->shared_MM_wb_amr_mode[2]= WB_AMR_CAPTURE_RECORD; 

		if(0==GetCtrlValue(BRCM_CTL_Mic_Capture_Mute))
			HAL_AUDIO_Ctrl(ACTION_AUD_UnmuteMic, NULL, NULL);
		else
			HAL_AUDIO_Ctrl(ACTION_AUD_MuteMic, NULL, NULL);
		mdelay(10); //to eleminate pop noise

		post_msg(COMMAND_AUDIO_ENABLE, 1, 1, 0);
		post_msg(COMMAND_AUDIO_TASK_START_REQUEST, 7, 0, 0);
	}
	
	return 0;
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//  Function Name: PCM16kCaptureStop
//
//  Description: Start PCM 16k capture
//
//------------------------------------------------------------
int 	PCM16kCaptureStop(struct snd_pcm_substream * substream)
{
//	brcm_alsa_chip_t *pChip = snd_pcm_substream_chip(substream);
	PTAMRWBCapture pPcmCap;

	pPcmCap = &gPcm16kCapture;

	//1. Stop
	{
		post_msg(COMMAND_AUDIO_ENABLE, 0, 0, 0);
		post_msg(COMMAND_MM_ENABLE, 0, WB_AMR_RECORD, 0);
		post_msg(COMMAND_CLEAR_VOIPMODE, 0, 0, 0);
		pPcmCap->pg0_sm->shared_16ISR_state = 0; //set to 8k
		
		pPcmCap->pg0_sm->shared_WB_AMR_Ctrl_state &= 0xff7f;		// Clear the flag for each session
		
	}


	//2.  Disable path
	{
		HAL_AUDIO_Control_Params_un_t audioControlParam;
		HAL_AUDIO_Get_Param_t getParam;
						
		//get present Audio channel and volume levels from HAL
		getParam.paramType = GET_AUDIO_CHNL;
		HAL_AUDIO_Ctrl( ACTION_AUD_GetParam, &getParam, NULL);
		audioControlParam.param_pathCfg.outputChnl= (AUDIO_CHANNEL_t) *(getParam.paramPtr);
		
		getParam.paramType = GET_AUDIO_VOLUMELEVEL;
		HAL_AUDIO_Ctrl( ACTION_AUD_GetParam, &getParam, NULL);
		audioControlParam.param_pathCfg.volumeLevel = (UInt16)*(getParam.paramPtr); 
		
		if(substream->number == 2) //AUX, wired headset
			audioControlParam.param_pathCfg.outputChnl = AUDIO_CHNL_HEADPHONE;
		audioControlParam.param_pathCfg.audioID = AUDIO_ID_AMRWB_RECORD;
		HAL_AUDIO_Ctrl( ACTION_AUD_DisablePath, &audioControlParam, NULL);
		DEBUG("\n %lx:Disable path outputChnl=%d vol=%d \n",jiffies, audioControlParam.param_pathCfg.outputChnl, audioControlParam.param_pathCfg.volumeLevel );
	}

	spin_lock(pPcmCap->mspinlock_pcm16kcap);
	pPcmCap->mu32Flags &= ~FLAG_PCM16_DSPENABLED;
	spin_unlock(pPcmCap->mspinlock_pcm16kcap);
	DEBUG("PCM16kCaptureStop done\n");
	return 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//  Function Name: DspCodecMsgHandler
//
//  Description: Handle status message from DSP
//
//------------------------------------------------------------

int	AmrWBMsgHandler(StatQ_t *pstatus_msg)
{
	int	ret = 0;
	PTAMRWBCapture pPcmCap;

	pPcmCap = &gPcm16kCapture;


//	DEBUG("AmrWBMsgHandler: msg = 0x%x,arg0 = 0x%x,arg1 = 0x%x,arg2 = 0x%x\n", pstatus_msg->status, pstatus_msg->arg0, pstatus_msg->arg1, pstatus_msg->arg2);		
	if( !(pPcmCap->mu32Flags & FLAG_PCM16_DSPENABLED) )
	{
		DEBUG("AmrWBMsgHandler: received msg before DSP enabled!\n");
		DEBUG("AmrWBMsgHandler: msg = 0x%x,arg0 = 0x%x,arg1 = 0x%x,arg2 = 0x%x\n", pstatus_msg->status, pstatus_msg->arg0, pstatus_msg->arg1, pstatus_msg->arg2);		
		return -1;
	}

	if( (!(pPcmCap->mu32Flags & FLAG_PCM16_DSPLOADED)) && (pstatus_msg->status != STATUS_TEST_PDMA) )
	{
			DEBUG("PCM16kCaptureStart:received msg before AMR-WB loaded\n");
			DEBUG("AmrWBMsgHandler: msg = 0x%x,arg0 = 0x%x,arg1 = 0x%x,arg2 = 0x%x\n", pstatus_msg->status, pstatus_msg->arg0, pstatus_msg->arg1, pstatus_msg->arg2);		
			return -2;
	}
	spin_lock(pPcmCap->mspinlock_pcm16kcap);

	
	switch(pstatus_msg->status)
	{
		case STATUS_TEST_PDMA:
			DEBUG("AmrWBMsgHandler: msg = 0x%x,arg0 = 0x%x,arg1 = 0x%x,arg2 = 0x%x\n", pstatus_msg->status, pstatus_msg->arg0, pstatus_msg->arg1, pstatus_msg->arg2);		
			//indicate encoder image is ready
			if((pstatus_msg->arg0 == 0xaaaa) && (pstatus_msg->arg1 == 0xbbbb))
				pPcmCap->mu32Flags |= FLAG_PCM16_DSPLOADED;
			else
				;//printk("status message above\n");//display status message
			break;
		case STATUS_PRAM_CODEC_DONE_CAPTURE:
			pPcmCap->u32TotalFrames++;

			//It is one frame of PCM data (20ms), but because we use the same callback function as 8k pcm, so I put 2 frames to tell we have 320*2 bytes data
			if(pPcmCap->mpfDumpFrames)
			{
				if(pPcmCap->u32TotalFrames>1)//drop first frame to eliminate clitch noise
					pPcmCap->mpfDumpFrames((Int8 *)&pPcmCap->mpPg5_sm->shared_dsp_capture16KPCM[0], 2);
			}
			
			pPcmCap->mpUnpage_sm->shared_MM_wb_amr_mode[3] = 0; //tell DSP we finish data copy
//			pPcmCap->mpUnpage_sm->shared_MM_wb_amr_mode[4] = 0;
			break;
		case STATUS_PRAM_CODEC_DONE_RECORD:
		case STATUS_PRAM_CODEC_INPUT_LOW:
		case STATUS_PRAM_CODEC_INPUT_EMPTY:
		case STATUS_PRAM_CODEC_OUTPUT_FULL:
		case STATUS_PRAM_CODEC_OUTPUT_LOW:
		case STATUS_PRAM_CODEC_DONEPLAY:
		case STATUS_PRAM_CODEC_CANCELPLAY:
			break;
		default:
			pr_info("\AmrWBMsgHandler:Unexpected message\n");		
			break;
	}

	spin_unlock(pPcmCap->mspinlock_pcm16kcap);

	return ret;
}
