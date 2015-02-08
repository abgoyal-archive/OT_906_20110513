/*****************************************************************************
*
*    (c) 2009 Broadcom Corporation
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
*  @file  dspif_voice_record.h
*
*  @brief DSP common interface APIs for voice record
*
*  @note
*****************************************************************************/
/**
*
* @defgroup Audio    Audio Component
*
* @brief    This group defines the common APIs for audio virtual driver
*
* @ingroup  Audio Component
*****************************************************************************/


#ifndef	__DSPIF_VOICE_RECORD_H__
#define	__DSPIF_VOICE_RECORD_H__

/**
*
* @addtogroup Audio
* @{
*/

#ifdef __cplusplus
extern "C" {
#endif

typedef enum VOCAPTURE_RECORD_MODE_t
{
	VOCAPTURE_RECORD_NONE,
	VOCAPTURE_RECORD_UL,		
	VOCAPTURE_RECORD_DL,
	VOCAPTURE_RECORD_BOTH
} VOCAPTURE_RECORD_MODE_t;

typedef enum
{
    VOCAPTURE_SPEECH_MODE_NONE = 0,
    VOCAPTURE_SPEECH_MODE_FR = 1,
    VOCAPTURE_SPEECH_MODE_EFR = 2,
    VOCAPTURE_SPEECH_MODE_LINEAR_PCM_8K= 3,
    VOCAPTURE_SPEECH_MODE_AMR = 4,
    VOCAPTURE_SPEECH_MODE_AMR_2 = 5,
    VOCAPTURE_SPEECH_MODE_LINEAR_PCM_16K= 6
} VOCAPTURE_SPEECH_MODE_t;						

typedef enum VOCAPTURE_VOICE_MIX_MODE_t
{
	VOCAPTURE_VOICE_MIX_NONE,
	VOCAPTURE_VOICE_MIX_DL,
	VOCAPTURE_VOICE_MIX_UL,
	VOCAPTURE_VOICE_MIX_BOTH
} VOCAPTURE_VOICE_MIX_MODE_t;

typedef void (*record_data_ready_cb_t)(UInt32 buf_index);

void dspif_VPU_record_init ();
void dspif_VPU_record_deinit ();


// ==========================================================================
//
// Function Name: dspif_VPU_record_start
//
// Description: Start the data transfer of VPU record
//
// =========================================================================
Result_t dspif_VPU_record_start ( VOCAPTURE_RECORD_MODE_t	recordMode,
								AUDIO_SAMPLING_RATE_t		samplingRate,
								UInt32						speechMode, // used by AMRNB and AMRWB
								UInt32						dataRate, // used by AMRNB and AMRWB
								Boolean						procEnable,
								Boolean						dexEnable,
								UInt32						numFramesPerInterrupt);

// ==========================================================================
//
// Function Name: dspif_VPU_record_stop
//
// Description: Stop immediately the data transfer of VPU voic record
//
// =========================================================================
Result_t dspif_VPU_record_stop ( void);


// ==========================================================================
//
// Function Name: dspif_VPU_record_pause
//
// Description: Pause the data transfer of VPU voice record
//
// =========================================================================
Result_t dspif_VPU_record_pause (void);


// ==========================================================================
//
// Function Name: dspif_VPU_record_resume
//
// Description: Resume the data transfer of VPU voice record
//
// =========================================================================
Result_t dspif_VPU_record_resume( void);



// ==========================================================================
//
// Function Name: dspif_VPU_record_read_AMRNB
//
// Description: Read data to shared memory for VPU voice record.
//
// =========================================================================
UInt32 dspif_VPU_record_read_AMRNB ( UInt8 *inBuf, UInt32 inSize, UInt16 bufIndex, UInt32 speechMode, UInt32 numFramesPerInterrupt );


// ==========================================================================
//
// Function Name: dspif_VPU_record_read_PCM
//
// Description: Read data to shared memory for VPU voice record.
//
// =========================================================================
UInt32 dspif_VPU_record_read_PCM ( UInt8 *inBuf, UInt32 inSize, UInt16 bufIndex, UInt32 speechMode, UInt32 numFramesPerInterrupt );


//
// APIs of AMRWB
//

// ==========================================================================
//
// Function Name: dspif_AMRWB_record_start
//
// Description: Start the data transfer of AMRWB record
//
// =========================================================================
Result_t dspif_AMRWB_record_start ( VOCAPTURE_RECORD_MODE_t	recordMode,
								AUDIO_SAMPLING_RATE_t		samplingRate,
								UInt32						speechMode, // used by AMRNB and AMRWB
								UInt32						dataRate, // used by AMRNB and AMRWB
								Boolean						procEnable,
								Boolean						dexEnable,
								UInt32						numFramesPerInterrupt);


// ==========================================================================
//
// Function Name: dspif_AMRWB_record_stop
//
// Description: Stop immediately the data transfer of AMRWB voic record
//
// =========================================================================
Result_t dspif_AMRWB_record_stop ( void);


// ==========================================================================
//
// Function Name: dspif_AMRWB_record_pause
//
// Description: Pause the data transfer of AMRWB voice record
//
// =========================================================================
Result_t dspif_AMRWB_record_pause (void);


// ==========================================================================
//
// Function Name: dspif_AMRWB_record_resume
//
// Description: Resume the data transfer of AMRWB voice record
//
// =========================================================================
Result_t dspif_AMRWB_record_resume( void);


// ==========================================================================
//
// Function Name: dspif_AMRWB_record_read
//
// Description: Read data to shared memory for AMRWB voice record.
//
// =========================================================================
UInt32 dspif_AMRWB_record_read ( UInt8 *inBuf, UInt32 inSize, UInt16 bufIndex, UInt32 numFramesPerInterrupt );

// ==========================================================================
//
// Function Name: dspif_AMRWB_record_GetReadIndex
//
// Description: Get the current read index of shared memory for AMRWB voice record.
//
// =========================================================================
UInt16 dspif_AMRWB_record_GetReadIndex (void);


// ==========================================================================
//
// Function Name: dspif_VPU_record_set_cb
//
// Description: sets the callback to be called when recording data is ready.
//
// =========================================================================

Result_t dspif_VPU_record_set_cb (record_data_ready_cb_t* record_data_ready);


#ifdef __cplusplus
}
#endif



#endif // __DSPIF_VOICE_PLAY_H__
