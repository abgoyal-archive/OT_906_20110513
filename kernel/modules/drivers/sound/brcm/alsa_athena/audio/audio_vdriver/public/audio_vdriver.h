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
*   @file   audio_vdriver.h
*
*   @brief  common APIs for audio
*
****************************************************************************/
/**
*
* @defgroup Audio    Audio Component
*
* @brief    This group defines the common APIs for audio virtual driver
*
* @ingroup  Audio Component
*****************************************************************************/

/**
*
*  @file  audio_vdriver.h
*
*  @brief Audio Virtual Driver API
*
*  @note
*****************************************************************************/

#ifndef	__AUDIO_VDRIVER_H__
#define	__AUDIO_VDRIVER_H__

/**
*
* @addtogroup Audio
* @{
*/

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	//AP->CP->DSP cmd to reuse the enum in dspcmd.h
	//AP->CP cmd to be listed here including filter loading.

	/**
	AUDDRV_DSP_FASTCMD,
	AUDDRV_DSP_VPUCMD,
	**/

	//CP:
	AUDDRV_CPCMD_TELEPHONY_INIT,
	AUDDRV_CPCMD_SetBasebandVolume,
	AUDDRV_CPCMD_SetAudioMode,
	AUDDRV_CPCMD_READ_AUDVOC_AEQMODE,// = 1,
	AUDDRV_CPCMD_WRITE_AUDVOC_AEQMODE,// = 2,
	AUDDRV_CPCMD_GET_CP_AUDIO_MODE,// = 3,
	AUDDRV_CPCMD_UPDATE_AUDIO_MODE,// = 4,
	AUDDRV_CPCMD_TELEPHONY_RATECHANGE,
	AUDDRV_CPCMD_ENABLE_DSP_DTX
} AUDDRV_CP_CMD_en_t;

typedef enum {
	AUDDRV_USER_GET_SPKPROT,
	AUDDRV_USER_ENA_SPKPROT,
} AudioDrvUserParam_t;

UInt32 audio_control_generic(
				UInt32 param1,
				UInt32 param2,
				UInt32 param3,
				UInt32 param4,
				UInt32 param5,
				UInt32 param6);

UInt32 audio_control_dsp(
				UInt32 param1,
				UInt32 param2,
				UInt32 param3,
				UInt32 param4,
				UInt32 param5,
				UInt32 param6);

void post_msg(
			UInt16 cmd,
			UInt16 arg0,
			UInt16 arg1,
			UInt16 arg2);

//  the control sequence for starting telephony audio.
void AUDDRV_Telephony_Init (
				AUDDRV_MIC_Enum_t   mic,
				AUDDRV_SPKR_Enum_t  speaker
				);

// the control sequence for ratechange of voice call.
void AUDDRV_Telephony_RateChange ( void );

// the control sequence for ending telephony audio.
//this func let DSP to turn off voice path, if need to resume apps operation on voice, controller needs to reenable voice path after phonce call ends.
void AUDDRV_Telephony_Deinit (void );


void AUDDRV_Telephony_SelectMicSpkr  (
				AUDDRV_MIC_Enum_t   mic,
				AUDDRV_SPKR_Enum_t  speaker
				);

// Enable audio output path and audio processing.
void AUDDRV_Enable_Output (
				AUDDRV_InOut_Enum_t     input_path_to_mixer,
				AUDDRV_SPKR_Enum_t      mixer_speaker_selection,
				Boolean                 enable_speaker,
				AUDIO_SAMPLING_RATE_t   sample_rate,
				AUDIO_CHANNEL_NUM_t     input_to_mixer
				);

// Disable audio output path. 
void AUDDRV_Disable_Output ( AUDDRV_InOut_Enum_t  path );

// Enable audio input and digital processing.
void AUDDRV_Enable_Input (
				AUDDRV_InOut_Enum_t 	 input_path,
				AUDDRV_MIC_Enum_t		 mic_selection,
				AUDIO_SAMPLING_RATE_t	 sample_rate
				);

// Disable audio input and digital processing.
void AUDDRV_Disable_Input (  AUDDRV_InOut_Enum_t  path );

Boolean AUDDRV_IsVoiceCallWB(AudioMode_t audio_mode);
Boolean AUDDRV_IsCall16K(AudioMode_t voiceMode);
Boolean AUDDRV_InVoiceCall( void );

void AUDDRV_SaveAudioMode( AudioMode_t audio_mode );
void AUDDRV_SetAudioMode ( AudioMode_t  audio_mode);
void AUDDRV_SetMusicMode ( AudioMode_t  audio_mode);
AudioMode_t AUDDRV_GetAudioMode( void );

void AUDDRV_GetAudioParam (
				AudioParam_t	audioParamType,
				void*			param
				);

void AUDDRV_SetAudioParam (
				AudioParam_t	audioParamType,
				void*			param
				);

void AUDDRV_User_CtrlDSP (
				AudioDrvUserParam_t	audioDrvUserParam,
				void			*user_CB,
				UInt32			param1,
				UInt32			param2
				);

void AUDDRV_User_HandleDSPInt ( UInt32 param1, UInt32 param2, UInt32 param3 );
void AUDDRV_SetPCMOnOff(Boolean	on_off);
AudioEqualizer_en_t AUDDRV_GetEquType( AUDDRV_TYPE_Enum_t path );
void AUDDRV_SetEquType( AUDDRV_TYPE_Enum_t path, AudioEqualizer_en_t equ_id );
#ifdef __cplusplus
}
#endif

#endif // __AUDIO_VDRIVER_H__
