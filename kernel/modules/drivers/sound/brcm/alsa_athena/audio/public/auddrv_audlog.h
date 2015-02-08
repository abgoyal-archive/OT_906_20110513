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
*  @file  auddrv_audlog.h
*  @brief Audio Driver API for audio logging
*
*****************************************************************************/
/**
*
* @defgroup AudioLoggingDriver    Audio Logging Driver
* @brief    This group defines the common APIs for the audio logging driver
*
*****************************************************************************/


#ifndef	__AUDDRV_AUDLOG_H__
#define	__AUDDRV_AUDLOG_H__

/**
*
* @addtogroup AudioLoggingDriver
* @{
*/

#define	DBG_MSG_ON_ATCMD

#define	LOG_FRAME_NUMBER						(20)
#define	LOG_FRAME_NUMBER_FOR_FLASH_MEMORY		(100)
#define	LOG_STREAM_NUMBER						(4)

#define	LOG_BUFFER_NUMBER	40
#define	LOG_FRAME_SIZE		162
#define	LOG_FILE_SIZE		(2*1024*1024)

/**
	Log message consumer type
**/
typedef enum
{
	LOG_TO_PC = 0,		//< Log message to PC/MTT 
	LOG_TO_FLASH,		//< Save log message to local flash
} AUDLOG_DEST_en_t;

/**
	Audvoc driver call back for logging message save to file system
**/
typedef struct audlog_cb_info_t
{
  UInt16 *p_LogRead;	//< shared memory read pointer
  UInt32 size_to_read;	//< read size
  UInt32 capture_point;	//< the capture point value in the stream
  UInt32 stream_index;	//< the stream number
} AUDLOG_CB_INFO;

typedef void (*AUDLOG_CB_FUNC)( AUDLOG_CB_INFO *);


/**
	Log data source defintion with DSP sharedmem interface
**/
typedef struct audvoc_log_t
{
	AUDLOG_CB_FUNC	LogMessageCallback;	//< call back function to save stream frame to flash
	AUDLOG_DEST_en_t	log_consumer[4];	//< log message consumer 0 : MTT  1 : file system 
} AUDLOG_INFO;


typedef struct audvoc_audio_buffer_t
{
	UInt16	*buffer;			///< buffer pointer
	UInt16	*current_ptr;		///< current position of buffer
	Int32	length;				///< buffer length
	UInt32	bits_per_sample;	///< number of bits per audio sample
	Int32	flag;				///< flag = 0, buffer this block, flag = 1, start to play immediately.
	Int32	buffer_type;		///< buffer_type = 0,	buffer mode one buffer, buffer_type = 1, stream mode, use queues. 
	UInt32	buffer_serial_index; ///< buffer serial ID
} AUDVOC_BUFFER_INFO;

typedef struct
{
	UInt32				type;
	AUDVOC_BUFFER_INFO	block_info;
} AUDIO_PLAY_MSG_t;

typedef	struct 
{
	UInt16	stream_index;
	UInt16	log_capture_control;
	UInt16	log_msg[LOG_FRAME_SIZE/2];	// VR_Lin_PCM_t log_msg;
} LOG_FRAME_t;	

typedef struct
{
	UInt16*	p_LogMsg;
	UInt32	i_LogMsgSize;
} LOG_MSG_SAVE_t;

#ifdef __cplusplus
extern "C" {
#endif


//Initialize driver internal variables and task queue. 
Result_t AUDDRV_AudLog_Init( void );

//Shut down driver internal variables and task queue. 
Result_t AUDDRV_AudLog_Shutdown( void );

//when driver finished the data in the buffer, driver generates this callback to let client use the buffer.

Result_t AUDDRV_AudLog_SetBufDoneCB (AUDLOG_CB_FUNC bufDone_cb);


//Read audio data from driver. 
UInt32 AUDDRV_AudLog_Read( 
                    UInt8*     pBuf,
                    UInt32	   nSize
					);

Result_t AUDDRV_AudLog_Start ( 
							  UInt32 log_stream,
							  UInt32 log_capture_point,
							  AUDLOG_DEST_en_t log_consumer,
							  char *filename
							  );

Result_t AUDDRV_AudLog_Pause ( void );

Result_t AUDDRV_AudLog_Resume( void );

Result_t AUDDRV_AudLog_Stop( UInt32 log_stream );

Result_t AUDDRV_AudLog_StartRetrieveFile( char *filename );
Result_t AUDDRV_AudLog_StopRetrieveFile( void );

void AUDLOG_ProcessLogChannel( StatQ_t * msg);

#ifdef __cplusplus
}
#endif

/** @} */

#endif // __AUDDRV_AUDLOG_H__
