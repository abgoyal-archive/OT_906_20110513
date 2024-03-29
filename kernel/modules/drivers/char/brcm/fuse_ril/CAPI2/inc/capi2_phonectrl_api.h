/****************************************************************************
*
*     Copyright (c) 2007-2008 Broadcom Corporation
*
*   Unless you and Broadcom execute a separate written software license
*   agreement governing use of this software, this software is licensed to you
*   under the terms of the GNU General Public License version 2, available
*    at http://www.gnu.org/licenses/old-licenses/gpl-2.0.html (the "GPL").
*
*   Notwithstanding the above, under no circumstances may you combine this
*   software in any way with any other Broadcom software provided under a license
*   other than the GPL, without Broadcom's express prior written consent.
*
****************************************************************************/
/**
*
*   @file   capi2_phonectrl_api.h
*
*   @brief  This file defines the capi2 api's related to phone control
*
****************************************************************************/
#ifndef CAPI2_PHONECTRL_API_H
#define CAPI2_PHONECTRL_API_H

#ifdef __cplusplus
extern "C" {
#endif

#define INCLUDE_IPC_PATH

#include "capi2_types.h"
#include "capi2_pchtypes.h"
#include "capi2_ds_api.h"
#include "capi2_sim_api.h"

#include "capi2_global.h"


#include "uelbs_api.h"


/****************************************************************************/
/**

*   @defgroup   CAPI2_PhoneControlSysInit   System Initialization API
*
*   @brief      This group defines the interfaces to init CAPI2 system
*
*
****************************************************************************/

#ifndef RPC_INCLUDED

/**
	CAPI2 Ack result types
**/
typedef enum{

	ACK_SUCCESS,	///< capi2 request ack succeed
	ACK_FAILED,		///< capi2 ack fail for unknown reasons
	ACK_TRANSMIT_FAIL,	///< capi2 ack fail due to fifo full, fifo mem full etc.
	ACK_CRITICAL_ERROR	///< capi2 ack fail due to comms processor reset ( The use case for this error is TBD )
} CAPI2_ACK_Result_t;

/**
	CAPI2 property types
**/
typedef enum
{
	CAPI2_PROP_START_AP = IPC_PROPERTY_START_AP_FOR_CAPI2,			///< ( AP is Read/Write, CP is Read only )
 	CAPI2_PROP_BACKLIGHT_LEVEL,		///< 0 is off, 1 is low, 2 is high
 	CAPI2_PROP_CHARGER_PRESENT,		///< 1 is present, 0 is not present
 	CAPI2_PROP_AP_IN_DEEPSLEEP,		///< 1 is deepsleep, 0 otherwise
	CAPI2_PROP_END_AP = IPC_PROPERTY_END_AP_FOR_CAPI2,

	CAPI2_PROP_START_CP = IPC_PROPERTY_START_CP_FOR_CAPI2,			///< ( CP is Read/Write, AP is Read only )
 	CAPI2_PROP_CP_IN_DEEPSLEEP,		///< 1 is deepsleep, 0 otherwise
	CAPI2_PROP_END_CP = IPC_PROPERTY_END_CP_FOR_CAPI2,

	CAPI2_MAX_PROP_TYPE = IPC_NUM_OF_PROPERTIES,		///< TBD: Align with IPC max
} CAPI2_PropType_t;

#endif

/**
	audio modes
**/
typedef enum 
{
	AUDIO_MODE_HANDSET = 0,
	AUDIO_MODE_HEADSET,
	AUDIO_MODE_HANDSFREE,
	AUDIO_MODE_BLUETOOTH,
	AUDIO_MODE_SPEAKERPHONE,
	AUDIO_MODE_TTY,
	AUDIO_MODE_INVALID  // numbers of modes above this line has to be the same as the AUDIO_MODE_NUMBER
} AudioMode_t;  // Audio Profiles

/**
	tone types
**/
typedef enum
{
// DTMF Tones
	SPEAKERTONE_ZERO = 0,
	SPEAKERTONE_ONE,
	SPEAKERTONE_TWO,
	SPEAKERTONE_THREE,
	SPEAKERTONE_FOUR,
	SPEAKERTONE_FIVE,
	SPEAKERTONE_SIX,
	SPEAKERTONE_SEVEN,
	SPEAKERTONE_EIGHT,
	SPEAKERTONE_NINE,
	SPEAKERTONE_POUND,
	SPEAKERTONE_STAR,

// Supervisory Tones
	SPEAKERTONE_DIALING,
	SPEAKERTONE_BUSY,
	SPEAKERTONE_RINGING,
	SPEAKERTONE_CONGESTION,
	SPEAKERTONE_RADIO_ACKN,
	SPEAKERTONE_NO_RADIO_PATH,
	SPEAKERTONE_CALL_WAITING,
	SPEAKERTONE_ERROR,
	SPEAKERTONE_SUPERIMPOSE_BEGIN = 0x8000 + SPEAKERTONE_ZERO,
	SPEAKERTONE_SUPERIMPOSE_END   = 0x8000 + SPEAKERTONE_ERROR
} SpeakerTone_t;


/**
	CAPI system states
**/
typedef enum
{
 	SYSTEM_STATE_OFF,				///< System is off, waiting for power-on
 	SYSTEM_STATE_ON,				///< System is on
 	SYSTEM_STATE_ON_NO_RF,			///< System is On but no radio activity
 	SYSTEM_STATE_OFF_IN_PROGRESS,	///< System is powering down in progress
 	SYSTEM_STATE_OFF_CHARGING		///< Charger is plugged in while handset is off
} SystemState_t;				///< System state

typedef enum
{
	EM_BATTMGR_NOT_CHARGING = 0,		///< Not charging currently
	EM_BATTMGR_NORMAL_CHARGING,			///< Normal charging
	EM_BATTMGR_MAINTENANCE_CHARGING,	///< Maintenance charging
	EM_BATTMGR_ERROR_CHARGING			///< There is an error during charging 
} EM_BATTMGR_ChargingStatus_en_t;





/// Rx Level
typedef struct {
	UInt8				RxLev;		///< RxLev
	UInt8				RxQual;		///< RxQual
}MsRxLevelData_t;

/// Rx Signal Level
typedef struct
{
	UInt8 rssi;			///< in GSM mode, this is RXLEV as defined in 3GPP 05.08
						///< in UMTS mode, this is RSCP as defined in 3GPP 25.133
	UInt8 qual;			///< in GSM mode, this is RXQUAL as defined in 3GPP 05.08
						///< in UMTS mode, this is Ec/Io as defined in 3GPP 25.133
} RxSignalInfo_t;

/// The serving cell information.
typedef struct
{
	UInt8			mRAT;		///< Current Radio Access Technology: RAT_NOT_AVAILABLE(0),RAT_GSM(1),RAT_UMTS(2)
	T_UE_LBS_PARAMS mLbsParams; ///< The LBS parameters
} ServingCellInfo_t;

/// Rx Signal Level
typedef struct
{
	Boolean signal_lev_changed;
	Boolean signal_qual_changed;
} RX_SIGNAL_INFO_CHG_t;

/// AT Command response
typedef struct {
	Int16				len;		///< At Response string len
	UInt8				*buffer;	///< Response Buffer
	Boolean				IsUnsolicited; ///< TRUE if unsolicited response
	UInt8				chan; ///< AT channel on AP
}AtResponse_t;

/// Audio Params
typedef struct 
{
	UInt16 speaker_pga;  //level, index
	UInt16 mic_pga;      //level, index
	UInt16 audvoc_vslopgain;
	UInt16 audvoc_vcfgr;
	UInt16 voice_volume_max;  //in dB.
	UInt16 voice_volume_init; //in dB.
	UInt16 audvoc_pslopgain;
	UInt16 audvoc_aslopgain;
	UInt16 audvoc_mixergain;	
}Capi2AudioParams_t;

#define GPIO_INIT_FIELD_NUM 5
#define GPIO_INIT_REC_NUM 64

/// GPIO Field Value
typedef UInt8 GPIO_Field_t[GPIO_INIT_FIELD_NUM];

/// GPIO Record Params
typedef struct
{
	GPIO_Field_t 	gpioRecInfo[ GPIO_INIT_REC_NUM ];
}Capi2GpioValue_t;

/// CAPI2 Response callback result data buffer handle.
#ifndef UNDER_LINUX
typedef void* ResultDataBufHandle_t;
#endif

typedef UInt8* CAPI2_Patch_Revision_Ptr_t;
#define PATCH_REVISION_LEN	16

/**
 * @addtogroup CAPI2_PhoneControlSysInit
 * @{
 */

#ifndef RPC_INCLUDED

//***************************************************************************************
/**
    Function callback for Async CAPI2 responses and also to receive unsolicited notifications
	@param		tid (in) Transaction ID passed in the original request ( INVALID_TID in case of unsolicited notification)
	@param		clientID (in) client ID
	@param		msgType (in) Response msg type, see MsgType_t
	@param		result (in) Result code ( if RESULT_OK and for some msgType types, the dataBuf may need to be examined )
	@param		dataBuf (in) The result data ( needs to be cast to response structure, based on msgType )
	@param		dataLength (in) length/size of dataBuf
	@param		dataBufHandle (in) handle to the result data.
	@return		None
	@note
	Call CAPI2_SYSFreeResultDataBuffer(dataBufHandle) to free the result data memory.

**/
typedef void (CAPI2_ResponseCallbackFunc_t) (UInt32 tid, UInt8 clientID, MsgType_t	msgType, Result_t result, void*	dataBuf, UInt32	dataLength, ResultDataBufHandle_t dataBufHandle);

//***************************************************************************************
/**
    Function callback for Async Acknowledgement in response to CAPI2 request.
	@param		tid (in) Transaction ID passed in the original request
	@param		clientID (in) Transaction ID
	@param		ackResult (in ) See CAPI2_ACK_Result_t for more details.
	@param		isCapiPending (in ) Set to TRUE when the CAPI2 async response is pending
	@return		None
	@note

**/
typedef void (CAPI2_AckCallbackFunc_t) (UInt32 tid, UInt8 clientID, CAPI2_ACK_Result_t ackResult, Boolean isCapiPending);

//***************************************************************************************
/**
    Function callback to dispatch the CAPI2 event handling to different task/thread
	When finished handover the CAPI2_HandleEvent is to be called
	@param		eventHandle (in) event handle to be passed in CAPI2_HandleEvent
	@return		None
	@note

**/
typedef void (CAPI2_EventCallbackFunc_t) (void* eventHandle);

//***************************************************************************************
/**
    Function to initialize IPC mechanism for CAPI2 system ( Should be the first API to call )
	@param		processorType (in) CAPI2_APPS or CAPI2_COMMS
	@return		\n RESULT_OK for success,
				\n RESULT_ERROR for failure
**/
Result_t CAPI2_SYS_EndPointRegister(Capi2ProcessorType_t processorType);

//***************************************************************************************
/**
    Function to initialize CAPI2 system
	@param		eventCb (in) Pointer to callback function to handover capi2 events to a Task
	@return		\n RESULT_OK for success,
				\n RESULT_ERROR for failure
**/
Result_t CAPI2_SYS_Init(CAPI2_EventCallbackFunc_t eventCb);


//***************************************************************************************
/**
    Function to be called to handle the capi2 event previously notified in CAPI2_EventCallbackFunc_t
	@return		\n RESULT_OK for success,
				\n RESULT_ERROR for failure
**/
void CAPI2_HandleEvent(void* eventHandle);

//***************************************************************************************
/**
    Function to free the result data buffer passed in the response callback.
	@param		dataBufHandle (in) free the result data buffer for the response callback
	@return		None
	@note
**/
void CAPI2_SYSFreeResultDataBuffer(ResultDataBufHandle_t dataBufHandle);


//***************************************************************************************
/**
    Function to register new client.
	The client needs to provide callbacks to receive Aysnc ack and Async responses
	@param		respCb (in) Pointer to callback function to receive Async responses ( Mandatory )
	@param		asyncCb (in) Pointer to callback function to receive Async acknowledgement for request sent  ( client can set this to NULL )
	@param		flowControlCb (in) Pointer to callback function to flow control notification ( client can set this to NULL )
	@return		Valid Client ID on success or INVALID_CLIENT_ID on failure.
	@note
	The CAPI2_SYS_Init() is to be called before calling this function
**/
UInt8 CAPI2_SYS_RegisterClient(CAPI2_ResponseCallbackFunc_t respCb,
							   CAPI2_AckCallbackFunc_t		asyncCb,
							   CAPI2_FlowControlCallbackFunc_t	flowControlCb
							   );

//***************************************************************************************
/**
    Function to de-register client from CAPI
	@param		clientID (in) Client ID to de-register
	@return		TRUE if the client ID is valid
**/
Boolean CAPI2_SYS_DeregisterClient(UInt8 clientID);

#endif

//***************************************************************************************
/**
    Function to set a registered client's Registered Event Mask List.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		maskList (in) List of event masks.
	@param		maskLen (in) Number of event masks passed in "maskList".
	@return		None
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_SYS_SET_REGISTERED_EVENT_MASK_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	Boolean ( TRUE for success; FALSE for failure. )

	@note
	This function is called to set the Registered Event Mask list for a registered client.
	For an unsolicited event to be broadcast to a specific client, the event's message type must
	be included in the client's registered event mask list. Specifically, there must be at least
	one registered event mask in the list whose logical AND operation with the event's message type
	is equal to the event's message type itself.

    Typically the client can just use the default setting created by SYS_RegisterForMSEvent()
	which is: one registered event mask of 0xFFFF to receive all unsolicited events.
	If the default setting is used, there is no need to call this function.

    This function can be called for more flexibility with the event mask set to the XOR value of
	the message types of all unsolicited events the client wants to receive.

**/

void CAPI2_SYS_SetRegisteredEventMask(UInt32 tid, UInt8 clientID, UInt16 *maskList, UInt8 maskLen);

//***************************************************************************************
/**
    Function to set a registered client's Filtered Event Mask List.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		maskList (in) List of event masks.
	@param		maskLen (in) Number of event masks passed in "maskList".
	@param		enableFlag (in) Flag to control the filter.
	@return		None
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_SYS_SET_FILTERED_EVENT_MASK_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	Boolean ( TRUE for success; FALSE for failure. )

	@note
	This function is called to set the Filtered Event Mask list for a registered client.
	For an unsolicited event to be broadcast to a specific client, the event's message type must
	NOT be included in the client's filtered event mask list. Specifically, there must not be
	one filtered event mask in the list whose logical AND operation with the event's message type
	is equal to the event's message type itself.

	Typically the client can just use the default setting created by SYS_RegisterForMSEvent()
	which is: one registered event mask of 0xFFFF to receive all unsolicited events.
	If the default setting is used, there is no need to call this function.

    This function can be called for more flexibility with the event mask set to the XOR value of
	the message types of all unsolicited events the client do not want to receive.
**/

void CAPI2_SYS_SetFilteredEventMask(UInt32 tid, UInt8 clientID, UInt16 *maskList, UInt8 maskLen, SysFilterEnable_t enableFlag);

//***************************************************************************************
/**
    Function to de-register from receiving MS events ( unsolicited events )
	@param		clientID (in) Client ID for which to stop receiving events
	@return		TRUE if the client ID is valid
	@note
	This function is used to stop receiving unsolicited events for specified Client ID.
**/
Boolean CAPI2_SYS_DeRegisterForMSEvent(UInt8 clientID);


//***************************************************************************************
/**
    Function to set CP (Modem processor) to Sleep mode
	@param	bSleepMode (in) TRUE will set the Sleep mode, FALSE to set to normal mode
	@return	TRUE on success or FALSE
	@note
	This function is used to stop receiving call registration status , signal strength etc when AP goes to deep sleep.
**/
Boolean CAPI2_SYS_SetSleepMode(Boolean bSleepMode);

//***************************************************************************************
/**
    Called by Open OS layer when interrupt is triggered from CP to AP. The CAPI will internally determine which
	interface needs processing and consequently trigggers callback
	for either Ack, response, packet data or CSD Data.

	@return	none
	@note
	This function should not be called in ISR directly but in high level task.
**/
void CAPI2_SYS_CP_NotificationHandler(void);

#ifndef RPC_INCLUDED

//***************************************************************************************
/**
    Function to set system properties by either AP or CP based on the granted permission.
	@param	type (in) property type
	@param	value (in) property value
	@return	TRUE on success or FALSE
	@note
**/
Boolean CAPI2_SetProperty(CAPI2_PropType_t type, UInt32 value);

//***************************************************************************************
/**
    Function to get system properties by either AP or CP based on the granted permission.
	@param	type (in) property type
	@param	value (in) property value returned to caller
	@return	TRUE on success or FALSE
	@note
**/
Boolean CAPI2_GetProperty(CAPI2_PropType_t type, UInt32 *value);

#endif

/** @} */

/****************************************************************************/
/**
*   @defgroup   CAPI2_PhoneControlAPIGroup	Phone Control API
*   @ingroup    SystemGroup				System API
*
*   @brief      This group defines the interfaces for network registration
*               and platform power up/down.
****************************************************************************/

typedef enum
{
	PLMN_SELECT_AUTO				= 0,	///< automatic
	PLMN_SELECT_MANUAL				= 1,	///< manual
	PLMN_SELECT_DEREG				= 2,	///< deregister from network
	PLMN_SELECT_SET_ONLY			= 3,	///< set only
	PLMN_SELECT_MANUAL_AUTO			= 4,	///< if manual selection fails, automatic mode is entered
	PLMN_SELECT_MANUAL_FORCE_AUTO	= 5,	///< After manual selection finish, switch to Automatic mode
	PLMN_SELECT_INVALID				= 6		///< Invalid Plmn selection
}PlmnSelectMode_t;

typedef enum
{
	PLMN_FORMAT_LONG			= 0,	///< long format alphanumeric
	PLMN_FORMAT_SHORT			= 1,	///< short format alphanumeric
	PLMN_FORMAT_NUMERIC			= 2,	///< numeric
	PLMN_FORMAT_INVALID			= 3		//
}PlmnSelectFormat_t;

/// Network Identity and Time Zone Network Name
typedef struct
{
	UInt8		longName[255];		///< Long AlphaNumeric name
	UInt8		shortName[255];		///< Short for of network name
} nitzNetworkName_t;



#define MAX_PLMN_SEARCH					10		///< Maximum PLMN Search value


/// Real Time Clock Time structure
typedef struct{
	UInt8  Sec;		///< 0-59 seconds
	UInt8  Min;		///< 0-59 minutes
	UInt8  Hour;	///< 0-23 hours
	UInt8  Week;	///< 0-6==sun-sat week
	UInt8  Day;		///< 1-31 day
	UInt8  Month;	///< 1-12 Month
	UInt16 Year;	///< (RTC_YEARBASE) - (RTC_YEARBASE + 99)
} RTCTime_t;

/// Time Zone and Date structure
typedef struct
{
	Int8		timeZone;
	UInt8		dstAdjust;	///< Possible value (0, 1, 2 or "INVALID_DST_VALUE"). "INVALID_DST_VALUE" means network does not pass DST
							///< (Daylight Saving Time) info in MM Information or GMM Information messages. See Section 10.5.3.12 of 3GPP 24.008.
	RTCTime_t	adjustedTime;	///< Real time clock interface time
} TimeZoneDate_t;

/// MS PLMN Info
typedef struct {
	Boolean					matchResult;	///< Match result
	UInt16					mcc;			///< mcc
	UInt16					mnc;			///< mnc
	PLMN_NAME_t				longName;		///< Long plmn name
	PLMN_NAME_t				shortName;		///< Short plmn name
	PLMN_NAME_t				countryName;	///< Country Name
}MsPlmnInfo_t;

/**
Band select info
**/
typedef enum
{
	BAND_NULL									= 0,			///< 0
	BAND_AUTO									= 0x0001,		///< 0 0000 0001
	BAND_GSM900_ONLY							= 0x0002,		///< 0 0000 0010
	BAND_DCS1800_ONLY							= 0x0004,		///< 0 0000 0100
	BAND_GSM900_DCS1800							= 0x0006,		///< 0 0000 0110
	BAND_PCS1900_ONLY							= 0x0008,		///< 0 0000 1000
	BAND_GSM850_ONLY							= 0x0010,		///< 0 0001 0000
	BAND_PCS1900_GSM850							= 0x0018,		///< 0 0001 1000
	BAND_ALL_GSM								= 0x001E,		///< 0 0001 1110 All GSM band(900/1800/850/1900)
	BAND_UMTS2100_ONLY							= 0x0020,		///< 0 0010 0000
	BAND_GSM900_DCS1800_UMTS2100				= 0x0026,		///< 0 0010 0110
	BAND_UMTS1900_ONLY							= 0x0040,		///< 0 0100 0000
	BAND_PCS1900_GSM850_UMTS1900_UMTS850		= 0x01D8,		///< 1 1101 1000 Add 1700 to this group temporarily
	BAND_UMTS850_ONLY							= 0x0080,		///< 0 1000 0000
    BAND_UMTS1700_ONLY							= 0x0100,		///< 1 0000 0000
   BAND_UMTS900_ONLY                   = 0x0200,       ///< 10 0000 0000 - GNATS TR 13152 - Band 8
   BAND_UMTS1800_ONLY                     = 0x0400,       ///< 100 0000 0000 - GNATS TR 13152 - Band 3
   BAND_ALL_UMTS                       = 0x07E0,      ///< 111 1110 0000 All UMTS band(2100/1900/850/1700/900/1800)
   BAND_ALL                                    = 0x07FF     ///< 111 1111 1111
} BandSelect_t;

/**
RAT select info
**/
typedef enum
{
	GSM_ONLY=0,				///< GSM Only
	DUAL_MODE_GSM_PREF=1,	///< GSM Prefer
	DUAL_MODE_UMTS_PREF=2,	///< UMTS prefer
	UMTS_ONLY=3,			///< UMTS only
	INVALID_RAT				///< Invalid RAT
} RATSelect_t;

#define		START_BAND_INVALID			0
#define		START_BAND_GSM900_DCS1800_UMTS2100	1
#define		START_BAND_PCS1900_GSM850_UMTS1900_UMTS850	2

/// Last good band info
typedef struct
{
	UInt8			lastGoodBand;			///< band info
	Boolean			IsLastGoodBandStored;	///< denotes if last band is saved
} LastGoodBandInfo_t;


#define MAX_IMEI_STR	20 ///< Should be atleast 16

#define SYS_IMEI_LEN	8  ///< Max len for IMEI in BCD format	

/// IMEI string
typedef struct
{
	UInt8	imei_str[MAX_IMEI_STR];	///< imei string
}MSImeiStr_t;

/// Phone Registration Info: MCC/MNC/LAC/Cell_ID/RAT/Band value valid only if GSM or GPRS is in Normal Service or Limited Service.
typedef struct {
	MSRegState_t	gsm_reg_state;		///< GSM Registration state
	MSRegState_t	gprs_reg_state;		///< GPRS Registration state
	UInt16			mcc;				///< MCC in Raw format (may include 3rd MNC digit), e.g. 0x13F0 for AT&T in Sunnyvale, CA
	UInt8			mnc;				///< MNC in Raw format, e.g. 0x71 for AT&T in Sunnyvale, CA
	UInt16			lac;				///< Location Area Code
	UInt16			cell_id;			///< Cell ID
	UInt8			rat;				///< Current Radio Access Technology, RAT_NOT_AVAILABLE, RAT_GSM or RAT_UMTS
	UInt8			band;				///< Current band. For possible values see MS_GetCurrentBand()
} MSRegStateInfo_t;


//=========================================================================
// MS State/Information Functions
//=========================================================================
//GSM & GPRS status

/**
 * @addtogroup CAPI2_PhoneControlAPIGroup
 * @{
 */

//***************************************************************************************
/**
	Function to check if MS is GSM registered.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_GSM_REGISTERED_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	Boolean
	@note This function is used to check if the MS has camped on a cell and thereby ready
	to perform normal GSM functions.
**/
void CAPI2_MS_IsGSMRegistered(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
	Function to check if MS is GPRS registered.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_GPRS_REGISTERED_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	Boolean
	@note This function is used to check if the MS is GPRS attached. If true, the MS is ready
	to perform normal GPRS operations.
**/
void CAPI2_MS_IsGPRSRegistered(UInt32 tid, UInt8 clientID);


//***************************************************************************************
/**
	Function to get the GSM registration reject cause generated locally
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_GSM_CAUSE_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	NetworkCause_t
	@note 	This function returns the GSM registration reject cause generated locally.
**/
void CAPI2_MS_GetGSMRegCause(UInt32 tid, UInt8 clientID);


//***************************************************************************************
/**
	Function to get the GPRS registration reject cause generated locally
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_GPRS_CAUSE_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	NetworkCause_t
	@note This function returns the GPRS registration reject cause generated locally.
**/
void CAPI2_MS_GetGPRSRegCause(UInt32 tid, UInt8 clientID);



//***************************************************************************************
/**
	Function to get the MS' registration Location Area Code
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_REGISTERED_LAC_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	LACode_t
**/
void CAPI2_MS_GetRegisteredLAC(UInt32 tid, UInt8 clientID);


//***************************************************************************************
/**
	Function to get the MS' registred Mobile Country Code
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_GET_PLMN_MCC_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	UInt16 The mobile country code of the MS
	@note	This function returns the mobile country code of the network on which the
	MS is registered.
**/
void CAPI2_MS_GetPlmnMCC(UInt32 tid, UInt8 clientID);


//***************************************************************************************
/**
	Function to get the MS' registred Mobile Network Code
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_GET_PLMN_MNC_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	UInt8 The mobile network code of the MS
	@note	This function returns the mobile network code of the network on which the
	MS is registered.
**/
void CAPI2_MS_GetPlmnMNC(UInt32 tid, UInt8 clientID);


//***************************************************************************************
/**
	This function de-registers the mobile from the network and
	powers down the system.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_SYS_POWERDOWN_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	None
**/
void CAPI2_SYS_ProcessPowerDownReq(UInt32 tid, UInt8 clientID);


//***************************************************************************************
/**
	This function powers-up the platform with No RF activity.  In this state
	the system is functional but can not access the network.  It is typically
	used to allow access to run the mobile in a restricted environment such as
	an airplane.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_SYS_POWERUP_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	None
**/
void CAPI2_SYS_ProcessNoRfReq(UInt32 tid, UInt8 clientID);


//***************************************************************************************
/**
	This function powers-up the platform.  The mobile will start to search for
	a network on which to camp and will broadcast events to registered clients.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_SYS_POWERUP_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	None
**/
void CAPI2_SYS_ProcessPowerUpReq(UInt32 tid, UInt8 clientID);


//***************************************************************************************
/**
    This function sends request to power up the COMM (Modem) processor without RF enabled
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_SYS_POWERON_CAUSE_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	PowerOnCause_t	Reason for Power-On
**/
void CAPI2_SYS_GetMSPowerOnCause(UInt32 tid, UInt8 clientID);


//***************************************************************************************
/**
	This is a utility to check if GPRS is allowed
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_GPRS_ALLOWED_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	Boolean TRUE if GPRS is allowed
	@note
	This function is used to check if GPRS is allowed. If the MS is a class B
	device, GPRS should be allowed during GSM voice or data call.
**/
void CAPI2_MS_IsGprsAllowed(UInt32 tid, UInt8 clientID);



//***************************************************************************************
/**
	Function to request the current RAT
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_GET_CURRENT_RAT_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	UInt8 Current RAT
	@note
	This function is used to check the current Radio Access Technology(RAT).
	Possible return values are :<br>
	RAT_NOT_AVAILABLE = 0<br>
	RAT_GSM			  = 1<br>
	RAT_UMTS		  = 2
**/
void CAPI2_MS_GetCurrentRAT(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
	Function to request the current frequency band.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_GET_CURRENT_BAND_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	UInt8 Current frequency band
	@note
	This function is used to return the current frequency band the MS is registered on.
	Possible return values are :<br>
		BAND_GSM_900	<br>
		BAND_GSM_900_P<br>
		BAND_GSM_900_E	<br>
		BAND_GSM_900_R	<br>
		BAND_GSM_1800	<br>
		BAND_GSM_1900	<br>
		BAND_GSM_850	<br>
		BAND_GSM_450	<br>
		BAND_GSM_480	<br>
		BAND_GSM750		<br>
		BAND_GSM_T_GSM_380	<br>
		BAND_GSM_T_GSM_410	<br>
		BAND_GSM_T_GSM_900	<br>
		BAND_UMTS_BAND_I	(UMTS 2100 MHz band)<br>
		BAND_UMTS_BAND_II	(UMTS 1900 MHz band)<br>
		BAND_UMTS_BAND_III	<br>
		BAND_UMTS_BAND_IV	(UMTS 1700 MHz band)<br>
		BAND_UMTS_BAND_V	(UMTS 850 MHz band)<br>
		BAND_UMTS_BAND_VI
**/
void CAPI2_MS_GetCurrentBand(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
    This function perform the selection of PLMN network.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		ucs2 (in) Set if the plmn is in ucs2 format
	@param		selectMode (in) PLMN selection mode
	@param		format (in) Format of plmn
	@param		plmnValue (in) PLMN value
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_PLMN_SELECT_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	UInt16 ( One of the PCHRejectCause_t values )
**/
void CAPI2_MS_PlmnSelect(
						UInt32 tid,
						UInt8 clientID,
						Boolean ucs2,
						PlmnSelectMode_t selectMode,
						PlmnSelectFormat_t format,
						char *plmnValue);

//***************************************************************************************
/**
    This function performs either: the abortion of current manual PLMN selection; revert back to 
	previous PLMN selection mode before manual PLMN selection (typically used after manual PLMN selection fails). 
	
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_PLMN_ABORT_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	None
**/
void CAPI2_MS_AbortPlmnSelect(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
    This function get the current PLMN select mode.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_GET_PLMN_MODE_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	PlmnSelectMode_t
**/
void CAPI2_MS_GetPlmnMode(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
    This function set the current PLMN select mode.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		mode (in) PLMN select mode
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_SET_PLMN_MODE_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	None
**/
void CAPI2_MS_SetPlmnMode(UInt32 tid, UInt8 clientID, PlmnSelectMode_t mode);

//***************************************************************************************
/**
    This function get the current PLMN select format.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_GET_PLMN_FORMAT_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	PlmnSelectFormat_t
**/
void CAPI2_MS_GetPlmnFormat(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
    This function set the current PLMN select format.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		format (in) PLMN format
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_SET_PLMN_FORMAT_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	None
**/
void CAPI2_MS_SetPlmnFormat(UInt32 tid, UInt8 clientID, PlmnSelectFormat_t format);


//***************************************************************************************
/**
    This function converts the passed PLMN name to a string
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		ucs2 (in) encoding of plmn
	@param		plmn_name (in) plmn name
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_CONVERT_PLMN_STRING_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	If the PLMN name string is ASCII encoded, the returned string is ASCII encoded and NULL terminated.
						If the PLMN name string is UCS2 encoded, the returned string is Hex string encoded.
**/
void CAPI2_MS_ConvertPLMNNameStr(UInt32 tid, UInt8 clientID, Boolean ucs2, PLMN_NAME_t *plmn_name);

//***************************************************************************************
/**
	This function determines if the passed MCC-MNC matches the MCC-MNC returned from the network.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		net_mcc (in) mcc
	@param		net_mnc (in) mnc
	@param		mcc (in) mcc
	@param		mnc (in) mnc
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_MATCH_PLMN_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	Boolean

	@note
	The passed MCC-MNC supports the use of "wild-carding" in MCC and MNC:
	if a digit is set to Hex 'D', it matches any digit value in network MCC-MNC.
**/
void CAPI2_MS_IsMatchedPLMN(UInt32 tid, UInt8 clientID, UInt16 net_mcc, UInt8 net_mnc, UInt16 mcc, UInt8 mnc);

//***************************************************************************************
/**
    This function perform the search of available PLMN.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_SEARCH_PLMN_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	None
**/
void CAPI2_MS_SearchAvailablePLMN(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
    This function aborts the search of available PLMN.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_ABORT_PLMN_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	None
**/
void CAPI2_MS_AbortSearchPLMN(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
	This function get the network names (long and short) based upon the  MCC/MNC/LAC tuple.
	The order of priority is as follows:
	-# EONS (based on MCC, MNC, LAC and EF-OPL & EF-PNN in EONS-enabled SIM
	-# CPHS ONS & ONSS (based on EF-ONS & EF-ONSS in CPHS-enabled SIM)
	-# Internal MCC/MNC lookup table (based on SE.13, NAPRD10 and carrier requirements)

	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		mcc registered MCC in raw format, e.g. 0x13F0 for Cingular Wireless
	@param		mnc registered MNC in raw format, e.g. 0x71 for Cingular Wireless
	@param		lac registered LAC in raw format
	@param		ucs2 TRUE if UCS2 format of long name and short name is perferred
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_PLMN_NAME_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	MsPlmnName_t
**/
void CAPI2_MS_GetPLMNNameByCode(UInt32 tid, UInt8 clientID,	UInt16 mcc, UInt8 mnc, UInt16 lac, Boolean ucs2);

//***************************************************************************************
/**
    This function requests an immediate auto network search.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_AUTO_SEARCH_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	None
**/
void CAPI2_MS_AutoSearchReq(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
	Function to get the country code string by mcc
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		mcc (in) country code
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_GET_MCC_COUNTRY_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	Null terminated country string
**/
void CAPI2_PLMN_GetCountryByMcc(UInt32 tid, UInt8 clientID, UInt16 mcc);

//***************************************************************************************
/**
	Function to query the plmn list size
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_PLMN_LIST_SIZE_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	UInt16 ( size of plmn list )
**/
void CAPI2_MS_GetPLMNListSize(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
	Function to gets the plmn info based in the input index
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		index (in) index from the table to retrieve record
	@param		ucs2 TRUE if UCS2 format of long name and short name is perferred
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_PLMN_INFO_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	MsPlmnInfo_t
**/
void CAPI2_MS_GetPLMNEntryByIndex(
					UInt32 tid,
					UInt8 clientID,
					UInt16	index,
					Boolean	ucs2);


//***************************************************************************************
/**
	Function to gets the plmn info based plmn codes
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		ucs2 (in) TRUE if UCS2 format of long name and short name is perferred
	@param		plmn_mcc (in) mcc
	@param		plmn_mnc (in) mnc
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_PLMN_INFO_BY_INDEX_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	MsPlmnInfo_t
**/
void CAPI2_MS_GetPLMNByCode(
					UInt32 tid,
					UInt8 clientID,
					Boolean ucs2,
					UInt16	plmn_mcc,
					UInt16	plmn_mnc);

//***************************************************************************************
/**
    This function sends request to to check if system reset is caused by assert
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_SYS_QUERY_RESET_CAUSE_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	Boolean
**/
void CAPI2_SYS_IsResetCausedByAssert(UInt32 tid, UInt8 clientID);



//***************************************************************************************
/**
    This function queries system state
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_SYS_GET_SYSTEM_STATE_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	SystemState_t
**/
void CAPI2_SYS_GetSystemState(UInt32 tid, UInt8 clientID);


//***************************************************************************************
/**
    This function sends request to set system state
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		state (in) System state
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_SYS_SET_SYSTEM_STATE_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	None
**/
void CAPI2_SYS_SetSystemState(UInt32 tid, UInt8 clientID, SystemState_t state);


//***************************************************************************************
/**
    This function queries the Rx Signal level
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_SYS_GET_RX_LEVEL_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	MsRxLevelData_t
**/
void CAPI2_SYS_GetRxSignalInfo(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
	This function set the flag to enable/disable broadcasting the cell info message.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param      inEnableCellInfoMsg (in) flag to update
	@return		None
	@note
	@n@b Responses 

**/
void CAPI2_SYS_EnableCellInfoMsg(UInt32 tid, UInt8 clientID, Boolean inEnableCellInfoMsg);

//GSM & GPRS status

//***************************************************************************************
/**
    This function checks the GSM registration status
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_SYS_GET_GSMREG_STATUS_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	RegisterStatus_t
**/
void CAPI2_SYS_GetGSMRegistrationStatus(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
    This function checks the GPRS registration status
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_SYS_GET_GPRSREG_STATUS_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	RegisterStatus_t
**/
void CAPI2_SYS_GetGPRSRegistrationStatus(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
    This function checks if either GSM or GPRS is registered
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_SYS_GET_REG_STATUS_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	Boolean
**/
void CAPI2_SYS_IsRegisteredGSMOrGPRS(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
    This function checks the GSM registration status cause
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_SYS_GET_GSMREG_CAUSE_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	PCHRejectCause_t
**/
void CAPI2_SYS_GetGSMRegistrationCause(UInt32 tid, UInt8 clientID);



//***************************************************************************************
/**
    This function checks if the PLMN is forbid
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_SYS_IS_PLMN_FORBIDDEN_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	Boolean
**/
void CAPI2_MS_IsPlmnForbidden(UInt32 tid, UInt8 clientID);


//***************************************************************************************
/**
    This function checks if home plmn is registered
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_HOME_PLMN_REG_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	Boolean
**/
void CAPI2_MS_IsRegisteredHomePLMN(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
    This function sets the power down timer
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		powerDownTimer (in) timer value
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_SET_POWER_DOWN_TIMER_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	None
**/
void CAPI2_MS_SetPowerDownTimer(UInt32 tid, UInt8 clientID, UInt8 powerDownTimer);


//***************************************************************************************
/**
	Function request to set the pre-power-cycle frequency band.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		startBand (in) starting band
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_START_BAND_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	Boolean
	@note
	The valid band should be:
		START_BAND_GSM900_DCS1800_UMTS2100<br>
		START_BAND_PCS1900_GSM850_UMTS1900_UMTS850

**/
void CAPI2_MS_SetStartBand(UInt32 tid, UInt8 clientID, UInt8 startBand);

//***************************************************************************************
/**
	Function to select band to 900/1800/1900/Dual
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		bandSelect (in) band to select
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_SELECT_BAND_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	None
**/
void CAPI2_SYS_SelectBand( UInt32 tid, UInt8 clientID, BandSelect_t bandSelect );

//***************************************************************************************
/**
	This function sets the RATs (Radio Access Technologies) and bands to be supported by platform.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param RAT_cap	GSM_ONLY(0), DUAL_MODE_GSM_PREF(1), DUAL_MODE_UMTS_PREF(2), or UMTS_ONLY(3)
	@param band_cap	combination of GSM and/or UMTS bands (no need to set BAND_AUTO bit)<br>
					e.g., BAND_GSM900_DCS1800 | BAND_PCS1900_ONLY (triband GSM, no UMTS)<br>
						  BAND_UMTS2100_ONLY (single band UMTS, no GSM)<br>
						  BAND_GSM900_DCS1800 | BAND_PCS1900_ONLY | BAND_UMTS2100_ONLY (triband GSM + single band UMTS)<br>
						  BAND_GSM900_DSC1800 | BAND_PCS1900_GSM850 | BAND_UMTS2100_ONLY (quadband GSM + single band UMTS)<br>
						(if RAT is GSM_ONLY, the UMTS band setting will be ignored;
						 if RAT is UMTS_ONLY, the GSM band setting will be ignored)
						or
						BAND_NULL
						(this resumes supported band setting to system setting from sysparm)
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_SET_RAT_BAND_RSP
	@n@b Result_t :		RESULT_OK if the RATs and bands specified is allowed by sysparm and
						they are consistent with each other
	@n@b ResultData :	None
	@note
	The settings specified by this function won't be in effect until
	SYS_ProcessPowerUpReq() (in power-off state) or
	SYS_SelectBand(BAND_AUTO) (in power-on state) is called.
**/
void CAPI2_MS_SetSupportedRATandBand(UInt32 tid, UInt8 clientID, RATSelect_t RAT_cap, BandSelect_t band_cap);


//******************************************************************************
/**
	This function returns the system RAT setting supported in platform which is defined
	in System Parameter file.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID

	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_MS_GET_SYSTEM_RAT_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::RATSelect_t

**/
void CAPI2_MS_GetSystemRAT(UInt32 tid, UInt8 clientID);

//******************************************************************************
/**
	This function returns the current RAT setting supported in platform. The supported
	RAT is the same as system RAT by default upon powerup, but supported RAT can be
	changed to be a subset of system RAT through CAPI2_MS_SetSupportedRATandBand().
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID

	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_MS_GET_SUPPORTED_RAT_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::RATSelect_t

*/
void CAPI2_MS_GetSupportedRAT(UInt32 tid, UInt8 clientID);

//******************************************************************************
/**
	This function returns the system band setting supported in platform which is defined
	in System Parameter file.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID

	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_MS_GET_SYSTEM_BAND_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::BandSelect_t
*/
void CAPI2_MS_GetSystemBand(UInt32 tid, UInt8 clientID);


//******************************************************************************
/**
	This function returns the current band setting supported in platform. The supported
	band is the same as system band by default upon powerup, but supported band can be
	changed to be a subset of system band through CAPI2_MS_SetSupportedRATandBand().
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID

	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_MS_GET_SUPPORTED_BAND_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::BandSelect_t
*/
void CAPI2_MS_GetSupportedBand(UInt32 tid, UInt8 clientID);


//******************************************************************************
/**
	This function is used to issue test AT commands.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		channel (in) Unused for now
	@param		cmdStr (in) AT Command string

	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_CAPI2_AT_COMMAND_RSP when AT command is run.
					::MSG_CAPI2_AT_COMMAND_IND for at command response ( unsolicited )
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: AtResponse_t for MSG_CAPI2_AT_RESPONSE_IND
	@n@b ResultData: None for MSG_CAPI2_AT_COMMAND_RSP
*/
void CAPI2_AT_ProcessCmd(UInt32 tid, UInt8 clientID, UInt8 channel, UInt8* cmdStr);

//******************************************************************************
/**
	This function returns the registration info
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID

	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_MS_GET_REG_INFO_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::MSRegStateInfo_t

**/
void CAPI2_MS_GetRegistrationInfo(UInt32 tid, UInt8 clientID);


//******************************************************************************
/**
	This function returns the current GPRS multi-slot class. The multi-slot classes
	we support are 1~12 and 33. See Annex B of 3GPP 45.002 for the multi-slot class definition.

	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID

	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_SYSPARAM_GET_MSCLASS_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::UInt16

**/
void CAPI2_SYSPARM_GetMSClass( UInt32 tid, UInt8 clientID );


//******************************************************************************
/** @cond internal*/
/**
    Obsolete API
	This function returns the current microphone gain setting
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID

	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_AUDIO_GET_MIC_GAIN_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::UInt8, range from 0~9 which each step being 3dB (0~27dB)

**/
void CAPI2_AUDIO_GetMicrophoneGainSetting (UInt32 tid, UInt8 clientID);

//******************************************************************************
/**
    Obsolete API
	This function gets the volume setting
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID

	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_AUDIO_GET_SPEAKER_VOL_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::UInt8, range from 0~9 which each step being 3dB (0~27dB)

**/
void CAPI2_AUDIO_GetSpeakerVol(UInt32 tid, UInt8 clientID);


//******************************************************************************
/**
    Obsolete API
	This function sets the volume setting
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		vol (in) Volume setting, range from 0~9 which each step being 3dB (0~27dB)

	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_AUDIO_SET_SPEAKER_VOL_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: None

**/
void CAPI2_AUDIO_SetSpeakerVol(UInt32 tid, UInt8 clientID, UInt8 vol);

//******************************************************************************
/**
    Obsolete API
	This function sets the microphone gain
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		gain (in) mic gain, range from 0~9 which each step being 3dB (0~27dB)

	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_AUDIO_SET_MIC_GAIN_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::UInt16

**/
void CAPI2_AUDIO_SetMicrophoneGain (UInt32 tid, UInt8 clientID, UInt8 gain);
/** @endcond */

//******************************************************************************
/**
	This function returns the manufacture name
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID

	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_SYSPARAM_GET_MNF_NAME_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: Null terminated UInt8*

**/
void CAPI2_SYSPARM_GetManufacturerName(UInt32 tid, UInt8 clientID);

//******************************************************************************
/**
	This function returns the MS model name
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID

	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_SYSPARAM_GET_MODEL_NAME_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: Null terminated UInt8*

**/
void CAPI2_SYSPARM_GetModelName(UInt32 tid, UInt8 clientID) ;

//******************************************************************************
/**
	This function returns the software version
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID

	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_SYSPARAM_GET_SW_VERSION_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: Null terminated UInt8*

**/
void CAPI2_SYSPARM_GetSWVersion(UInt32 tid, UInt8 clientID);


//******************************************************************************
/**
	Get GPRS multislot class: Support for 1~12 and 33. See Section Annex B of 3GPP 45.002.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID

	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_SYSPARAM_GET_EGPRS_CLASS_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: UInt16

**/
void CAPI2_SYSPARM_GetEGPRSMSClass( UInt32 tid, UInt8 clientID );

//******************************************************************************
/**
	This function returns the MS IMEI number in string format
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID

	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_SYSPARAM_GET_IMEI_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: Null terminated UInt8*

**/
void CAPI2_UTIL_ExtractImei(UInt32 tid, UInt8 clientID);

//******************************************************************************
/**
	Function to set the threshold parameters to control whether RSSI indication message MSG_RSSI_IND is
	posted to clients. Once this function is called, the difference between the new and old RXLEV/RXQUAL
	values (if current RAT is GSM) or RSCP/Ec/Io values (if current RAT is UMTS) must be larger or equal
	to the threshold in order for the MSG_RSSI_IND message to be sent.

    The caller can pass 0 to a threshold to disable the threshold checking.

	@param tid (in) Unique exchange/transaction id which is passed back in the response
	@param clientID (in) Client ID
	@param gsm_rxlev_thresold (in) GSM RXLEV threshold. See section 8.1.4 of GSM 05.08 for RXLEV values.
	@param gsm_rxqual_thresold (in) GSM RXQUAL threshold. See Section 8.2.4 of GSM 05.08 for RXQUAL values.
	@param umts_rscp_thresold (in) UMTS RSCP threshold. See Section 9.1.1.3 of 3GPP 25.133 for RSCP values.
	@param umts_ecio_thresold (in) UMTS Ec/Io threshold. See Section 9.1.2.3 of 3GPP 25.133 for Ec/Io values.

	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_SYS_SET_RSSI_THRESHOLD_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: None

**/
void CAPI2_SYS_SetRssiThreshold(UInt32 tid, UInt8 clientID, UInt8 gsm_rxlev_thresold, UInt8 gsm_rxqual_thresold, UInt8 umts_rscp_thresold, UInt8 umts_ecio_thresold);

//******************************************************************************
/**
	This function gets the audio related settings
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		audioMode (in) Currently not used

	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_AUDIO_GET_SETTINGS_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::Capi2AudioParams_t

**/
void CAPI2_AUDIO_GetSettings(UInt32 tid, UInt8 clientID, UInt8 audioMode);

/** @} */

//=========================================================================
// MS Database API's for accessing of the elements
//=========================================================================
#define MAX_TEST_ARFCNS 8		///< 8 is because in the current code only 8 ARFCNs are taken from the list.


/**
Structure : Database Element Notify Indication
			The following structure is used for the contents of the message
			MSG_MS_LOCAL_ELEM_NOTIFY_IND. The type of the element which is updated.
**/
typedef struct
{
	MS_Element_t elementType;		///< Database Element Type
} MS_LocalElemNotifyInd_t;			///< MS Database Element Notify Indicaiton


/**
Structure:	Production Test Data for each band. For use with MS_LOCAL_ELEM_TEST_CHAN
			The following structure is used by the upper layers to
			set the production test frequencies for each band.
*/
typedef struct
{
	BandSelect_t	band;			///< the band which the ARFCNs belong to.
 									///< Only BAND_GSM_900_ONLY, BAND_DCS1800_ONLY,
 									///< BAND_PCS1900_ONLY and BAND_GSM850_ONLY are allowed as inputs currently.
	UInt16			numChan;		///< Number of ARFCNs in the ChanListPtr
	UInt16*			chanListPtr;	///< pointer to the list of ARFCNs
} MS_TestChan_t;					///< Test Channel Type

/// MS Element
typedef struct
{
	MS_Element_t  inElemType;
	union
	{
		UInt8				u8Data;
		UInt16				u16Data;
		Boolean				bData;
		MSNetAccess_t		netAccess;
		MSNwOperationMode_t	netOper;
		MSNwType_t			netMsType;
		MS_TestChan_t		testChannel;
		PLMN_t				plmn;
		PlmnSelectMode_t	mode;
		PlmnSelectFormat_t	format;
		UInt8				u3Bytes[3];
		UInt8				u10Bytes[10];
        ECC_REC_LIST_t      ecc_list;
		NVRAMClassmark_t	stackClassmark;
	    UInt8               imeidata[15];
	}data_u;
}CAPI2_MS_Element_t;

/**
 * @addtogroup CAPI2_PhoneControlAPIGroup
 * @{
 */

//**************************************************************************************
/**
	This function is a generic interface that will be used by any clients (external/
	internal) to update the MS Database elements. This function will copy the element
	passed in the third argument in to the database.
	Note: After proper range checks in neccessary the value will be updated.

	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID, used to verify whether the client is allowed to modify
				the element. Currently this is not done. Could be implemented in future.
	@param		data (in) The database element type and data

	@n@b Responses
	@n@b MsgType_t: ::MSG_MS_SET_ELEMENT_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: None
**/
void CAPI2_MS_SetElement(UInt32 tid, UInt8 clientID, CAPI2_MS_Element_t *data);

//**************************************************************************************
/**
	This function is a generic interface that will be used by any clients (external/
	internal) to read any MS Database elements. This function will copy the contents of
	the database value to the memory location passed in by the last argument.
	The calling entity should know what will be the typecast used to retreive the element.

	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID, used to identify that the client is allowd to
				access the element or not.
	@param		inElemType (in) The database element type.

	@n@b Responses
	@n@b MsgType_t: ::MSG_MS_GET_ELEMENT_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: CAPI2_MS_Element_t
**/
void CAPI2_MS_GetElement(UInt32 tid, UInt8 clientID, MS_Element_t inElemType);

/** @} */

/****************************************************************************/
/**

*   @defgroup   CAPI2_AuxAdcDrvGroup   Aux ADC Group
*
*   @brief      This group defines the interfaces to access ADC functions
*
*
****************************************************************************/
/**
 * @addtogroup CAPI2_AuxAdcDrvGroup
 * @{
 */

//******************************************************************************
/**
	This function starts ADC measurement
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		init_value (in) BMR value to start measurement
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_ADC_START_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::UInt16
	@note
	The ADC data will be available within 120us-180us so we just need to
	 wait till the result come back. This function doesn't support multi-entry
*/
void CAPI2_ADCMGR_Start(UInt32 tid, UInt8 clientID, UInt16 init_value);

/** @} */

/**
*	HAL EM BATTMGR function call result
**/
typedef enum
{
	EM_BATTMGR_SUCCESS = 0,						///< Successful
	EM_BATTMGR_ERROR_ACTION_NOT_SUPPORTED,		///< Not supported by platform HW
	EM_BATTMGR_ERROR_INTERNAL_ERROR,			///< Internal error: i2c, comm failure, etc.
	EM_BATTMGR_ERROR_EVENT_HAS_A_CLIENT,		///< Error if trying to register more than
												///< 1 client to an event with one client only requirement
	EM_BATTMGR_ERROR_OTHERS						///< Undefined error
} HAL_EM_BATTMGR_Result_en_t;

/**
*	BATTMGR EVENT types
*	Upper layer can register a function to get notification on event below
*	Historically, only one client registered for callback (MMMI), therefore
*	this event management is designed to take one client only.
**/
typedef enum
{
	EM_BATTMGR_CHARGER_PLUG_IN_EVENT,			///< Charger plug in event for both USB and Wall (basic notification of
												///< charging current existence to APP and
												///< show start charging dialog and animate batt icon).
												///< To be more speficic, ie. if it's USB or WAC, use PMU HAL.
	EM_BATTMGR_CHARGER_PLUG_OUT_EVENT,			///< Charger plug out event
	EM_BATTMGR_ENDOFCHARGE_EVENT,		   		///< End of Charge event. Battery is full - charging is done.
												///< APP uses this to show BATT FULL dialog box.
	EM_BATTMGR_BATT_EXTREME_TEMP_EVENT,			///< BATT temp is outside window (safety) or extreme temperature
	EM_BATTMGR_LOW_BATT_EVENT,					///< BATT low is detected
	EM_BATTMGR_EMPTY_BATT_EVENT,				///< BATT empty is detected
	EM_BATTMGR_BATTLEVEL_CHANGE_EVENT			///< BATT level change is detected
} HAL_EM_BATTMGR_Event_en_t;

/// HAL EM BATTMGR Level notification structure
typedef struct
{
	HAL_EM_BATTMGR_Event_en_t	eventType; ///< The event type
	UInt8 inLevel;			///< The battery level, 0~N, depend the sysparm
	UInt16 inAdc_avg;		///< Adc value in mV. Ex, 4000 is 4.0V, 3800 is 3.8V
	UInt8 inTotal_levels;	///< total levels
}HAL_EM_BatteryLevel_t;

//! HAL EM BATTMGR charger type
typedef enum
{
	EM_BATTMGR_WALL_CHARGER = 0,
	EM_BATTMGR_USB_CHARGER
} HAL_EM_BATTMGR_Charger_t;

//! HAL EM BATTMGR charger in/out
typedef enum
{
	EM_BATTMGR_CHARGER_UNPLUGGED = 0,
	EM_BATTMGR_CHARGER_PLUGGED
} HAL_EM_BATTMGR_Charger_InOut_t;

typedef struct
{
	UInt16 badr_value;
	UInt32 Context;
}ADC_StartRsp_t;

#define MAX_ADC_CHANNELS 10

typedef struct
{
	UInt8 arrayLen;
	UInt16 adcChNum[MAX_ADC_CHANNELS];
}CAPI2_ADC_ChannelReq_t;

typedef struct
{
	UInt8 arrayLen;
	UInt16 IoAdc[MAX_ADC_CHANNELS];
}CAPI2_ADC_ChannelRsp_t;


/**
 * @addtogroup CAPI2_AuxAdcDrvGroup
 * @{
 */

//******************************************************************************
/**
	This function request the registration with Battery Manager on the CP
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		event (in) Battery level event type
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_PMU_BATT_LEVEL_REGISTER_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::HAL_EM_BATTMGR_Result_en_t
	@note
	The notification will be sent via MSG_PMU_BATT_LEVEL_IND with HAL_EM_BatteryLevel_t as payload
*/
void CAPI2_PMU_Battery_Register(UInt32 tid, UInt8 clientID, HAL_EM_BATTMGR_Event_en_t event);


//******************************************************************************
/**
	This function request the immediate battery level notification
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_PMU_BATT_LEVEL_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::None
	@note
	The notification will be sent via MSG_PMU_BATT_LEVEL_IND with HAL_EM_BatteryLevel_t as payload
*/
void CAPI2_PMU_BattADCReq(UInt32 tid, UInt8 clientID);

//******************************************************************************
/**
	This function communicates a change in the charging state from the AP to the CP
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		chargeType (in) Type of charger (USB or Wall)
	@param		inOut (in) Battery level event type (charger plug or unplug)
	@param		status (in) status
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_PMU_BATT_CHARGING_NOTIFICATION_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::None
*/
void CAPI2_PMU_BattChargingNotification(UInt32 tid, UInt8 clientID, HAL_EM_BATTMGR_Charger_t chargeType, HAL_EM_BATTMGR_Charger_InOut_t inOut, UInt8 status);

//******************************************************************************
/**
	This function communicates a change in the charging state to CAPI interface
	@param		charge_type (in) Type of charger (USB or Wall)
	@param		in_out		(in) Battery level event type (charger plug or unplug)
	@return		None
*/
void CAPI2_ChangeChargeState(HAL_EM_BATTMGR_Charger_t charge_type, HAL_EM_BATTMGR_Charger_InOut_t in_out);

//**************************************************************************************
/**
	This function handles the Batt Level Indication originating in the CP Batt Manager.  
	@param		resp (in) Structure indicating type of event and battery levels
	@return		None 
**/
void CEMU_HandleBattLevelInd(HAL_EM_BatteryLevel_t* resp); 

//******************************************************************************
/**
	This 4 ADC use defined (adc_ch_num[4]) channels are read and loaded into the CAPI2_ADC_ChannelRsp_t io array.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		ch (in) Input channel array struct ( only 4 are supported )
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_ADC_MULTI_CH_START_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::CAPI2_ADC_ChannelRsp_t
*/
void CAPI2_ADCMGR_MultiChStart(UInt32 tid, UInt8 clientID, CAPI2_ADC_ChannelReq_t *ch);

//**************************************************************************************
/**
	This function registers events on the AP to events in the batt manager on the CP  
	@param		event (in) Event type
	@return		Result 
**/
HAL_EM_BATTMGR_Result_en_t CEMU_BATTMGR_RegisterStatus(HAL_EM_BATTMGR_Event_en_t event);

/** @} */
/************************************************ INTERNAL CAPI2 Functions for CSS APPS ONLY ****************************************************/
 /** @cond */

typedef struct
{
	UInt16			pgsm_supported;
	UInt16			egsm_supported;
	UInt16			dcs_supported;
	UInt16			pcs_supported;
	UInt16			gsm850_supported;
	UInt16			pgsm_pwr_class;
	UInt16			egsm_pwr_class;
	UInt16			dcs_pwr_class;
	UInt16			pcs_pwr_class;
	UInt16			gsm850_pwr_class;
	UInt16			ms_class_hscsd;
} CAPI2_Class_t;

typedef UInt8* CAPI2_SYSPARM_IMEI_PTR_t;

typedef enum
{
	TIMEZONE_UPDATEMODE_NONE,			///< No time zone update
	TIMEZONE_UPDATEMODE_AUTO,			///< Automatic time zone update
	TIMEZONE_UPDATEMODE_MANUAL,			///< Manual update
	TIMEZONE_UPDATEMODE_USERCONFIRM		///< Time zone update pending user confirmation
} TimeZoneUpdateMode_t;

#define MAX_BATTMGR_LEVELS			10

typedef struct
{
	UInt16 batt_level_table[ MAX_BATTMGR_LEVELS ];
}Batt_Level_Table_t;

//******************************************************************************
/**
	This function
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::UInt16
	@note
*/
void CAPI2_SYSPARM_GetSysparmIndPartFileVersion(UInt32 tid, UInt8 clientID);

//******************************************************************************
/**
	This function
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::char*
	@note
*/
void CAPI2_SYS_GetBootLoaderVersion(UInt32 tid, UInt8 clientID);

//******************************************************************************
/**
	This function
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::char*
	@note
*/
void CAPI2_SYS_GetDSFVersion(UInt32 tid, UInt8 clientID);

//******************************************************************************
/**
	This function
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::UInt16
	@note
*/
void CAPI2_SYSPARM_GetChanMode( UInt32 tid, UInt8 clientID );

//******************************************************************************
/**
	This function
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::CAPI2_Class_t
	@note
*/
void CAPI2_SYSPARM_GetClassmark( UInt32 tid, UInt8 clientID );

//******************************************************************************
/**
	This function
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::char*
	@note
*/
void CAPI2_SYSPARM_GetIMEI( UInt32 tid, UInt8 clientID );

//******************************************************************************
/**
	This function
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::
	@note
*/
void CAPI2_SYSPARM_SetDARPCfg(UInt32 tid, UInt8 clientID, UInt8 darp_cfg);

//******************************************************************************
/**
	This function
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::
	@note
*/
void CAPI2_SYSPARM_SetEGPRSMSClass( UInt32 tid, UInt8 clientID, UInt16 egprs_class );

//******************************************************************************
/**
	This function
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::
	@note
*/
void CAPI2_SYSPARM_SetGPRSMSClass( UInt32 tid, UInt8 clientID, UInt16 gprs_class );

//******************************************************************************
/**
	This function gets BATT_LEVEL_TABLE in system parameter
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_SYSPARM_GETBATT_TABLE_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: :: Batt_Level_Table_t
	@note
*/
void CAPI2_SYSPARM_GetBattTable(UInt32 tid, UInt8 clientID);

//******************************************************************************
/**
	This function
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::
	@note
*/
void CAPI2_SYS_SetMSPowerOnCause( UInt32 tid, UInt8 clientID, UInt8 temp );

//******************************************************************************
/**
	This function This function erases the NITZ Network Name saved in RAM &
	file system. This is required to pass NITZ GCF TC 44.2.9.1.2.

	After calling this function, MMI may call MS_GetPLMNNameByCode()
	again to refresh the registered PLMN name displayed.

	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::None
	@note
*/
void CAPI2_TIMEZONE_DeleteNetworkName(UInt32 tid, UInt8 clientID);

//******************************************************************************
/**
	This function
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::None
	@note
*/
void CAPI2_TIMEZONE_UpdateRTC(UInt32 tid, UInt8 clientID, Boolean updateFlag);


//******************************************************************************
/**
    Function to get time zone update mode
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::TimeZoneUpdateMode_t
	@note
	Possible update modes are manual, auto, user_confirm, none.
*/
void CAPI2_TIMEZONE_GetTZUpdateMode(UInt32 tid, UInt8 clientID);


//******************************************************************************
/**
    Function to set time zone update mode
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		mode (in) TimeZoneUpdateMode_t
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::None
	@note
*/
void CAPI2_TIMEZONE_SetTZUpdateMode(UInt32 tid, UInt8 clientID, TimeZoneUpdateMode_t mode);


//***************************************************************************************
/**
	Function to
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_xx_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	??
**/
void CAPI2_MS_GetGPRSRegState(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
	Function to
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_xx_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	??
**/
void CAPI2_MS_GetGSMRegState(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
	Function to
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_xx_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	??
**/
void CAPI2_MS_GetRegisteredCellInfo(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
	Function to
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_xx_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	??
**/
void CAPI2_MS_GetStartBand(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
	Function to
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_MS_xx_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	??
**/
void CAPI2_MS_SetMEPowerClass(UInt32 tid, UInt8 clientID, UInt8 band, UInt8 pwrClass);

//***************************************************************************************
/**
	Function to check if de-register is in progress
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_IS_DEREGISTER_IN_PROGRESS_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	Boolean
**/
void CAPI2_MS_IsDeRegisterInProgress(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
	Function to check if register is in progress
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_IS_REGISTER_IN_PROGRESS_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	Boolean
**/
void CAPI2_MS_IsRegisterInProgress(UInt32 tid, UInt8 clientID);

//***************************************************************************************
/**
	Function to select plmn rat
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		manual_rat (in) RAT_NOT_AVAILABLE (no RAT specified); RAT_GSM; RAT_UMTS
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_SET_PLMN_SELECT_RAT_RSP
	@n@b Result_t :		RESULT_OK, RESULT_ERROR
	@n@b ResultData :	None
**/
void CAPI2_MS_SetPlmnSelectRat(UInt32 tid, UInt8 clientID, UInt8 manual_rat);

//******************************************************************************
/** 
	Function to request the protocol stack to return test parameters (e.g. measurement report)
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		inPeriodicReport, True to request stack to report TestParam periodically
	@param      inTimeInterval		The time interval between peiodic reports
       @return		None
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_DIAG_MEASURE_REPORT_RSP
	@n@b Result_t :		RESULT_OK
	@n@b ResultData :	None

**/
void CAPI2_DIAG_ApiMeasurmentReportReq(UInt32 tid, UInt8 clientID, Boolean inPeriodicReport, UInt32 inTimeInterval);

//******************************************************************************
/**
	Function to enable or disable Cell Lock Feature
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		cell_lockEnable, TRUE if to enable Cell Lock , FALSE otherwise
      @return		None
      	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_DIAG_CELLLOCK_RSP
	@n@b Result_t :		RESULT_OK
	@n@b ResultData :	None

**/
void CAPI2_DIAG_ApiCellLockReq(UInt32 tid, UInt8 clientID, Boolean cell_lockEnable);

//******************************************************************************
/** 
	Function to query the Cell Lock Feature staus.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
       @return		None
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t :	MSG_DIAG_CELLLOCK_STATUS_RSP
	@n@b Result_t :		RESULT_OK
	@n@b ResultData :	Boolean

**/
void CAPI2_DIAG_ApiCellLockStatus(UInt32 tid, UInt8 clientID);

//******************************************************************************
/**
	This function gets the actual low voltage from the system dependent param
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_SYSPARM_GET_ACTUAL_LOW_VOLT_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::UInt16 ( Low Volt reading )
	@note
*/
void CAPI2_SYSPARM_GetActualLowVoltReading( UInt32 tid, UInt8 clientID );

//******************************************************************************
/**
	This function gets the actual 4p2 voltage reading from the system dependent param
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_SYSPARM_GET_ACTUAL_4P2_VOLT_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::UInt16 (  Volt reading )
	@note
*/
void CAPI2_SYSPARM_GetActual4p2VoltReading( UInt32 tid, UInt8 clientID);

//******************************************************************************
/**
	This function gets the low voltage from the system dependent param
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_SYSPARM_GET_LOW_THRESHOLD_VOLT
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::UInt16 ( Low Volt reading )
	@note
*/
void CAPI2_SYSPARM_GetBattLowThresh( UInt32 tid, UInt8 clientID );

//******************************************************************************
/**
	This function gets the default 4p2 voltage reading from the system dependent param
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_SYSPARM_GET_DEFAULT_4P2_VOLT
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::UInt16 (  4P Volt reading )
	@note
*/
void CAPI2_SYSPARM_GetDefault4p2VoltReading( UInt32 tid, UInt8 clientID);


//******************************************************************************
/**
	This function gets the default gpio values from sysparm from CP.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_SYSPARM_GET_GPIO_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::Capi2GpioValue_t (  Complete GPIO values)
	@note
*/
void CAPI2_SYSPARM_GetGPIO_Value(UInt32 tid, UInt8 clientID);

//******************************************************************************
/**
	This function gets the default gpio values from sysparm from CP.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		inHsdpaPhyCat (in) HSDPA Cat
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_SYS_SET_HSDPA_CATEGORY_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::None
	@note
*/
void CAPI2_SYSPARM_SetHSDPAPHYCategory(UInt32 tid, UInt8 clientID, UInt32 hsdpaPhyCat);

//******************************************************************************
/**
	This function gets the default gpio values from sysparm from CP.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_SYS_GET_HSDPA_CATEGORY_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::UInt32
	@note
*/
void CAPI2_SYSPARM_GetHSDPAPHYCategory(UInt32 tid, UInt8 clientID);

//******************************************************************************
/**
	This function gets the charging status of the battery.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_BATTMGR_GETCHARGINGSTATUS_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::EM_BATTMGR_ChargingStatus_en_t
	@note
*/
void CAPI2_BATTMGR_GetChargingStatus(UInt32 tid, UInt8 clientID);

//******************************************************************************
/**
	This function gets the charging level (percentage) of the battery.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_BATTMGR_GETPERCENTAGELEVEL_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::UInt16
	@note
*/
void CAPI2_BATTMGR_GetPercentageLevel(UInt32 tid, UInt8 clientID);

//******************************************************************************
/**
	This function queries whether the battery is present or not.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_BATTMGR_ISBATTERYPRESENT_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::Boolean
	@note
*/
void CAPI2_BATTMGR_IsBatteryPresent(UInt32 tid, UInt8 clientID);

//******************************************************************************
/**
	This function queries whether the charger is plugged in.
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_BATTMGR_ISCHARGERPLUGIN_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::Boolean
	@note
*/
void CAPI2_BATTMGR_IsChargerPlugIn(UInt32 tid, UInt8 clientID);


//******************************************************************************
/**
	This function queries current battery level
	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	The Async SYS response is as follows
	@n@b Responses
	@n@b MsgType_t: ::MSG_BATTMGR_GET_LEVEL_RSP
	@n@b Result_t: ::RESULT_OK.
	@n@b ResultData: ::UInt16
	@note
*/
void CAPI2_BATTMGR_GetLevel(UInt32 tid, UInt8 clientID);

/** @endcond */

void CAPI2_MS_InitCallCfg(UInt32 tid, UInt8 clientID);
void CAPI2_MS_InitFaxConfig(UInt32 tid, UInt8 clientID);
void CAPI2_MS_InitVideoCallCfg(UInt32 tid, UInt8 clientID);
void CAPI2_MS_InitCallCfgAmpF(UInt32 tid, UInt8 clientID);

void CAPI2_AUDIO_ASIC_SetAudioMode(UInt32 tid, UInt8 clientID, AudioMode_t mode);
void CAPI2_SPEAKER_StartTone(UInt32 tid, UInt8 clientID, SpeakerTone_t tone, UInt8 duration);
void CAPI2_SPEAKER_StartGenericTone(UInt32 tid, UInt8 clientID, Boolean superimpose, UInt16 tone_duration, UInt16 f1, UInt16 f2, UInt16 f3);
void CAPI2_SPEAKER_StopTone(UInt32 tid, UInt8 clientID);
void CAPI2_AUDIO_Turn_EC_NS_OnOff(UInt32 tid, UInt8 clientID, Boolean ec_on_off, Boolean ns_on_off);
void CAPI2_ECHO_SetDigitalTxGain(UInt32 tid, UInt8 clientID, short digital_gain_step);
void CAPI2_RIPCMDQ_Connect_Uplink(UInt32 tid, UInt8 clientID, Boolean Uplink);
void CAPI2_RIPCMDQ_Connect_Downlink(UInt32 tid, UInt8 clientID, Boolean Downlink);
void CAPI2_PMU_BattChargingNotification(UInt32 tid, UInt8 clientID, HAL_EM_BATTMGR_Charger_t chargeType, HAL_EM_BATTMGR_Charger_InOut_t inOut, UInt8 status);
void CAPI2_PATCH_GetRevision(UInt32 tid, UInt8 clientID);
void CAPI2_RTC_SetDST_RSP(UInt32 tid, UInt8 clientID, Boolean status);
void CAPI2_RTC_SetTimeZone_RSP(UInt32 tid, UInt8 clientID, Boolean status);
void CAPI2_RTC_SetTime_RSP(UInt32 tid, UInt8 clientID, Boolean status);
void CAPI2_TestCmds(UInt32 tid, UInt8 clientID, UInt32 testId, UInt32 param1, UInt32 param2, unsigned char* buffer);
void CAPI2_program_equalizer(UInt32 tid, UInt8 clientID, Int32 equalizer_type);
void CAPI2_program_poly_equalizer(UInt32 tid, UInt8 clientID, Int32 equalizer_type);
void CAPI2_program_FIR_IIR_filter(UInt32 tid, UInt8 clientID, UInt16 audio_mode);
void CAPI2_program_poly_FIR_IIR_filter(UInt32 tid, UInt8 clientID, UInt16 audio_mode);
void CAPI2_audio_control_generic(UInt32 tid, UInt8 clientID, UInt32 param1, UInt32 param2, UInt32 param3, UInt32 param4);
void CAPI2_audio_control_dsp(UInt32 tid, UInt8 clientID, UInt32 param1, UInt32 param2, UInt32 param3, UInt32 param4);
Int32 GetCapi2MSElementSize(MS_Element_t inElemType);//Utility function and not really capi2 req/resp
void CAPI2_VOLUMECTRL_SetBasebandVolume(UInt32 tid, UInt8 clientID, UInt8 level, UInt8 chnl, UInt16 *audio_atten, UInt8 extid);
#ifdef __cplusplus
}
#endif

#endif

