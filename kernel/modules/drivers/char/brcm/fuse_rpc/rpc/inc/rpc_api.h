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
*   @file   rpc_api.h
*
*   @brief  This file defines the rpc public api's
*
****************************************************************************/
#ifndef ARPC_SYS_API_H
#define ARPC_SYS_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mobcom_types.h"
#include "rpc_global.h"

#define INCLUDE_CAPI2_MSG_IDS
#include "resultcode.h"
#include "taskmsgs.h"

#include "rpc_ipc.h"
#include "xdr.h"

/****************************************************************************/
/**

*   @defgroup   RPC_SysApi   RPC API
*
*   @brief      This group defines the interfaces to init RPC system
*
*
****************************************************************************/


/**
 * @addtogroup RPC_SysApi
 * @{
 */

/**
	XDR table record field
**/
typedef struct 
{
	MsgType_t	msgType;	///< Message ID for the message
	const char* msgTypeStr;	///< string for Message ID
	xdrproc_t	xdr_proc;	///< XDR proc for this message id
	UInt32		xdr_size;	///< actual size for this message
	u_int32_t	maxMsgSize;	///< max size for this message
}RPC_XdrInfo_t;

/**
	RPC Message
**/
typedef struct tag_XDR_Msg_t
{
	MsgType_t	msgId;	///< Message ID for the message
	UInt32		tid;	///< Transaction ID
	UInt8		clientID;	///< Client ID
	void*		dataBuf;	///< Payload
	UInt32		dataLen;	///< Payload len
} RPC_Msg_t;


/**
	Ack result types
**/
typedef enum{

	ACK_SUCCESS,	///< request ack succeed
	ACK_FAILED,		///< ack fail for unknown reasons
	ACK_TRANSMIT_FAIL,	///< ack fail due to fifo full, fifo mem full etc.
	ACK_CRITICAL_ERROR	///< ack fail due to comms processor reset ( The use case for this error is TBD )
} RPC_ACK_Result_t;



/**
Result data buffer handle.
**/
typedef void* ResultDataBufHandle_t;

/**
Client RPC handle
**/
typedef UInt8 RPC_Handle_t;


//***************************************************************************************
/**
    Function callback for Async responses and also to receive unsolicited notifications
	@param		inMsg (in) RPC message
	@param		dataBufHandle (in) The Data buffer handle associate with RPC message.
	@param		userContextData (in) User Context Data
	@return		None
	@note
	Call RPC_SYSFreeResultDataBuffer(dataBufHandle) to free the result data memory.

**/
typedef void (RPC_ResponseCallbackFunc_t) (RPC_Msg_t* inMsg, ResultDataBufHandle_t dataBufHandle, UInt32 userContextData);

//***************************************************************************************
/**
    Function callback for Async Acknowledgement in response to request.
	@param		tid (in) Transaction ID passed in the original request
	@param		rpcClientID (in) Transaction ID
	@param		ackResult (in ) See RPC_ACK_Result_t for more details.
	@param		ackUsrData (in ) Can be used send pending flag to say the response may be delayed
	@return		None
	@note

**/
typedef void (RPC_AckCallbackFunc_t) (UInt32 tid, UInt8 rpcClientID, RPC_ACK_Result_t ackResult, UInt32 ackUsrData);

//***************************************************************************************
/**
    Function callback for notify request message to the client
	@param		inMsg (in) RPC message
	@param		dataBufHandle (in) The Data buffer handle associate with RPC message.
	@param		userContextData (in) User Context Data
	@return		None
	@note
	Call RPC_SYSFreeResultDataBuffer(dataBufHandle) to free the result data memory.

**/
typedef void (RPC_RequestCallbackFunc_t) (RPC_Msg_t* inMsg, ResultDataBufHandle_t dataBufHandle, UInt32 userContextData);


//***************************************************************************************
/**
    Optional Function callback called by RPC framework when pre-header need to be serialized/de-serialized.
	If this callback is NULL then the RPC framework will trriger XDR routine from the clients XDR table
	@param		xdrs (in) Current XDR context
	@param		data (in) The pointer to the clients message data
	@param		msgId (in) The message ID for the message
	@param		proc (in) The xdr proc associated with message ID from XDR table.
	@return		None
	@note

**/
typedef Boolean (RPC_MainXDRCallback_t) (XDR *xdrs, void** data, MsgType_t msgId, xdrproc_t proc);

//***************************************************************************************
/**
    Function callback to dispatch the event handling to different task/thread
	When finished handover the RPC_HandleEvent is to be called
	@param		eventHandle (in) event handle to be passed in RPC_HandleEvent
	@return		None
	@note

**/
typedef void (RPC_EventCallbackFunc_t) (void* eventHandle);

/**
RPC Init params
**/
typedef struct
{
	RPC_RequestCallbackFunc_t*		reqCb;	///< Request callback
	RPC_ResponseCallbackFunc_t*		respCb; ///< Response callback
	RPC_AckCallbackFunc_t*			ackCb;	///< Ack calback
	RPC_FlowControlCallbackFunc_t*	flowCb;	///< Flow control event callback
	RPC_InterfaceType_t				iType;	///< Interface type
	RPC_XdrInfo_t					*xdrtbl;	///< Pointer XDR map table
	UInt16							table_size;	///< The size of the table
	UInt32							userData;	///< User context data
	xdrproc_t						mainProc;	///< Optional Pre-Header XDR proc
	UInt32							maxDataBufSize; 	///< Max size of the recv message
}RPC_InitParams_t;


//***************************************************************************************
/**
    Function to initialize RPC system
	@param		eventCb (in) Pointer to callback function to handover RPC events to a Task
	@return		\n RESULT_OK for success,
				\n RESULT_ERROR for failure
**/
Result_t RPC_SYS_Init(RPC_EventCallbackFunc_t eventCb);


//***************************************************************************************
/**
    Function called to handle the RPC event previously notified in RPC_EventCallbackFunc_t
	@return		\n RESULT_OK for success,
				\n RESULT_ERROR for failure
**/
void RPC_HandleEvent(void* eventHandle);

//***************************************************************************************
/**
    Function to free the result data buffer passed in the response callback.
	@param		dataBufHandle (in) free the result data buffer for the response callback
	@return		None
	@note
**/
void RPC_SYSFreeResultDataBuffer(ResultDataBufHandle_t dataBufHandle);


//***************************************************************************************
/**
    Function to register new RPC client.
	The client needs to provide callbacks to receive Aysnc ack and Async responses
	@param		params (in) RPC Init params
	@return		Non Zero RPC_Handle_t
	@note
	The RPC_SYS_Init() is to be called before calling this function
**/
RPC_Handle_t RPC_SYS_RegisterClient(const RPC_InitParams_t *params); 


//***************************************************************************************
/**
    Function to de-register current RPC client
	@param		handle (in) RPC Handle
	@return		TRUE if the client handle is valid
**/
Boolean RPC_SYS_DeregisterClient(RPC_Handle_t handle);



//***************************************************************************************
/**
    Function to set system properties by either AP or CP based on the granted permission.
	@param	type (in) property type
	@param	value (in) property value
	@return	TRUE on success or FALSE
	@note
**/
Boolean RPC_SetProperty(RPC_PropType_t type, UInt32 value);

//***************************************************************************************
/**
    Function to get system properties by either AP or CP based on the granted permission.
	@param	type (in) property type
	@param	value (in) property value returned to caller
	@return	TRUE on success or FALSE
	@note
**/
Boolean RPC_GetProperty(RPC_PropType_t type, UInt32 *value);

//***************************************************************************************
/**
    Function to serialize RPC request message and send the packed message via IPC
	@param	rpcMsg (in) RPC message to be serialized
	@return		\n RESULT_OK for success,
				\n RESULT_ERROR for failure
	@note
**/
Result_t RPC_SerializeReq(RPC_Msg_t* rpcMsg);

//***************************************************************************************
/**
    Function to serialize RPC response message and send the packed message via IPC
	@param	rpcMsg (in) RPC message to be serialized
	@return		\n RESULT_OK for success,
				\n RESULT_ERROR for failure
	@note
**/
Result_t RPC_SerializeRsp(RPC_Msg_t* rpcMsg);

//***************************************************************************************
/**
    Function to send ack for received request message
	@param	handle (in) RPC buf handle of the received request message
	@param	ackUsrData (in) User Data sent transparently in Ack message
	@return		\n RESULT_OK for success,
				\n RESULT_ERROR for failure
	@note
**/
Result_t RPC_SendAckForRequest(ResultDataBufHandle_t handle, UInt32 ackUsrData);

//***************************************************************************************
/**
    Function to check if the msgID is valid for RPC. RPC checks if the xdr entry exist in the XDR table.
	Clients MUST include the message id in the XDR table for this message to be handled
	@param	msgID (in) Message ID to check
	@return	TRUE on success or FALSE
	@note
**/
Boolean RPC_IsValidMsg(MsgType_t msgID);

//***************************************************************************************
/**
    Function to get payload size for the given message id
	@param	msgID (in) Message ID
	@return	size of the payload
	@note
**/
UInt32 RPC_GetMsgPayloadSize(MsgType_t msgID);

Boolean RPC_SYS_BindClientID(RPC_Handle_t handle, UInt8 userClientID);

XDR_ENUM_DECLARE(MsgType_t)


void  RPC_GetPersistentData(IPC_PersistentDataStore_t *thePersistentData);

void RPC_IPC_SetCPCrash(void);

RpcProcessorType_t RPC_GetProcessorType(void);

/** @} */


#ifdef __cplusplus
}
#endif

#endif
