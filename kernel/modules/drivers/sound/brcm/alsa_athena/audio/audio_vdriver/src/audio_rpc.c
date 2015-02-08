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
//#include "string.h"
#include "mobcom_types.h"
#include "resultcode.h"
#include "taskmsgs.h"

#include "ipcproperties.h"
#include "rpc_global.h"
#include "rpc_ipc.h"

#include "xdr_porting_layer.h"
#include "xdr.h"
#include "rpc_api.h"

#include "xassert.h"
#include "audio_consts.h"
#include "auddrv_def.h"
#include "csl_aud_drv.h"
#include "audio_vdriver.h"

#include "rpc_sync_api.h"

#include "dspcmd.h"
#include "ripcmdq.h"
#include "audioapi_asic.h"
#include "log.h"

static UInt8 audioClientId = 0;
static RPC_Handle_t rpcHandle ;


//If this struct is changed then please change xdr_Audio_Params_t() also.
typedef struct
{
	UInt32 param1;
	UInt32 param2;
	UInt32 param3;
	UInt32 param4;
	UInt32 param5;
	UInt32 param6;
}Audio_Params_t;

static bool_t xdr_Audio_Params_t(void* xdrs, Audio_Params_t *rsp);
#define _T(a) a

/*************************************  FRAMEWORK CODE *******************************************************************/
static RPC_XdrInfo_t AUDIO_Prim_dscrm[] = {
	
	/* Add phonebook message serialize/deserialize routine map */
	{ MSG_AUDIO_CTRL_GENERIC_REQ,_T("MSG_AUDIO_CTRL_GENERIC_REQ"), (xdrproc_t) xdr_Audio_Params_t,  sizeof( Audio_Params_t ), 0},
	{ MSG_AUDIO_CTRL_GENERIC_RSP,_T("MSG_AUDIO_CTRL_GENERIC_RSP"), (xdrproc_t)xdr_UInt32, sizeof( UInt32 ), 0 },
	{ MSG_AUDIO_CTRL_DSP_REQ,_T("MSG_AUDIO_CTRL_DSP_REQ"), (xdrproc_t) xdr_Audio_Params_t,  sizeof( Audio_Params_t ), 0},
	{ MSG_AUDIO_CTRL_DSP_RSP,_T("MSG_AUDIO_CTRL_DSP_RSP"), (xdrproc_t)xdr_UInt32, sizeof( UInt32 ), 0 },
	{ (MsgType_t)__dontcare__, "",NULL_xdrproc_t, 0,0 } 
};


#if defined(FUSE_COMMS_PROCESSOR) 
static Result_t SendAudioRspForRequest(RPC_Msg_t* req, MsgType_t msgType, void* payload)
{
	RPC_Msg_t rsp;
	
	rsp.msgId = msgType;
	rsp.tid = req->tid;
	rsp.clientID = req->clientID;
	rsp.dataBuf = (void*)payload;
	rsp.dataLen = 0;
	
	return RPC_SerializeRsp(&rsp);
}
#endif

void HandleAudioEventReqCb(RPC_Msg_t* pMsg, 
						 ResultDataBufHandle_t dataBufHandle, 
						 UInt32 userContextData)
{
	Log_DebugPrintf(LOGID_MISC, "HandleAudioEventRspCb msg=0x%x clientID=%d ", pMsg->msgId, 0);

#if defined(FUSE_COMMS_PROCESSOR) 

	 RPC_SendAckForRequest(dataBufHandle, 0);

	if(pMsg->msgId == MSG_AUDIO_CTRL_GENERIC_REQ)
	{
		Audio_Params_t* p = (Audio_Params_t*)pMsg->dataBuf;
		UInt32 val = audio_control_generic(p->param1,p->param2,p->param3,p->param4,p->param5,p->param6);

		SendAudioRspForRequest(pMsg, MSG_AUDIO_CTRL_GENERIC_RSP, &val);
	}
	else if(pMsg->msgId == MSG_AUDIO_CTRL_DSP_REQ)
	{
		Audio_Params_t* p = (Audio_Params_t*)pMsg->dataBuf;
		UInt32 val = audio_control_dsp(p->param1,p->param2,p->param3,p->param4,p->param5,p->param6);
		
		SendAudioRspForRequest(pMsg, MSG_AUDIO_CTRL_DSP_RSP, &val);
	}
	else
		xassert(0, pMsg->msgId);
#endif

	RPC_SYSFreeResultDataBuffer(dataBufHandle);
}

static Boolean AudioCopyPayload( MsgType_t msgType, 
						 void* srcDataBuf, 
						 UInt32 destBufSize,
						 void* destDataBuf, 
						 UInt32* outDestDataSize, 
						 Result_t *outResult)
{
	UInt32 len;

	xassert(srcDataBuf != NULL, 0);

	len = RPC_GetMsgPayloadSize(msgType);

	*outResult = RESULT_OK;
	*outDestDataSize = len;

	if(destDataBuf && srcDataBuf && len <= destBufSize)
	{
		memcpy(destDataBuf, srcDataBuf, len);
		return TRUE;
	}
	return FALSE;
}



void Audio_InitRpc(void)
{
	static int first_time = 1;

	if(first_time)
	{
		RPC_InitParams_t params={0};
		RPC_SyncInitParams_t syncParams;

		params.iType = INTERFACE_RPC_DEFAULT;
		params.table_size = (sizeof(AUDIO_Prim_dscrm)/sizeof(RPC_XdrInfo_t));
		params.xdrtbl = AUDIO_Prim_dscrm;
		params.respCb = NULL;
		params.reqCb = HandleAudioEventReqCb;
		syncParams.copyCb = AudioCopyPayload;

        rpcHandle = RPC_SyncRegisterClient(&params,&syncParams);
        audioClientId = RPC_SYS_GetClientID(rpcHandle);

		
		//audioClientId = RPC_SyncRegisterClient(&params,&syncParams);


		first_time = 0;
		Log_DebugPrintf(LOGID_MISC, "Audio_InitRpc %d", audioClientId);
	}
}

/*************************************  AUDIO API CODE *******************************************************************/


void CAPI2_audio_control_generic(UInt32 tid, UInt8 clientID, Audio_Params_t* params)
{
	RPC_Msg_t msg;

	msg.msgId = MSG_AUDIO_CTRL_GENERIC_REQ;
	msg.tid = tid;
	msg.clientID = clientID;
	msg.dataBuf = (void*)params;
	msg.dataLen = 0;
	RPC_SerializeReq(&msg);
}

void CAPI2_audio_control_dsp(UInt32 tid, UInt8 clientID, Audio_Params_t* params)
{
	RPC_Msg_t msg;
	
	msg.msgId = MSG_AUDIO_CTRL_DSP_REQ;
	msg.tid = tid;
	msg.clientID = clientID;
	msg.dataBuf = (void*)params;
	msg.dataLen = 0;
	RPC_SerializeReq(&msg);
}

bool_t xdr_Audio_Params_t(void* xdrs, Audio_Params_t *rsp)
{
	XDR_LOG(xdrs,"Audio_Params_t")

	if(
		xdr_UInt32(xdrs, &rsp->param1) &&
		xdr_UInt32(xdrs, &rsp->param2) &&
		xdr_UInt32(xdrs, &rsp->param3) &&
		xdr_UInt32(xdrs, &rsp->param4) &&
		xdr_UInt32(xdrs, &rsp->param5) &&
		xdr_UInt32(xdrs, &rsp->param6) &&
	1)
		return TRUE;
	else
		return FALSE;
}

#if defined(FUSE_APPS_PROCESSOR) 

UInt32 audio_control_generic(UInt32 param1,UInt32 param2,UInt32 param3,UInt32 param4,UInt32 param5,UInt32 param6)
{
	Audio_Params_t audioParam;
	UInt32 tid;
	MsgType_t msgType;
	RPC_ACK_Result_t ackResult;
	UInt32 val = (UInt32)0;

	audioParam.param1 = param1;
	audioParam.param2 = param2;
	audioParam.param3 = param3;
	audioParam.param4 = param4;
	audioParam.param5 = param5;
	audioParam.param6 = param6;

	tid = RPC_SyncCreateTID( &val, sizeof( UInt32 ) );
	CAPI2_audio_control_generic(tid, audioClientId,&audioParam);
	RPC_SyncWaitForResponse( tid,audioClientId, &ackResult, &msgType, NULL );
	return val;
}

UInt32 audio_control_dsp(UInt32 param1,UInt32 param2,UInt32 param3,UInt32 param4,UInt32 param5,UInt32 param6)
{
	Audio_Params_t audioParam;
	UInt32 tid;
	MsgType_t msgType;
	RPC_ACK_Result_t ackResult;
	UInt32 val = (UInt32)0;
	Log_DebugPrintf(LOGID_AUDIO, "\n\r\t* audio_control_dsp (AP) param1 %d, param2 %d param3 %d param4 %d *\n\r", param1, param2, param3, param4);

	switch (param1)
	{


		case DSPCMD_TYPE_COMMAND_DIGITAL_SOUND:
			RIPCMDQ_DigitalSound((UInt16)param2);
			
			break;

		case DSPCMD_TYPE_COMMAND_SET_ARM2SP:
			RIPCMDQ_SetARM2SP((UInt16)param2, (UInt16)param3);
			
			break;

		case DSPCMD_TYPE_COMMAND_SET_ARM2SP2:
			RIPCMDQ_SetARM2SP2((UInt16)param2, (UInt16)param3);
			
			break;
			
		case DSPCMD_TYPE_COMMAND_SET_BT_NB:
			RIPCMDQ_SetBTNarrowBand((UInt16)param2);
			
			break;

		case DSPCMD_TYPE_COMMAND_USB_HEADSET:
			RIPCMDQ_USBHeadset((UInt16)param2);

			break;		


		case DSPCMD_TYPE_MM_VPU_ENABLE:
				RIPCMDQ_MMVPUEnable((UInt16)param2);
				
			break;
			
		case DSPCMD_TYPE_MM_VPU_DISABLE:
				RIPCMDQ_MMVPUDisable();
				
			break;
			
#if 1	// AMCR PCM enable bit is controlled by ARM audio		
		case DSPCMD_TYPE_AUDIO_SET_PCM:
			RIPCMDQ_DigitalSound((UInt16)param2);

			break;			
#endif							

		case DSPCMD_TYPE_SET_VOCODER_INIT:
			RIPCMD_Vocoder_Init((UInt16)param2);
			
			break;		
			
		case DSPCMD_TYPE_COMMAND_VOIF_CONTROL:
			RIPCMDQ_VOIFControl((UInt16)param2);
			
			break;	
			
		default:
            Log_DebugPrintf(LOGID_AUDIO, "\n\r\t* audio_control_dsp (AP) default case param1 %d, param2 %d param3 %d param4 %d *\n\r", param1, param2, param3, param4);

			audioParam.param1 = param1;
			audioParam.param2 = param2;
			audioParam.param3 = param3;
			audioParam.param4 = param4;
			audioParam.param5 = param5;
			audioParam.param6 = param6;

			tid = RPC_SyncCreateTID( &val, sizeof( UInt32 ) );
			CAPI2_audio_control_dsp(tid, audioClientId,&audioParam);
			RPC_SyncWaitForResponse( tid,audioClientId, &ackResult, &msgType, NULL );
	
			break;
	} 
	
	return val;

}

#endif

