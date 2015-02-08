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
*   @file   capi2_isim_api.c
*
*   @brief  This file defines the interface for CAPI2 ISIM API.
*
****************************************************************************/
/**

*   @defgroup   CAPI2_SIMAPIGroup   SIM API
*   @ingroup    CAPI2_SIMGroup
*
*   @brief      This group defines the interfaces to the ISIM system and provides
*				API documentation.
****************************************************************************/


//******************************************************************************
//	 			include block
//******************************************************************************
#include	"xdr.h"
#include	"capi2_reqrep.h"

//***************************************************************************************
/**
    This function sends request to check on if the inserted SIM/USIM supports ISIM feature
	and Sys Parm indicates we support ISIM. Note that ISIM can be supported only on USIM.

	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	@n@b Responses 
	@n@b MsgType_t : MSG_ISIM_ISISIMSUPPORTED_RSP
	@n@b Result_t :		RESULT_OK
	@n@b ResultData : Boolean TRUE if ISIM supported; FALSE otherwise. 
	
**/	

void CAPI2_ISIM_IsIsimSupported(UInt32 tid, UInt8 clientID)
{
	CAPI2_ReqRep_t req;
	char* stream;
	UInt32 len;
	Result_t result = RESULT_OK;
	/* Create the request */
	/* Init the request */
	memset(&req, 0, sizeof(CAPI2_ReqRep_t));

	/* message specific */
	/* NONE */

	/* serialize */
	CAPI2_SerializeReqRsp(&req, tid, MSG_ISIM_ISISIMSUPPORTED_REQ, clientID, result, &stream, &len);

	/*     Send it accross to the IPC(stream, len)    */
#ifdef BYPASS_IPC
	CAPI2_DeserializeAndHandleReq(stream, len);
#endif

}

//***************************************************************************************
/**
    This function sends request to check on if the ISIM application is activated in the SIM/USIM. 
	If the ISIM application is activated, the socket ID in the response message will be greater than 0, 
	otherwise the socket ID will be 0.

	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@return		None
	@note
	@n@b Responses 
	@n@b MsgType_t : MSG_ISIM_ISISIMACTIVATED_RSP
	@n@b Result_t :		RESULT_OK
	@n@b ResultData : UInt8 socket_id - Socket ID for the ISIM application; 0 if ISIM is not activated.
	
**/	

void CAPI2_ISIM_IsIsimActivated(UInt32 tid, UInt8 clientID)
{
	CAPI2_ReqRep_t req;
	char* stream;
	UInt32 len;
	Result_t result = RESULT_OK;
	/* Create the request */
	/* Init the request */
	memset(&req, 0, sizeof(CAPI2_ReqRep_t));

	/* message specific */
	/* NONE */

	/* serialize */
	CAPI2_SerializeReqRsp(&req, tid, MSG_ISIM_ISISIMACTIVATED_REQ, clientID, result, &stream, &len);

	/*     Send it accross to the IPC(stream, len)    */
#ifdef BYPASS_IPC
	CAPI2_DeserializeAndHandleReq(stream, len);
#endif

}

//***************************************************************************************
/**
    This function activates the ISIM application in the SIM/USIM. 
	This function is applicable only if ISIM_IsIsimSupported() returns TRUE. 

	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		socket_id (in) pointer to the socket ID
	@return		None
	@note
	@n@b Responses 
	@n@b MsgType_t : MSG_ISIM_ACTIVATE_RSP
	@n@b Result_t :		RESULT_OK
	@n@b ResultData : ISIM_ACTIVATE_RSP_t

**/	

void CAPI2_ISIM_ActivateIsimAppli(UInt32 tid, UInt8 clientID)
{
	CAPI2_ReqRep_t req;
	char* stream;
	UInt32 len;
	Result_t result = RESULT_OK;
	/* Create the request */
	/* Init the request */
	memset(&req, 0, sizeof(CAPI2_ReqRep_t));

	/* message specific */
	req.respId = MSG_ISIM_ACTIVATE_RSP;


	/* serialize */
	CAPI2_SerializeReqRsp(&req, tid, MSG_ISIM_ACTIVATEISIMAPPLI_REQ, clientID, result, &stream, &len);

	/*     Send it accross to the IPC(stream, len)    */
#ifdef BYPASS_IPC
	CAPI2_DeserializeAndHandleReq(stream, len);
#endif

}

/**
    This function sends the Authenticate command for IMS AKA Security Context (see Section
	7.1.2.1 of 3GPP 31.103). 
	This function is applicable only if ISIM_IsIsimSupported() returns TRUE and ISIM application is activated.

	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		rand_data (in) RAND data
	@param		rand_len (in) Number of bytes in RAND data
	@param		autn_data (in) AUTN data
	@param		autn_len (in) Number of bytes in AUTN data

	@return		None
	@note
	@n@b Responses 
	@n@b MsgType_t : MSG_ISIM_AUTHEN_AKA_RSP
	@n@b Result_t :		RESULT_OK
	@n@b ResultData : ISIM_AUTHEN_AKA_RSP_t

**/	

void CAPI2_ISIM_SendAuthenAkaReq(UInt32 tid, UInt8 clientID, const UInt8 *rand_data, UInt16 rand_len, const UInt8 *autn_data, UInt16 autn_len)
{
	CAPI2_ReqRep_t req;
	char* stream;
	UInt32 len;
	Result_t result = RESULT_OK;
	/* Create the request */
	/* Init the request */
	memset(&req, 0, sizeof(CAPI2_ReqRep_t));

	/* message specific */
	req.req_rep_u.CAPI2_ISIM_SendAuthenAkaReq_Req.rand.str = (char*) rand_data;
	req.req_rep_u.CAPI2_ISIM_SendAuthenAkaReq_Req.rand.len = rand_len;
	req.req_rep_u.CAPI2_ISIM_SendAuthenAkaReq_Req.autn.str = (char*) autn_data;
	req.req_rep_u.CAPI2_ISIM_SendAuthenAkaReq_Req.autn.len = autn_len;
	req.respId = MSG_ISIM_AUTHEN_AKA_RSP;

	/* serialize */
	CAPI2_SerializeReqRsp(&req, tid, MSG_ISIM_SENDAUTHENAKAREQ_REQ, clientID, result, &stream, &len);

	/*     Send it accross to the IPC(stream, len)    */
#ifdef BYPASS_IPC
	CAPI2_DeserializeAndHandleReq(stream, len);
#endif

}

/**
    This function sends the Authenticate command for HTTP Digest Security Context (see Section
	7.1.2.2 of 3GPP 31.103).
	This function is applicable only if ISIM_IsIsimSupported() returns TRUE and ISIM application is activated.

	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		realm_data (in) REALM data
	@param		realm_len (in) Number of bytes in REALM data
	@param		nonce_data (in) NONCE data
	@param		nonce_len (in) Number of bytes in NONCE data
	@param		cnonce_data (in) CNONCE data
	@param		cnonce_len (in) Number of bytes in CNONCE data

	@return		None
	@note
	@n@b Responses 
	@n@b MsgType_t : MSG_ISIM_AUTHEN_HTTP_RSP
	@n@b Result_t :		RESULT_OK
	@n@b ResultData : ISIM_AUTHEN_HTTP_RSP_t

**/	

void CAPI2_ISIM_SendAuthenHttpReq(UInt32 tid, UInt8 clientID, const UInt8 *realm_data, UInt16 realm_len, const UInt8 *nonce_data, UInt16 nonce_len, const UInt8 *cnonce_data, UInt16 cnonce_len)
{
	CAPI2_ReqRep_t req;
	char* stream;
	UInt32 len;
	Result_t result = RESULT_OK;
	/* Create the request */
	/* Init the request */
	memset(&req, 0, sizeof(CAPI2_ReqRep_t));

	/* message specific */
	req.req_rep_u.CAPI2_ISIM_SendAuthenHttpReq_Req.realm.str = (char*) realm_data;
	req.req_rep_u.CAPI2_ISIM_SendAuthenHttpReq_Req.realm.len = realm_len;
	req.req_rep_u.CAPI2_ISIM_SendAuthenHttpReq_Req.nonce.str = (char*) nonce_data;
	req.req_rep_u.CAPI2_ISIM_SendAuthenHttpReq_Req.nonce.len = nonce_len;
	req.req_rep_u.CAPI2_ISIM_SendAuthenHttpReq_Req.cnonce.str = (char*) cnonce_data;
	req.req_rep_u.CAPI2_ISIM_SendAuthenHttpReq_Req.cnonce.len = cnonce_len;
	req.respId = MSG_ISIM_AUTHEN_HTTP_RSP;

	/* serialize */
	CAPI2_SerializeReqRsp(&req, tid, MSG_ISIM_SENDAUTHENHTTPREQ_REQ, clientID, result, &stream, &len);

	/*     Send it accross to the IPC(stream, len)    */
#ifdef BYPASS_IPC
	CAPI2_DeserializeAndHandleReq(stream, len);
#endif

}

/**
    This function sends the Authenticate command for GBA Security Context in Bootstrapping Mode (see Section
	7.1.2.3 of 3GPP 31.103) to ISIM. 
	This function is applicable only if ISIM_IsIsimSupported() returns TRUE. 

	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		rand_data (in) RAND data
	@param		rand_len (in) Number of bytes in RAND data
	@param		autn_data (in) AUTN data
	@param		autn_len (in) Number of bytes in AUTN data

	@return		None
	@note
	@n@b Responses 
	@n@b MsgType_t : MSG_ISIM_AUTHEN_GBA_BOOT_RSP
	@n@b Result_t :		RESULT_OK
	@n@b ResultData : ISIM_AUTHEN_GBA_BOOT_RSP_t

**/	

void CAPI2_ISIM_SendAuthenGbaBootReq(UInt32 tid, UInt8 clientID, const UInt8 *rand_data, UInt16 rand_len, const UInt8 *autn_data, UInt16 autn_len)
{
	CAPI2_ReqRep_t req;
	char* stream;
	UInt32 len;
	Result_t result = RESULT_OK;
	/* Create the request */
	/* Init the request */
	memset(&req, 0, sizeof(CAPI2_ReqRep_t));

	/* message specific */
	req.req_rep_u.CAPI2_ISIM_SendAuthenGbaBootReq_Req.rand.str = (char*) rand_data;
	req.req_rep_u.CAPI2_ISIM_SendAuthenGbaBootReq_Req.rand.len = rand_len;
	req.req_rep_u.CAPI2_ISIM_SendAuthenGbaBootReq_Req.autn.str = (char*) autn_data;
	req.req_rep_u.CAPI2_ISIM_SendAuthenGbaBootReq_Req.autn.len = autn_len;
	req.respId = MSG_ISIM_AUTHEN_GBA_BOOT_RSP;

	/* serialize */
	CAPI2_SerializeReqRsp(&req, tid, MSG_ISIM_SENDAUTHENGBABOOTREQ_REQ, clientID, result, &stream, &len);

	/*     Send it accross to the IPC(stream, len)    */
#ifdef BYPASS_IPC
	CAPI2_DeserializeAndHandleReq(stream, len);
#endif

}

/**
    This function sends the Authenticate command for GBA Security Context in NAF Derivation Mode (see Section
	7.1.2.4 of 3GPP 31.103) to ISIM. 
	This function is applicable only if ISIM_IsIsimSupported() returns TRUE and ISIM application is activated.

	@param		tid (in) Unique exchange/transaction id which is passed back in the response
	@param		clientID (in) Client ID
	@param		naf_id_data (in) NAF ID data
	@param		naf_id_len (in) Number of bytes in NAF ID data

	@return		None
	@note
	@n@b Responses 
	@n@b MsgType_t : MSG_ISIM_AUTHEN_GBA_NAF_RSP
	@n@b Result_t :		RESULT_OK
	@n@b ResultData : ISIM_AUTHEN_GBA_NAF_RSP_t

**/	

void CAPI2_ISIM_SendAuthenGbaNafReq(UInt32 tid, UInt8 clientID, const UInt8 *naf_id_data, UInt16 naf_id_len)
{
	CAPI2_ReqRep_t req;
	char* stream;
	UInt32 len;
	Result_t result = RESULT_OK;
	/* Create the request */
	/* Init the request */
	memset(&req, 0, sizeof(CAPI2_ReqRep_t));

	/* message specific */
	req.req_rep_u.CAPI2_ISIM_SendAuthenGbaNafReq_Req.naf_id.str = (char*) naf_id_data;
	req.req_rep_u.CAPI2_ISIM_SendAuthenGbaNafReq_Req.naf_id.len = naf_id_len;
	req.respId = MSG_ISIM_AUTHEN_GBA_NAF_RSP;

	/* serialize */
	CAPI2_SerializeReqRsp(&req, tid, MSG_ISIM_SENDAUTHENGBANAFREQ_REQ, clientID, result, &stream, &len);

	/*     Send it accross to the IPC(stream, len)    */
#ifdef BYPASS_IPC
	CAPI2_DeserializeAndHandleReq(stream, len);
#endif

}

