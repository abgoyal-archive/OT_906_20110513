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


#ifdef INCLUDE_IPC_PATH
	#ifdef UNDER_LINUX
		
		#define CSL_TYPES_H

		#include <linux/broadcom/IPCInterface.h>
		#include <linux/broadcom/IPCProperties.h>
	#else
		#include <IPCInterface.h>
		#include <IPCProperties.h>
	#endif
#endif

#ifndef __CAPI2_GLOBAL
#define __CAPI2_GLOBAL

//capi2_global.h
//Do not include any include files here

#ifndef _CAPI2_TRACE_DEF
#define _CAPI2_TRACE_DEF



//undefine this in local source file to disable log
#define _DBG_(a) a	//by default logs are enabled
void Capi2_DebugOutputString(char* pStr);

#if defined(UNDER_CE) || defined(WIN32)
#define snprintf _snprintf
#endif

#ifdef WIN32
#include "stdlib.h"
#include "string.h"

#ifndef UNDER_CE
#include	"assert.h"
#endif

//Windows
#define CAPI2_TRACE	printf
#define CAPI2_TRACE_DETAIL printf
#define CAPI2_TRACE_DATA_DETAIL printf
#define DETAIL_LOG_ENABLED FALSE
#define DETAIL_DATA_LOG_ENABLED FALSE
#define capi2_malloc(x)	malloc(x)
#define capi2_free(x)	free(x)
extern unsigned long g_dwLogLEVEL;

#define BASIC_LOG_ENABLED (!g_dwLogLEVEL)

#if defined(UNDER_CE)
 #define xassert(e,v) ((e) ? (void)0 : CAPI2_Assert(#e, __FILE__, __LINE__, (UInt32)v))
 #define assert(e) ((e) ? (void)0 : CAPI2_Assert(#e, __FILE__, __LINE__, 0))
#else
	#ifndef xassert
	#define xassert(a,b)
	#endif
#endif
#elif UNDER_LINUX

#define CAPI2_TRACE	printk
#define CAPI2_TRACE_DETAIL printk
#define CAPI2_TRACE_DATA_DETAIL printk
#define capi2_malloc(x)	kmalloc(x, GFP_KERNEL)
#define capi2_free(x)	kfree(x)

extern void CAPI2_Assert(char *expr, char *file, int line, int value);
#define xassert(e,v) ((e) ? (void)0 : CAPI2_Assert(#e, __FILE__, __LINE__, (UInt32)v))
#define assert(e) ((e) ? (void)0 : CAPI2_Assert(#e, __FILE__, __LINE__, 0))

#else
//Target
#include "stdlib.h"
#include "string.h"
#include "logapi.h"
#include "assert.h"
#include "xassert.h"
#include "osheap.h"

#define IPC_PATH(f) < ## f ## >

extern int Log_DebugPrintf(UInt16 logID, char *fmt, ...);
extern void	Log_DebugOutputString(UInt16 logID, char* dbgString);
extern Boolean Log_IsLoggingEnable(UInt16 logID);

#define CAPI2_TRACE(...) Log_DebugPrintf(LOGID_CAPI2_BASIC, __VA_ARGS__)
#define CAPI2_TRACE_DETAIL(a) Log_DebugOutputString(LOGID_CAPI2_DETAIL, a)
#define CAPI2_TRACE_DATA_DETAIL(...) Log_DebugPrintf(LOGID_DATA_DETAIL, __VA_ARGS__)
#define DETAIL_LOG_ENABLED Log_IsLoggingEnable(LOGID_CAPI2_DETAIL)
#define BASIC_LOG_ENABLED Log_IsLoggingEnable(LOGID_CAPI2_BASIC)
#define DETAIL_DATA_LOG_ENABLED Log_IsLoggingEnable(LOGID_DATA_DETAIL)
#define capi2_malloc(x)	OSHEAP_Alloc(x)
#define capi2_free(x)	OSHEAP_Delete(x)

#endif
#endif


#ifndef RPC_INCLUDED
typedef enum
{
   CAPI2_APPS,
   CAPI2_COMMS
}Capi2ProcessorType_t;
#endif

//void CAPI2_LogBinaryData(MsgType_t msgId, UInt8* stream , UInt32 len);

#endif
