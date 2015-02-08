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

//***************************************************************************
/**
*
*   @file   bcm_agps_driver.c
*
*   @brief  This driver is typically used for handling AT command through capi2 api.
*
*
****************************************************************************/
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/semaphore.h>
#include <linux/sched.h>
#include <linux/wait.h>

#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/ioctl.h>

#include "mobcom_types.h"

#include "capi2_reqrep.h"
#include "rpc_api.h"
#include "bcm_agps_driver.h"


#define RAT_NOT_AVAILABLE		0
#define RAT_GSM					1
#define RAT_UMTS				2

ServingCellInfo_t gCellInfo;

#define KAGPS_MODULE_NAME  "bcm_agps"

#define KAGPS_TRACE_ON
#ifdef KAGPS_TRACE_ON
#define KAGPS_TRACE(str) printk str
#else
#define KAGPS_TRACE(str) {}
#endif

#define HW_GPIO_GPS_CNTIN  (59)

extern UInt8 CAPI2_SYS_RegisterClient(RPC_RequestCallbackFunc_t reqCb,
									RPC_ResponseCallbackFunc_t respCb, 
									RPC_AckCallbackFunc_t		ackCb,
									RPC_FlowControlCallbackFunc_t	flowControlCb);

void LcsPrintCellInfo(const ServingCellInfo_t* inCellInfo);

/**
 *  file ops 
 */
static int KAGPS_Open(struct inode *inode, struct file *filp);
static int KAGPS_Read(struct file *filep, char __user *buf, size_t size, loff_t *off);
static int KAGPS_Release(struct inode *inode, struct file *filp) ;
static int KAGPS_Ioctl(struct inode *inode, struct file *filp, unsigned int cmd, UInt32 arg);

static struct class *cntin_class;

static char cntinStatus = 0; 
static int cntinResponseLength = 0;

static UInt8 gIsOpened = FALSE;

//Platform device data
typedef struct _PlatformDevData_t
{
	int init;	
} PlatformDevData_t;

static PlatformDevData_t sgDeviceInfo =
{
	.init = 0,
}; 

typedef struct
{
	struct file          *file;
    struct mutex         recv_mutex;
}KAGPS_Param_t;

static KAGPS_Param_t gKAgpsParam;

#define BCM_KAGPS_TID_INIT   0X01
#define BCM_KAGPS_TID_MAX    0XFFFFF

UInt32 gKagpsTID = BCM_KAGPS_TID_INIT;
UInt8 gKagpsClientID = 110; //0;
static UInt8 gRpcHdl= 0;

UInt32 KagpsGetNewTID(void)
{
    if(gKagpsTID > BCM_KAGPS_TID_MAX) 
    {
        gKagpsTID = BCM_KAGPS_TID_INIT;
    }
    gKagpsTID++;
    KAGPS_TRACE(("gKagpsTID:%lu\n", gKagpsTID));

    return gKagpsTID;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//  Function Name: DeviceRelease
//
//  Description: 'release' call back function for platform device
//
//------------------------------------------------------------
static void DeviceRelease(struct device *pdev)
{

}


//Platform device structure
static struct platform_device sgPlatformDevice =
{
	.name		= "brcm_cntin_device",
	.dev		= 
	{
		.platform_data	= &sgDeviceInfo,
		.release = DeviceRelease,
	},
	.id		= -1,
};

static struct file_operations sFileOperations = 
{
    .owner      = THIS_MODULE,
    .open       = KAGPS_Open,
    .read       = KAGPS_Read,
    .write      = NULL,
    .ioctl      = KAGPS_Ioctl,
    .poll       = NULL,
    .mmap       = NULL,
    .release    = KAGPS_Release
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//  Function Name: DriverProbe
//
//  Description: 'probe' call back function
//
//------------------------------------------------------------
static int __devinit DriverProbe(struct platform_device *pdev)
{
      KAGPS_TRACE(( "CNTIN DriverProbe \n") ) ;
      return 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//  Function Name: DriverRemove
//
//  Description: 'remove' call back function
//
//------------------------------------------------------------
static int DriverRemove(struct platform_device *pdev)
{
	return 0;
}


static int DriverSuspend(
		struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//  Function Name: DriverResume
//
//  Description: 'resume' call back function
//
//------------------------------------------------------------
static int DriverResume(struct platform_device *pdev)
{
	return 0;
}


//Platfoorm driver structure
static struct platform_driver sgPlatformDriver =
{
	.probe		= DriverProbe,
	.remove 	= DriverRemove,
	.suspend	= DriverSuspend,
	.resume		= DriverResume,
	.driver		= 
		{
		.name	= "brcm_cntin_device",
		.owner	= THIS_MODULE,
		},
};

void KAGPS_RPC_RespCbk(RPC_Msg_t* pMsg, ResultDataBufHandle_t dataBufHandle, UInt32 userContextData)
{
    UInt32 len;
    void* dataBuf;
    CAPI2_ReqRep_t * const reqRep = (CAPI2_ReqRep_t*)pMsg->dataBuf;
    
     if(gIsOpened == FALSE)
    { 
    	return;
    }

    KAGPS_TRACE(( "KAGPS_RPC_RespCbk pMsg->msgId=0x%x. L1_BB=0x%x, CELL_INFO_IND=0x%x.\n", pMsg->msgId, MSG_L1_BB_ISLOCKED_RSP, MSG_SERVING_CELL_INFO_IND) ) ;
    
    CAPI2_GetPayloadInfo(reqRep, pMsg->msgId, &dataBuf, &len);

	//printk("===pMsg->tid=%x, pMsg->clientID=%x, pMsg->msgId=%x, reqRep->result=%x, dataBuf=%x, value=%d, len=%x, dataBufHandle=%x\n",pMsg->tid, pMsg->clientID, pMsg->msgId, reqRep->result, dataBuf,*((UInt32 *) dataBuf),len, dataBufHandle);

    //if (pMsg->msgId==0x42c8 /*MSG_L1_BB_ISLOCKED_RSP*/)
	//if (pMsg->tid==1)
	switch(pMsg->msgId)
    {
	case MSG_L1_BB_ISLOCKED_RSP:
		{

			KAGPS_TRACE(( "KAGPS received MSG_L1_BB_ISLOCKED_RSP. len=%d.\n", len) ) ;
			cntinResponseLength=len;
			cntinStatus=*((Boolean *) dataBuf);
			if (cntinStatus)
			{
				KAGPS_TRACE(( "CNTIN clock is locked\n") ) ;
				if (gpio_request(HW_GPIO_GPS_CNTIN, "GPS CNTIN")==0)
				{
					KAGPS_TRACE(( "CNTIN GPIO request granted\n") ) ;			
					gpio_direction_output(HW_GPIO_GPS_CNTIN, 1);
					gpio_set_value(HW_GPIO_GPS_CNTIN, 1);
				}  
				else
					KAGPS_TRACE(( "CNTIN GPIO request error\n") ) ;
			}
			else
				KAGPS_TRACE(( "CNTIN clock is not locked\n") ) ;
		}
		break;
	case MSG_SERVING_CELL_INFO_IND:
		{
			ServingCellInfo_t* pRevCellInfo = (ServingCellInfo_t*)dataBuf;
			UInt16		mnc = pRevCellInfo->mLbsParams.lbs_mm_params.mnc; 
			UInt16		mcc = pRevCellInfo->mLbsParams.lbs_mm_params.mcc;
			
			
			KAGPS_TRACE(( "KAGPS received MSG_SERVING_CELL_INFO_IND. size=%d len=%d. rat=%d.\n", sizeof(ServingCellInfo_t), len, pRevCellInfo->mRAT) ) ;
			
			mutex_lock(&gKAgpsParam.recv_mutex);
			//if(gKagpsClientID)
			if(gRpcHdl)
			{
				//test
				//LcsPrintCellInfo(pRevCellInfo);
				if(mnc != 0xff) //0xff is Invalid mnc
				{
					//Valid
					gCellInfo.mLbsParams.lbs_mm_params.mnc = mnc;
					gCellInfo.mLbsParams.lbs_mm_params.mcc = mcc;
				}
				else if((gCellInfo.mLbsParams.lbs_mm_params.mnc == 0) && (gCellInfo.mLbsParams.lbs_mm_params.mcc == 0))
				{
					//no valid MCC and MNC is stored, query
					CAPI2_MS_GetPlmnMCC(KagpsGetNewTID(), gKagpsClientID);
					CAPI2_MS_GetPlmnMNC(KagpsGetNewTID(), gKagpsClientID);				
				}
				
				if(pRevCellInfo->mRAT == RAT_NOT_AVAILABLE)
				{
					memcpy(&gCellInfo, dataBuf, sizeof(ServingCellInfo_t));
					CAPI2_MS_GetCurrentRAT(KagpsGetNewTID(), gKagpsClientID);
				}
				else
				{
					gCellInfo.mRAT = pRevCellInfo->mRAT;
				}
				
				if(pRevCellInfo->mRAT == RAT_GSM)
				{
					UInt16 prevLac = gCellInfo.mLbsParams.lbs_mm_params.lac;
					UInt16 prevGsmCellId = gCellInfo.mLbsParams.lbs_gsm_params.scell.cell_id;
					
					UInt16 lac = pRevCellInfo->mLbsParams.lbs_mm_params.lac;
					UInt16 gsmCellId = pRevCellInfo->mLbsParams.lbs_gsm_params.scell.cell_id;	
					
					memcpy(&(gCellInfo.mLbsParams.lbs_gsm_params), &(pRevCellInfo->mLbsParams.lbs_gsm_params), sizeof(T_LBS_GSM_Params));
					if((lac == 0xffff) || (lac == 0))
					{
						//Invalid, query
						gCellInfo.mLbsParams.lbs_mm_params.lac = prevLac;
						CAPI2_MS_GetRegisteredLAC(KagpsGetNewTID(), gKagpsClientID);
					}
					else
					{
						gCellInfo.mLbsParams.lbs_mm_params.lac = lac;
					}
					if((gsmCellId == 0x0000) || (gsmCellId == 0xffff))
					{
						gCellInfo.mLbsParams.lbs_gsm_params.scell.cell_id = prevGsmCellId;
						CAPI2_MS_GetRegisteredCellInfo(KagpsGetNewTID(), gKagpsClientID);
					}
					//else
					//{
					//	gCellInfo.mLbsParams.lbs_gsm_params.scell.cell_id = gsmCellId;
					//}
				}	
				else if(pRevCellInfo->mRAT == RAT_UMTS)
				{	
					T_LBS_UMTS_MeasFreq* pUmtsFreqSrv = &(pRevCellInfo->mLbsParams.lbs_umts_params.active);
					UInt32 prevCellId = gCellInfo.mLbsParams.lbs_umts_params.active.cells[0].cell_identity;
					
					gCellInfo.mLbsParams.lbs_mm_params.lac = 0;
					memcpy(&(gCellInfo.mLbsParams.lbs_umts_params), &(pRevCellInfo->mLbsParams.lbs_umts_params), sizeof(T_LBS_UMTS_Params));
					if((pUmtsFreqSrv->num_cell == 0) || (pUmtsFreqSrv->cells[0].cell_identity == 0xffffffff))
					{
						gCellInfo.mLbsParams.lbs_umts_params.active.cells[0].cell_identity = prevCellId;
						CAPI2_MS_GetRegisteredCellInfo(KagpsGetNewTID(), gKagpsClientID);
						CAPI2_MS_GetElement(KagpsGetNewTID(), gKagpsClientID, MS_NETWORK_ELEM_RNC);
					}
				}
			}
			else
			{
				memcpy(&gCellInfo, dataBuf, sizeof(ServingCellInfo_t));
			}
			//test
			//LcsPrintCellInfo(&gCellInfo);
			mutex_unlock(&gKAgpsParam.recv_mutex);
		}
		break;
	case MSG_MS_GET_PLMN_MCC_RSP:
		{	
			mutex_lock(&gKAgpsParam.recv_mutex);		
			gCellInfo.mLbsParams.lbs_mm_params.mcc = *((UInt16 *) dataBuf);
			mutex_unlock(&gKAgpsParam.recv_mutex);
			KAGPS_TRACE(( "KAGPS received MSG_MS_GET_PLMN_MCC_RSP. mcc=%d len=%d.\n", gCellInfo.mLbsParams.lbs_mm_params.mcc, len) ) ;
		}
		break;
	case MSG_MS_GET_PLMN_MNC_RSP:
		{	
			mutex_lock(&gKAgpsParam.recv_mutex);			
			gCellInfo.mLbsParams.lbs_mm_params.mnc = *((UInt8 *) dataBuf);
			mutex_unlock(&gKAgpsParam.recv_mutex);
			KAGPS_TRACE(( "KAGPS received MSG_MS_GET_PLMN_MNC_RSP. mnc=%d len=%d.\n", gCellInfo.mLbsParams.lbs_mm_params.mnc, len) ) ;
		}
		break;
	case MSG_MS_GET_CURRENT_RAT_RSP:
		{	
			mutex_lock(&gKAgpsParam.recv_mutex);			
			gCellInfo.mRAT = *((UInt8 *) dataBuf);
			mutex_unlock(&gKAgpsParam.recv_mutex);
			KAGPS_TRACE(( "KAGPS received MSG_MS_GET_CURRENT_RAT_RSP. rat=%d len=%d.\n", gCellInfo.mRAT, len) ) ;
		}
		break;
	case MSG_MS_REGISTERED_LAC_RSP:
		{	
			mutex_lock(&gKAgpsParam.recv_mutex);			
			gCellInfo.mLbsParams.lbs_mm_params.lac = *((UInt16 *) dataBuf);
			mutex_unlock(&gKAgpsParam.recv_mutex);
			KAGPS_TRACE(( "KAGPS received MSG_MS_REGISTERED_LAC_RSP. lac=%d len=%d.\n", gCellInfo.mLbsParams.lbs_mm_params.lac, len) ) ;
		}
		break;
	case MSG_MS_GET_CELL_INFO_RSP:
		{			
			UInt16 cellID = *((UInt16 *) dataBuf);
			mutex_lock(&gKAgpsParam.recv_mutex);	
			if(gCellInfo.mRAT == RAT_UMTS)
			{
				gCellInfo.mLbsParams.lbs_umts_params.active.cells[0].cell_identity &= 0xff00;
				gCellInfo.mLbsParams.lbs_umts_params.active.cells[0].cell_identity += cellID;
				KAGPS_TRACE(( "KAGPS received MSG_MS_GET_CELL_INFO_RSP. cellID=0x%x 3G cell_identity=0x%lx.\n", cellID, gCellInfo.mLbsParams.lbs_umts_params.active.cells[0].cell_identity) ) ;
			}
			else if(gCellInfo.mRAT == RAT_GSM)
			{
				KAGPS_TRACE(( "KAGPS received MSG_MS_GET_CELL_INFO_RSP. cellID=0x%x 2G scell.cell_id=0x%lx.\n", cellID, gCellInfo.mLbsParams.lbs_gsm_params.scell.cell_id) ) ;
				gCellInfo.mLbsParams.lbs_gsm_params.scell.cell_id = cellID;
			}
			mutex_unlock(&gKAgpsParam.recv_mutex);
		}
		break;
	case MSG_MS_GET_ELEMENT_RSP:
		{			
			CAPI2_MS_Element_t* pMsElement = (CAPI2_MS_Element_t *) dataBuf;
			if(pMsElement->inElemType == MS_NETWORK_ELEM_RNC)
			{
				UInt16 rncValue = pMsElement->data_u.u16Data;
				mutex_lock(&gKAgpsParam.recv_mutex);	
				gCellInfo.mLbsParams.lbs_umts_params.active.cells[0].cell_identity &= 0x00ff;
				gCellInfo.mLbsParams.lbs_umts_params.active.cells[0].cell_identity += (rncValue << 16);
				mutex_unlock(&gKAgpsParam.recv_mutex);
				
				KAGPS_TRACE(( "KAGPS received MS_NETWORK_ELEM_RNC. rncValue=0x%x cell_identity=0x%lx.\n", rncValue, gCellInfo.mLbsParams.lbs_umts_params.active.cells[0].cell_identity) ) ;
			}
		}
		break;
	default:
	    {
	        //KAGPS_TRACE(( "KAGPS_RPC_RespCbk default\n") ) ;
	    }
	    break;
    }

    RPC_SYSFreeResultDataBuffer(dataBufHandle);
}

/* Ack call back */
void KAGPS_Capi2HandleAckCbk(UInt32 tid, UInt8 clientid, RPC_ACK_Result_t ackResult, UInt32 ackUsrData)
{
    switch (ackResult)
    {
    case ACK_SUCCESS:
    {
        KAGPS_TRACE(( "KAGPS_Capi2HandleAckCbk ACK_SUCCESS\n") ) ;
        //capi2 request ack succeed
        //KAGPS_TRACE(("KRIL_HandleCapi2AckCbk::AckCbk ACK_SUCCESS tid=%lu\n", tid);
        //printk("===HAL_Capi2HandleAckCbk::AckCbk ACK_SUCCESS tid=%lu\n", tid);

    }
    break;

    case ACK_FAILED:
    {
		KAGPS_TRACE(( "KAGPS_Capi2HandleAckCbk ACK_FAILED\n") ) ;
        //KRIL_DEBUG(DBG_ERROR, "KRIL_HandleCapi2AckCbk::AckCbk ACK_FAILED\n");
        //printk("===HAL_Capi2HandleAckCbk::AckCbk ACK_FAILED\n");
        //capi2 ack fail for unknown reasons
    }
    break;

    case ACK_TRANSMIT_FAIL:
    {
        KAGPS_TRACE(( "KAGPS_Capi2HandleAckCbk ACK_TRANSMIT_FAIL\n") ) ;
        // KRIL_DEBUG(DBG_ERROR, "KRIL_HandleCapi2AckCbk::AckCbk ACK_TRANSMIT_FAIL\n");
        //printk("===HAL_Capi2HandleAckCbk::AckCbk ACK_TRANSMIT_FAIL\n");
        //capi2 ack fail due to fifo full, fifo mem full etc.
    }
    break;

    case ACK_CRITICAL_ERROR:
    {
        KAGPS_TRACE(( "KAGPS_Capi2HandleAckCbk ACK_CRITICAL_ERROR\n") ) ;
        //KRIL_DEBUG(DBG_ERROR, "KRIL_HandleCapi2AckCbk::AckCbk ACK_CRITICAL_ERROR\n");
        //printk("===HAL_Capi2HandleAckCbk::AckCbk ACK_CRITICAL_ERROR\n");
        //capi2 ack fail due to comms processor reset ( The use case for this error is TBD )
    }
    break;

    default:
    {
        KAGPS_TRACE(( "KAGPS_Capi2HandleAckCbk default\n") ) ;
        //KRIL_DEBUG(DBG_ERROR, "KRIL_HandleCapi2AckCbk::AckCbk ackResult error!\n");
        // printk("===HAL_Capi2HandleAckCbk::AckCbk ackResult error!\n");
    }
    break;
    }
}

void KAGPS_Capi2HandleFlowCtrl(RPC_FlowCtrlEvent_t event, UInt8 channel)
{
    /*NOT HANDLED*/
}


//======================================File operations==================================================
//***************************************************************************
/**
 *  Called by Linux I/O system to handle open() call.   
 *  @param  (in)    not used
 *  @param  (io)    file pointer    
 *  @return int     0 if success, -1 if error
 *  @note
 *      API is defined by struct file_operations 'open' member.
 */

static int KAGPS_Open(struct inode *inode, struct file *filp)
{

    UInt32 ret = 0;
    //const UInt8 ClientID = 110;
    
    
    gIsOpened = TRUE;

	cntinResponseLength=0;
	    
    //if (!gKagpsClientID)
    if(!gRpcHdl)
	{
		KAGPS_TRACE(( "KAGPS_Open CAPI2_SYS_RegisterClient.\n")) ;
	    //gKagpsClientID = CAPI2_SYS_RegisterClient(NULL, KAGPS_RPC_RespCbk, KAGPS_Capi2HandleAckCbk, KAGPS_Capi2HandleFlowCtrl);
	    //RPC_SYS_BindClientID(handle, ClientID);
	    gRpcHdl = CAPI2_SYS_RegisterClient(NULL, KAGPS_RPC_RespCbk, KAGPS_Capi2HandleAckCbk, KAGPS_Capi2HandleFlowCtrl);
	    RPC_SYS_BindClientID(gRpcHdl, gKagpsClientID);
	}

	KAGPS_TRACE(( "KAGPS_Open gKagpsClientID=%d. gKagpsTID=%d. gRpcHdl=%d.\n", gKagpsClientID, gKagpsTID, gRpcHdl)) ;

	//if(gKagpsClientID)
	if(gRpcHdl)
	{
		gKAgpsParam.file = filp;
	    filp->private_data = &gKAgpsParam;

	    //SetHALClientID(ClientID);
		CAPI2_L1_bb_isLocked(KagpsGetNewTID(), gKagpsClientID, false);
		CAPI2_SYS_EnableCellInfoMsg(KagpsGetNewTID(), gKagpsClientID, TRUE);
	}
    return(ret);
}

static int KAGPS_Read(struct file *filep, char __user *buf, size_t size, loff_t *off)
{
	KAGPS_TRACE(( "KAGPS_Read\n") ) ;

	if (cntinResponseLength)
	{
		if (put_user(cntinStatus, buf)==0)
		{
			KAGPS_TRACE(( "put_user success cntinStatus=%d size=%d \n",cntinStatus,cntinResponseLength) ) ;
		}
		else
			return 0;

		KAGPS_TRACE(( "KAGPS_Read cntinStatus=%d size=%d \n",cntinStatus,cntinResponseLength) ) ;
		return 1;
	}
	else
		return 0;
}

static int KAGPS_Release(struct inode *inode, struct file *filp)
{
	KAGPS_TRACE(( "KAGPS_Release\n") ) ;
	gpio_set_value(HW_GPIO_GPS_CNTIN, 0);
	gpio_free(HW_GPIO_GPS_CNTIN);
	CAPI2_SYS_EnableCellInfoMsg(KagpsGetNewTID(), gKagpsClientID, FALSE);
	
	gIsOpened = FALSE;
	
	return 0;
}


//***************************************************************************
/**
 *  Called by Linux I/O system to initialize module.   
 *  @return int     0 if success, -1 if error
 *  @note
 *      API is defined by module_init macro
 */
static int __init KAGPS_ModuleInit(void)
{
    int err = 1;
	struct device *myDevice;
	dev_t myDev;

    KAGPS_TRACE(("enter KAGPS_ModuleInit()\n"));
    
	/*err =  platform_device_register(&sgPlatformDevice);
	if (err)
		return err;

	err = platform_driver_register(&sgPlatformDriver);*/
			
    //drive driver process:
    if (register_chrdev(BCM_CNTIN_MAJOR, KAGPS_MODULE_NAME, &sFileOperations) < 0 )
    {
        KAGPS_TRACE(("register_chrdev failed\n" ) );
        return -1 ;
    }

    cntin_class = class_create(THIS_MODULE, KAGPS_MODULE_NAME);
    if (IS_ERR(cntin_class))
    {
        return PTR_ERR(cntin_class);
    }
    myDev=MKDEV(BCM_CNTIN_MAJOR, 0);

	KAGPS_TRACE(("mydev = %d\n",myDev ) );

    myDevice=device_create(cntin_class, NULL, myDev,NULL, KAGPS_MODULE_NAME);

    err = PTR_ERR(myDevice);
    if (IS_ERR(myDevice))
    {
        KAGPS_TRACE(("device create failed\n" ) );
        return -1 ;
    }

	memset(&gCellInfo, 0, sizeof(ServingCellInfo_t));
	mutex_init(&(gKAgpsParam.recv_mutex));

    KAGPS_TRACE(("exit sucessfuly KAGPS_ModuleInit()\n"));
    return 0;
}

//***************************************************************************
/**
 *  Called by Linux I/O system to exit module.   
 *  @return int     0 if success, -1 if error
 *  @note
 *      API is defined by module_exit macro
 **/
static void __exit KAGPS_ModuleExit(void)
{
    KAGPS_TRACE(("KAGPS_ModuleExit()\n"));
    unregister_chrdev( BCM_CNTIN_MAJOR, KAGPS_MODULE_NAME ) ;
}



/**
 *  export module init and export functions
 **/
module_init(KAGPS_ModuleInit);
module_exit(KAGPS_ModuleExit);
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cntin clock Driver");

//*************************************************************/
Int32 KAGPS_SendCmd(unsigned int cmd, unsigned long arg)
{
    //KRIL_CmdQueue_t *kril_cmd = NULL;
    void *tdata = NULL;
	KAGPS_SetCmd_t kagpsSetCmd;

	KAGPS_TRACE(("KAGPS_SendCmd cmd=0x%x.\n", cmd));

    //validate CMD
    if (_IOC_TYPE(cmd) != BCM_KAGPS_IO_MAGIC) 
    {
        KAGPS_TRACE(("BCM_KAGPS_IO_MAGIC:0x%x _IOC_TYPE(cmd):0x%x  error!\n", BCM_KAGPS_IO_MAGIC, _IOC_TYPE(cmd)));
        return -ENOTTY;
    }

	copy_from_user(&kagpsSetCmd, (KAGPS_SetCmd_t *)arg, sizeof(KAGPS_SetCmd_t));

    KAGPS_TRACE(("KAGPS_SendCmd CmdID:0x%lx datalen:%d\n", kagpsSetCmd.CmdID, kagpsSetCmd.datalen));

    if (0 != kagpsSetCmd.datalen)
    {
		void *tdata = NULL;
        tdata = kmalloc(kagpsSetCmd.datalen, GFP_KERNEL);
        if(NULL == tdata)
        {
            KAGPS_TRACE(("KAGPS_SendCmd tdata memory allocate fail!\n"));
        }
        else
        {
            //memcpy(tdata, kagpsSetCmd.data, kagpsSetCmd.datalen);
			copy_from_user(tdata, kagpsSetCmd.data, kagpsSetCmd.datalen);
            KAGPS_TRACE(("KAGPS_SendCmd tdata memory allocate success tdata:%p\n", kagpsSetCmd.data));
            kagpsSetCmd.data = tdata;
        }
    }
    else
    {
        KAGPS_TRACE(("KAGPS_SendCmd datalen is 0\n"));
        kagpsSetCmd.data = NULL;
    }
/*
	switch(kagpsSetCmd.CmdID)
	{
		//Call CAPI2
	case default:
		break;
	}
*/

    if(tdata)
	{
		kfree(tdata);
	}
    KAGPS_TRACE(("KAGPS_SendCmd  cmd:%ld\n", kagpsSetCmd.CmdID));

    return 0;
}

//void LcsInitCellInfo(void);

Int32 KAGPS_GetCmd(struct file *filp, unsigned int cmd, unsigned long arg)
{
    Int32 rv = 0;
    
    KAGPS_GetCmd_t getCmd;
    
    KAGPS_GetCmd_t* pGetCmd = (KAGPS_GetCmd_t*)arg;

	KAGPS_TRACE(("KAGPS_GetCmd cmd=0x%x. arg=0x%lx. sizeGetCmd=%d\n", cmd, arg, sizeof(KAGPS_GetCmd_t)));

    if (copy_from_user(&getCmd, (KAGPS_GetCmd_t*)arg, sizeof(KAGPS_GetCmd_t)))
    {
		KAGPS_TRACE(("KAGPS_GetCmd copy_from_user failed cmd=0x%x.\n", cmd));
        rv = -EFAULT;
        return rv;
    }
	KAGPS_TRACE(("KAGPS_GetCmd getCmd.CmdID=0x%lx.OrgCmdID=0x%x. iodatalen=%d, size=%d. orgLen=%d.data=0x%x.\n", getCmd.CmdID, pGetCmd->CmdID, getCmd.iodatalen, sizeof(ServingCellInfo_t), pGetCmd->iodatalen, getCmd.data));

	getCmd.CmdID = BCM_KAGPS_GET_CELLINFO_CMD;
	switch(getCmd.CmdID)
	{
	case BCM_KAGPS_GET_CELLINFO_CMD:
		//Test
		//LcsInitCellInfo();
		mutex_lock(&gKAgpsParam.recv_mutex);
		if ((gCellInfo.mRAT != RAT_NOT_AVAILABLE))// && (getCmd.iodatalen >= sizeof(ServingCellInfo_t)))
		{
			if (copy_to_user(getCmd.data, &gCellInfo, sizeof(ServingCellInfo_t)))
			{
				rv = -EFAULT;
				KAGPS_TRACE(("ERROR: KAGPS_GetCmd copy response dara to user Fail\n"));
				getCmd.result = 0;
				getCmd.iodatalen = 0;
			}
			else
			{
				getCmd.result = 1;
				getCmd.iodatalen = sizeof(ServingCellInfo_t);
			}
		}
		else
		{
			getCmd.result = 0;
			getCmd.iodatalen = 0;
		}

		if (copy_to_user((KAGPS_GetCmd_t*)arg, &getCmd, sizeof(KAGPS_GetCmd_t)))
		{
			rv = -EFAULT;
			KAGPS_TRACE(("ERROR: KAGPS_GetCmd copy response infor to user Fail\n"));
		}
		//Test
		LcsPrintCellInfo(&gCellInfo);
		
		KAGPS_TRACE(("KAGPS_GetCmd getCmd.result=%d. getCmd.iodatalen=%d, \n", getCmd.result, getCmd.iodatalen));
		mutex_unlock(&gKAgpsParam.recv_mutex);
		break;
	}
    return rv;
}


static int KAGPS_Ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
    int rc = -EINVAL;

	KAGPS_TRACE(("KAGPS_Ioctl cmd=0x%x. arg=0x%x.\n", cmd, arg));
    switch(cmd)
    {
        case BCM_KAGPS_SET_CMD:
            rc = KAGPS_SendCmd(cmd, arg);
            break;

        case BCM_KAGPS_GET_CMD:
            rc = KAGPS_GetCmd(filp, cmd, arg);
            break;

        default:
            KAGPS_TRACE(("we don't process the KAGPS_ioctl cmd:0x%x!\n", cmd));
            break;
    }

	KAGPS_TRACE(("KAGPS_Ioctl cmd=0x%x.rc=%d.\n", cmd, rc));
    return(rc);
}

void LcsPrintCellInfo(const ServingCellInfo_t* inCellInfo)
{
	KAGPS_TRACE(("KAGPS: LcsPrintCellInfo inCellInfo->mRAT = %d.\n", inCellInfo->mRAT));

	KAGPS_TRACE(("KAGPS: LBS MCC=%d, MNC=%d, LAC=%d.\n", 
		inCellInfo->mLbsParams.lbs_mm_params.mcc,
		inCellInfo->mLbsParams.lbs_mm_params.mnc,
		inCellInfo->mLbsParams.lbs_mm_params.lac));

	if(inCellInfo->mRAT == 2) //UMTS
	{
	KAGPS_TRACE(("KAGPS: LBS UMTS CID=0x%lx, uarfcn_DL=%d, uarfcn_UL=%d, rssi=%d, num_cell=%d.num_umts_freq=%d.\n", 
		inCellInfo->mLbsParams.lbs_umts_params.active.cells[0].cell_identity,
		inCellInfo->mLbsParams.lbs_umts_params.active.uarfcn_DL,
		inCellInfo->mLbsParams.lbs_umts_params.active.uarfcn_UL,
		inCellInfo->mLbsParams.lbs_umts_params.active.rssi,
		inCellInfo->mLbsParams.lbs_umts_params.active.num_cell,
		inCellInfo->mLbsParams.lbs_umts_params.num_umts_freq
		));
	KAGPS_TRACE(("KAGPS: LBS UMTS num_gsm_ncells=%d, cell_id=%d.arfcn=%d,rxlev=%d,bsic=%d,\n", 
		inCellInfo->mLbsParams.lbs_umts_params.num_gsm_ncell,
		inCellInfo->mLbsParams.lbs_umts_params.gsm_ncells[0].cell_id,
		inCellInfo->mLbsParams.lbs_umts_params.gsm_ncells[0].arfcn,
		inCellInfo->mLbsParams.lbs_umts_params.gsm_ncells[0].rxlev,
		inCellInfo->mLbsParams.lbs_umts_params.gsm_ncells[0].bsic
		//inCellInfo->mLbsParams.lbs_umts_params.gsm_ncells[0].lac
		));
	}

	if(inCellInfo->mRAT == 1) //GSM
	{
	KAGPS_TRACE(("KAGPS: LBS GSM scell.cell_id=%d.scell.arfcn=%d,scell.rxlev=%d,scell.bsic=%d,scell.\n", 
		inCellInfo->mLbsParams.lbs_gsm_params.scell.cell_id,
		inCellInfo->mLbsParams.lbs_gsm_params.scell.arfcn,
		inCellInfo->mLbsParams.lbs_gsm_params.scell.rxlev,
		inCellInfo->mLbsParams.lbs_gsm_params.scell.bsic
		//inCellInfo->mLbsParams.lbs_gsm_params.scell.lac
		));

	KAGPS_TRACE(("KAGPS: LBS GSM num_gsm_ncells=%d, cell[0] cell_id=%d.arfcn=%d,rxlev=%d,bsic=%d.\n", 
		inCellInfo->mLbsParams.lbs_gsm_params.num_gsm_ncells,
		inCellInfo->mLbsParams.lbs_gsm_params.gsm_ncells[0].cell_id,
		inCellInfo->mLbsParams.lbs_gsm_params.gsm_ncells[0].arfcn,
		inCellInfo->mLbsParams.lbs_gsm_params.gsm_ncells[0].rxlev,
		inCellInfo->mLbsParams.lbs_gsm_params.gsm_ncells[0].bsic
		//inCellInfo->mLbsParams.lbs_gsm_params.gsm_ncells[0].lac
		));
	}
}

void LcsInitCellInfo(void)
{
	memset(&gCellInfo, 0, sizeof(ServingCellInfo_t));

	gCellInfo.mRAT = 1;
	KAGPS_TRACE(("LCS: LcsInitCellInfo inCellInfo->mRAT = %d.", gCellInfo.mRAT));

	gCellInfo.mLbsParams.lbs_mm_params.mcc = 422;
	gCellInfo.mLbsParams.lbs_mm_params.mnc = 533;
	gCellInfo.mLbsParams.lbs_mm_params.lac	= 215;

	if(gCellInfo.mRAT == 2) //UMTS
	{
		gCellInfo.mLbsParams.lbs_umts_params.active.cells[0].cell_identity = 2134567;
		gCellInfo.mLbsParams.lbs_umts_params.active.uarfcn_DL = 666;
		gCellInfo.mLbsParams.lbs_umts_params.active.uarfcn_UL = 777;
		gCellInfo.mLbsParams.lbs_umts_params.active.rssi = 88;
		gCellInfo.mLbsParams.lbs_umts_params.active.num_cell = 99;
		gCellInfo.mLbsParams.lbs_umts_params.num_umts_freq = 9;

		gCellInfo.mLbsParams.lbs_umts_params.num_gsm_ncell = 1;
		gCellInfo.mLbsParams.lbs_umts_params.gsm_ncells[0].cell_id = 223;
		gCellInfo.mLbsParams.lbs_umts_params.gsm_ncells[0].arfcn = 222;
		gCellInfo.mLbsParams.lbs_umts_params.gsm_ncells[0].rxlev = 33;
		gCellInfo.mLbsParams.lbs_umts_params.gsm_ncells[0].bsic = 44;
		//gCellInfo.mLbsParams.lbs_umts_params.gsm_ncells[0].lac = 555;
	}

	if(gCellInfo.mRAT == 1) //GSM
	{
		gCellInfo.mLbsParams.lbs_gsm_params.scell.cell_id = 215;
		gCellInfo.mLbsParams.lbs_gsm_params.scell.arfcn = 888;
		gCellInfo.mLbsParams.lbs_gsm_params.scell.rxlev = 11;
		gCellInfo.mLbsParams.lbs_gsm_params.scell.bsic = 22;
		//gCellInfo.mLbsParams.lbs_gsm_params.scell.lac = 555;

		gCellInfo.mLbsParams.lbs_gsm_params.num_gsm_ncells = 1;
		gCellInfo.mLbsParams.lbs_gsm_params.gsm_ncells[0].cell_id = 136;
		gCellInfo.mLbsParams.lbs_gsm_params.gsm_ncells[0].arfcn = 444;
		gCellInfo.mLbsParams.lbs_gsm_params.gsm_ncells[0].rxlev = 33;
		gCellInfo.mLbsParams.lbs_gsm_params.gsm_ncells[0].bsic = 222;
		//gCellInfo.mLbsParams.lbs_gsm_params.gsm_ncells[0].lac  = 111;
	}
	
	LcsPrintCellInfo(&gCellInfo);
}

