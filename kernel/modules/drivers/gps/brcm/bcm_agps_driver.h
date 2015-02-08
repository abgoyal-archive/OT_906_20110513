/****************************************************************************
*
*     Copyright (c) 2009 Broadcom Corporation
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

//***************************************************************************
/**
*
*   @file   bcm_agps_driver.h
*
*   @brief  This driver is used for handling AGPS messages through capi2 api.
*
*
****************************************************************************/
#ifndef __BCM_AGPS_DRIVER_H__
#define __BCM_AGPS_DRIVER_H__

typedef struct
{
    unsigned long CmdID;    // Kernel AGPS Command ID
    void *data;             // pointer to user buffer
    size_t datalen;         // length of user buffer
} KAGPS_SetCmd_t;

typedef struct 
{
    unsigned long CmdID;	//Kernel AGPS Command ID
    int result;				//Response result, TRUE success, FALSE fail.
    void *data;				//pointer to user buffer, this buffer should be allocated by the requester and the maximum buffer size should be provided in iodatalen
    size_t iodatalen;		//length of user buffer. The requester should provide the maximum buffer size. This filed returns the data length copied in data buffer
} KAGPS_GetCmd_t;

#define BCM_KAGPS_IO_MAGIC 'G'

#define BCM_KAGPS_SET_IOC_NR 0
#define BCM_KAGPS_GET_IOC_NR 1

//#define BCM_KAGPS_SET_CMD     _IOR(BCM_KAGPS_IO_MAGIC, BCM_KAGPS_SET_IOC_NR, KAGPS_SetCmd_t)
//#define BCM_KAGPS_GET_CMD     _IOR(BCM_KAGPS_IO_MAGIC, BCM_KAGPS_GET_IOC_NR, KAGPS_GetCmd_t)

#define BCM_KAGPS_SET_CMD     0x5400
#define BCM_KAGPS_GET_CMD     0x5401

#define BCM_KAGPS_GET_CELLINFO_CMD		0x01

#endif //__BCM_AGPS_DRIVER_H__

