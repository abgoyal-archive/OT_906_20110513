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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#define CSL_TYPES_H
#include <linux/broadcom/bcm_fuse_net_if.h>
#ifdef CONFIG_BRCM_FUSE_RIL_CIB
#include "capi_common_rpc.h"
#include "capi_gen_rpc.h"
#include "capi_rpc.h"
#endif

#include "rpc_ipc.h"

#include "capi2_ds_api.h"
#include "bcm_net_debug.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif


static int __init bcm_fuse_net_init_module(void);
static void __exit bcm_fuse_net_exit_module(void);


/**
   Packet Data EndPoint buffer pool info
 */
#define BCM_NET_MAX_DATA_LEN       1600 //bytes
#define BCM_NET_MAX_NUM_PKTS       64 //packets
#define BCM_NET_START_THRESHOLD     1 
#define BCM_NET_STOP_THRESHOLD      0


typedef enum
{
    EFree = 0,
    EInUse = 1
}net_dev_entry_stat_t;

typedef struct 
{
    net_dev_entry_stat_t entry_stat;
    struct net_device *dev_ptr;
    uint8_t  pdp_context_id;
    unsigned long  ip_addr;
    struct net_device_stats stats;
}net_drvr_info_t; 


spinlock_t g_dev_lock;
static net_drvr_info_t g_net_dev_tbl[BCM_NET_MAX_PDP_CNTXS];
static unsigned char g_NetClientId = 0;

static void bcm_fuse_net_fc_cb(RPC_FlowCtrlEvent_t event, unsigned char cid);
static RPC_Result_t bcm_fuse_net_bd_cb(PACKET_InterfaceType_t interfaceType, unsigned char cid, PACKET_BufHandle_t dataBufHandle);
static void bcm_fuse_net_init(struct net_device *dev_ptr);
static net_drvr_info_t *bcm_fuse_net_device_pdp_lookup(uint8_t pdp_context_id);
static net_drvr_info_t *bcm_fuse_net_device_cid_lookup(unsigned char cid);
static uint8_t bcm_fuse_net_pdp_id(net_drvr_info_t *drvr_info_ptr);
static uint8_t bcm_fuse_net_find_entry(net_drvr_info_t *ndrvr_info_ptr);
static void bcm_fuse_net_free_entry(uint8_t pdp_cid);


/**
   @fn void bcm_fuse_net_fc_cb(RPC_FlowCtrlEvent_t event, unsigned char8 cid);
 */
static void bcm_fuse_net_fc_cb(RPC_FlowCtrlEvent_t event, unsigned char cid)
{
    struct net_device *dev_ptr = NULL;
    net_drvr_info_t *ndrvr_info_ptr = NULL;

    BNET_DEBUG(DBG_INFO,"%s: flow control cid:%d \n", __FUNCTION__, cid);

    //ndrvr_info_ptr = &g_net_dev_tbl[0];//bcm_fuse_net_device_pool_lookup(pool);
    ndrvr_info_ptr = bcm_fuse_net_device_cid_lookup(cid);
    if (ndrvr_info_ptr == NULL)
    {
        if (printk_ratelimit())
            BNET_DEBUG(DBG_ERROR,"%s: bcm_fuse_net_device_cid_lookup() failed\n", __FUNCTION__);
        return;
    }

    dev_ptr = ndrvr_info_ptr->dev_ptr;

    switch(event)
    {
        case RPC_FLOW_START :
        {
            BNET_DEBUG(DBG_INFO,"%s: RECVD RPC_FLOW_START !!\n", __FUNCTION__);
            if (netif_queue_stopped(dev_ptr))
                netif_wake_queue(dev_ptr);
        }
        break;

        case RPC_FLOW_STOP :
        {
            BNET_DEBUG(DBG_INFO,"%s: RECVD RPC_FLOW_STOP !!\n", __FUNCTION__);
            netif_stop_queue(dev_ptr);
        }
        break;

        default:
        {
            BNET_DEBUG(DBG_ERROR,"%s: RECVD Unknown Flow Control Message on cid %d!!\n", __FUNCTION__, cid);
        }
        break;
    }
    return;
}


/**
   @fn RPC_Result_t bcm_fuse_net_bd_cb(PACKET_InterfaceType_t interfaceType, unsigned char cid, PACKET_BufHandle_t dataBufHandle);
*/
static RPC_Result_t bcm_fuse_net_bd_cb(PACKET_InterfaceType_t interfaceType, unsigned char cid, PACKET_BufHandle_t dataBufHandle)
{
    unsigned long data_len = 0;
    struct sk_buff *skb = NULL;
    unsigned char *data_ptr = NULL;
    net_drvr_info_t *ndrvr_info_ptr = NULL;

    //BNET_DEBUG(DBG_INFO,"%s: receive packet \n", __FUNCTION__);

    data_len = RPC_PACKET_GetBufferLength(dataBufHandle);

    //BNET_DEBUG(DBG_INFO,"%s: RECVD Buffer Delivery on AP Packet channel, cid[%d] size[%d]!!\n", __FUNCTION__, cid, data_len);

    //ndrvr_info_ptr = &g_net_dev_tbl[0];
    ndrvr_info_ptr = bcm_fuse_net_device_pdp_lookup(cid);
    if (ndrvr_info_ptr == NULL)
    {
        if (printk_ratelimit())
            BNET_DEBUG(DBG_ERROR,"%s: bcm_fuse_net_device_pdp_lookup failed\n", __FUNCTION__);

        return RPC_RESULT_ERROR;
    }

    skb = dev_alloc_skb(data_len);
    if (skb == NULL)
    {
        if (printk_ratelimit())
            BNET_DEBUG(DBG_ERROR,"%s: dev_alloc_skb() failed - packet dropped\n", __FUNCTION__);

        ndrvr_info_ptr->stats.rx_dropped++;
        return RPC_RESULT_ERROR;
    }

    data_ptr = (unsigned char*)RPC_PACKET_GetBufferData(dataBufHandle);
    memcpy(skb_put(skb, data_len), data_ptr, data_len);

    skb->dev = ndrvr_info_ptr->dev_ptr;
    skb->protocol=htons(ETH_P_IP);
    skb->ip_summed = CHECKSUM_UNNECESSARY; /* don't check it */
    skb->pkt_type = PACKET_HOST;
    ndrvr_info_ptr->dev_ptr->last_rx = jiffies;

    ndrvr_info_ptr->stats.rx_packets++;
    ndrvr_info_ptr->stats.rx_bytes += data_len;

    netif_rx(skb);

    return RPC_RESULT_OK;
}


static int bcm_fuse_net_open(struct net_device *dev)
{
    int i;
    net_drvr_info_t ndrvr_info_ptr;
    unsigned long flags;
    uint8_t idx = BCM_NET_MAX_PDP_CNTXS;//0;
    int ret = 0;
    static int IsFirstCall = 0;

    ndrvr_info_ptr.dev_ptr = dev;

    /**
       Register callbacks with the RPC Proxy server
    */
    if (0 == IsFirstCall)
    {
        ret = RPC_PACKET_RegisterDataInd (g_NetClientId, INTERFACE_PACKET, bcm_fuse_net_bd_cb, bcm_fuse_net_fc_cb);
        if (RPC_RESULT_OK != ret)
        {
            return(-1);
        }
        IsFirstCall++;
    }

    for (i = 0; i < BCM_NET_MAX_PDP_CNTXS; i++)
    {
        BNET_DEBUG(DBG_INFO,"%s: g_net_dev_tbl[%d]=0x%x,dev_ptr 0x%x, dev 0x%x, 0x%x \n", __FUNCTION__,
            i, (unsigned int)(&g_net_dev_tbl[i]), (unsigned int)(g_net_dev_tbl[i].dev_ptr),
            (unsigned int)dev, (unsigned int)(&ndrvr_info_ptr));
    }

    idx = bcm_fuse_net_find_entry(&ndrvr_info_ptr);
    if (idx == BCM_NET_MAX_PDP_CNTXS)
    {
        BNET_DEBUG(DBG_ERROR,"%s: No free device interface to assign for pdp_cid[%d]\n", __FUNCTION__, idx);
        return(-EISCONN);
    }

    spin_lock_irqsave(&g_dev_lock, flags);
    g_net_dev_tbl[idx].pdp_context_id = idx+1;

    spin_unlock_irqrestore(&g_dev_lock, flags);
    BNET_DEBUG(DBG_INFO,"%s: BCM_FUSE_NET_ACTIVATE_PDP: pdp_info.cid [%d]\n", __FUNCTION__, g_net_dev_tbl[idx].pdp_context_id);

    netif_start_queue(dev);
    return 0;
}

static int bcm_fuse_net_stop(struct net_device *dev)
{
    int i;
    BNET_DEBUG(DBG_INFO,"%s: <<\n", __FUNCTION__);
    for (i = 0; i < BCM_NET_MAX_PDP_CNTXS; i++)
    {
        if (g_net_dev_tbl[i].dev_ptr == dev)
        {
            bcm_fuse_net_free_entry(g_net_dev_tbl[i].pdp_context_id);
            BNET_DEBUG(DBG_INFO,"%s: free g_net_dev_tbl[%d].cid:%d\n", __FUNCTION__, i, g_net_dev_tbl[i].pdp_context_id);
            break;
        }
    }
    netif_stop_queue(dev);
    return 0;
}




static int bcm_fuse_net_tx(struct sk_buff *skb, struct net_device *dev)
{
    void *buff_data_ptr;
    uint8_t pdp_cid = BCM_NET_MAX_PDP_CNTXS;
    PACKET_BufHandle_t buffer;
    static net_drvr_info_t *ndrvr_info_ptr = NULL; //consider for multiple case??
    int i;

    //ndrvr_info_ptr = &g_net_dev_tbl[0];
    if (NULL == ndrvr_info_ptr) //only run the first tx
    {
        for (i = 0; i < BCM_NET_MAX_PDP_CNTXS; i++)
        {
            if (g_net_dev_tbl[i].dev_ptr == dev)
            {
                ndrvr_info_ptr = &g_net_dev_tbl[i];
                BNET_DEBUG(DBG_INFO,"%s: g_net_dev_tbl[%d]=0x%x \n", __FUNCTION__, i, (unsigned int)(&g_net_dev_tbl[i]));
                break;
            }
        }
    }

    if (NULL == ndrvr_info_ptr)
    {
        return(-EINVAL);
    }

    if (BCM_NET_MAX_DATA_LEN < skb->len) 
    {
        BNET_DEBUG(DBG_ERROR,"%s: len[%d] exceeds supported len[%d] failed\n", __FUNCTION__, skb->len, BCM_NET_MAX_DATA_LEN);
        ndrvr_info_ptr->stats.tx_errors++;
        return(-1);
    }

    if(0 == skb->len)
    {
        BNET_DEBUG(DBG_ERROR,"%s: len[%d] is zero size failed\n", __FUNCTION__, skb->len);
        return(-1);
    }

    //pdp_cid = 1;
    pdp_cid = bcm_fuse_net_pdp_id(ndrvr_info_ptr);
    if (BCM_NET_INVALID_PDP_CNTX == pdp_cid)
    {
        BNET_DEBUG(DBG_ERROR,"%s: net device to pdp context id mapping failed\n", __FUNCTION__);
        ndrvr_info_ptr->stats.tx_errors++;
        return(-1);
    }

    //Allocate a buffer
    buffer = RPC_PACKET_AllocateBuffer(INTERFACE_PACKET, skb->len, pdp_cid);
    if(!buffer) 
    {
        BNET_DEBUG(DBG_ERROR,"%s: Error buffer Handle cid %d\n", __FUNCTION__, pdp_cid);
        ndrvr_info_ptr->stats.tx_errors++;
        return(-ENOBUFS);
    }

    //transfer data from skb to ipc_buffer
    buff_data_ptr = RPC_PACKET_GetBufferData(buffer);
    if (buff_data_ptr == NULL)
    {
        BNET_DEBUG(DBG_ERROR,"%s: RPC_PACKET_GetBufferData() failed\n", __FUNCTION__);
        ndrvr_info_ptr->stats.tx_errors++;
        return(-ENOBUFS);
    }

    memset(buff_data_ptr, 0, BCM_NET_MAX_DATA_LEN);
    memcpy(buff_data_ptr, skb->data, skb->len);

    RPC_PACKET_SetBufferLength(buffer, skb->len);

    dev->trans_start = jiffies; /* save the timestamp */

    RPC_PACKET_SendData(g_NetClientId, INTERFACE_PACKET, pdp_cid, buffer);

    /**
      The IPC buffer is freed by the receiving end point.
    */
    ndrvr_info_ptr->stats.tx_packets++;
    ndrvr_info_ptr->stats.tx_bytes += skb->len;

    dev_kfree_skb(skb);

    return(0);
}


static struct net_device_stats *bcm_fuse_net_stats(struct net_device *dev)
{
    net_drvr_info_t *ndrvr_info_ptr = NULL;
    int i;

    for (i = 0; i < BCM_NET_MAX_PDP_CNTXS; i++)
    {
       if (g_net_dev_tbl[i].dev_ptr == dev)
       {
           ndrvr_info_ptr = &g_net_dev_tbl[i];
           //BNET_DEBUG(DBG_INFO,"%s: g_net_dev_tbl[%d]=0x%x \n", __FUNCTION__, i, &g_net_dev_tbl[i]);
           break;
       }
    }

    return(&ndrvr_info_ptr->stats);
}


/**
   @fn int bcm_fuse_net_config(struct net_device *dev_ptr, struct ifmap *map);
*/
int bcm_fuse_net_config(struct net_device *dev_ptr, struct ifmap *map)
{
    BNET_DEBUG(DBG_INFO,"%s: <<\n", __FUNCTION__);
    if (dev_ptr->flags & IFF_UP) /* can't act on a running interface */
        return -EBUSY;

    return 0;
}


/**
   Initialize a net device. (Called from kernel in alloc_netdev())

   @fn static void bcm_fuse_net_init(struct net_device *dev_ptr);
*/

static const struct net_device_ops bcm_netdev_ops = {
        .ndo_open               = bcm_fuse_net_open,
        .ndo_stop               = bcm_fuse_net_stop,
        .ndo_start_xmit         = bcm_fuse_net_tx,
        .ndo_get_stats          = bcm_fuse_net_stats,
        .ndo_set_config         = bcm_fuse_net_config,
};

static void bcm_fuse_net_init(struct net_device *dev)
{
    dev->netdev_ops = &bcm_netdev_ops;

    dev->mtu             = BCM_NET_MAX_DATA_LEN;
    dev->tx_queue_len    = (BCM_NET_MAX_NUM_PKTS - BCM_NET_START_THRESHOLD);
    dev->flags       = IFF_NOARP | IFF_DYNAMIC;
}


/**
   @fn static int bcm_fuse_net_attach(unsigned int dev_index);
*/
static int bcm_fuse_net_attach(unsigned int dev_index)
{
    int ret = 0;
    unsigned long flags;
    struct net_device *dev_ptr = NULL;

    if (dev_index >= BCM_NET_MAX_PDP_CNTXS)
    {
        BNET_DEBUG(DBG_ERROR,"%s: Invalid dev_index[%d] passed as parameter \n", __FUNCTION__, dev_index);
        return(-1);
    }

    /* Allocate etherdev, including space for private structure */
    if ((dev_ptr = alloc_netdev(sizeof(net_drvr_info_t *), BCM_NET_DEV_STR, bcm_fuse_net_init)) == NULL) 
    {
        BNET_DEBUG(DBG_ERROR,"%s: Out of memory\n", __FUNCTION__);
        return(-1);
    }

    //Allocate Buffer pool for this channel

    spin_lock_irqsave(&g_dev_lock, flags);

    //dev_ptr->priv = &g_net_dev_tbl[dev_index]; //from kernel 2.6.29
    g_net_dev_tbl[dev_index].dev_ptr = dev_ptr;
    g_net_dev_tbl[dev_index].entry_stat = EFree;
    g_net_dev_tbl[dev_index].pdp_context_id = BCM_NET_MAX_PDP_CNTXS;
    BNET_DEBUG(DBG_INFO,"%s: g_net_dev_tbl[%d] = 0x%x, dev_ptr 0x%x\n", __FUNCTION__, dev_index, (unsigned int)(&g_net_dev_tbl[dev_index]), (unsigned int)dev_ptr);
    BNET_DEBUG(DBG_INFO,"%s: entry_stat 0x%x\n", __FUNCTION__, g_net_dev_tbl[dev_index].entry_stat);

    spin_unlock_irqrestore(&g_dev_lock, flags);

    if ((ret = register_netdev(dev_ptr)) != 0)
    {
        BNET_DEBUG(DBG_ERROR,"%s: Error [%d] registering device \"%s\"\n", __FUNCTION__, ret, BCM_NET_DEV_STR);
        return(-1);
    }

    return(0);
}


/**
   @fn static int bcm_fuse_net_deattach(unsigned int  dev_index);
*/
static int bcm_fuse_net_deattach(unsigned int dev_index)
{
    unsigned long flags;

    if (dev_index >= BCM_NET_MAX_PDP_CNTXS)
    {
        BNET_DEBUG(DBG_ERROR,"%s: Invalid dev_index[%d] passed as parameter \n", __FUNCTION__, dev_index);
        return(-1);
    }

    spin_lock_irqsave(&g_dev_lock, flags);

    unregister_netdev(g_net_dev_tbl[dev_index].dev_ptr);
    free_netdev(g_net_dev_tbl[dev_index].dev_ptr);
     
    //Reset most entries except for the buffer pool
    g_net_dev_tbl[dev_index].dev_ptr = NULL;
    g_net_dev_tbl[dev_index].entry_stat = EFree;
    g_net_dev_tbl[dev_index].ip_addr = 0;
    g_net_dev_tbl[dev_index].pdp_context_id = BCM_NET_MAX_PDP_CNTXS;
    memset(&g_net_dev_tbl[dev_index].stats, 0, sizeof(struct net_device_stats));

    spin_unlock_irqrestore(&g_dev_lock, flags);

    return(0);
}


/**
   @fn static net_drvr_info_t *bcm_fuse_net_device_pdp_lookup(uint8_t pdp_context_id);
*/
static net_drvr_info_t *bcm_fuse_net_device_pdp_lookup(uint8_t pdp_context_id)
{  
    int i = 0;
    unsigned long flags;
    net_drvr_info_t *drvr_info_ptr = NULL;

    spin_lock_irqsave(&g_dev_lock, flags);

    for (i = 0; i < BCM_NET_MAX_PDP_CNTXS; i++)
    {
        if (g_net_dev_tbl[i].entry_stat == EFree)
            continue;

        if (g_net_dev_tbl[i].pdp_context_id == pdp_context_id)
        {
            drvr_info_ptr = &g_net_dev_tbl[i];
            goto FOUND_ENTRY;
        }
    }

    BNET_DEBUG(DBG_ERROR,"%s: No network device mapping for PDP context id[%d]\n", __FUNCTION__, pdp_context_id);

FOUND_ENTRY:
    spin_unlock_irqrestore(&g_dev_lock, flags);

    return(drvr_info_ptr);
}


/**
   @fn static uint8_t bcm_fuse_net_find_entry(net_drvr_info_t *ndrvr_info_ptr);
*/
static uint8_t bcm_fuse_net_find_entry(net_drvr_info_t *ndrvr_info_ptr)
{
    int i = 0;
    unsigned long flags;
    uint8_t dev_idx = BCM_NET_MAX_PDP_CNTXS;

    if (ndrvr_info_ptr == NULL)
    {
        return(dev_idx);
    }

    spin_lock_irqsave(&g_dev_lock, flags);

    for (i = 0; i < BCM_NET_MAX_PDP_CNTXS; i++)
    {
        if ((g_net_dev_tbl[i].dev_ptr == ndrvr_info_ptr->dev_ptr) &&
          (g_net_dev_tbl[i].entry_stat == EFree))
        {
            dev_idx = i;         
            g_net_dev_tbl[i].entry_stat = EInUse;
            goto FOUND_ENTRY;
        }
    }

    BNET_DEBUG(DBG_ERROR,"%s: Entry for device interface in use\n", __FUNCTION__);

FOUND_ENTRY:
    spin_unlock_irqrestore(&g_dev_lock, flags);

    return(dev_idx);
}


/**
   @fn static void bcm_fuse_net_free_entry(uint8_t pdp_cid);
*/
static void bcm_fuse_net_free_entry(uint8_t pdp_cid)
{
    int i = 0;
    unsigned long flags;

    if (pdp_cid == BCM_NET_INVALID_PDP_CNTX)
        return;

    spin_lock_irqsave(&g_dev_lock, flags);

    for (i = 0; i < BCM_NET_MAX_PDP_CNTXS; i++)
    {
        if (g_net_dev_tbl[i].pdp_context_id == pdp_cid)
        {
            g_net_dev_tbl[i].entry_stat = EFree;
            g_net_dev_tbl[i].pdp_context_id = BCM_NET_INVALID_PDP_CNTX;
            g_net_dev_tbl[i].ip_addr = 0;
            goto FREE_ENTRY_EXIT;
        }
    }

FREE_ENTRY_EXIT:
    spin_unlock_irqrestore(&g_dev_lock, flags);

    return;
}

/**
   @fn static uint8_t bcm_fuse_net_pdp_id(net_drvr_info_t *drvr_info_ptr);
*/
static uint8_t bcm_fuse_net_pdp_id(net_drvr_info_t *drvr_info_ptr)
{
    int i = 0;
    unsigned long flags;
    uint8_t pdp_cid = BCM_NET_INVALID_PDP_CNTX;

    if (drvr_info_ptr == NULL)
    {
        return(pdp_cid);
    }

   	spin_lock_irqsave(&g_dev_lock, flags);

    for (i = 0; i < BCM_NET_MAX_PDP_CNTXS; i++)
    {
        if (g_net_dev_tbl[i].entry_stat == EFree)
            continue;

        if (g_net_dev_tbl[i].dev_ptr == drvr_info_ptr->dev_ptr)
        {
            pdp_cid = g_net_dev_tbl[i].pdp_context_id;
            goto FOUND_ENTRY;
        }
    }

    BNET_DEBUG(DBG_ERROR,"%s: No PDP context id for net devie [%p]\n", __FUNCTION__, drvr_info_ptr->dev_ptr);

FOUND_ENTRY:
    spin_unlock_irqrestore(&g_dev_lock, flags);

    return(pdp_cid);
}


/**
   @fn static net_drvr_info_t *bcm_fuse_net_device_cid_lookup(unsigned char cid);
*/
static net_drvr_info_t *bcm_fuse_net_device_cid_lookup(unsigned char cid)
{
    int i = 0;
    unsigned long flags;
    net_drvr_info_t *drvr_info_ptr = NULL;

    spin_lock_irqsave(&g_dev_lock, flags);

    for (i = 0; i < BCM_NET_MAX_PDP_CNTXS; i++)
    {
        if (g_net_dev_tbl[i].entry_stat == EFree)
            continue;

        if (g_net_dev_tbl[i].pdp_context_id == cid)
        {
            drvr_info_ptr = &g_net_dev_tbl[i];
            goto FOUND_ENTRY;
        }
    }

    BNET_DEBUG(DBG_ERROR,"%s: No network device mapping for cid[%d]\n", __FUNCTION__, cid);

FOUND_ENTRY:
    spin_unlock_irqrestore(&g_dev_lock, flags);

    return(drvr_info_ptr);
}


static int __init bcm_fuse_net_init_module(void)
{
    unsigned int i = 0;

    BNET_DEBUG(DBG_INFO,"%s: << \n", __FUNCTION__);
    spin_lock_init(&g_dev_lock);

    for (i = 0; i < BCM_NET_MAX_PDP_CNTXS; i++)
    {
        memset(&g_net_dev_tbl[i], 0, sizeof(net_drvr_info_t));
    }

    for (i = 0; i < BCM_NET_MAX_PDP_CNTXS; i++)
    {
        bcm_fuse_net_attach(i);
    }

    return(0);
}


/**
   @fn static void __exit bcm_fuse_net_exit_module(void);
*/
static void __exit bcm_fuse_net_exit_module(void)
{
   unsigned int i = 0;

   for (i = 0; i < BCM_NET_MAX_PDP_CNTXS; i++)
   {
      bcm_fuse_net_deattach(i);
   }

   return;
}

module_init(bcm_fuse_net_init_module);
module_exit(bcm_fuse_net_exit_module);
