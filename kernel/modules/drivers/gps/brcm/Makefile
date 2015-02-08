#******************************************************************************
#*
#*    Copyright 2007  Broadcom Corporation
#*    All Rights Reserved
#*
#*    No portions of this material may be reproduced in any form without the
#*    written permission of:
#*
#*       Broadcom Corporation
#*       16215 Alton Parkway
#*       Irvine, California 92619-7013
#*
#*    All information contained in this document is Broadcom Corporation
#*    company private, proprietary, and trade secret.
#*
#******************************************************************************
EXTRA_CFLAGS += -DUNDER_LINUX
EXTRA_CFLAGS += -DRPC_INCLUDED
EXTRA_CFLAGS += -DSTACK_wedge
EXTRA_CFLAGS += -D_BCM2153_
EXTRA_CFLAGS += -DUMTS

EXTRA_CFLAGS += -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_2157/inc/
EXTRA_CFLAGS += -I ../modules/drivers/char/brcm/fuse_rpc/rpc_2157/inc/
EXTRA_CFLAGS += -I ../modules/drivers/char/brcm/fuse_ipc/ipc_2157/Implementation/Headers/
EXTRA_CFLAGS += -I ../modules/drivers/char/brcm/fuse_ipc/inc_2157/

obj-$(CONFIG_BRCM_CNTIN) = bcm_agps_driver.o

