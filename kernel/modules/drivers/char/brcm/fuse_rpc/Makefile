#Makefile for RPC sources
#EXTRA_CFLAGS := -Werror

EXTRA_CFLAGS += -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_2157/inc/
EXTRA_CFLAGS += -I ../modules/drivers/char/brcm/fuse_rpc/rpc_2157/inc/

EXTRA_CFLAGS += -Ifs/cifs/
EXTRA_CFLAGS += -DUNDER_LINUX
EXTRA_CFLAGS += -DRPC_INCLUDED

obj-$(CONFIG_BRCM_FUSE_RIL_2157SDB) += \
          fuse_rpc_main.o \
          rpc_2157/src/


