#Makefile for KRIL & CAPI2
#EXTRA_CFLAGS := -Werror

EXTRA_CFLAGS += -I ../modules/drivers/char/brcm/fuse_ipc/ipc_2157/Implementation/Headers/
EXTRA_CFLAGS += -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_2157/inc/
EXTRA_CFLAGS += -I ../modules/drivers/char/brcm/fuse_ril/crypto/
EXTRA_CFLAGS += -I ../modules/drivers/char/brcm/fuse_rpc/rpc_2157/inc/
EXTRA_CFLAGS += -I ../modules/drivers/char/brcm/fuse_log/

# video telephony support
ifdef CONFIG_VIDEO_TELEPHONY_ENABLE
EXTRA_CFLAGS += -DVIDEO_TELEPHONY_ENABLE
endif

EXTRA_CFLAGS += -DRPC_INCLUDED
EXTRA_CFLAGS += -DUNDER_LINUX
EXTRA_CFLAGS += -D_BCM2153_
EXTRA_CFLAGS += -DUMTS


obj-$(CONFIG_BRCM_FUSE_RIL_2157SDB) += \
	bcm_kril_main.o \
	bcm_kril_ioctl.o \
	bcm_kril_cmd_handler.o \
	bcm_kril_capi2_handler.o \
	bcm_kril_notify_handler.o \
	bcm_kril_callctrl_handler.o \
	bcm_kril_suppsvc_handler.o \
	bcm_kril_msg_handler.o \
	bcm_kril_network_handler.o \
	bcm_kril_misc_handler.o \
	bcm_kril_sim_handler.o \
	bcm_kril_stk_handler.o \
	bcm_kril_pdp_handler.o \
	bcm_kril_simlockfun.o \
	bcm_cp_cmd_handler.o \
	CAPI2_2157/ \
	crypto/

