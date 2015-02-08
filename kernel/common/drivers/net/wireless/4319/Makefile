ifeq ($(SRCBASE),)
SRCBASE := $(shell /bin/pwd)/src
endif

# bcm4329
DHDCFLAGS = -DLINUX -DBCMDRIVER -DBCMDONGLEHOST -DDHDTHREAD -DBCMWPA2         \
	-DUNRELEASEDCHIP -Dlinux -DDHD_SDALIGN=64 -DMAX_HDR_READ=64           \
	-DDHD_FIRSTREAD=64 -DDHD_GPL -DDHD_SCHED -DBDC -DTOE -DDHD_BCMEVENTS  \
	-DSHOW_EVENTS -DBCMSDIO -DDHD_GPL -DBCMLXSDMMC -DBCMPLATFORM_BUS      \
	-Wall -Wstrict-prototypes -Werror -DSRCBASE=\"$(SRCBASE)\" \
	-DOEM_ANDROID -DEMBEDDED_PLATFORM -DPNO_SUPPORT -DBCMWAPI_WPI


DHDCFLAGS += -DDHD_DEBUG

DHDCFLAGS +=  -DSOFTAP

# For 2153 4319
#DHDCFLAGS += -DSDIO_ISR_THREAD -DCUSTOMER_HW

# For 2153 Using OOB
DHDCFLAGS += -DOOB_INTR_ONLY  -DMMC_SDIO_ABORT -DCUSTOMER_HW -DCUSTOM_OOB_GPIO_NUM=25

############### Uncomment below line to enable SW OOB option #############
## The nvram.txt should contain the necessary options for enabling fw oob
#DHDCFLAGS += -DDHD_USE_STATIC_BUF -DOOB_INTR_ONLY -DCUSTOMER_HW2  -DMMC_SDIO_ABORT

# OPTIONAL as mentioned in the TWIKI
#DHDCFLAGS += -DCUSTOM_OOB_GPIO_NUM=152

############### Uncomment below line to enable HW OOB option ############
#DHDCFLAGS += -DHW_OOB

############### Uncomment below line to build for APSTA option ##########
#DHDCFLAGS += -DAP -DAPSTA_PINGTEST

EXTRA_CFLAGS +=	-I$(src)/src/include/ \
	-I$(src)/src/dhd/sys/ \
	-I$(src)/src/wl/sys/ \
	-I$(src)/src/dongle/ \
	-I$(src)/src/bcmsdio/sys/ \
	-I$(src)/src/shared/

DHDOFILES = src/dhd/sys/dhd_linux.o \
	src/shared/linux_osl.o \
	src/shared/bcmutils.o \
	src/dhd/sys/dhd_common.o \
	src/dhd/sys/dhd_custom_gpio.o \
	src/wl/sys/wl_iw.o \
	src/shared/siutils.o \
	src/shared/sbutils.o \
	src/shared/aiutils.o \
	src/shared/hndpmu.o \
	src/shared/bcmwifi.o \
	src/dhd/sys/dhd_sdio.o   \
	src/dhd/sys/dhd_linux_sched.o \
	src/dhd/sys/dhd_cdc.o \
	src/bcmsdio/sys/bcmsdh_sdmmc.o \
	src/bcmsdio/sys/bcmsdh.o \
	src/bcmsdio/sys/bcmsdh_linux.o    \
	src/bcmsdio/sys/bcmsdh_sdmmc_linux.o

obj-m += bcm4319.o
bcm4319-objs += $(DHDOFILES)
EXTRA_CFLAGS += $(DHDCFLAGS)
EXTRA_LDFLAGS += --strip-debug
