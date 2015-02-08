#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y					:= timer.o irq.o
obj-$(CONFIG_ARCH_BCM116X)		+= dma.o
obj-y					+= sysfs.o
obj-$(CONFIG_GPIOLIB)			+= gpio.o
obj-$(CONFIG_COMMON_CLKDEV)		+= clock.o
obj-$(CONFIG_BCM2153_CPUFREQ_DRV) 	+= cpufreq_drv.o
obj-$(CONFIG_OSDAL_SUPPORT)		+= dma_drv.o osdal_os_service.o csl/ chal/ 
obj-$(CONFIG_BCM_PWM_CORE)		+= pwm_core.o
obj-$(CONFIG_BCM_RFKILL) += bcm_rfkill.o

EXTRA_CFLAGS += -ggdb

ifeq ($(CONFIG_ATHENA),y)
EXTRA_CFLAGS				+= -D_ATHENA_
EXTRA_CFLAGS				+= -DCHAL_NDEBUG_BUILD
endif

ifeq ($(CONFIG_ARCH_BCM21553_A0),y)
EXTRA_CFLAGS				+= -DCHIP_REVISION=10
endif
ifeq ($(CONFIG_ARCH_BCM21553_B0),y)
EXTRA_CFLAGS				+= -DCHIP_REVISION=20
endif

