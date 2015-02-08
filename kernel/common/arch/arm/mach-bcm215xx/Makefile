#
# Makefile for the linux kernel, BCM21553 machine.
#
obj-$(CONFIG_COMMON_CLKDEV)		+= clock-21553.o
obj-$(CONFIG_BOARD_ATHENARAY)	+= board-athenaray.o
obj-$(CONFIG_BOARD_ATHENARAY_EDN20)		+= board-athenarayEDN20.o
obj-$(CONFIG_BOARD_THUNDERBIRD) += board-thunderbird.o
obj-$(CONFIG_BOARD_THUNDERBIRD_EDN31) += board-thunderbirdEDN31.o
obj-$(CONFIG_BCM215XX_PM)      += bcm215xx_sleep.o bcm215xx_pm.o
obj-$(CONFIG_OSDAL_SUPPORT) += osdal_21553_clock.o
obj-y  += cpu-bcm21553.o device.o mm.o

obj-$(CONFIG_USB_GADGET_DWC_OTG) += lm.o
