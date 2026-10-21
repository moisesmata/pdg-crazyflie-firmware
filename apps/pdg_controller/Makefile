.DEFAULT_GOAL := all
CRAZYFLIE_BASE := $(abspath ../../crazyflie-firmware)
OOT := $(CURDIR)
OOT_CONFIG := $(CURDIR)/app-config
COREUTILS := $(firstword $(wildcard /opt/homebrew/opt/coreutils/libexec/gnubin /usr/local/opt/coreutils/libexec/gnubin))
GNU_SED := $(firstword $(wildcard /opt/homebrew/opt/gnu-sed/libexec/gnubin /usr/local/opt/gnu-sed/libexec/gnubin))
LOCAL_ARM := $(firstword $(wildcard $(abspath ../../.tools)/xpack-arm-none-eabi-gcc-*/bin))
export PATH := $(LOCAL_ARM):$(COREUTILS):$(GNU_SED):$(PATH)
include $(CRAZYFLIE_BASE)/tools/make/oot.mk
