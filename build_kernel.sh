#!/bin/bash
###############################################################################
#
#                           Kernel Build Script 
#
###############################################################################
# 2012-07-30 xuefy        : modified
# 2011-10-24 effectivesky : modified
# 2010-12-29 allydrop     : created
###############################################################################
##############################################################################
# set toolchain
##############################################################################
export ARCH=arm
export CROSS_COMPILE=/home/xuefy/android/cm_gb/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-

##############################################################################
# make zImage
##############################################################################
mkdir -p ./obj/KERNEL_OBJ/
make O=./obj/KERNEL_OBJ/ ef65l_defconfig
make -j4 O=./obj/KERNEL_OBJ/

##############################################################################
# Copy Kernel Image and Modules
##############################################################################
cp -f ./obj/KERNEL_OBJ/arch/arm/boot/zImage ./obj/
#cp -f ./obj/KERNEL_OBJ/drivers/net/wireless/bcm4330_b2/wlan.ko ./obj/
#cp -f ./obj/KERNEL_OBJ/drivers/scsi/scsi_wait_scan.ko ./obj/
mkdir -p ./obj/modules/
cp -r `find ./obj/KERNEL_OBJ/ -iname '*.ko'` ./obj/modules/

