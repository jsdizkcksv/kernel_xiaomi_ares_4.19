# SPDX-License-Identifier: GPL-2.0
#
# Makefile for nfc devices
#
# MediaTek platform headers (<mach/upmu_*.h>) needed by the pn553 NCI driver.
# MTK_PLATFORM is derived from CONFIG_MTK_PLATFORM (stored quoted in .config).
MTK_PLATFORM := $(subst $(quote),,$(CONFIG_MTK_PLATFORM))
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include

obj-$(CONFIG_NFC_FDP)		+= fdp/
obj-$(CONFIG_NFC_PN544)		+= pn544/
obj-$(CONFIG_NFC_MICROREAD)	+= microread/
obj-$(CONFIG_NFC_PN533)		+= pn533/
obj-$(CONFIG_NFC_MEI_PHY)	+= mei_phy.o
obj-$(CONFIG_NFC_SIM)		+= nfcsim.o
obj-$(CONFIG_NFC_PORT100)	+= port100.o
obj-$(CONFIG_NFC_MRVL)		+= nfcmrvl/
obj-$(CONFIG_NFC_TRF7970A)	+= trf7970a.o
obj-$(CONFIG_NFC_ST21NFCA)  	+= st21nfca/
obj-$(CONFIG_NFC_ST_NCI)	+= st-nci/
obj-$(CONFIG_NFC_NXP_NCI)	+= nxp-nci/
obj-$(CONFIG_NFC_S3FWRN5)	+= s3fwrn5/
obj-$(CONFIG_NFC_ST95HF)	+= st95hf/
obj-$(CONFIG_NFC_PN553_DEVICES)	+= pn553.o
