#
# Copyright (c) 2019-2023, Intel Corporation. All rights reserved.
# Copyright (c) 2024-2025, Altera Corporation. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#

PLAT_INCLUDES		:=	\
			-Iplat/intel/soc/n5x/include/			\
			-Iplat/intel/soc/common/drivers/		\
			-Iplat/intel/soc/common/include/

# Include GICv2 driver files
include drivers/arm/gic/v2/gicv2.mk
DM_GICv2_SOURCES	:=	\
			${GICV2_SOURCES}                                \
			plat/common/plat_gicv2.c


PLAT_BL_COMMON_SOURCES	:=	\
			${DM_GICv2_SOURCES}				\
			drivers/delay_timer/delay_timer.c		\
			drivers/delay_timer/generic_delay_timer.c  	\
			drivers/ti/uart/aarch64/16550_console.S		\
			lib/xlat_tables/aarch64/xlat_tables.c 		\
			lib/xlat_tables/xlat_tables_common.c 		\
			plat/intel/soc/common/aarch64/platform_common.c \
			plat/intel/soc/common/aarch64/plat_helpers.S	\
			plat/intel/soc/common/socfpga_delay_timer.c     \
			plat/intel/soc/common/drivers/ccu/ncore_ccu.c

BL2_SOURCES     +=

BL31_SOURCES	+=	\
		drivers/arm/cci/cci.c					\
		lib/cpus/aarch64/aem_generic.S				\
		lib/cpus/aarch64/cortex_a53.S				\
		plat/common/plat_psci_common.c				\
		plat/intel/soc/n5x/bl31_plat_setup.c			\
		plat/intel/soc/n5x/soc/n5x_clock_manager.c		\
		plat/intel/soc/common/socfpga_psci.c			\
		plat/intel/soc/common/socfpga_sip_svc.c			\
		plat/intel/soc/common/socfpga_sip_svc_v2.c		\
		plat/intel/soc/common/socfpga_topology.c		\
		plat/intel/soc/common/sip/socfpga_sip_ecc.c             \
		plat/intel/soc/common/sip/socfpga_sip_fcs.c		\
		plat/intel/soc/common/soc/socfpga_mailbox.c		\
		plat/intel/soc/common/soc/socfpga_reset_manager.c

# Don't have the Linux kernel as a BL33 image by default
ARM_LINUX_KERNEL_AS_BL33	:=	0
$(eval $(call assert_boolean,ARM_LINUX_KERNEL_AS_BL33))
$(eval $(call add_define,ARM_LINUX_KERNEL_AS_BL33))
$(eval $(call add_define,ARM_PRELOADED_DTB_BASE))

# Configs for Boot Source
SOCFPGA_BOOT_SOURCE_SDMMC		?=	0
SOCFPGA_BOOT_SOURCE_QSPI		?=	0
SOCFPGA_BOOT_SOURCE_NAND		?=	0

$(eval $(call assert_booleans,\
	$(sort \
		SOCFPGA_BOOT_SOURCE_SDMMC \
		SOCFPGA_BOOT_SOURCE_QSPI \
		SOCFPGA_BOOT_SOURCE_NAND \
)))
$(eval $(call add_defines,\
	$(sort \
		SOCFPGA_BOOT_SOURCE_SDMMC \
		SOCFPGA_BOOT_SOURCE_QSPI \
		SOCFPGA_BOOT_SOURCE_NAND \
)))

PROGRAMMABLE_RESET_ADDRESS	:= 0
RESET_TO_BL2			:= 1
BL2_INV_DCACHE			:= 0
USE_COHERENT_MEM		:= 1

#To get the TF-A version via SMC calls
DEFINES += -DVERSION_MAJOR=${VERSION_MAJOR}
DEFINES += -DVERSION_MINOR=${VERSION_MINOR}
DEFINES += -DVERSION_PATCH=${VERSION_PATCH}
