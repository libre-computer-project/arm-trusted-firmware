/*
 * Copyright (c) 2017-2023, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef ARM_TZC_DRAM_LD_S
#define ARM_TZC_DRAM_LD_S

#include <lib/xlat_tables/xlat_tables_defs.h>

MEMORY {
    EL3_SEC_DRAM (rw): ORIGIN = ARM_EL3_TZC_DRAM1_BASE, LENGTH = ARM_EL3_TZC_DRAM1_SIZE
}

SECTIONS
{
	. = ARM_EL3_TZC_DRAM1_BASE;
	ASSERT(. == ALIGN(PAGE_SIZE),
	"ARM_EL3_TZC_DRAM_BASE address is not aligned on a page boundary.")
	.el3_tzc_dram (NOLOAD) : ALIGN(PAGE_SIZE) {
	__PLAT_SPMC_SHMEM_DATASTORE_START__ = .;
	*(.arm_spmc_shmem_datastore)
	__PLAT_SPMC_SHMEM_DATASTORE_END__ = .;
	__EL3_SEC_DRAM_START__ = .;
	*(.arm_el3_tzc_dram)
	__EL3_SEC_DRAM_UNALIGNED_END__ = .;

	. = ALIGN(PAGE_SIZE);
	__EL3_SEC_DRAM_END__ = .;
	} >EL3_SEC_DRAM
}

#endif /* ARM_TZC_DRAM_LD_S */
