/*
 * Copyright (c) 2014-2025, Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <asm_macros.S>
#include <assert_macros.S>
#include <cpu_macros.S>
#include <lib/psci/psci.h>
#include <platform_def.h>

	.globl	psci_do_pwrdown_cache_maintenance
	.globl	psci_do_pwrup_cache_maintenance
	.globl	psci_power_down_wfi

/* -----------------------------------------------------------------------
 * void psci_do_pwrdown_cache_maintenance(unsigned int power level);
 *
 * This function performs cache maintenance for the specified power
 * level. The levels of cache affected are determined by the power
 * level which is passed as the argument i.e. level 0 results
 * in a flush of the L1 cache. Both the L1 and L2 caches are flushed
 * for a higher power level.
 *
 * Additionally, this function also ensures that stack memory is correctly
 * flushed out to avoid coherency issues due to a change in its memory
 * attributes after the data cache is disabled.
 * -----------------------------------------------------------------------
 */
func psci_do_pwrdown_cache_maintenance
	stp     x29, x30, [sp,#-16]!
	stp     x19, x20, [sp,#-16]!

	/* ---------------------------------------------
	 * Invoke CPU-specific power down operations for
	 * the appropriate level
	 * ---------------------------------------------
	 */
	bl	prepare_cpu_pwr_dwn

	/* ---------------------------------------------
	 * Do stack maintenance by flushing the used
	 * stack to the main memory and invalidating the
	 * remainder.
	 * ---------------------------------------------
	 */
	bl	plat_get_my_stack

	/* ---------------------------------------------
	 * Calculate and store the size of the used
	 * stack memory in x1.
	 * ---------------------------------------------
	 */
	mov	x19, x0
	mov	x1, sp
	sub	x1, x0, x1
	mov	x0, sp
	bl	flush_dcache_range

	/* ---------------------------------------------
	 * Calculate and store the size of the unused
	 * stack memory in x1. Calculate and store the
	 * stack base address in x0.
	 * ---------------------------------------------
	 */
	sub	x0, x19, #PLATFORM_STACK_SIZE
	sub	x1, sp, x0
	bl	inv_dcache_range

	ldp	x19, x20, [sp], #16
	ldp	x29, x30, [sp], #16
	ret
endfunc psci_do_pwrdown_cache_maintenance


/* -----------------------------------------------------------------------
 * void psci_do_pwrup_cache_maintenance(void);
 *
 * This function performs cache maintenance after this cpu is powered up.
 * Currently, this involves managing the used stack memory before turning
 * on the data cache.
 * -----------------------------------------------------------------------
 */
func psci_do_pwrup_cache_maintenance
	stp	x29, x30, [sp,#-16]!

	/* ---------------------------------------------
	 * Ensure any inflight stack writes have made it
	 * to main memory.
	 * ---------------------------------------------
	 */
	dmb	st

	/* ---------------------------------------------
	 * Calculate and store the size of the used
	 * stack memory in x1. Calculate and store the
	 * stack base address in x0.
	 * ---------------------------------------------
	 */
	bl	plat_get_my_stack
	mov	x1, sp
	sub	x1, x0, x1
	mov	x0, sp
	bl	inv_dcache_range

	/* ---------------------------------------------
	 * Enable the data cache.
	 * ---------------------------------------------
	 */
	mrs	x0, sctlr_el3
	orr	x0, x0, #SCTLR_C_BIT
	msr	sctlr_el3, x0
	isb

	ldp	x29, x30, [sp], #16
	ret
endfunc psci_do_pwrup_cache_maintenance

/* -----------------------------------------------------------------------
 * void psci_power_down_wfi(void); This function is called to indicate to the
 * power controller that it is safe to power down this cpu. It may exit if the
 * request was denied and reset did not occur
 * -----------------------------------------------------------------------
 */
func psci_power_down_wfi
	apply_erratum cortex_a510, ERRATUM(2684597), ERRATA_A510_2684597

	dsb	sy		// ensure write buffer empty
	wfi

	/*
	 * in case the WFI wasn't terminal, we have to undo errata mitigations.
	 * These will be smart enough to handle being called the same way
	 */
	apply_erratum cortex_a710, ERRATUM(2291219), ERRATA_A710_2291219
	apply_erratum cortex_x3,   ERRATUM(2313909), ERRATA_X3_2313909, NO_GET_CPU_REV
	apply_erratum neoverse_n2, ERRATUM(2326639), ERRATA_N2_2326639, NO_GET_CPU_REV

	ret
endfunc psci_power_down_wfi
