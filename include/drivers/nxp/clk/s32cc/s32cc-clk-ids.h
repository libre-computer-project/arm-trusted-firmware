/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2024 NXP
 */
#ifndef S32CC_CLK_IDS_H
#define S32CC_CLK_IDS_H

#include <stdint.h>
#include <lib/utils_def.h>

/**
 * Clock ID encoding:
 *     31:30 bits = Type of the clock
 *     29:0  bits = Clock ID within the clock category
 */
#define S32CC_CLK_ID_MASK	GENMASK_64(29U, 0U)
#define S32CC_CLK_TYPE_MASK	GENMASK_64(31U, 30U)
#define S32CC_CLK_ID(ID)	(((unsigned long)(ID)) & S32CC_CLK_ID_MASK)
#define S32CC_CLK_TYPE(ID)	(((unsigned long)(ID)) & S32CC_CLK_TYPE_MASK)
#define S32CC_CLK(TAG, ID)	(S32CC_CLK_ID(ID) | (S32CC_CLK_TYPE((TAG) << 30U)))
#define S32CC_HW_CLK(ID)	S32CC_CLK(0UL, U(ID))
#define S32CC_SW_CLK(SUB, ID)	S32CC_CLK(2UL | ((SUB) & 1UL), U(ID))

/* SW clocks subcategories */
#define S32CC_ARCH_CLK(ID)	S32CC_SW_CLK(0UL, ID)
#define S32CC_PLAT_CLK(ID)	S32CC_SW_CLK(1UL, ID)

/* IDs for clock selectors listed in S32CC Reference Manuals  */
#define S32CC_CLK_FIRC				S32CC_HW_CLK(0)
#define S32CC_CLK_SIRC				S32CC_HW_CLK(1)
#define S32CC_CLK_FXOSC				S32CC_HW_CLK(2)
#define S32CC_CLK_ARM_PLL_PHI0			S32CC_HW_CLK(4)
#define S32CC_CLK_ARM_PLL_PHI1			S32CC_HW_CLK(5)
#define S32CC_CLK_ARM_PLL_PHI2			S32CC_HW_CLK(6)
#define S32CC_CLK_ARM_PLL_PHI3			S32CC_HW_CLK(7)
#define S32CC_CLK_ARM_PLL_PHI4			S32CC_HW_CLK(8)
#define S32CC_CLK_ARM_PLL_PHI5			S32CC_HW_CLK(9)
#define S32CC_CLK_ARM_PLL_PHI6			S32CC_HW_CLK(10)
#define S32CC_CLK_ARM_PLL_PHI7			S32CC_HW_CLK(11)
#define S32CC_CLK_ARM_PLL_DFS1			S32CC_HW_CLK(12)
#define S32CC_CLK_ARM_PLL_DFS2			S32CC_HW_CLK(13)
#define S32CC_CLK_ARM_PLL_DFS3			S32CC_HW_CLK(14)
#define S32CC_CLK_ARM_PLL_DFS4			S32CC_HW_CLK(15)
#define S32CC_CLK_ARM_PLL_DFS5			S32CC_HW_CLK(16)
#define S32CC_CLK_ARM_PLL_DFS6			S32CC_HW_CLK(17)
#define S32CC_CLK_PERIPH_PLL_PHI0		S32CC_HW_CLK(18)
#define S32CC_CLK_PERIPH_PLL_PHI1		S32CC_HW_CLK(19)
#define S32CC_CLK_PERIPH_PLL_PHI2		S32CC_HW_CLK(20)
#define S32CC_CLK_PERIPH_PLL_PHI3		S32CC_HW_CLK(21)
#define S32CC_CLK_PERIPH_PLL_PHI4		S32CC_HW_CLK(22)
#define S32CC_CLK_PERIPH_PLL_PHI5		S32CC_HW_CLK(23)
#define S32CC_CLK_PERIPH_PLL_PHI6		S32CC_HW_CLK(24)
#define S32CC_CLK_PERIPH_PLL_PHI7		S32CC_HW_CLK(25)
#define S32CC_CLK_PERIPH_PLL_DFS1		S32CC_HW_CLK(26)
#define S32CC_CLK_PERIPH_PLL_DFS2		S32CC_HW_CLK(27)
#define S32CC_CLK_PERIPH_PLL_DFS3		S32CC_HW_CLK(28)
#define S32CC_CLK_PERIPH_PLL_DFS4		S32CC_HW_CLK(29)
#define S32CC_CLK_PERIPH_PLL_DFS5		S32CC_HW_CLK(30)
#define S32CC_CLK_PERIPH_PLL_DFS6		S32CC_HW_CLK(31)
#define S32CC_CLK_FTM0_EXT_REF			S32CC_HW_CLK(34)
#define S32CC_CLK_FTM1_EXT_REF			S32CC_HW_CLK(35)
#define S32CC_CLK_DDR_PLL_PHI0			S32CC_HW_CLK(36)
#define S32CC_CLK_GMAC0_EXT_TX			S32CC_HW_CLK(37)
#define S32CC_CLK_GMAC0_EXT_RX			S32CC_HW_CLK(38)
#define S32CC_CLK_GMAC0_EXT_REF			S32CC_HW_CLK(39)
#define S32CC_CLK_SERDES0_LANE0_TX		S32CC_HW_CLK(40)
#define S32CC_CLK_SERDES0_LANE0_CDR		S32CC_HW_CLK(41)
#define S32CC_CLK_GMAC0_EXT_TS			S32CC_HW_CLK(44)
#define S32CC_CLK_GMAC0_REF_DIV			S32CC_HW_CLK(45)

/* Software defined clock IDs */
#define S32CC_CLK_ARM_PLL_MUX			S32CC_ARCH_CLK(0)
#define S32CC_CLK_ARM_PLL_VCO			S32CC_ARCH_CLK(1)

/* ARM CGM1 clocks */
#define S32CC_CLK_MC_CGM1_MUX0			S32CC_ARCH_CLK(2)
#define S32CC_CLK_A53_CORE			S32CC_ARCH_CLK(3)
#define S32CC_CLK_A53_CORE_DIV2			S32CC_ARCH_CLK(4)
#define S32CC_CLK_A53_CORE_DIV10		S32CC_ARCH_CLK(5)

/* XBAR clock*/
#define S32CC_CLK_MC_CGM0_MUX0			S32CC_ARCH_CLK(6)
#define S32CC_CLK_XBAR_2X			S32CC_ARCH_CLK(7)
#define S32CC_CLK_XBAR				S32CC_ARCH_CLK(8)
#define S32CC_CLK_XBAR_DIV2			S32CC_ARCH_CLK(9)
#define S32CC_CLK_XBAR_DIV3			S32CC_ARCH_CLK(10)
#define S32CC_CLK_XBAR_DIV4			S32CC_ARCH_CLK(11)
#define S32CC_CLK_XBAR_DIV6			S32CC_ARCH_CLK(12)

/* Periph PLL */
#define S32CC_CLK_PERIPH_PLL_MUX		S32CC_ARCH_CLK(13)
#define S32CC_CLK_PERIPH_PLL_VCO		S32CC_ARCH_CLK(14)

#define S32CC_CLK_MC_CGM0_MUX8			S32CC_ARCH_CLK(15)
#define S32CC_CLK_LINFLEX_BAUD			S32CC_ARCH_CLK(16)
#define S32CC_CLK_LINFLEX			S32CC_ARCH_CLK(17)

/* DDR PLL */
#define S32CC_CLK_DDR_PLL_MUX			S32CC_ARCH_CLK(18)
#define S32CC_CLK_DDR_PLL_VCO			S32CC_ARCH_CLK(19)

/* DDR clock */
#define S32CC_CLK_MC_CGM5_MUX0			S32CC_ARCH_CLK(20)
#define S32CC_CLK_DDR				S32CC_ARCH_CLK(21)

#endif /* S32CC_CLK_IDS_H */
