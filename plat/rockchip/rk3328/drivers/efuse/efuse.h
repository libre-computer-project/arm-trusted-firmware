/*
 * Copyright (C) 2016, Fuzhou Rockchip Electronics Co., Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef EFUSE_H
#define EFUSE_H

#include <platform_def.h>

/* CRU controller register */
#define CRU_WRITE_MASK		(16)

#define GRF_GPIO2A_IOMUX	(0x0020)
#define EFUSE_SGRF_SOC_CON5	(0X0014)

#define CRU_CLKGATE_CON2	(0x0208)
#define CRU_CLKGATE_CON15	(0x023C)
#define EFUSE_SRC_CLK_EN	(1 << 13)
#define EFUSE_1024_PCLK_EN	(1 << 9)

#define RK322XH_S_EFUSE_START	0
#define RK322XH_S_EFUSE_WORDS	24
#define RK322XH_NS_EFUSE_START	(RK322XH_S_EFUSE_START + RK322XH_S_EFUSE_WORDS)
#define RK322XH_NS_EFUSE_WORDS	8

/* SGRF controller register */
#define SGRF_WRITE_MASK		(16)

/* eFuse controller register */
#define CRU_CLKSEL_CON5		(0x0114)
#define REG_EFUSE_MOD		(0x0000)
#define efuse_pwren		(1 << 6)
#define EFUSE_RD_ENB_USER	(1 << 6)
#define EFUSE_PG_ENB_USER	(1 << 5)
#define EFUSE_STROBE_POL	(1 << 4)
#define EFUSE_LOAD_POL		(1 << 3)
#define EFUSE_PGENB_POL		(1 << 2)
#define EFUSE_CSB_POL		(1 << 1)
/* 0:auto mode; 1:user mode */
#define EFUSE_USER_MODE		(1 << 0)

#define REG_EFUSE_RD_MASK_S	(0x0004)
#define REG_EFUSE_PG_MASK_S	(0x0008)

#define REG_EFUSE_RD_MASK_NS	(0x000C)
#define REG_EFUSE_PG_MASK_NS	(0x0010)

#define REG_EFUSE_INT_CON	(0x0014)
#define REG_EFUSE_INT_STATUS	(0x0018)
#define REG_EFUSE_USER_CTRL	(0x001C)
#define EFUSE_A_SHIFT		(16)
#define EFUSE_A_MASK		(0x3FF)
#define EFUSE_PGENB		(1 << 3) /* active low */
#define EFUSE_LOAD		(1 << 2)
#define EFUSE_STROBE		(1 << 1)
#define EFUSE_CSB		(1 << 0) /* active low */
#define REG_EFUSE_DOUT		(0x0020)
#define REG_EFUSE_AUTO_CTRL	(0x0024)
/* 0:programming mode; 1:read mode */
#define EFUSE_AUTO_RD		(1 << 1)
#define EFUSE_AUTO_ENABLE	(1 << 0)

#define EFUSE_PG_ENB_MODE	(0 << 1)

#define REG_EFUSE_T_CSB_P	(0x0028)
#define REG_EFUSE_T_PGENB_P	(0x002C)
#define REG_EFUSE_T_LOAD_P	(0x0030)
#define REG_EFUSE_T_ADDR_P	(0x0034)
#define REG_EFUSE_T_STROBE_P	(0x0038)
#define REG_EFUSE_T_CSB_R	(0x003C)
#define REG_EFUSE_T_PGENB_R	(0x0040)
#define REG_EFUSE_T_LOAD_R	(0x0044)
#define REG_EFUSE_T_ADDR_R	(0x0048)
#define REG_EFUSE_T_STROBE_R	(0x004C)
#define REG_EFUSE_REVISION	(0x0050)
#define REG_EFUSE_CTRL		REG_EFUSE_USER_CTRL

#define T_CSB_P_S		1
#define T_PGENB_P_S		1
#define T_LOAD_P_S		1
#define T_ADDR_P_S		1
#define T_STROBE_P_S		2
#define T_CSB_P_L		241
#define T_PGENB_P_L		241
#define T_LOAD_P_L		241
#define T_ADDR_P_L		241
#define T_STROBE_P_L		240
#define T_CSB_R_S		1
#define T_PGENB_R_S		1
#define T_LOAD_R_S		1
#define T_ADDR_R_S		1
#define T_STROBE_R_S		2
#define T_CSB_R_L		4
#define T_PGENB_R_L		4
#define T_LOAD_R_L		4
#define T_ADDR_R_L		4
#define T_STROBE_R_L		3

/*
 * readregs function is real-time read, from the efuse hardware.
 * read function is not real-time read, read the value stored in
 * memory when machine starting up.
 */
int rk_efuse8_readregs(uint32_t addr, uint32_t length, uint8_t *buf);
int rk_efuse8_write(uint32_t addr, uint8_t val);

int rk_efuse32_readregs(uint32_t addr, uint32_t length, uint32_t *buf);
int rk_efuse32_write(uint32_t addr, uint32_t val);
int rk_efuse32_read(uint32_t addr, uint32_t length, uint32_t *buf);

int enable_efuse_clk();

void mode_init(int sec);
void rk_efuse_prog_en(int n);

int rk_efuse_init(void);

#endif /* RK_EFUSE_H */

