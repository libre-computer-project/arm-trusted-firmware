/*
 * Copyright (c) 2017-2021, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <endian.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <drivers/ufs.h>
#include <lib/mmio.h>

#define CDB_ADDR_MASK			127
#define ALIGN_CDB(x)			(((x) + CDB_ADDR_MASK) & ~CDB_ADDR_MASK)
#define ALIGN_8(x)			(((x) + 7) & ~7)

#define UFS_DESC_SIZE			0x400
#define MAX_UFS_DESC_SIZE		0x8000		/* 32 descriptors */

#define MAX_PRDT_SIZE			0x40000		/* 256KB */

static ufs_params_t ufs_params;
static int nutrs;	/* Number of UTP Transfer Request Slots */

/*
 * ufs_uic_error_handler - UIC error interrupts handler
 * @ignore_linereset: set to ignore PA_LAYER_GEN_ERR (UIC error)
 *
 * Returns
 * 0 - ignore error
 * -EIO - fatal error, needs re-init
 * -EAGAIN - non-fatal error, retries are sufficient
 */
static int ufs_uic_error_handler(bool ignore_linereset)
{
	uint32_t data;
	int result = 0;

	data = mmio_read_32(ufs_params.reg_base + UECPA);
	if (data & UFS_UIC_PA_ERROR_MASK) {
		if (data & PA_LAYER_GEN_ERR) {
			if (!ignore_linereset) {
				return -EIO;
			}
		} else {
			result = -EAGAIN;
		}
	}

	data = mmio_read_32(ufs_params.reg_base + UECDL);
	if (data & UFS_UIC_DL_ERROR_MASK) {
		if (data & PA_INIT_ERR) {
			return -EIO;
		}
		result = -EAGAIN;
	}

	/* NL/TL/DME error requires retries */
	data = mmio_read_32(ufs_params.reg_base + UECN);
	if (data & UFS_UIC_NL_ERROR_MASK) {
		result = -EAGAIN;
	}

	data = mmio_read_32(ufs_params.reg_base + UECT);
	if (data & UFS_UIC_TL_ERROR_MASK) {
		result = -EAGAIN;
	}

	data = mmio_read_32(ufs_params.reg_base + UECDME);
	if (data & UFS_UIC_DME_ERROR_MASK) {
		result = -EAGAIN;
	}

	return result;
}

/*
 * ufs_error_handler - error interrupts handler
 * @status: interrupt status
 * @ignore_linereset: set to ignore PA_LAYER_GEN_ERR (UIC error)
 *
 * Returns
 * 0 - ignore error
 * -EIO - fatal error, needs re-init
 * -EAGAIN - non-fatal error, retries are sufficient
 */
static int ufs_error_handler(uint32_t status, bool ignore_linereset)
{
	int result;

	if (status & UFS_INT_UE) {
		result = ufs_uic_error_handler(ignore_linereset);
		if (result != 0) {
			return result;
		}
	}

	/* Return I/O error on fatal error, it is upto the caller to re-init UFS */
	if (status & UFS_INT_FATAL) {
		return -EIO;
	}

	/* retry for non-fatal errors */
	return -EAGAIN;
}

/*
 * ufs_wait_for_int_status - wait for expected interrupt status
 * @expected: expected interrupt status bit
 * @timeout_ms: timeout in milliseconds to poll for
 * @ignore_linereset: set to ignore PA_LAYER_GEN_ERR (UIC error)
 *
 * Returns
 * 0 - received expected interrupt and cleared it
 * -EIO - fatal error, needs re-init
 * -EAGAIN - non-fatal error, caller can retry
 * -ETIMEDOUT - timed out waiting for interrupt status
 */
static int ufs_wait_for_int_status(const uint32_t expected_status,
				   unsigned int timeout_ms,
				   bool ignore_linereset)
{
	uint32_t interrupt_status, interrupts_enabled;
	int result = 0;

	interrupts_enabled = mmio_read_32(ufs_params.reg_base + IE);
	do {
		interrupt_status = mmio_read_32(ufs_params.reg_base + IS) & interrupts_enabled;
		if (interrupt_status & UFS_INT_ERR) {
			mmio_write_32(ufs_params.reg_base + IS, interrupt_status & UFS_INT_ERR);
			result = ufs_error_handler(interrupt_status, ignore_linereset);
			if (result != 0) {
				return result;
			}
		}

		if (interrupt_status & expected_status) {
			break;
		}
		mdelay(1);
	} while (timeout_ms-- > 0);

	if (!(interrupt_status & expected_status)) {
		return -ETIMEDOUT;
	}

	mmio_write_32(ufs_params.reg_base + IS, expected_status);

	return result;
}


int ufshc_send_uic_cmd(uintptr_t base, uic_cmd_t *cmd)
{
	unsigned int data;
	int result, retries;

	if (base == 0 || cmd == NULL)
		return -EINVAL;

	for (retries = 0; retries < 100; retries++) {
		data = mmio_read_32(base + HCS);
		if ((data & HCS_UCRDY) != 0) {
			break;
		}
		mdelay(1);
	}
	if (retries >= 100) {
		return -EBUSY;
	}

	mmio_write_32(base + IS, ~0);
	mmio_write_32(base + UCMDARG1, cmd->arg1);
	mmio_write_32(base + UCMDARG2, cmd->arg2);
	mmio_write_32(base + UCMDARG3, cmd->arg3);
	mmio_write_32(base + UICCMD, cmd->op);

	result = ufs_wait_for_int_status(UFS_INT_UCCS, UIC_CMD_TIMEOUT_MS,
					 cmd->op == DME_SET);
	if (result != 0) {
		return result;
	}

	return mmio_read_32(base + UCMDARG2) & CONFIG_RESULT_CODE_MASK;
}

int ufshc_dme_get(unsigned int attr, unsigned int idx, unsigned int *val)
{
	uintptr_t base;
	int result, retries;
	uic_cmd_t cmd;

	assert(ufs_params.reg_base != 0);

	if (val == NULL)
		return -EINVAL;

	base = ufs_params.reg_base;
	cmd.arg1 = (attr << 16) | GEN_SELECTOR_IDX(idx);
	cmd.arg2 = 0;
	cmd.arg3 = 0;
	cmd.op = DME_GET;

	for (retries = 0; retries < UFS_UIC_COMMAND_RETRIES; ++retries) {
		result = ufshc_send_uic_cmd(base, &cmd);
		if (result == 0)
			break;
		/* -EIO requires UFS re-init */
		if (result == -EIO) {
			return result;
		}
	}
	if (retries >= UFS_UIC_COMMAND_RETRIES)
		return -EIO;

	*val = mmio_read_32(base + UCMDARG3);
	return 0;
}

int ufshc_dme_set(unsigned int attr, unsigned int idx, unsigned int val)
{
	uintptr_t base;
	int result, retries;
	uic_cmd_t cmd;

	assert((ufs_params.reg_base != 0));

	base = ufs_params.reg_base;
	cmd.arg1 = (attr << 16) | GEN_SELECTOR_IDX(idx);
	cmd.arg2 = 0;
	cmd.arg3 = val;
	cmd.op = DME_SET;

	for (retries = 0; retries < UFS_UIC_COMMAND_RETRIES; ++retries) {
		result = ufshc_send_uic_cmd(base, &cmd);
		if (result == 0)
			break;
		/* -EIO requires UFS re-init */
		if (result == -EIO) {
			return result;
		}
	}
	if (retries >= UFS_UIC_COMMAND_RETRIES)
		return -EIO;

	return 0;
}

static int ufshc_hce_enable(uintptr_t base)
{
	unsigned int data;
	int retries;

	/* Enable Host Controller */
	mmio_write_32(base + HCE, HCE_ENABLE);

	/* Wait until basic initialization sequence completed */
	for (retries = 0; retries < HCE_ENABLE_INNER_RETRIES; ++retries) {
		data = mmio_read_32(base + HCE);
		if (data & HCE_ENABLE) {
			break;
		}
		udelay(HCE_ENABLE_TIMEOUT_US);
	}
	if (retries >= HCE_ENABLE_INNER_RETRIES) {
		return -ETIMEDOUT;
	}

	return 0;
}

static int ufshc_hce_disable(uintptr_t base)
{
	unsigned int data;
	int timeout;

	/* Disable Host Controller */
	mmio_write_32(base + HCE, HCE_DISABLE);
	timeout = HCE_DISABLE_TIMEOUT_US;
	do {
		data = mmio_read_32(base + HCE);
		if ((data & HCE_ENABLE) == HCE_DISABLE) {
			break;
		}
		udelay(1);
	} while (--timeout > 0);

	if (timeout <= 0) {
		return -ETIMEDOUT;
	}

	return 0;
}


static int ufshc_reset(uintptr_t base)
{
	unsigned int data;
	int retries, result;

	/* disable controller if enabled */
	if (mmio_read_32(base + HCE) & HCE_ENABLE) {
		result = ufshc_hce_disable(base);
		if (result != 0) {
			return -EIO;
		}
	}

	for (retries = 0; retries < HCE_ENABLE_OUTER_RETRIES; ++retries) {
		result = ufshc_hce_enable(base);
		if (result == 0) {
			break;
		}
	}
	if (retries >= HCE_ENABLE_OUTER_RETRIES) {
		return -EIO;
	}

	/* Enable UIC Interrupts alone. We can ignore other interrupts until
	 * link is up as there might be spurious error interrupts during link-up
	 */
	data = UFS_INT_UCCS | UFS_INT_UHES | UFS_INT_UHXS | UFS_INT_UPMS;
	mmio_write_32(base + IE, data);

	return 0;
}

static int ufshc_dme_link_startup(uintptr_t base)
{
	uic_cmd_t cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_LINKSTARTUP;
	return ufshc_send_uic_cmd(base, &cmd);
}

static int ufshc_link_startup(uintptr_t base)
{
	int data, result;
	int retries;

	for (retries = DME_LINKSTARTUP_RETRIES; retries > 0; retries--) {
		result = ufshc_dme_link_startup(base);
		if (result != 0) {
			/* Reset controller before trying again */
			result = ufshc_reset(base);
			if (result != 0) {
				return result;
			}
			continue;
		}
		assert(mmio_read_32(base + HCS) & HCS_DP);
		data = mmio_read_32(base + IS);
		if (data & UFS_INT_ULSS)
			mmio_write_32(base + IS, UFS_INT_ULSS);

		/* clear UE set due to line-reset */
		if (data & UFS_INT_UE) {
			mmio_write_32(base + IS, UFS_INT_UE);
		}
		/* clearing line-reset, UECPA is cleared on read */
		mmio_read_32(base + UECPA);
		return 0;
	}
	return -EIO;
}

/* Read Door Bell register to check if slot zero is available */
static int is_slot_available(void)
{
	if (mmio_read_32(ufs_params.reg_base + UTRLDBR) & 0x1) {
		return -EBUSY;
	}
	return 0;
}

static void get_utrd(utp_utrd_t *utrd)
{
	uintptr_t base;
	int result;
	utrd_header_t *hd;

	assert(utrd != NULL);
	result = is_slot_available();
	assert(result == 0);

	/* clear utrd */
	memset((void *)utrd, 0, sizeof(utp_utrd_t));
	base = ufs_params.desc_base;
	/* clear the descriptor */
	memset((void *)base, 0, UFS_DESC_SIZE);

	utrd->header = base;
	utrd->task_tag = 1; /* We always use the first slot */
	/* CDB address should be aligned with 128 bytes */
	utrd->upiu = ALIGN_CDB(utrd->header + sizeof(utrd_header_t));
	utrd->resp_upiu = ALIGN_8(utrd->upiu + sizeof(cmd_upiu_t));
	utrd->size_upiu = utrd->resp_upiu - utrd->upiu;
	utrd->size_resp_upiu = ALIGN_8(sizeof(resp_upiu_t));
	utrd->prdt = utrd->resp_upiu + utrd->size_resp_upiu;

	hd = (utrd_header_t *)utrd->header;
	hd->ucdba = utrd->upiu & UINT32_MAX;
	hd->ucdbau = (utrd->upiu >> 32) & UINT32_MAX;
	/* Both RUL and RUO is based on DWORD */
	hd->rul = utrd->size_resp_upiu >> 2;
	hd->ruo = utrd->size_upiu >> 2;
	(void)result;
}

/*
 * Prepare UTRD, Command UPIU, Response UPIU.
 */
static int ufs_prepare_cmd(utp_utrd_t *utrd, uint8_t op, uint8_t lun,
			   int lba, uintptr_t buf, size_t length)
{
	utrd_header_t *hd;
	cmd_upiu_t *upiu;
	prdt_t *prdt;
	unsigned int ulba;
	unsigned int lba_cnt;
	uintptr_t desc_limit;
	uintptr_t prdt_end;

	hd = (utrd_header_t *)utrd->header;
	upiu = (cmd_upiu_t *)utrd->upiu;

	hd->i = 1;
	hd->ct = CT_UFS_STORAGE;
	hd->ocs = OCS_MASK;

	upiu->trans_type = CMD_UPIU;
	upiu->task_tag = utrd->task_tag;
	upiu->cdb[0] = op;
	ulba = (unsigned int)lba;
	lba_cnt = (unsigned int)(length >> UFS_BLOCK_SHIFT);
	switch (op) {
	case CDBCMD_TEST_UNIT_READY:
		break;
	case CDBCMD_READ_CAPACITY_10:
		hd->dd = DD_OUT;
		upiu->flags = UPIU_FLAGS_R | UPIU_FLAGS_ATTR_S;
		upiu->lun = lun;
		break;
	case CDBCMD_READ_10:
		hd->dd = DD_OUT;
		upiu->flags = UPIU_FLAGS_R | UPIU_FLAGS_ATTR_S;
		upiu->lun = lun;
		upiu->cdb[1] = RW_WITHOUT_CACHE;
		/* set logical block address */
		upiu->cdb[2] = (ulba >> 24) & 0xff;
		upiu->cdb[3] = (ulba >> 16) & 0xff;
		upiu->cdb[4] = (ulba >> 8) & 0xff;
		upiu->cdb[5] = ulba & 0xff;
		/* set transfer length */
		upiu->cdb[7] = (lba_cnt >> 8) & 0xff;
		upiu->cdb[8] = lba_cnt & 0xff;
		break;
	case CDBCMD_WRITE_10:
		hd->dd = DD_IN;
		upiu->flags = UPIU_FLAGS_W | UPIU_FLAGS_ATTR_S;
		upiu->lun = lun;
		upiu->cdb[1] = RW_WITHOUT_CACHE;
		/* set logical block address */
		upiu->cdb[2] = (ulba >> 24) & 0xff;
		upiu->cdb[3] = (ulba >> 16) & 0xff;
		upiu->cdb[4] = (ulba >> 8) & 0xff;
		upiu->cdb[5] = ulba & 0xff;
		/* set transfer length */
		upiu->cdb[7] = (lba_cnt >> 8) & 0xff;
		upiu->cdb[8] = lba_cnt & 0xff;
		break;
	default:
		assert(0);
		break;
	}
	if (hd->dd == DD_IN) {
		flush_dcache_range(buf, length);
	} else if (hd->dd == DD_OUT) {
		inv_dcache_range(buf, length);
	}

	utrd->prdt_length = 0;
	if (length) {
		upiu->exp_data_trans_len = htobe32(length);
		assert(lba_cnt <= UINT16_MAX);
		prdt = (prdt_t *)utrd->prdt;

		desc_limit = ufs_params.desc_base + ufs_params.desc_size;
		while (length > 0) {
			if ((uintptr_t)prdt + sizeof(prdt_t) > desc_limit) {
				ERROR("UFS: Exceeded descriptor limit. Image is too large\n");
				panic();
			}
			prdt->dba = (unsigned int)(buf & UINT32_MAX);
			prdt->dbau = (unsigned int)((buf >> 32) & UINT32_MAX);
			/* prdt->dbc counts from 0 */
			if (length > MAX_PRDT_SIZE) {
				prdt->dbc = MAX_PRDT_SIZE - 1;
				length = length - MAX_PRDT_SIZE;
			} else {
				prdt->dbc = length - 1;
				length = 0;
			}
			buf += MAX_PRDT_SIZE;
			prdt++;
			utrd->prdt_length++;
		}
		hd->prdtl = utrd->prdt_length;
		hd->prdto = (utrd->size_upiu + utrd->size_resp_upiu) >> 2;
	}

	prdt_end = utrd->prdt + utrd->prdt_length * sizeof(prdt_t);
	flush_dcache_range(utrd->header, prdt_end - utrd->header);
	return 0;
}

static int ufs_prepare_query(utp_utrd_t *utrd, uint8_t op, uint8_t idn,
			     uint8_t index, uint8_t sel,
			     uintptr_t buf, size_t length)
{
	utrd_header_t *hd;
	query_upiu_t *query_upiu;


	hd = (utrd_header_t *)utrd->header;
	query_upiu = (query_upiu_t *)utrd->upiu;

	hd->i = 1;
	hd->ct = CT_UFS_STORAGE;
	hd->ocs = OCS_MASK;

	query_upiu->trans_type = QUERY_REQUEST_UPIU;
	query_upiu->task_tag = utrd->task_tag;
	query_upiu->data_segment_len = htobe16(length);
	query_upiu->ts.desc.opcode = op;
	query_upiu->ts.desc.idn = idn;
	query_upiu->ts.desc.index = index;
	query_upiu->ts.desc.selector = sel;
	switch (op) {
	case QUERY_READ_DESC:
		query_upiu->query_func = QUERY_FUNC_STD_READ;
		query_upiu->ts.desc.length = htobe16(length);
		break;
	case QUERY_WRITE_DESC:
		query_upiu->query_func = QUERY_FUNC_STD_WRITE;
		query_upiu->ts.desc.length = htobe16(length);
		memcpy((void *)(utrd->upiu + sizeof(query_upiu_t)),
		       (void *)buf, length);
		break;
	case QUERY_READ_ATTR:
	case QUERY_READ_FLAG:
		query_upiu->query_func = QUERY_FUNC_STD_READ;
		break;
	case QUERY_CLEAR_FLAG:
	case QUERY_SET_FLAG:
		query_upiu->query_func = QUERY_FUNC_STD_WRITE;
		break;
	case QUERY_WRITE_ATTR:
		query_upiu->query_func = QUERY_FUNC_STD_WRITE;
		query_upiu->ts.attr.value = htobe32(*((uint32_t *)buf));
		break;
	default:
		assert(0);
		break;
	}
	flush_dcache_range((uintptr_t)utrd->header, UFS_DESC_SIZE);
	return 0;
}

static void ufs_prepare_nop_out(utp_utrd_t *utrd)
{
	utrd_header_t *hd;
	nop_out_upiu_t *nop_out;

	hd = (utrd_header_t *)utrd->header;
	nop_out = (nop_out_upiu_t *)utrd->upiu;

	hd->i = 1;
	hd->ct = CT_UFS_STORAGE;
	hd->ocs = OCS_MASK;

	nop_out->trans_type = 0;
	nop_out->task_tag = utrd->task_tag;
	flush_dcache_range((uintptr_t)utrd->header, UFS_DESC_SIZE);
}

static void ufs_send_request(int task_tag)
{
	unsigned int data;
	int slot;

	slot = task_tag - 1;
	/* clear all interrupts */
	mmio_write_32(ufs_params.reg_base + IS, ~0);

	mmio_write_32(ufs_params.reg_base + UTRLRSR, 1);
	assert(mmio_read_32(ufs_params.reg_base + UTRLRSR) == 1);

	data = UTRIACR_IAEN | UTRIACR_CTR | UTRIACR_IACTH(0x1F) |
	       UTRIACR_IATOVAL(0xFF);
	mmio_write_32(ufs_params.reg_base + UTRIACR, data);
	/* send request */
	mmio_setbits_32(ufs_params.reg_base + UTRLDBR, 1U << slot);
}

static int ufs_check_resp(utp_utrd_t *utrd, int trans_type, unsigned int timeout_ms)
{
	utrd_header_t *hd;
	resp_upiu_t *resp;
	sense_data_t *sense;
	unsigned int data;
	int slot, result;

	hd = (utrd_header_t *)utrd->header;
	resp = (resp_upiu_t *)utrd->resp_upiu;

	result = ufs_wait_for_int_status(UFS_INT_UTRCS, timeout_ms, false);
	if (result != 0) {
		return result;
	}

	slot = utrd->task_tag - 1;

	data = mmio_read_32(ufs_params.reg_base + UTRLDBR);
	assert((data & (1 << slot)) == 0);
	/*
	 * Invalidate the header after DMA read operation has
	 * completed to avoid cpu referring to the prefetched
	 * data brought in before DMA completion.
	 */
	inv_dcache_range((uintptr_t)hd, UFS_DESC_SIZE);
	assert(hd->ocs == OCS_SUCCESS);
	assert((resp->trans_type & TRANS_TYPE_CODE_MASK) == trans_type);

	sense = &resp->sd.sense;
	if (sense->resp_code == SENSE_DATA_VALID &&
	    sense->sense_key == SENSE_KEY_UNIT_ATTENTION && sense->asc == 0x29 &&
	    sense->ascq == 0) {
		WARN("Unit Attention Condition\n");
		return -EAGAIN;
	}

	(void)resp;
	(void)slot;
	(void)data;
	return 0;
}

static void ufs_send_cmd(utp_utrd_t *utrd, uint8_t cmd_op, uint8_t lun, int lba, uintptr_t buf,
			 size_t length)
{
	int result, i;

	for (i = 0; i < UFS_CMD_RETRIES; ++i) {
		get_utrd(utrd);
		result = ufs_prepare_cmd(utrd, cmd_op, lun, lba, buf, length);
		assert(result == 0);
		ufs_send_request(utrd->task_tag);
		result = ufs_check_resp(utrd, RESPONSE_UPIU, CMD_TIMEOUT_MS);
		if (result == 0 || result == -EIO) {
			break;
		}
	}
	assert(result == 0);
	(void)result;
}

#ifdef UFS_RESP_DEBUG
static void dump_upiu(utp_utrd_t *utrd)
{
	utrd_header_t *hd;
	int i;

	hd = (utrd_header_t *)utrd->header;
	INFO("utrd:0x%x, ruo:0x%x, rul:0x%x, ocs:0x%x, UTRLDBR:0x%x\n",
		(unsigned int)(uintptr_t)utrd, hd->ruo, hd->rul, hd->ocs,
		mmio_read_32(ufs_params.reg_base + UTRLDBR));
	for (i = 0; i < sizeof(utrd_header_t); i += 4) {
		INFO("[%lx]:0x%x\n",
			(uintptr_t)utrd->header + i,
			*(unsigned int *)((uintptr_t)utrd->header + i));
	}

	for (i = 0; i < sizeof(cmd_upiu_t); i += 4) {
		INFO("cmd[%lx]:0x%x\n",
			utrd->upiu + i,
			*(unsigned int *)(utrd->upiu + i));
	}
	for (i = 0; i < sizeof(resp_upiu_t); i += 4) {
		INFO("resp[%lx]:0x%x\n",
			utrd->resp_upiu + i,
			*(unsigned int *)(utrd->resp_upiu + i));
	}
	for (i = 0; i < sizeof(prdt_t); i += 4) {
		INFO("prdt[%lx]:0x%x\n",
			utrd->prdt + i,
			*(unsigned int *)(utrd->prdt + i));
	}
}
#endif

static void ufs_verify_init(void)
{
	utp_utrd_t utrd;
	int result;

	get_utrd(&utrd);
	ufs_prepare_nop_out(&utrd);
	ufs_send_request(utrd.task_tag);
	result = ufs_check_resp(&utrd, NOP_IN_UPIU, NOP_OUT_TIMEOUT_MS);
	assert(result == 0);
	(void)result;
}

static void ufs_verify_ready(void)
{
	utp_utrd_t utrd;
	ufs_send_cmd(&utrd, CDBCMD_TEST_UNIT_READY, 0, 0, 0, 0);
}

static void ufs_query(uint8_t op, uint8_t idn, uint8_t index, uint8_t sel,
		      uintptr_t buf, size_t size)
{
	utp_utrd_t utrd;
	query_resp_upiu_t *resp;
	int result;

	switch (op) {
	case QUERY_READ_FLAG:
	case QUERY_READ_ATTR:
	case QUERY_READ_DESC:
	case QUERY_WRITE_DESC:
	case QUERY_WRITE_ATTR:
		assert(((buf & 3) == 0) && (size != 0));
		break;
	default:
		/* Do nothing in default case */
		break;
	}
	get_utrd(&utrd);
	ufs_prepare_query(&utrd, op, idn, index, sel, buf, size);
	ufs_send_request(utrd.task_tag);
	result = ufs_check_resp(&utrd, QUERY_RESPONSE_UPIU, QUERY_REQ_TIMEOUT_MS);
	assert(result == 0);
	resp = (query_resp_upiu_t *)utrd.resp_upiu;
#ifdef UFS_RESP_DEBUG
	dump_upiu(&utrd);
#endif
	assert(resp->query_resp == QUERY_RESP_SUCCESS);

	switch (op) {
	case QUERY_READ_FLAG:
		*(uint32_t *)buf = (uint32_t)resp->ts.flag.value;
		break;
	case QUERY_READ_DESC:
		memcpy((void *)buf,
		       (void *)(utrd.resp_upiu + sizeof(query_resp_upiu_t)),
		       size);
		break;
	case QUERY_READ_ATTR:
		*(uint32_t *)buf = htobe32(resp->ts.attr.value);
		break;
	default:
		/* Do nothing in default case */
		break;
	}
	(void)result;
}

unsigned int ufs_read_attr(int idn)
{
	unsigned int value;

	ufs_query(QUERY_READ_ATTR, idn, 0, 0,
		  (uintptr_t)&value, sizeof(value));
	return value;
}

void ufs_write_attr(int idn, unsigned int value)
{
	ufs_query(QUERY_WRITE_ATTR, idn, 0, 0,
		  (uintptr_t)&value, sizeof(value));
}

unsigned int ufs_read_flag(int idn)
{
	unsigned int value;

	ufs_query(QUERY_READ_FLAG, idn, 0, 0,
		  (uintptr_t)&value, sizeof(value));
	return value;
}

void ufs_set_flag(int idn)
{
	ufs_query(QUERY_SET_FLAG, idn, 0, 0, 0, 0);
}

void ufs_clear_flag(int idn)
{
	ufs_query(QUERY_CLEAR_FLAG, idn, 0, 0, 0, 0);
}

void ufs_read_desc(int idn, int index, uintptr_t buf, size_t size)
{
	ufs_query(QUERY_READ_DESC, idn, index, 0, buf, size);
}

void ufs_write_desc(int idn, int index, uintptr_t buf, size_t size)
{
	ufs_query(QUERY_WRITE_DESC, idn, index, 0, buf, size);
}

static int ufs_read_capacity(int lun, unsigned int *num, unsigned int *size)
{
	utp_utrd_t utrd;
	resp_upiu_t *resp;
	sense_data_t *sense;
	unsigned char data[CACHE_WRITEBACK_GRANULE << 1];
	uintptr_t buf;
	int retries = UFS_READ_CAPACITY_RETRIES;

	assert((ufs_params.reg_base != 0) &&
	       (ufs_params.desc_base != 0) &&
	       (ufs_params.desc_size >= UFS_DESC_SIZE) &&
	       (num != NULL) && (size != NULL));

	/* align buf address */
	buf = (uintptr_t)data;
	buf = (buf + CACHE_WRITEBACK_GRANULE - 1) &
	      ~(CACHE_WRITEBACK_GRANULE - 1);
	do {
		ufs_send_cmd(&utrd, CDBCMD_READ_CAPACITY_10, lun, 0,
			    buf, READ_CAPACITY_LENGTH);
#ifdef UFS_RESP_DEBUG
		dump_upiu(&utrd);
#endif
		resp = (resp_upiu_t *)utrd.resp_upiu;
		sense = &resp->sd.sense;
		if (!((sense->resp_code == SENSE_DATA_VALID) &&
		    (sense->sense_key == SENSE_KEY_UNIT_ATTENTION) &&
		    (sense->asc == 0x29) && (sense->ascq == 0))) {
			inv_dcache_range(buf, CACHE_WRITEBACK_GRANULE);
			/* last logical block address */
			*num = be32toh(*(unsigned int *)buf);
			if (*num)
				*num += 1;
			/* logical block length in bytes */
			*size = be32toh(*(unsigned int *)(buf + 4));

			return 0;
		}

	} while (retries-- > 0);

	return -ETIMEDOUT;
}

size_t ufs_read_blocks(int lun, int lba, uintptr_t buf, size_t size)
{
	utp_utrd_t utrd;
	resp_upiu_t *resp;

	assert((ufs_params.reg_base != 0) &&
	       (ufs_params.desc_base != 0) &&
	       (ufs_params.desc_size >= UFS_DESC_SIZE));

	ufs_send_cmd(&utrd, CDBCMD_READ_10, lun, lba, buf, size);
#ifdef UFS_RESP_DEBUG
	dump_upiu(&utrd);
#endif
	/*
	 * Invalidate prefetched cache contents before cpu
	 * accesses the buf.
	 */
	inv_dcache_range(buf, size);
	resp = (resp_upiu_t *)utrd.resp_upiu;
	return size - resp->res_trans_cnt;
}

size_t ufs_write_blocks(int lun, int lba, const uintptr_t buf, size_t size)
{
	utp_utrd_t utrd;
	resp_upiu_t *resp;

	assert((ufs_params.reg_base != 0) &&
	       (ufs_params.desc_base != 0) &&
	       (ufs_params.desc_size >= UFS_DESC_SIZE));

	ufs_send_cmd(&utrd, CDBCMD_WRITE_10, lun, lba, buf, size);
#ifdef UFS_RESP_DEBUG
	dump_upiu(&utrd);
#endif
	resp = (resp_upiu_t *)utrd.resp_upiu;
	return size - resp->res_trans_cnt;
}

static int ufs_set_fdevice_init(void)
{
	unsigned int result;
	int timeout;

	ufs_set_flag(FLAG_DEVICE_INIT);

	timeout = FDEVICEINIT_TIMEOUT_MS;
	do {
		result = ufs_read_flag(FLAG_DEVICE_INIT);
		if (!result) {
			break;
		}
		mdelay(5);
		timeout -= 5;
	} while (timeout > 0);

	if (result != 0U) {
		return -ETIMEDOUT;
	}

	return 0;
}

static void ufs_enum(void)
{
	unsigned int blk_num, blk_size;
	int i, result;

	mmio_write_32(ufs_params.reg_base + UTRLBA,
		      ufs_params.desc_base & UINT32_MAX);
	mmio_write_32(ufs_params.reg_base + UTRLBAU,
		      (ufs_params.desc_base >> 32) & UINT32_MAX);

	ufs_verify_init();
	ufs_verify_ready();

	result = ufs_set_fdevice_init();
	assert(result == 0);

	blk_num = 0;
	blk_size = 0;

	/* dump available LUNs */
	for (i = 0; i < UFS_MAX_LUNS; i++) {
		result = ufs_read_capacity(i, &blk_num, &blk_size);
		if (result != 0) {
			WARN("UFS LUN%d dump failed\n", i);
		}
		if (blk_num && blk_size) {
			INFO("UFS LUN%d contains %d blocks with %d-byte size\n",
			     i, blk_num, blk_size);
		}
	}

	(void)result;
}

static void ufs_get_device_info(struct ufs_dev_desc *card_data)
{
	uint8_t desc_buf[DESC_DEVICE_MAX_SIZE];

	ufs_read_desc(DESC_TYPE_DEVICE, 0, (uintptr_t)desc_buf, DESC_DEVICE_MAX_SIZE);

	/*
	 * getting vendor (manufacturerID) and Bank Index in big endian
	 * format
	 */
	card_data->wmanufacturerid = (uint16_t)((desc_buf[DEVICE_DESC_PARAM_MANF_ID] << 8) |
				     (desc_buf[DEVICE_DESC_PARAM_MANF_ID + 1]));
}

int ufs_init(const ufs_ops_t *ops, ufs_params_t *params)
{
	int result;
	unsigned int data;
	uic_cmd_t cmd;
	struct ufs_dev_desc card = {0};

	assert((params != NULL) &&
	       (params->reg_base != 0) &&
	       (params->desc_base != 0) &&
	       (params->desc_size >= UFS_DESC_SIZE));

	memcpy(&ufs_params, params, sizeof(ufs_params_t));

	/* 0 means 1 slot */
	nutrs = (mmio_read_32(ufs_params.reg_base + CAP) & CAP_NUTRS_MASK) + 1;
	if (nutrs > (ufs_params.desc_size / UFS_DESC_SIZE)) {
		nutrs = ufs_params.desc_size / UFS_DESC_SIZE;
	}


	if (ufs_params.flags & UFS_FLAGS_SKIPINIT) {
		mmio_write_32(ufs_params.reg_base + UTRLBA,
			      ufs_params.desc_base & UINT32_MAX);
		mmio_write_32(ufs_params.reg_base + UTRLBAU,
			      (ufs_params.desc_base >> 32) & UINT32_MAX);

		result = ufshc_dme_get(0x1571, 0, &data);
		assert(result == 0);
		result = ufshc_dme_get(0x41, 0, &data);
		assert(result == 0);
		if (data == 1) {
			/* prepare to exit hibernate mode */
			memset(&cmd, 0, sizeof(uic_cmd_t));
			cmd.op = DME_HIBERNATE_EXIT;
			result = ufshc_send_uic_cmd(ufs_params.reg_base,
						    &cmd);
			assert(result == 0);
			data = mmio_read_32(ufs_params.reg_base + UCMDARG2);
			assert(data == 0);
			do {
				data = mmio_read_32(ufs_params.reg_base + IS);
			} while ((data & UFS_INT_UHXS) == 0);
			mmio_write_32(ufs_params.reg_base + IS, UFS_INT_UHXS);
			data = mmio_read_32(ufs_params.reg_base + HCS);
			assert((data & HCS_UPMCRS_MASK) == HCS_PWR_LOCAL);
		}
		result = ufshc_dme_get(0x1568, 0, &data);
		assert(result == 0);
		assert((data > 0) && (data <= 3));
	} else {
		assert((ops != NULL) && (ops->phy_init != NULL) &&
		       (ops->phy_set_pwr_mode != NULL));

		result = ufshc_reset(ufs_params.reg_base);
		assert(result == 0);
		ops->phy_init(&ufs_params);
		result = ufshc_link_startup(ufs_params.reg_base);
		assert(result == 0);

		/* enable all interrupts */
		data = UFS_INT_UCCS | UFS_INT_UHES | UFS_INT_UHXS | UFS_INT_UPMS;
		data |= UFS_INT_UTRCS | UFS_INT_ERR;
		mmio_write_32(ufs_params.reg_base + IE, data);

		ufs_enum();

		ufs_get_device_info(&card);
		if (card.wmanufacturerid == UFS_VENDOR_SKHYNIX) {
			ufs_params.flags |= UFS_FLAGS_VENDOR_SKHYNIX;
		}

		ops->phy_set_pwr_mode(&ufs_params);
	}

	(void)result;
	return 0;
}
