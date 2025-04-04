/*
 * Copyright (c) 2023, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef RSE_PLATFORM_API_H
#define RSE_PLATFORM_API_H

#include <stdint.h>

#include "psa/error.h"
#if CRYPTO_SUPPORT
#include <rse_crypto_defs.h>
#endif

#define RSE_PLATFORM_API_ID_NV_READ       (1010)
#define RSE_PLATFORM_API_ID_NV_INCREMENT  (1011)

/*
 * Increments the given non-volatile (NV) counter by one
 *
 * counter_id	NV counter ID.
 *
 * PSA_SUCCESS if the value is read correctly. Otherwise,
 *	it returns a PSA_ERROR.
 */
psa_status_t
rse_platform_nv_counter_increment(uint32_t counter_id);

/*
 * Reads the given non-volatile (NV) counter
 *
 * counter_id	NV counter ID.
 * size		Size of the buffer to store NV counter value
 *			in bytes.
 * val		Pointer to store the current NV counter value.
 *
 * PSA_SUCCESS if the value is read correctly. Otherwise,
 *	it returns a PSA_ERROR.
 */
psa_status_t
rse_platform_nv_counter_read(uint32_t counter_id,
		uint32_t size, uint8_t *val);

#if CRYPTO_SUPPORT
/*
 * Reads the public key or the public part of a key pair in binary format.
 *
 * key		Identifier of the key to export.
 * data		Buffer where the key data is to be written.
 * data_size	Size of the data buffer in bytes.
 * data_length	On success, the number of bytes that make up the key data.
 *
 * PSA_SUCCESS if the value is read correctly. Otherwise,
 *	it returns a PSA_ERROR.
 */
psa_status_t
rse_platform_key_read(enum rse_key_id_builtin_t key, uint8_t *data,
		size_t data_size, size_t *data_length);

/*
 * Gets the entropy.
 *
 * data		Buffer where the entropy data is to be written.
 * data_size	Size of the data buffer in bytes.
 *
 * PSA_SUCCESS if the entropy is generated successfully. Otherwise,
 *	it returns a PSA_ERROR.
 */
psa_status_t
rse_platform_get_entropy(uint8_t *data, size_t data_size);
#endif

#endif /* RSE_PLATFORM_API_H */
