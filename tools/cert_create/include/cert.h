/*
 * Copyright (c) 2015-2022, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef CERT_H
#define CERT_H

#include <openssl/ossl_typ.h>
#include <openssl/x509.h>
#include "ext.h"
#include "key.h"

#define CERT_MAX_EXT			9

/*
 * This structure contains information related to the generation of the
 * certificates. All these fields must be known and specified at build time
 * except for the file name, which is picked up from the command line at
 * run time.
 *
 * One instance of this structure must be created for each of the certificates
 * present in the chain of trust.
 *
 * If the issuer points to this same instance, the generated certificate will
 * be self-signed.
 */
typedef struct cert_s cert_t;
struct cert_s {
	int id;			/* Unique identifier */

	const char *opt;	/* Command line option to pass filename */
	const char *fn;		/* Filename to save the certificate */
	const char *cn;		/* Subject CN (Company Name) */
	const char *help_msg;	/* Help message */

	/* These fields must be defined statically */
	int key;		/* Key to be signed */
	int issuer;		/* Issuer certificate */
	int ext[CERT_MAX_EXT];	/* Certificate extensions */
	int num_ext;		/* Number of extensions in the certificate */

	X509 *x;		/* X509 certificate container */
};

/* Exported API */
int cert_init(void);
cert_t *cert_get_by_opt(const char *opt);
int cert_add_ext(X509 *issuer, X509 *subject, int nid, char *value);
int cert_new(
	int md_alg,
	cert_t *cert,
	int days,
	int ca,
	STACK_OF(X509_EXTENSION) * sk);
void cert_cleanup(void);

/* Macro to register the certificates used in the CoT */
#define REGISTER_COT(_certs) \
	cert_t *def_certs = &_certs[0]; \
	const unsigned int num_def_certs = sizeof(_certs)/sizeof(_certs[0])

/* Macro to register the platform defined certificates used in the CoT */
#define PLAT_REGISTER_COT(_pdef_certs) \
	cert_t *pdef_certs = &_pdef_certs[0]; \
	const unsigned int num_pdef_certs = sizeof(_pdef_certs)/sizeof(_pdef_certs[0])

/* Exported variables */
extern cert_t *def_certs;
extern const unsigned int num_def_certs;
extern cert_t *pdef_certs;
extern const unsigned int num_pdef_certs;

extern cert_t *certs;
extern unsigned int num_certs;
#endif /* CERT_H */
