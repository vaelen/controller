/*
 * OpenSSL stub functions for RTEMS.
 *
 * Provides missing OpenSSL functions that are referenced but not
 * implemented in the rtems-libbsd OpenSSL port.
 *
 * Copyright (c) 2025 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include <openssl/bio.h>

/*
 * BIO_f_prefix - Create a prefix BIO filter.
 *
 * This function is referenced by OpenSSL's ASN.1 parsing code but is not
 * implemented in the rtems-libbsd OpenSSL port. We provide a stub that
 * returns NULL to indicate the feature is not available.
 */
const BIO_METHOD *BIO_f_prefix(void)
{
    return NULL;
}
