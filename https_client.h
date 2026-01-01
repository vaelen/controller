/*
 * HTTPS Client for RTEMS satellite tracking controller.
 *
 * Provides simple HTTPS GET functionality using OpenSSL.
 * Designed for embedded systems with streaming support to handle
 * large responses without excessive memory allocation.
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#ifndef HTTPS_CLIENT_H
#define HTTPS_CLIENT_H

#include <stdbool.h>
#include <stddef.h>

// ============================================================================
// Error Codes
// ============================================================================

typedef enum https_error {
    HTTPS_SUCCESS = 0,
    HTTPS_ERROR_NULL_POINTER = -1,
    HTTPS_ERROR_INVALID_URL = -2,
    HTTPS_ERROR_CONNECTION_FAILED = -3,
    HTTPS_ERROR_SSL_FAILED = -4,
    HTTPS_ERROR_HTTP_ERROR = -5,
    HTTPS_ERROR_TIMEOUT = -6,
    HTTPS_ERROR_BUFFER_TOO_SMALL = -7,
    HTTPS_ERROR_DNS_FAILED = -8,
    HTTPS_ERROR_WRITE_FAILED = -9,
    HTTPS_ERROR_READ_FAILED = -10
} https_error_t;

// ============================================================================
// Callback Types
// ============================================================================

/*
 * Callback function type for streaming response data.
 *
 * @param data      Pointer to received data chunk
 * @param len       Length of data chunk
 * @param user_data Opaque pointer passed to https_get_stream()
 * @return          true to continue reading, false to stop
 */
typedef bool (*https_data_callback_t)(const char *data, size_t len, void *user_data);

// ============================================================================
// Public Functions
// ============================================================================

/*
 * Perform an HTTPS GET request with streaming callback.
 *
 * The callback is invoked for each chunk of response data received.
 * This allows processing large responses without buffering the entire
 * response in memory.
 *
 * @param url           Full URL (e.g., "https://celestrak.org/...")
 * @param timeout_sec   Connection timeout in seconds (0 = default 30s)
 * @param callback      Function called for each chunk of response data
 * @param user_data     Opaque pointer passed to callback
 * @return              HTTPS_SUCCESS or error code
 */
https_error_t https_get_stream(const char *url, int timeout_sec,
                               https_data_callback_t callback, void *user_data);

/*
 * Get a human-readable error message for an HTTPS error code.
 *
 * @param err   Error code from https_get_stream()
 * @return      Static string describing the error
 */
const char *https_error_string(https_error_t err);

#endif /* HTTPS_CLIENT_H */
