/*
 * HTTPS Client implementation using OpenSSL.
 *
 * Provides simple HTTPS GET functionality for embedded RTEMS systems.
 * Uses OpenSSL's BIO and SSL APIs for TLS connections.
 *
 * Copyright (c) 2025 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include "https_client.h"
#include "log.h"

#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>

#include <openssl/ssl.h>
#include <openssl/err.h>
#include <openssl/bio.h>

#define LOG_TAG "HTTPS"
#define DEFAULT_TIMEOUT 30
#define READ_BUFFER_SIZE 4096
#define DEFAULT_HTTPS_PORT 443
#define DEFAULT_HTTP_PORT 80

// ============================================================================
// Internal Helper Functions
// ============================================================================

/*
 * Parse URL into host, port, and path components.
 * Returns 0 on success, -1 on error.
 */
static int parse_url(const char *url, char *host, size_t host_size,
                     int *port, char *path, size_t path_size, int *use_ssl)
{
    const char *p = url;

    /* Check scheme */
    if (strncmp(p, "https://", 8) == 0) {
        *use_ssl = 1;
        *port = DEFAULT_HTTPS_PORT;
        p += 8;
    } else if (strncmp(p, "http://", 7) == 0) {
        *use_ssl = 0;
        *port = DEFAULT_HTTP_PORT;
        p += 7;
    } else {
        return -1;
    }

    /* Extract host */
    const char *host_end = p;
    while (*host_end && *host_end != ':' && *host_end != '/') {
        host_end++;
    }

    size_t host_len = host_end - p;
    if (host_len == 0 || host_len >= host_size) {
        return -1;
    }
    memcpy(host, p, host_len);
    host[host_len] = '\0';

    p = host_end;

    /* Check for port */
    if (*p == ':') {
        p++;
        char *end;
        long port_val = strtol(p, &end, 10);
        if (port_val <= 0 || port_val > 65535) {
            return -1;
        }
        *port = (int)port_val;
        p = end;
    }

    /* Extract path */
    if (*p == '\0') {
        strncpy(path, "/", path_size - 1);
        path[path_size - 1] = '\0';
    } else {
        strncpy(path, p, path_size - 1);
        path[path_size - 1] = '\0';
    }

    return 0;
}

/*
 * Connect to host:port and return socket fd.
 * Returns -1 on error.
 */
static int connect_to_host(const char *host, int port, int timeout_sec)
{
    struct addrinfo hints, *result, *rp;
    int sockfd = -1;
    char port_str[16];

    snprintf(port_str, sizeof(port_str), "%d", port);

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    int err = getaddrinfo(host, port_str, &hints, &result);
    if (err != 0) {
        LOG_ERROR(LOG_TAG, "DNS lookup failed for %s: %s", host, gai_strerror(err));
        return -1;
    }

    for (rp = result; rp != NULL; rp = rp->ai_next) {
        sockfd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (sockfd == -1) {
            continue;
        }

        /* Set socket timeout */
        struct timeval tv;
        tv.tv_sec = timeout_sec;
        tv.tv_usec = 0;
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        if (connect(sockfd, rp->ai_addr, rp->ai_addrlen) == 0) {
            break;  /* Success */
        }

        close(sockfd);
        sockfd = -1;
    }

    freeaddrinfo(result);

    if (sockfd == -1) {
        LOG_ERROR(LOG_TAG, "Failed to connect to %s:%d", host, port);
    }

    return sockfd;
}

/*
 * Send HTTP GET request.
 */
static int send_http_request(BIO *bio, const char *host, const char *path)
{
    char request[1024];
    int len = snprintf(request, sizeof(request),
                       "GET %s HTTP/1.1\r\n"
                       "Host: %s\r\n"
                       "User-Agent: SatTrackController/1.0\r\n"
                       "Accept: */*\r\n"
                       "Connection: close\r\n"
                       "\r\n",
                       path, host);

    if (len >= (int)sizeof(request)) {
        LOG_ERROR(LOG_TAG, "Request too large");
        return -1;
    }

    int written = BIO_write(bio, request, len);
    if (written != len) {
        LOG_ERROR(LOG_TAG, "Failed to send request");
        return -1;
    }

    return 0;
}

/*
 * Skip HTTP headers and return pointer to body start.
 * Returns number of body bytes in buffer, or -1 on error.
 */
static int skip_http_headers(char *buffer, int buffer_len, int *http_status)
{
    *http_status = 0;

    /* Parse status line */
    if (strncmp(buffer, "HTTP/1.", 7) != 0) {
        LOG_ERROR(LOG_TAG, "Invalid HTTP response");
        return -1;
    }

    /* Extract status code */
    const char *status_start = buffer + 9;
    *http_status = atoi(status_start);

    /* Find end of headers */
    char *body = strstr(buffer, "\r\n\r\n");
    if (body == NULL) {
        /* Headers not complete in buffer */
        return 0;
    }

    body += 4;  /* Skip \r\n\r\n */
    int header_len = body - buffer;
    int body_len = buffer_len - header_len;

    /* Move body to start of buffer */
    if (body_len > 0) {
        memmove(buffer, body, body_len);
    }

    return body_len;
}

// ============================================================================
// Public Functions
// ============================================================================

https_error_t https_get_stream(const char *url, int timeout_sec,
                               https_data_callback_t callback, void *user_data)
{
    if (url == NULL || callback == NULL) {
        return HTTPS_ERROR_NULL_POINTER;
    }

    if (timeout_sec <= 0) {
        timeout_sec = DEFAULT_TIMEOUT;
    }

    https_error_t result = HTTPS_SUCCESS;
    char host[256];
    char path[512];
    int port;
    int use_ssl;
    int sockfd = -1;
    SSL_CTX *ssl_ctx = NULL;
    SSL *ssl = NULL;
    BIO *bio = NULL;

    /* Parse URL */
    if (parse_url(url, host, sizeof(host), &port, path, sizeof(path), &use_ssl) != 0) {
        LOG_ERROR(LOG_TAG, "Invalid URL: %s", url);
        return HTTPS_ERROR_INVALID_URL;
    }

    LOG_DEBUG(LOG_TAG, "Connecting to %s:%d%s (SSL=%d)", host, port, path, use_ssl);

    /* Connect to host */
    sockfd = connect_to_host(host, port, timeout_sec);
    if (sockfd < 0) {
        return HTTPS_ERROR_CONNECTION_FAILED;
    }

    /* Create BIO from socket */
    bio = BIO_new_socket(sockfd, BIO_CLOSE);
    if (bio == NULL) {
        LOG_ERROR(LOG_TAG, "Failed to create socket BIO");
        close(sockfd);
        return HTTPS_ERROR_CONNECTION_FAILED;
    }

    /* Setup SSL if needed */
    if (use_ssl) {
        ssl_ctx = SSL_CTX_new(TLS_client_method());
        if (ssl_ctx == NULL) {
            LOG_ERROR(LOG_TAG, "Failed to create SSL context");
            result = HTTPS_ERROR_SSL_FAILED;
            goto cleanup;
        }

        /* Skip certificate verification (embedded system, no CA store) */
        SSL_CTX_set_verify(ssl_ctx, SSL_VERIFY_NONE, NULL);

        ssl = SSL_new(ssl_ctx);
        if (ssl == NULL) {
            LOG_ERROR(LOG_TAG, "Failed to create SSL object");
            result = HTTPS_ERROR_SSL_FAILED;
            goto cleanup;
        }

        /* Set hostname for SNI */
        SSL_set_tlsext_host_name(ssl, host);

        /* Wrap socket BIO with SSL */
        SSL_set_bio(ssl, bio, bio);
        bio = NULL;  /* SSL now owns the BIO */

        /* Perform SSL handshake */
        if (SSL_connect(ssl) <= 0) {
            unsigned long err = ERR_get_error();
            char err_buf[256];
            ERR_error_string_n(err, err_buf, sizeof(err_buf));
            LOG_ERROR(LOG_TAG, "SSL handshake failed: %s", err_buf);
            result = HTTPS_ERROR_SSL_FAILED;
            goto cleanup;
        }

        LOG_DEBUG(LOG_TAG, "SSL connection established: %s", SSL_get_cipher(ssl));

        /* Create SSL BIO for I/O */
        bio = BIO_new(BIO_f_ssl());
        BIO_set_ssl(bio, ssl, BIO_NOCLOSE);
    }

    /* Send HTTP request */
    if (send_http_request(bio, host, path) != 0) {
        result = HTTPS_ERROR_WRITE_FAILED;
        goto cleanup;
    }

    /* Read response */
    char buffer[READ_BUFFER_SIZE];
    int total_read = 0;
    int headers_skipped = 0;
    int http_status = 0;

    while (1) {
        int bytes_read = BIO_read(bio, buffer + total_read,
                                  sizeof(buffer) - total_read - 1);
        if (bytes_read <= 0) {
            if (BIO_should_retry(bio)) {
                continue;
            }
            break;  /* EOF or error */
        }

        total_read += bytes_read;
        buffer[total_read] = '\0';

        /* Skip headers on first read */
        if (!headers_skipped) {
            int body_len = skip_http_headers(buffer, total_read, &http_status);
            if (body_len < 0) {
                result = HTTPS_ERROR_HTTP_ERROR;
                goto cleanup;
            }
            if (body_len == 0) {
                /* Need more data for headers */
                continue;
            }

            headers_skipped = 1;
            total_read = body_len;

            /* Check HTTP status */
            if (http_status < 200 || http_status >= 300) {
                LOG_ERROR(LOG_TAG, "HTTP error: %d", http_status);
                result = HTTPS_ERROR_HTTP_ERROR;
                goto cleanup;
            }

            LOG_DEBUG(LOG_TAG, "HTTP status: %d", http_status);
        }

        /* Pass data to callback */
        if (total_read > 0) {
            if (!callback(buffer, total_read, user_data)) {
                LOG_DEBUG(LOG_TAG, "Callback requested stop");
                break;
            }
            total_read = 0;
        }
    }

cleanup:
    if (bio != NULL) {
        BIO_free_all(bio);
    }
    if (ssl != NULL) {
        SSL_shutdown(ssl);
        SSL_free(ssl);
    }
    if (ssl_ctx != NULL) {
        SSL_CTX_free(ssl_ctx);
    }

    return result;
}

const char *https_error_string(https_error_t err)
{
    switch (err) {
        case HTTPS_SUCCESS:             return "Success";
        case HTTPS_ERROR_NULL_POINTER:  return "Null pointer";
        case HTTPS_ERROR_INVALID_URL:   return "Invalid URL";
        case HTTPS_ERROR_CONNECTION_FAILED: return "Connection failed";
        case HTTPS_ERROR_SSL_FAILED:    return "SSL/TLS error";
        case HTTPS_ERROR_HTTP_ERROR:    return "HTTP error";
        case HTTPS_ERROR_TIMEOUT:       return "Timeout";
        case HTTPS_ERROR_BUFFER_TOO_SMALL: return "Buffer too small";
        case HTTPS_ERROR_DNS_FAILED:    return "DNS lookup failed";
        case HTTPS_ERROR_WRITE_FAILED:  return "Write failed";
        case HTTPS_ERROR_READ_FAILED:   return "Read failed";
        default:                        return "Unknown error";
    }
}
