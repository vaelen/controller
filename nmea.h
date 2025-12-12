/*
 * NMEA 0183 Parser
 *
 * Parses NMEA sentences for GPS data extraction.
 * Supports GGA (position) and RMC (date/time) sentences
 * from all GNSS constellations (GP, GN, GA, GB, GL, GQ).
 *
 * Copyright (c) 2025 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#ifndef NMEA_H
#define NMEA_H

#include <stdbool.h>

#define NMEA_BUFFER_SIZE    256    /* Max NMEA sentence is 82 chars */
#define NMEA_MAX_FIELDS     20

/* Receive buffer for accumulating serial data */
typedef struct {
    char buffer[NMEA_BUFFER_SIZE];
    int write_pos;
} nmea_buffer_t;

/* UTC time from NMEA sentence */
typedef struct {
    bool valid;
    int hour;
    int minute;
    double second;
} nmea_time_t;

/* Date from NMEA sentence */
typedef struct {
    bool valid;
    int year;      /* Full 4-digit year (e.g., 2025) */
    int month;     /* 1-12 */
    int day;       /* 1-31 */
} nmea_date_t;

/* Parsed GGA sentence data */
typedef struct {
    bool valid;
    double lat_deg;       /* Latitude in degrees (+ = N, - = S) */
    double lon_deg;       /* Longitude in degrees (+ = E, - = W) */
    double alt_m;         /* Altitude in meters above MSL */
    int fix_quality;      /* 0=invalid, 1=GPS, 2=DGPS */
    int num_satellites;
    nmea_time_t time;
} nmea_gga_t;

/* Parsed RMC sentence data */
typedef struct {
    bool valid;
    nmea_date_t date;
    nmea_time_t time;
    double lat_deg;       /* Latitude in degrees (+ = N, - = S) */
    double lon_deg;       /* Longitude in degrees (+ = E, - = W) */
    char status;          /* 'A'=active, 'V'=void */
} nmea_rmc_t;

/*
 * Buffer management functions
 */

/* Initialize buffer to empty state */
void nmea_buffer_init(nmea_buffer_t *buf);

/* Add data to buffer. Returns number of bytes added. */
int nmea_buffer_add(nmea_buffer_t *buf, const char *data, int len);

/* Extract a complete line from buffer (ending with \n).
 * Returns true if a line was extracted, false if no complete line available.
 * The extracted line is null-terminated and includes the \n. */
bool nmea_buffer_get_line(nmea_buffer_t *buf, char *line, int max_len);

/*
 * Sentence validation and parsing functions
 */

/* Validate NMEA checksum. Returns true if valid or no checksum present. */
bool nmea_validate_checksum(const char *sentence);

/* Check if sentence is a GGA sentence (any talker ID) */
bool nmea_is_gga(const char *sentence);

/* Check if sentence is an RMC sentence (any talker ID) */
bool nmea_is_rmc(const char *sentence);

/* Parse GGA sentence. Returns true on success. */
bool nmea_parse_gga(const char *sentence, nmea_gga_t *gga);

/* Parse RMC sentence. Returns true on success. */
bool nmea_parse_rmc(const char *sentence, nmea_rmc_t *rmc);

#endif /* NMEA_H */
