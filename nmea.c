/*
 * NMEA 0183 Parser Implementation
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include "nmea.h"
#include "log.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
#include <math.h>

/*
 * Internal helper functions
 */

/* Convert hex character to integer value */
static int hex_char_to_int(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* Split sentence into fields by comma delimiter.
 * Modifies sentence in-place. Returns number of fields. */
static int nmea_split_fields(char *sentence, char *fields[], int max_fields) {
    int count = 0;
    char *p = sentence;

    /* Skip leading $ or ! */
    if (*p == '$' || *p == '!') {
        p++;
    }

    while (*p && count < max_fields) {
        fields[count++] = p;

        /* Find next delimiter or end */
        while (*p && *p != ',' && *p != '*' && *p != '\r' && *p != '\n') {
            p++;
        }

        if (*p == ',') {
            *p++ = '\0';
        } else if (*p == '*' || *p == '\r' || *p == '\n') {
            *p = '\0';
            break;
        } else {
            break;
        }
    }

    return count;
}

/* Parse NMEA coordinate format (ddmm.mmmm or dddmm.mmmm) to decimal degrees.
 * dir should be 'N', 'S', 'E', or 'W'. */
static double nmea_parse_coord(const char *value, const char *dir) {
    if (!value || !dir || !*value || !*dir) {
        return 0.0;
    }

    double raw = atof(value);
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    double decimal_deg = degrees + (minutes / 60.0);

    if (*dir == 'S' || *dir == 'W') {
        decimal_deg = -decimal_deg;
    }

    // Truncate to 4 decimal places
    decimal_deg = floor(decimal_deg * 10000.0) / 10000.0;

    return decimal_deg;
}

/* Parse NMEA time format (hhmmss.ss) */
static bool nmea_parse_time_field(const char *field, nmea_time_t *time) {
    if (!field || !time || strlen(field) < 6) {
        return false;
    }

    char buf[3] = {0};

    /* Hours */
    buf[0] = field[0];
    buf[1] = field[1];
    time->hour = atoi(buf);

    /* Minutes */
    buf[0] = field[2];
    buf[1] = field[3];
    time->minute = atoi(buf);

    /* Seconds (including fractional part) */
    time->second = atof(&field[4]);

    time->valid = true;
    return true;
}

/* Parse NMEA date format (ddmmyy) */
static bool nmea_parse_date_field(const char *field, nmea_date_t *date) {
    if (!field || !date || strlen(field) < 6) {
        return false;
    }

    char buf[3] = {0};

    /* Day */
    buf[0] = field[0];
    buf[1] = field[1];
    date->day = atoi(buf);

    /* Month */
    buf[0] = field[2];
    buf[1] = field[3];
    date->month = atoi(buf);

    /* Year (2-digit, assume 20xx) */
    buf[0] = field[4];
    buf[1] = field[5];
    date->year = 2000 + atoi(buf);

    date->valid = true;
    return true;
}

/*
 * Buffer management functions
 */

void nmea_buffer_init(nmea_buffer_t *buf) {
    if (buf) {
        memset(buf->buffer, 0, NMEA_BUFFER_SIZE);
        buf->write_pos = 0;
    }
}

void nmea_buffer_log_contents(nmea_buffer_t *buf) {
    if (buf) {
        char output[NMEA_BUFFER_SIZE * 5];
        memset(output, 0, sizeof(output));
        char *p = output;
        for (int i = 0; i < buf->write_pos; i++) {
            snprintf(p, 4, "%02X ", (unsigned char)buf->buffer[i]);
            p += 3;
        }
        p[0] = ' ';
        p[1] = '|';
        p[2] = ' ';
        p += 3;
        for (int i = 0; i < buf->write_pos; i++) {
            if (isprint((unsigned char)buf->buffer[i])) {
                p[0] = buf->buffer[i];
            } else {
                p[0] = '.';
            }
            p ++;
        }
        LOG_DEBUG("NMEA", "Pos: %d, Contents: %s",  buf->write_pos, output);
    }
}

int nmea_buffer_add(nmea_buffer_t *buf, const char *data) {
    if (!buf || !data) {
        LOG_WARN("NMEA", "Invalid arguments to nmea_buffer_add");
        return 0;
    }

    int len = strlen(data);
    int space = NMEA_BUFFER_SIZE - buf->write_pos - 1;
    int to_copy = (len < space) ? len : space;

    if (to_copy > 0) {
        strncpy(&buf->buffer[buf->write_pos], data, to_copy);
        buf->write_pos += to_copy;
        buf->buffer[buf->write_pos] = '\0';
        //nmea_buffer_log_contents(buf);
    }

    return to_copy;
}

bool nmea_buffer_get_line(nmea_buffer_t *buf, char *line, int max_len) {
    if (!buf || !line || max_len <= 0) {
        LOG_WARN("NMEA", "Invalid arguments to nmea_buffer_get_line");
        return false;
    }

    /* Find newline */
    char *newline = strchr(buf->buffer, '\n');
    if (!newline) {
        return false;
    }

    int line_len = (newline - buf->buffer) + 1;

    /* Copy line (truncate if necessary) */
    int copy_len = (line_len < max_len - 1) ? line_len : max_len - 1;
    memcpy(line, buf->buffer, copy_len);
    line[copy_len] = '\0';

    /* Shift remaining data */
    int remaining = buf->write_pos - line_len;
    if (remaining > 0) {
        memmove(buf->buffer, newline + 1, remaining);
    }
    buf->write_pos = remaining;
    buf->buffer[buf->write_pos] = '\0';

    return true;
}

/*
 * Checksum validation
 */

bool nmea_validate_checksum(const char *sentence) {
    if (!sentence) {
        return false;
    }

    /* Find start delimiter */
    const char *start = strchr(sentence, '$');
    if (!start) {
        start = strchr(sentence, '!');
    }
    if (!start) {
        return false;
    }
    start++;  /* Skip the delimiter */

    /* Find checksum delimiter */
    const char *asterisk = strchr(start, '*');
    if (!asterisk) {
        /* No checksum present - consider valid per NMEA spec for optional checksum */
        return true;
    }

    /* Calculate XOR checksum */
    unsigned char calc_checksum = 0;
    for (const char *p = start; p < asterisk; p++) {
        calc_checksum ^= (unsigned char)*p;
    }

    /* Parse provided checksum (2 hex digits) */
    if (strlen(asterisk) < 3) {
        return false;
    }

    int high = hex_char_to_int(asterisk[1]);
    int low = hex_char_to_int(asterisk[2]);

    if (high < 0 || low < 0) {
        return false;
    }

    unsigned char provided_checksum = (high << 4) | low;

    return calc_checksum == provided_checksum;
}

/*
 * Sentence type detection
 */

bool nmea_is_gga(const char *sentence) {
    if (!sentence) {
        return false;
    }

    /* Find start delimiter */
    const char *p = strchr(sentence, '$');
    if (!p) {
        p = strchr(sentence, '!');
    }
    if (!p) {
        return false;
    }

    /* Check for GGA at position 3-5 (after 2-char talker ID) */
    if (strlen(p) >= 6) {
        return (p[3] == 'G' && p[4] == 'G' && p[5] == 'A');
    }

    return false;
}

bool nmea_is_rmc(const char *sentence) {
    if (!sentence) {
        return false;
    }

    /* Find start delimiter */
    const char *p = strchr(sentence, '$');
    if (!p) {
        p = strchr(sentence, '!');
    }
    if (!p) {
        return false;
    }

    /* Check for RMC at position 3-5 (after 2-char talker ID) */
    if (strlen(p) >= 6) {
        return (p[3] == 'R' && p[4] == 'M' && p[5] == 'C');
    }

    return false;
}

/*
 * GGA Parser
 *
 * Format: $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
 * Fields:
 *   0: Sentence ID (e.g., GPGGA, GNGGA)
 *   1: UTC time (hhmmss.ss)
 *   2: Latitude (llll.ll)
 *   3: N/S indicator
 *   4: Longitude (yyyyy.yy)
 *   5: E/W indicator
 *   6: Fix quality (0=invalid, 1=GPS, 2=DGPS)
 *   7: Number of satellites
 *   8: HDOP
 *   9: Altitude above MSL
 *  10: Altitude units (M)
 *  11: Geoidal separation
 *  12: Geoidal separation units (M)
 *  13: Age of differential GPS data
 *  14: Differential reference station ID
 */
bool nmea_parse_gga(const char *sentence, nmea_gga_t *gga) {
    if (!sentence || !gga) {
        return false;
    }

    /* Initialize output */
    memset(gga, 0, sizeof(*gga));
    gga->valid = false;

    /* Make a working copy */
    char work[NMEA_BUFFER_SIZE];
    strncpy(work, sentence, NMEA_BUFFER_SIZE - 1);
    work[NMEA_BUFFER_SIZE - 1] = '\0';

    /* Split into fields */
    char *fields[NMEA_MAX_FIELDS];
    int num_fields = nmea_split_fields(work, fields, NMEA_MAX_FIELDS);

    if (num_fields < 10) {
        return false;
    }

    /* Parse time (field 1) */
    if (strlen(fields[1]) >= 6) {
        nmea_parse_time_field(fields[1], &gga->time);
    }

    /* Parse latitude (fields 2-3) */
    if (strlen(fields[2]) > 0 && strlen(fields[3]) > 0) {
        gga->lat_deg = nmea_parse_coord(fields[2], fields[3]);
    }

    /* Parse longitude (fields 4-5) */
    if (strlen(fields[4]) > 0 && strlen(fields[5]) > 0) {
        gga->lon_deg = nmea_parse_coord(fields[4], fields[5]);
    }

    /* Parse fix quality (field 6) */
    if (strlen(fields[6]) > 0) {
        gga->fix_quality = atoi(fields[6]);
    }

    /* Parse number of satellites (field 7) */
    if (strlen(fields[7]) > 0) {
        gga->num_satellites = atoi(fields[7]);
    }

    /* Parse altitude (field 9) */
    if (strlen(fields[9]) > 0) {
        gga->alt_m = atof(fields[9]);
    }

    gga->valid = true;
    return true;
}

/*
 * RMC Parser
 *
 * Format: $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a,a*hh
 * Fields:
 *   0: Sentence ID (e.g., GPRMC, GNRMC)
 *   1: UTC time (hhmmss.ss)
 *   2: Status (A=active, V=void)
 *   3: Latitude (llll.ll)
 *   4: N/S indicator
 *   5: Longitude (yyyyy.yy)
 *   6: E/W indicator
 *   7: Speed over ground (knots)
 *   8: Course over ground (degrees)
 *   9: Date (ddmmyy)
 *  10: Magnetic variation
 *  11: Magnetic variation direction
 *  12: Mode indicator (optional)
 */
bool nmea_parse_rmc(const char *sentence, nmea_rmc_t *rmc) {
    if (!sentence || !rmc) {
        return false;
    }

    /* Initialize output */
    memset(rmc, 0, sizeof(*rmc));
    rmc->valid = false;

    /* Make a working copy */
    char work[NMEA_BUFFER_SIZE];
    strncpy(work, sentence, NMEA_BUFFER_SIZE - 1);
    work[NMEA_BUFFER_SIZE - 1] = '\0';

    /* Split into fields */
    char *fields[NMEA_MAX_FIELDS];
    int num_fields = nmea_split_fields(work, fields, NMEA_MAX_FIELDS);

    if (num_fields < 10) {
        return false;
    }

    /* Parse time (field 1) */
    if (strlen(fields[1]) >= 6) {
        nmea_parse_time_field(fields[1], &rmc->time);
    }

    /* Parse status (field 2) */
    if (strlen(fields[2]) > 0) {
        rmc->status = fields[2][0];
    }

    /* Parse latitude (fields 3-4) */
    if (strlen(fields[3]) > 0 && strlen(fields[4]) > 0) {
        rmc->lat_deg = nmea_parse_coord(fields[3], fields[4]);
    }

    /* Parse longitude (fields 5-6) */
    if (strlen(fields[5]) > 0 && strlen(fields[6]) > 0) {
        rmc->lon_deg = nmea_parse_coord(fields[5], fields[6]);
    }

    /* Parse date (field 9) */
    if (strlen(fields[9]) >= 6) {
        nmea_parse_date_field(fields[9], &rmc->date);
    }

    rmc->valid = true;
    return true;
}
