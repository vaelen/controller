/*
 * Pass Management Tasks
 *
 * Tasks for TLE database management, pass prediction, and pass execution.
 * Includes the TLE updater, pass calculator, and pass executor tasks.
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include <rtems.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "pass.h"
#include "shared.h"
#include "sgp4.h"
#include "https_client.h"
#include "log.h"

// ============================================================================
// TLE Stream Parser for Download Filtering
// ============================================================================

/*
 * Parser states for streaming TLE data.
 */
typedef enum tle_parser_state {
    TLE_STATE_NAME = 0,   /* Expecting name line (line 0) */
    TLE_STATE_LINE1 = 1,  /* Expecting TLE line 1 */
    TLE_STATE_LINE2 = 2   /* Expecting TLE line 2 */
} tle_parser_state_t;

/*
 * TLE stream parser context for filtering during download.
 */
typedef struct tle_stream_parser {
    char line_buf[256];              /* Buffer for accumulating current line */
    int line_len;                    /* Current line length */
    char name_line[SGP4_TLE_NAME_LEN]; /* Satellite name */
    char line1[SGP4_TLE_LINE_LEN];   /* TLE line 1 */
    char line2[SGP4_TLE_LINE_LEN];   /* TLE line 2 */
    tle_parser_state_t state;        /* Current parser state */
    const int *norad_ids;            /* NORAD IDs to filter (NULL = accept all) */
    int norad_count;                 /* Number of NORAD IDs in filter */
    FILE *output_file;               /* File to write filtered TLE data */
    int total_parsed;                /* Total TLE entries parsed */
    int matches_found;               /* Number of matching satellites */
    int max_matches;                 /* Maximum matches to accept */
} tle_stream_parser_t;

/*
 * Initialize TLE stream parser.
 */
static void tle_parser_init(tle_stream_parser_t *parser, const int *norad_ids,
                            int norad_count, FILE *output_file, int max_matches)
{
    memset(parser, 0, sizeof(*parser));
    parser->norad_ids = norad_ids;
    parser->norad_count = norad_count;
    parser->output_file = output_file;
    parser->max_matches = max_matches;
    parser->state = TLE_STATE_NAME;
}

/*
 * Check if NORAD ID is in the filter list.
 * Returns true if the ID should be accepted.
 */
static bool tle_parser_should_accept(const tle_stream_parser_t *parser, int norad_id)
{
    /* If no filter list, accept based on max_matches limit */
    if (parser->norad_ids == NULL || parser->norad_count == 0) {
        return parser->matches_found < parser->max_matches;
    }

    /* Check if ID is in the filter list */
    for (int i = 0; i < parser->norad_count; i++) {
        if (parser->norad_ids[i] == norad_id) {
            return true;
        }
    }
    return false;
}

/*
 * Extract NORAD ID from TLE line 1.
 * Line 1 format: "1 NNNNN..." where NNNNN is the 5-digit NORAD ID at columns 3-7.
 */
static int tle_parser_extract_norad_id(const char *line1)
{
    if (strlen(line1) < 7) {
        return -1;
    }
    if (line1[0] != '1') {
        return -1;
    }

    /* Extract NORAD ID from columns 3-7 (0-indexed: 2-6) */
    char id_str[6];
    memcpy(id_str, &line1[2], 5);
    id_str[5] = '\0';

    return atoi(id_str);
}

/*
 * Process a complete TLE entry (name, line1, line2).
 * Returns true if the entry was accepted and written.
 */
static bool tle_parser_process_entry(tle_stream_parser_t *parser)
{
    parser->total_parsed++;

    /* Extract NORAD ID from line 1 */
    int norad_id = tle_parser_extract_norad_id(parser->line1);
    if (norad_id < 0) {
        LOG_DEBUG("TLE", "Invalid TLE line 1: %s", parser->line1);
        return false;
    }

    /* Check if we should accept this satellite */
    if (!tle_parser_should_accept(parser, norad_id)) {
        return false;
    }

    /* Write to output file */
    if (parser->output_file != NULL) {
        fprintf(parser->output_file, "%s\n%s\n%s\n",
                parser->name_line, parser->line1, parser->line2);
    }

    parser->matches_found++;
    LOG_DEBUG("TLE", "Accepted satellite: %s (NORAD %d), %d/%d",
              parser->name_line, norad_id, parser->matches_found, parser->max_matches);

    return true;
}

/*
 * Process a single line from the TLE stream.
 * Called for each complete line received.
 */
static void tle_parser_process_line(tle_stream_parser_t *parser, const char *line)
{
    /* Skip empty lines */
    size_t len = strlen(line);
    if (len == 0) {
        return;
    }

    /* Determine line type based on first character */
    if (line[0] == '1' && len >= 69) {
        /* This is TLE line 1 */
        if (parser->state == TLE_STATE_NAME) {
            /* No name line before line 1, use empty name */
            parser->name_line[0] = '\0';
        }
        strncpy(parser->line1, line, SGP4_TLE_LINE_LEN - 1);
        parser->line1[SGP4_TLE_LINE_LEN - 1] = '\0';
        parser->state = TLE_STATE_LINE2;
    }
    else if (line[0] == '2' && len >= 69) {
        /* This is TLE line 2 */
        if (parser->state == TLE_STATE_LINE2) {
            strncpy(parser->line2, line, SGP4_TLE_LINE_LEN - 1);
            parser->line2[SGP4_TLE_LINE_LEN - 1] = '\0';

            /* Process complete TLE entry */
            tle_parser_process_entry(parser);
        }
        parser->state = TLE_STATE_NAME;
    }
    else {
        /* This is a name line */
        strncpy(parser->name_line, line, SGP4_TLE_NAME_LEN - 1);
        parser->name_line[SGP4_TLE_NAME_LEN - 1] = '\0';

        /* Trim trailing whitespace from name */
        int name_len = strlen(parser->name_line);
        while (name_len > 0 && (parser->name_line[name_len - 1] == ' ' ||
                                 parser->name_line[name_len - 1] == '\r' ||
                                 parser->name_line[name_len - 1] == '\n')) {
            parser->name_line[--name_len] = '\0';
        }

        parser->state = TLE_STATE_LINE1;
    }
}

/*
 * Feed data chunk to parser.
 * Handles partial lines across chunks.
 */
static void tle_parser_feed(tle_stream_parser_t *parser, const char *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        char c = data[i];

        if (c == '\n') {
            /* End of line - process it */
            parser->line_buf[parser->line_len] = '\0';

            /* Strip trailing CR if present */
            if (parser->line_len > 0 && parser->line_buf[parser->line_len - 1] == '\r') {
                parser->line_buf[parser->line_len - 1] = '\0';
            }

            tle_parser_process_line(parser, parser->line_buf);

            parser->line_len = 0;

            /* Stop if we've reached max matches */
            if (parser->matches_found >= parser->max_matches) {
                return;
            }
        }
        else if (parser->line_len < (int)sizeof(parser->line_buf) - 1) {
            /* Add character to line buffer */
            parser->line_buf[parser->line_len++] = c;
        }
    }
}

/*
 * HTTPS callback for TLE download.
 * Parses incoming data and filters satellites.
 */
static bool tle_download_callback(const char *data, size_t len, void *user_data)
{
    tle_stream_parser_t *parser = (tle_stream_parser_t *)user_data;

    tle_parser_feed(parser, data, len);

    /* Continue reading unless we've found enough matches */
    return parser->matches_found < parser->max_matches;
}

// ============================================================================
// TLE Database Loading
// ============================================================================

/*
 * Load TLE database from file into global state.
 * Must be called with g_tle_database_mutex held.
 */
static int load_tle_database(const char *filepath)
{
    FILE *file = fopen(filepath, "r");
    if (file == NULL) {
        LOG_DEBUG("TLE", "Cannot open TLE file: %s", filepath);
        return -1;
    }

    int count = 0;
    sgp4_tle_t tle;
    sgp4_error_t err;

    while ((err = sgp4_read_tle_stream(file, &tle)) == SGP4_SUCCESS && count < MAX_SATELLITES) {
        /* Convert TLE to SGP4 elements and initialize state */
        sgp4_elements_t elements;
        err = sgp4_tle_to_elements(&tle, &elements);
        if (err != SGP4_SUCCESS) {
            LOG_WARN("TLE", "Failed to convert TLE for %s: %d", tle.name, err);
            continue;
        }

        sgp4_state_t state;
        err = sgp4_init(&state, &elements);
        if (err != SGP4_SUCCESS) {
            LOG_WARN("TLE", "Failed to init SGP4 for %s: %d", tle.name, err);
            continue;
        }

        /* Store in database */
        g_state.satellites[count].valid = true;
        g_state.satellites[count].tle = tle;
        g_state.satellites[count].state = state;
        count++;

        LOG_DEBUG("TLE", "Loaded: %s (NORAD %d)", tle.name, tle.norad_id);
    }

    fclose(file);

    g_state.satellite_count = count;
    LOG_INFO("TLE", "Loaded %d satellites from %s", count, filepath);

    return count;
}

const char *find_satellite_name(int norad_id)
{
    for (int i = 0; i < g_state.satellite_count; i++) {
        if (g_state.satellites[i].valid &&
            g_state.satellites[i].tle.norad_id == norad_id) {
            return g_state.satellites[i].tle.name;
        }
    }
    return NULL;
}

/*
 * Check if network is available (has a valid IP address).
 */
static bool network_is_available(const char *ifname)
{
    char ipv4_addr[CONFIG_IPV4_ADDR_MAX];
    char ipv6_addr[CONFIG_IPV6_ADDR_MAX];

    network_get_addresses(ifname, ipv4_addr, sizeof(ipv4_addr),
                          ipv6_addr, sizeof(ipv6_addr));

    return (ipv4_addr[0] != '\0' || ipv6_addr[0] != '\0');
}

/*
 * Download TLE data from URL and save filtered results to file.
 * Returns number of satellites saved, or -1 on error.
 */
static int download_tle_data(const char *url, const int *norad_ids, int norad_count,
                             const char *output_path, int max_satellites)
{
    /* Open output file */
    FILE *output = fopen(output_path, "w");
    if (output == NULL) {
        LOG_ERROR("TLE", "Failed to create temp file: %s", output_path);
        return -1;
    }

    /* Initialize parser - static to reduce stack usage */
    static tle_stream_parser_t parser;
    tle_parser_init(&parser, norad_ids, norad_count, output, max_satellites);

    /* Download and parse */
    LOG_INFO("TLE", "Downloading TLE from: %s", url);

    https_error_t err = https_get_stream(url, 60, tle_download_callback, &parser);

    fclose(output);

    if (err != HTTPS_SUCCESS) {
        LOG_ERROR("TLE", "Download failed: %s", https_error_string(err));
        unlink(output_path);
        return -1;
    }

    LOG_INFO("TLE", "Download complete: parsed %d, matched %d satellites",
             parser.total_parsed, parser.matches_found);

    return parser.matches_found;
}

rtems_task tle_updater_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("TLE", "Updater task started");

    /* Get initial configuration - static to reduce stack usage */
    static config_t cfg;
    config_get_copy(&cfg);

    /* Try to load existing TLE from file on startup */
    rtems_semaphore_obtain(g_tle_database_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    int loaded = load_tle_database(TLE_FILE_PATH);
    rtems_semaphore_release(g_tle_database_mutex);

    if (loaded > 0) {
        /* Notify controller that TLE database is ready */
        tle_update_message_t msg;
        msg.type = MSG_TLE_DATABASE_UPDATED;
        rtems_message_queue_send(g_tle_queue, &msg, sizeof(msg));
    }

    /* Main update loop */
    while (true) {
        /* Refresh configuration */
        config_get_copy(&cfg);

        /* Check if networking is enabled */
        if (!cfg.network.enabled) {
            LOG_DEBUG("TLE", "Network disabled, skipping update");
            rtems_task_wake_after(cfg.tle_update_interval_hours * 60 * 60 *
                                  rtems_clock_get_ticks_per_second());
            continue;
        }

        /* Wait for network to be available */
        int retries = 0;
        while (!network_is_available(cfg.network.interface) && retries < 12) {
            LOG_DEBUG("TLE", "Waiting for network...");
            rtems_task_wake_after(5 * rtems_clock_get_ticks_per_second());
            retries++;
        }

        if (!network_is_available(cfg.network.interface)) {
            LOG_WARN("TLE", "Network not available after 60s, will retry later");
            rtems_task_wake_after(5 * 60 * rtems_clock_get_ticks_per_second());
            continue;
        }

        /* Download TLE data to temp file */
        int downloaded = download_tle_data(
            cfg.tle_url,
            cfg.satellite_count > 0 ? cfg.satellite_norad_ids : NULL,
            cfg.satellite_count,
            TLE_TEMP_FILE_PATH,
            MAX_SATELLITES
        );

        if (downloaded > 0) {
            /* Remove existing file first (required for FAT filesystems) */
            unlink(TLE_FILE_PATH);

            /* Rename temp file to final location */
            if (rename(TLE_TEMP_FILE_PATH, TLE_FILE_PATH) != 0) {
                LOG_ERROR("TLE", "Failed to rename temp file: %s", strerror(errno));
                unlink(TLE_TEMP_FILE_PATH);
            } else {
                /* Load new TLE data into memory */
                rtems_semaphore_obtain(g_tle_database_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
                load_tle_database(TLE_FILE_PATH);
                rtems_semaphore_release(g_tle_database_mutex);

                /* Notify controller */
                tle_update_message_t msg;
                msg.type = MSG_TLE_DATABASE_UPDATED;
                rtems_message_queue_send(g_tle_queue, &msg, sizeof(msg));
            }
        } else if (downloaded == 0) {
            LOG_WARN("TLE", "No matching satellites found in download");
            unlink(TLE_TEMP_FILE_PATH);
        } else {
            /* Download failed, keep existing data */
            LOG_WARN("TLE", "Download failed, keeping existing TLE data");
        }

        /* Wait for next update interval */
        rtems_task_wake_after(cfg.tle_update_interval_hours * 60 * 60 *
                              rtems_clock_get_ticks_per_second());
    }
}

// ============================================================================
// Pass Calculator Helper Functions
// ============================================================================

/* Time step for coarse pass scanning (seconds) */
#define PASS_SCAN_STEP_SEC 60

/* Buffer added to prediction window (minutes) */
#define PASS_WINDOW_BUFFER_MIN 5

/* Binary search precision (seconds) */
#define PASS_REFINE_PRECISION_SEC 1.0

/*
 * Calculate look angles at a specific time.
 * Returns true if calculation was successful.
 */
static bool calculate_look_angles_at_time(
    const sgp4_state_t *sat_state,
    const sgp4_geodetic_t *observer,
    double jd,
    sgp4_look_angles_t *angles)
{
    /* Calculate minutes since TLE epoch */
    double epoch_jd = sat_state->jdsatepoch + sat_state->jdsatepochF;
    double tsince_min = (jd - epoch_jd) * 1440.0;  /* days to minutes */

    /* Propagate satellite position */
    sgp4_result_t result;
    sgp4_error_t err = sgp4_propagate(sat_state, tsince_min, &result);
    if (err != SGP4_SUCCESS) {
        return false;
    }

    /* Convert ECI to ECEF */
    double gst = sgp4_gstime(jd);
    sgp4_vec3_t eci = { result.r[0], result.r[1], result.r[2] };
    sgp4_vec3_t ecef;
    sgp4_eci_to_ecef(&eci, gst, &ecef);

    /* Calculate look angles */
    sgp4_look_angles(&ecef, observer, angles);

    return true;
}

/*
 * Refine a visibility transition (AOS or LOS) using binary search.
 */
static double refine_transition(
    const sgp4_state_t *sat_state,
    const sgp4_geodetic_t *observer,
    double t_before,
    double t_after,
    double min_el_rad,
    bool looking_for_aos)
{
    double precision_jd = PASS_REFINE_PRECISION_SEC / 86400.0;

    while ((t_after - t_before) > precision_jd) {
        double t_mid = (t_before + t_after) / 2.0;

        sgp4_look_angles_t angles;
        if (!calculate_look_angles_at_time(sat_state, observer, t_mid, &angles)) {
            break;  /* Propagation error - return best estimate */
        }

        bool is_visible = angles.elevation_rad >= min_el_rad;

        if (looking_for_aos) {
            if (is_visible) {
                t_after = t_mid;
            } else {
                t_before = t_mid;
            }
        } else {  /* looking for LOS */
            if (is_visible) {
                t_before = t_mid;
            } else {
                t_after = t_mid;
            }
        }
    }

    return (t_before + t_after) / 2.0;
}

/*
 * Find maximum elevation during a pass using ternary search.
 */
static bool find_max_elevation(
    const sgp4_state_t *sat_state,
    const sgp4_geodetic_t *observer,
    double aos_jd,
    double los_jd,
    double *max_el_out)
{
    double low = aos_jd;
    double high = los_jd;
    double precision_jd = 1.0 / 86400.0;  /* 1 second */

    while ((high - low) > precision_jd) {
        double mid1 = low + (high - low) / 3.0;
        double mid2 = high - (high - low) / 3.0;

        sgp4_look_angles_t angles1, angles2;
        if (!calculate_look_angles_at_time(sat_state, observer, mid1, &angles1) ||
            !calculate_look_angles_at_time(sat_state, observer, mid2, &angles2)) {
            return false;
        }

        if (angles1.elevation_rad < angles2.elevation_rad) {
            low = mid1;
        } else {
            high = mid2;
        }
    }

    /* Get final elevation at peak */
    sgp4_look_angles_t angles;
    if (!calculate_look_angles_at_time(sat_state, observer, (low + high) / 2.0, &angles)) {
        return false;
    }

    *max_el_out = angles.elevation_rad;
    return true;
}

/*
 * Find the next satellite pass within the given time window.
 */
static bool find_next_pass(
    const sgp4_state_t *sat_state,
    int norad_id,
    const sgp4_geodetic_t *observer,
    double start_jd,
    double end_jd,
    double min_el_rad,
    pass_info_t *pass_out)
{
    double step_jd = PASS_SCAN_STEP_SEC / 86400.0;

    /* Check if satellite is currently visible (skip in-progress passes) */
    sgp4_look_angles_t initial_angles;
    if (calculate_look_angles_at_time(sat_state, observer, start_jd, &initial_angles)) {
        if (initial_angles.elevation_rad >= min_el_rad) {
            /* Satellite is already visible - skip to find next pass */
            /* Scan forward to find when it sets */
            double t = start_jd;
            bool was_visible = true;
            while (t < end_jd && was_visible) {
                t += step_jd;
                sgp4_look_angles_t angles;
                if (calculate_look_angles_at_time(sat_state, observer, t, &angles)) {
                    was_visible = angles.elevation_rad >= min_el_rad;
                } else {
                    break;
                }
            }
            /* Now start searching from when it set */
            start_jd = t;
        }
    }

    /* Coarse scan for AOS */
    bool prev_visible = false;
    double aos_approx = 0.0;
    double los_approx = 0.0;
    bool found_aos = false;

    for (double t = start_jd; t < end_jd; t += step_jd) {
        sgp4_look_angles_t angles;
        if (!calculate_look_angles_at_time(sat_state, observer, t, &angles)) {
            continue;  /* Propagation failed - skip this time point */
        }

        bool curr_visible = angles.elevation_rad >= min_el_rad;

        if (!prev_visible && curr_visible) {
            /* Found approximate AOS */
            aos_approx = t;
            found_aos = true;
        } else if (prev_visible && !curr_visible && found_aos) {
            /* Found approximate LOS */
            los_approx = t;

            /* Refine AOS */
            double aos_refined = refine_transition(
                sat_state, observer,
                aos_approx - step_jd, aos_approx,
                min_el_rad, true);

            /* Refine LOS */
            double los_refined = refine_transition(
                sat_state, observer,
                los_approx - step_jd, los_approx,
                min_el_rad, false);

            /* Find max elevation */
            double max_el;
            if (!find_max_elevation(sat_state, observer, aos_refined, los_refined, &max_el)) {
                max_el = 0.1;  /* Default if calculation fails */
            }

            /* Get AOS and LOS azimuths */
            sgp4_look_angles_t aos_angles, los_angles;
            calculate_look_angles_at_time(sat_state, observer, aos_refined, &aos_angles);
            calculate_look_angles_at_time(sat_state, observer, los_refined, &los_angles);

            /* Populate pass info */
            pass_out->norad_id = norad_id;
            pass_out->aos_jd = aos_refined;
            pass_out->los_jd = los_refined;
            pass_out->max_elevation_rad = max_el;
            pass_out->aos_azimuth_rad = aos_angles.azimuth_rad;
            pass_out->los_azimuth_rad = los_angles.azimuth_rad;

            return true;
        }

        prev_visible = curr_visible;
    }

    /* Check if pass extends beyond window */
    if (found_aos && prev_visible) {
        /* Pass started but didn't end within window - refine AOS and use window end */
        double aos_refined = refine_transition(
            sat_state, observer,
            aos_approx - step_jd, aos_approx,
            min_el_rad, true);

        double max_el;
        if (!find_max_elevation(sat_state, observer, aos_refined, end_jd, &max_el)) {
            max_el = 0.1;
        }

        sgp4_look_angles_t aos_angles, los_angles;
        calculate_look_angles_at_time(sat_state, observer, aos_refined, &aos_angles);
        calculate_look_angles_at_time(sat_state, observer, end_jd, &los_angles);

        pass_out->norad_id = norad_id;
        pass_out->aos_jd = aos_refined;
        pass_out->los_jd = end_jd;  /* Truncated to window */
        pass_out->max_elevation_rad = max_el;
        pass_out->aos_azimuth_rad = aos_angles.azimuth_rad;
        pass_out->los_azimuth_rad = los_angles.azimuth_rad;

        return true;
    }

    return false;
}

rtems_task pass_calculator_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("PASS", "Calculator task started");

    /* Wait for initial GPS fix and TLE load */
    rtems_task_wake_after(10 * rtems_clock_get_ticks_per_second());

    while (true) {
        /* Get configuration */
        config_t cfg;
        config_get_copy(&cfg);

        int prediction_window_min = cfg.pass.prediction_window_min;
        double min_el_rad = cfg.pass.min_elevation_deg * SGP4_DEG_TO_RAD;
        int calc_interval_sec = cfg.pass.calc_interval_sec;

        /* Get current state snapshot */
        rtems_semaphore_obtain(g_state_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
        bool location_valid = g_state.location_valid;
        bool time_valid = g_state.time_valid;
        sgp4_geodetic_t observer = g_state.observer_location;
        time_t current_time = g_state.current_time;
        rtems_semaphore_release(g_state_mutex);

        if (!location_valid || !time_valid) {
            LOG_DEBUG("PASS", "Waiting for GPS fix...");
            rtems_task_wake_after(10 * rtems_clock_get_ticks_per_second());
            continue;
        }

        /* Calculate time window */
        double start_jd = sgp4_unix_to_jd((double)current_time);
        double end_jd = start_jd + (prediction_window_min + PASS_WINDOW_BUFFER_MIN) / 1440.0;

        int passes_found = 0;

        /* Iterate over all satellites */
        rtems_semaphore_obtain(g_tle_database_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
        int sat_count = g_state.satellite_count;

        for (int i = 0; i < sat_count && i < MAX_SATELLITES; i++) {
            if (!g_state.satellites[i].valid) {
                continue;
            }

            pass_info_t pass;
            if (find_next_pass(&g_state.satellites[i].state,
                               g_state.satellites[i].tle.norad_id,
                               &observer,
                               start_jd, end_jd,
                               min_el_rad,
                               &pass)) {

                /* Send pass to controller */
                pass_message_t msg;
                msg.type = MSG_PASS_CALCULATED;
                msg.pass = pass;

                rtems_status_code status = rtems_message_queue_send(
                    g_pass_queue, &msg, sizeof(msg));

                if (status == RTEMS_SUCCESSFUL) {
                    passes_found++;
                    LOG_DEBUG("PASS", "Found pass: NORAD %d, max_el=%.1f deg",
                              pass.norad_id,
                              pass.max_elevation_rad * SGP4_RAD_TO_DEG);
                } else if (status == RTEMS_TOO_MANY) {
                    LOG_WARN("PASS", "Pass queue full, dropping pass for NORAD %d",
                             pass.norad_id);
                }
            }
        }

        rtems_semaphore_release(g_tle_database_mutex);

        if (passes_found > 0) {
            LOG_INFO("PASS", "Found %d passes in next %d minutes",
                     passes_found, prediction_window_min);
        }

        /* Sleep until next calculation cycle */
        rtems_task_wake_after(calc_interval_sec * rtems_clock_get_ticks_per_second());
    }
}

// ============================================================================
// Pass Executor Helper Functions
// ============================================================================

bool get_satellite_state(int norad_id, sgp4_state_t *state_out,
                         char *name_out, size_t name_size)
{
    for (int i = 0; i < g_state.satellite_count && i < MAX_SATELLITES; i++) {
        if (g_state.satellites[i].valid &&
            g_state.satellites[i].tle.norad_id == norad_id) {
            *state_out = g_state.satellites[i].state;
            if (name_out && name_size > 0) {
                strncpy(name_out, g_state.satellites[i].tle.name, name_size - 1);
                name_out[name_size - 1] = '\0';
            }
            return true;
        }
    }
    return false;
}

// ============================================================================
// Pass Executor State Machine Handlers
// ============================================================================

/* Forward declarations */
static void handle_start_pass(const pass_info_t *pass);
static void handle_abort_pass(void);
static void process_prepositioning(double current_jd, const sgp4_geodetic_t *observer);
static void process_waiting_aos(double current_jd);
static void process_tracking(double current_jd, const sgp4_geodetic_t *observer,
                             double rot_threshold_rad, double doppler_threshold_khz);
static void process_completing(void);

/*
 * Handle START_PASS command.
 */
static void handle_start_pass(const pass_info_t *pass) {
    /* Log pass details */
    double aos_unix = sgp4_jd_to_unix(pass->aos_jd);
    struct tm *aos_tm = gmtime((time_t *)&aos_unix);

    LOG_INFO("EXEC", "Starting pass for NORAD %d", pass->norad_id);
    LOG_INFO("EXEC", "  AOS: %02d:%02d:%02d, az=%.1f deg",
             aos_tm->tm_hour, aos_tm->tm_min, aos_tm->tm_sec,
             pass->aos_azimuth_rad * SGP4_RAD_TO_DEG);
    LOG_INFO("EXEC", "  Max elevation: %.1f deg",
             pass->max_elevation_rad * SGP4_RAD_TO_DEG);

    /* Copy satellite state from TLE database */
    rtems_semaphore_obtain(g_tle_database_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    bool found = get_satellite_state(pass->norad_id,
                                      &g_executor_state.sat_state,
                                      g_executor_state.sat_name,
                                      sizeof(g_executor_state.sat_name));
    rtems_semaphore_release(g_tle_database_mutex);

    if (!found) {
        LOG_ERROR("EXEC", "Satellite NORAD %d not found in TLE database",
                  pass->norad_id);
        return;
    }

    /* Initialize tracking state */
    g_executor_state.current_pass = *pass;
    g_executor_state.sat_state_valid = true;
    g_executor_state.rotator_commanded = false;
    g_executor_state.doppler_valid = false;
    g_executor_state.last_doppler_factor = 1.0;

    /* Send preposition command */
    rotator_command_message_t rot_cmd;
    rot_cmd.command = ROT_CMD_GOTO;
    rot_cmd.target_azimuth_rad = pass->aos_azimuth_rad;
    rot_cmd.target_elevation_rad = 0.0;  /* Start at horizon */

    rtems_message_queue_send(g_rotator_cmd_queue, &rot_cmd, sizeof(rot_cmd));

    g_executor_state.last_cmd_azimuth_rad = pass->aos_azimuth_rad;
    g_executor_state.last_cmd_elevation_rad = 0.0;
    g_executor_state.rotator_commanded = true;

    LOG_INFO("EXEC", "Prepositioning antenna to az=%.1f el=0.0",
             pass->aos_azimuth_rad * SGP4_RAD_TO_DEG);

    g_executor_state.state = EXEC_STATE_PREPOSITIONING;
}

/*
 * Handle ABORT_PASS command.
 */
static void handle_abort_pass(void) {
    LOG_WARN("EXEC", "Aborting pass for NORAD %d",
             g_executor_state.current_pass.norad_id);

    /* Send stop command to rotator */
    rotator_command_message_t rot_cmd;
    rot_cmd.command = ROT_CMD_STOP;
    rot_cmd.target_azimuth_rad = 0.0;
    rot_cmd.target_elevation_rad = 0.0;
    rtems_message_queue_send(g_rotator_cmd_queue, &rot_cmd, sizeof(rot_cmd));

    /* Reset state */
    g_executor_state.state = EXEC_STATE_IDLE;
    g_executor_state.sat_state_valid = false;
    g_executor_state.rotator_commanded = false;
    g_executor_state.doppler_valid = false;
}

/*
 * Process PREPOSITIONING state.
 * Transition to WAITING_AOS when close to AOS time.
 */
static void process_prepositioning(double current_jd,
                                    const sgp4_geodetic_t *observer) {
    (void)observer;
    config_t cfg;
    config_get_copy(&cfg);
    double prepos_margin_jd = cfg.pass.preposition_margin_sec / 86400.0;

    double time_to_aos = g_executor_state.current_pass.aos_jd - current_jd;

    if (time_to_aos <= prepos_margin_jd) {
        LOG_INFO("EXEC", "Preposition complete, waiting for AOS (%.0f sec)",
                 time_to_aos * 86400.0);
        g_executor_state.state = EXEC_STATE_WAITING_AOS;
    }
}

/*
 * Process WAITING_AOS state.
 * Transition to TRACKING when satellite rises above horizon.
 */
static void process_waiting_aos(double current_jd) {
    if (current_jd >= g_executor_state.current_pass.aos_jd) {
        LOG_INFO("EXEC", "AOS - Beginning tracking of %s",
                 g_executor_state.sat_name);
        g_executor_state.state = EXEC_STATE_TRACKING;
    }
}

/*
 * Process TRACKING state.
 * Update rotator and radio as needed.
 */
static void process_tracking(double current_jd, const sgp4_geodetic_t *observer,
                             double rot_threshold_rad, double doppler_threshold_khz) {
    /* Check if pass has ended */
    if (current_jd >= g_executor_state.current_pass.los_jd) {
        LOG_INFO("EXEC", "LOS - Pass complete for %s",
                 g_executor_state.sat_name);
        g_executor_state.state = EXEC_STATE_COMPLETING;
        return;
    }

    /* Calculate current satellite position and doppler */
    sgp4_look_angles_t angles;
    double doppler_factor;
    double range_rate;

    sgp4_error_t err = sgp4_calculate_tracking_data(&g_executor_state.sat_state,
                                                     observer, current_jd,
                                                     &angles, &doppler_factor,
                                                     &range_rate);
    if (err != SGP4_SUCCESS) {
        LOG_WARN("EXEC", "SGP4 propagation failed");
        return;
    }

    /* Update tracking state */
    g_executor_state.current_azimuth_rad = angles.azimuth_rad;
    g_executor_state.current_elevation_rad = angles.elevation_rad;
    g_executor_state.current_range_km = angles.range_km;
    g_executor_state.range_rate_km_s = range_rate;

    /* Check if rotator needs update */
    double az_diff = fabs(angles.azimuth_rad - g_executor_state.last_cmd_azimuth_rad);
    double el_diff = fabs(angles.elevation_rad - g_executor_state.last_cmd_elevation_rad);

    /* Handle azimuth wraparound */
    if (az_diff > SGP4_PI) {
        az_diff = SGP4_TWO_PI - az_diff;
    }

    if (az_diff >= rot_threshold_rad || el_diff >= rot_threshold_rad) {
        rotator_command_message_t rot_cmd;
        rot_cmd.command = ROT_CMD_GOTO;
        rot_cmd.target_azimuth_rad = angles.azimuth_rad;
        rot_cmd.target_elevation_rad = angles.elevation_rad;

        rtems_message_queue_send(g_rotator_cmd_queue, &rot_cmd, sizeof(rot_cmd));

        g_executor_state.last_cmd_azimuth_rad = angles.azimuth_rad;
        g_executor_state.last_cmd_elevation_rad = angles.elevation_rad;

        LOG_DEBUG("EXEC", "Rotator: az=%.1f el=%.1f deg",
                  angles.azimuth_rad * SGP4_RAD_TO_DEG,
                  angles.elevation_rad * SGP4_RAD_TO_DEG);
    }

    /* Check if doppler correction needs update */
    /* Use 145.9 MHz as reference for threshold calculation */
    double doppler_diff_hz = fabs(doppler_factor - g_executor_state.last_doppler_factor) *
                             145.9e6;

    if (!g_executor_state.doppler_valid ||
        doppler_diff_hz >= (doppler_threshold_khz * 1000.0)) {
        g_executor_state.last_doppler_factor = doppler_factor;
        g_executor_state.doppler_valid = true;

        /* Log doppler info (actual frequency commands stubbed out for now) */
        double shift_hz = (doppler_factor - 1.0) * 145.9e6;
        LOG_DEBUG("EXEC", "Doppler: factor=%.9f, shift=%.0f Hz at 145.9 MHz",
                  doppler_factor, shift_hz);

        /* TODO: Send radio frequency command */
        /* radio_set_doppler_offset(doppler_factor); */
    }
}

/*
 * Process COMPLETING state.
 * Clean up and return to IDLE.
 */
static void process_completing(void) {
    /* Park the rotator */
    rotator_command_message_t rot_cmd;
    rot_cmd.command = ROT_CMD_PARK;
    rot_cmd.target_azimuth_rad = 0.0;
    rot_cmd.target_elevation_rad = 0.0;
    rtems_message_queue_send(g_rotator_cmd_queue, &rot_cmd, sizeof(rot_cmd));

    LOG_INFO("EXEC", "Pass completed, returning to idle");

    g_executor_state.state = EXEC_STATE_IDLE;
    g_executor_state.sat_state_valid = false;
    g_executor_state.rotator_commanded = false;
    g_executor_state.doppler_valid = false;
}

// ============================================================================
// Pass Executor Task
// ============================================================================

rtems_task pass_executor_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("EXEC", "Pass executor task started");

    /* Initialize state */
    memset(&g_executor_state, 0, sizeof(g_executor_state));
    g_executor_state.state = EXEC_STATE_IDLE;

    while (true) {
        /* Get configuration */
        config_t cfg;
        config_get_copy(&cfg);

        int poll_ms = cfg.pass.tracking_poll_ms;
        double rot_threshold_rad = cfg.pass.rotator_threshold_deg * SGP4_DEG_TO_RAD;
        double doppler_threshold_khz = cfg.pass.doppler_threshold_khz;

        /* Calculate tick count for poll interval */
        rtems_interval poll_ticks = (poll_ms * rtems_clock_get_ticks_per_second()) / 1000;
        if (poll_ticks < 1) poll_ticks = 1;

        /* Check for commands from controller */
        executor_command_message_t cmd;
        size_t size;
        rtems_status_code status;

        if (g_executor_state.state == EXEC_STATE_IDLE) {
            /* Blocking wait when idle */
            status = rtems_message_queue_receive(
                g_executor_cmd_queue, &cmd, &size,
                RTEMS_WAIT, RTEMS_NO_TIMEOUT);
        } else {
            /* Non-blocking check during tracking */
            status = rtems_message_queue_receive(
                g_executor_cmd_queue, &cmd, &size,
                RTEMS_NO_WAIT, 0);
        }

        /* Process command if received */
        if (status == RTEMS_SUCCESSFUL) {
            switch (cmd.command) {
                case EXEC_CMD_START_PASS:
                    handle_start_pass(&cmd.pass);
                    break;

                case EXEC_CMD_ABORT_PASS:
                    handle_abort_pass();
                    break;

                default:
                    LOG_WARN("EXEC", "Unknown command type: %d", cmd.command);
                    break;
            }
        }

        /* State machine processing */
        if (g_executor_state.state != EXEC_STATE_IDLE) {
            /* Get current time */
            rtems_semaphore_obtain(g_state_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
            time_t current_time = g_state.current_time;
            bool time_valid = g_state.time_valid;
            sgp4_geodetic_t observer = g_state.observer_location;
            bool location_valid = g_state.location_valid;
            rtems_semaphore_release(g_state_mutex);

            if (!time_valid || !location_valid) {
                LOG_WARN("EXEC", "GPS not available during tracking");
                rtems_task_wake_after(poll_ticks);
                continue;
            }

            double current_jd = sgp4_unix_to_jd((double)current_time);

            switch (g_executor_state.state) {
                case EXEC_STATE_PREPOSITIONING:
                    process_prepositioning(current_jd, &observer);
                    break;

                case EXEC_STATE_WAITING_AOS:
                    process_waiting_aos(current_jd);
                    break;

                case EXEC_STATE_TRACKING:
                    process_tracking(current_jd, &observer, rot_threshold_rad,
                                     doppler_threshold_khz);
                    break;

                case EXEC_STATE_COMPLETING:
                    process_completing();
                    break;

                default:
                    break;
            }

            /* Sleep for poll interval */
            rtems_task_wake_after(poll_ticks);
        }
    }
}
