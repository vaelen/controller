/*
 * Pass Calculation Unit Tests
 *
 * Host-compilable tests for satellite pass prediction.
 * Tests the complete pass calculation pipeline using SGP4.
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "sgp4.h"
#include "priority_queue.h"

/* Test counters */
static int tests_run = 0;
static int tests_passed = 0;

/* Test helper macros */
#define TEST_ASSERT(condition, msg) do { \
    tests_run++; \
    if (!(condition)) { \
        printf("  FAIL: %s (line %d)\n", msg, __LINE__); \
        return 1; \
    } else { \
        tests_passed++; \
    } \
} while(0)

#define TEST_ASSERT_EQ(expected, actual, msg) do { \
    tests_run++; \
    if ((expected) != (actual)) { \
        printf("  FAIL: %s - expected %d, got %d (line %d)\n", \
               msg, (int)(expected), (int)(actual), __LINE__); \
        return 1; \
    } else { \
        tests_passed++; \
    } \
} while(0)

#define TEST_ASSERT_DOUBLE_NEAR(expected, actual, tolerance, msg) do { \
    tests_run++; \
    if (fabs((expected) - (actual)) > (tolerance)) { \
        printf("  FAIL: %s - expected %.8f, got %.8f (diff=%.8f) (line %d)\n", \
               msg, (expected), (actual), fabs((expected)-(actual)), __LINE__); \
        return 1; \
    } else { \
        tests_passed++; \
    } \
} while(0)

/* ============================================================================
 * Pass Calculation Helper Functions (copied from pass.c for testing)
 * ============================================================================ */

/* Time step for coarse pass scanning (seconds) */
#define PASS_SCAN_STEP_SEC 60

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

/* ============================================================================
 * Test Helper Functions
 * ============================================================================ */

/* Initialize SGP4 state from TLE lines */
static int init_sgp4_from_tle(const char *line1, const char *line2,
                               sgp4_state_t *state_out, sgp4_tle_t *tle_out)
{
    sgp4_error_t err = sgp4_parse_tle_2line(line1, line2, tle_out);
    if (err != SGP4_SUCCESS) return -1;

    sgp4_elements_t elements;
    err = sgp4_tle_to_elements(tle_out, &elements);
    if (err != SGP4_SUCCESS) return -1;

    err = sgp4_init(state_out, &elements);
    if (err != SGP4_SUCCESS) return -1;

    return 0;
}

/* Print pass information for debugging */
static void print_pass_info(const pass_info_t *pass, const char *label)
{
    double aos_unix = sgp4_jd_to_unix(pass->aos_jd);
    double los_unix = sgp4_jd_to_unix(pass->los_jd);
    double duration_min = (los_unix - aos_unix) / 60.0;

    printf("  %s: NORAD %d\n", label, pass->norad_id);
    printf("    AOS JD: %.6f (Unix: %.0f)\n", pass->aos_jd, aos_unix);
    printf("    LOS JD: %.6f (Unix: %.0f)\n", pass->los_jd, los_unix);
    printf("    Duration: %.1f minutes\n", duration_min);
    printf("    Max elevation: %.1f degrees\n", pass->max_elevation_rad * SGP4_RAD_TO_DEG);
    printf("    AOS azimuth: %.1f degrees\n", pass->aos_azimuth_rad * SGP4_RAD_TO_DEG);
    printf("    LOS azimuth: %.1f degrees\n", pass->los_azimuth_rad * SGP4_RAD_TO_DEG);
}

/* ============================================================================
 * Look Angles Tests
 * ============================================================================ */

/* Test look angles calculation at a known time */
static int test_look_angles_at_time(void)
{
    printf("test_look_angles_at_time...\n");

    /* ISS TLE for 2026-01-01 */
    const char *line1 = "1 25544U 98067A   26001.50000000  .00016717  00000-0  10270-3 0  9999";
    const char *line2 = "2 25544  51.6400 247.4627 0006703 130.5360 325.0288 15.72125391999993";

    sgp4_state_t state;
    sgp4_tle_t tle;
    int ret = init_sgp4_from_tle(line1, line2, &state, &tle);
    TEST_ASSERT_EQ(0, ret, "TLE init");

    /* Observer at Tokyo */
    sgp4_geodetic_t observer = {
        35.6762 * SGP4_DEG_TO_RAD,
        139.6503 * SGP4_DEG_TO_RAD,
        0.040
    };

    /* Calculate at epoch */
    double epoch_jd = state.jdsatepoch + state.jdsatepochF;
    sgp4_look_angles_t angles;
    bool success = calculate_look_angles_at_time(&state, &observer, epoch_jd, &angles);
    TEST_ASSERT(success, "Look angles calculation succeeded");

    /* Verify reasonable values */
    TEST_ASSERT(angles.elevation_rad >= -SGP4_PI/2.0, "Elevation >= -90 deg");
    TEST_ASSERT(angles.elevation_rad <= SGP4_PI/2.0, "Elevation <= 90 deg");
    TEST_ASSERT(angles.azimuth_rad >= 0.0, "Azimuth >= 0");
    TEST_ASSERT(angles.azimuth_rad < SGP4_TWO_PI, "Azimuth < 360 deg");
    TEST_ASSERT(angles.range_km > 0.0, "Range > 0");
    TEST_ASSERT(angles.range_km < 50000.0, "Range reasonable (< 50000 km)");

    printf("  At epoch: el=%.1f deg, az=%.1f deg, range=%.0f km\n",
           angles.elevation_rad * SGP4_RAD_TO_DEG,
           angles.azimuth_rad * SGP4_RAD_TO_DEG,
           angles.range_km);

    printf("  PASSED\n");
    return 0;
}

/* Test elevation varies with time (satellite motion) */
static int test_elevation_varies_with_time(void)
{
    printf("test_elevation_varies_with_time...\n");

    const char *line1 = "1 25544U 98067A   26001.50000000  .00016717  00000-0  10270-3 0  9999";
    const char *line2 = "2 25544  51.6400 247.4627 0006703 130.5360 325.0288 15.72125391999993";

    sgp4_state_t state;
    sgp4_tle_t tle;
    int ret = init_sgp4_from_tle(line1, line2, &state, &tle);
    TEST_ASSERT_EQ(0, ret, "TLE init");

    sgp4_geodetic_t observer = {
        35.6762 * SGP4_DEG_TO_RAD,
        139.6503 * SGP4_DEG_TO_RAD,
        0.040
    };

    double epoch_jd = state.jdsatepoch + state.jdsatepochF;

    /* Sample elevation at multiple times */
    double elevations[10];
    int different_count = 0;

    for (int i = 0; i < 10; i++) {
        double t = epoch_jd + (i * 10.0 / 1440.0);  /* Every 10 minutes */
        sgp4_look_angles_t angles;
        bool success = calculate_look_angles_at_time(&state, &observer, t, &angles);
        TEST_ASSERT(success, "Look angles calculation");
        elevations[i] = angles.elevation_rad;

        if (i > 0 && fabs(elevations[i] - elevations[i-1]) > 0.001) {
            different_count++;
        }
    }

    /* Elevation should change over time */
    TEST_ASSERT(different_count > 0, "Elevation changes over time");

    printf("  PASSED\n");
    return 0;
}

/* ============================================================================
 * Pass Finding Tests
 * ============================================================================ */

/* Test finding a pass for ISS */
static int test_find_iss_pass(void)
{
    printf("test_find_iss_pass...\n");

    /* ISS TLE */
    const char *line1 = "1 25544U 98067A   26001.50000000  .00016717  00000-0  10270-3 0  9999";
    const char *line2 = "2 25544  51.6400 247.4627 0006703 130.5360 325.0288 15.72125391999993";

    sgp4_state_t state;
    sgp4_tle_t tle;
    int ret = init_sgp4_from_tle(line1, line2, &state, &tle);
    TEST_ASSERT_EQ(0, ret, "TLE init");

    /* Observer at Tokyo */
    sgp4_geodetic_t observer = {
        35.6762 * SGP4_DEG_TO_RAD,
        139.6503 * SGP4_DEG_TO_RAD,
        0.040
    };

    /* Search for pass in next 24 hours from TLE epoch */
    double epoch_jd = state.jdsatepoch + state.jdsatepochF;
    double start_jd = epoch_jd;
    double end_jd = epoch_jd + 1.0;  /* 24 hours */
    double min_el_rad = 5.0 * SGP4_DEG_TO_RAD;  /* 5 degree minimum */

    pass_info_t pass;
    bool found = find_next_pass(&state, tle.norad_id, &observer,
                                 start_jd, end_jd, min_el_rad, &pass);

    /* Should find at least one pass in 24 hours */
    TEST_ASSERT(found, "Found a pass in 24 hours");

    if (found) {
        print_pass_info(&pass, "ISS Pass");

        /* Calculate duration */
        double aos_unix = sgp4_jd_to_unix(pass.aos_jd);
        double los_unix = sgp4_jd_to_unix(pass.los_jd);
        double duration_min = (los_unix - aos_unix) / 60.0;

        /* CRITICAL TEST: ISS pass should be < 20 minutes (typically 5-10 min) */
        TEST_ASSERT(duration_min > 0.0, "Duration positive");
        TEST_ASSERT(duration_min < 20.0, "Duration < 20 min for LEO satellite");

        printf("  Duration: %.1f minutes - ", duration_min);
        if (duration_min < 20.0) {
            printf("OK (valid LEO pass)\n");
        } else {
            printf("FAILED (too long for LEO)\n");
        }

        /* Max elevation should be at least min_el_rad */
        TEST_ASSERT(pass.max_elevation_rad >= min_el_rad, "Max elevation >= minimum");

        /* AOS should be before LOS */
        TEST_ASSERT(pass.aos_jd < pass.los_jd, "AOS before LOS");

        /* Azimuths should be valid */
        TEST_ASSERT(pass.aos_azimuth_rad >= 0.0, "AOS azimuth >= 0");
        TEST_ASSERT(pass.aos_azimuth_rad < SGP4_TWO_PI, "AOS azimuth < 360");
        TEST_ASSERT(pass.los_azimuth_rad >= 0.0, "LOS azimuth >= 0");
        TEST_ASSERT(pass.los_azimuth_rad < SGP4_TWO_PI, "LOS azimuth < 360");
    }

    printf("  PASSED\n");
    return 0;
}

/* Test that all found passes have reasonable durations */
static int test_all_passes_reasonable_duration(void)
{
    printf("test_all_passes_reasonable_duration...\n");

    const char *line1 = "1 25544U 98067A   26001.50000000  .00016717  00000-0  10270-3 0  9999";
    const char *line2 = "2 25544  51.6400 247.4627 0006703 130.5360 325.0288 15.72125391999993";

    sgp4_state_t state;
    sgp4_tle_t tle;
    int ret = init_sgp4_from_tle(line1, line2, &state, &tle);
    TEST_ASSERT_EQ(0, ret, "TLE init");

    sgp4_geodetic_t observer = {
        35.6762 * SGP4_DEG_TO_RAD,
        139.6503 * SGP4_DEG_TO_RAD,
        0.040
    };

    double epoch_jd = state.jdsatepoch + state.jdsatepochF;
    double search_start = epoch_jd;
    double min_el_rad = 5.0 * SGP4_DEG_TO_RAD;

    int passes_found = 0;
    int passes_too_long = 0;

    /* Find multiple passes over 3 days */
    while (search_start < epoch_jd + 3.0 && passes_found < 50) {
        double search_end = search_start + 0.5;  /* 12 hour window */

        pass_info_t pass;
        bool found = find_next_pass(&state, tle.norad_id, &observer,
                                     search_start, search_end, min_el_rad, &pass);

        if (found) {
            passes_found++;

            double aos_unix = sgp4_jd_to_unix(pass.aos_jd);
            double los_unix = sgp4_jd_to_unix(pass.los_jd);
            double duration_min = (los_unix - aos_unix) / 60.0;

            printf("  Pass %d: %.1f min, max_el=%.1f deg\n",
                   passes_found, duration_min, pass.max_elevation_rad * SGP4_RAD_TO_DEG);

            if (duration_min > 20.0) {
                passes_too_long++;
                printf("    *** TOO LONG ***\n");
            }

            /* Move search start past this pass */
            search_start = pass.los_jd + 0.001;  /* Small offset */
        } else {
            /* No pass in this window, move forward */
            search_start = search_end;
        }
    }

    printf("  Found %d passes, %d too long\n", passes_found, passes_too_long);

    TEST_ASSERT(passes_found > 0, "Found at least one pass");
    TEST_ASSERT_EQ(0, passes_too_long, "No passes too long");

    printf("  PASSED\n");
    return 0;
}

/* Test pass finding with satellite already visible */
static int test_satellite_already_visible(void)
{
    printf("test_satellite_already_visible...\n");

    const char *line1 = "1 25544U 98067A   26001.50000000  .00016717  00000-0  10270-3 0  9999";
    const char *line2 = "2 25544  51.6400 247.4627 0006703 130.5360 325.0288 15.72125391999993";

    sgp4_state_t state;
    sgp4_tle_t tle;
    int ret = init_sgp4_from_tle(line1, line2, &state, &tle);
    TEST_ASSERT_EQ(0, ret, "TLE init");

    sgp4_geodetic_t observer = {
        35.6762 * SGP4_DEG_TO_RAD,
        139.6503 * SGP4_DEG_TO_RAD,
        0.040
    };

    double epoch_jd = state.jdsatepoch + state.jdsatepochF;
    double min_el_rad = 5.0 * SGP4_DEG_TO_RAD;

    /* First find a pass */
    pass_info_t pass1;
    bool found1 = find_next_pass(&state, tle.norad_id, &observer,
                                  epoch_jd, epoch_jd + 1.0, min_el_rad, &pass1);
    TEST_ASSERT(found1, "Found initial pass");

    if (found1) {
        /* Now search starting mid-pass */
        double mid_pass = (pass1.aos_jd + pass1.los_jd) / 2.0;

        pass_info_t pass2;
        bool found2 = find_next_pass(&state, tle.norad_id, &observer,
                                      mid_pass, epoch_jd + 1.0, min_el_rad, &pass2);

        /* Should find the NEXT pass, not the current one */
        if (found2) {
            /* The second pass should start after the first pass ends */
            TEST_ASSERT(pass2.aos_jd >= pass1.los_jd, "Second pass AOS >= first pass LOS");
            printf("  Found next pass starting after current one ends\n");
        }
    }

    printf("  PASSED\n");
    return 0;
}

/* ============================================================================
 * Refinement Tests
 * ============================================================================ */

/* Test binary search refinement precision */
static int test_refinement_precision(void)
{
    printf("test_refinement_precision...\n");

    const char *line1 = "1 25544U 98067A   26001.50000000  .00016717  00000-0  10270-3 0  9999";
    const char *line2 = "2 25544  51.6400 247.4627 0006703 130.5360 325.0288 15.72125391999993";

    sgp4_state_t state;
    sgp4_tle_t tle;
    int ret = init_sgp4_from_tle(line1, line2, &state, &tle);
    TEST_ASSERT_EQ(0, ret, "TLE init");

    sgp4_geodetic_t observer = {
        35.6762 * SGP4_DEG_TO_RAD,
        139.6503 * SGP4_DEG_TO_RAD,
        0.040
    };

    double epoch_jd = state.jdsatepoch + state.jdsatepochF;
    double min_el_rad = 5.0 * SGP4_DEG_TO_RAD;

    /* Find a pass */
    pass_info_t pass;
    bool found = find_next_pass(&state, tle.norad_id, &observer,
                                 epoch_jd, epoch_jd + 1.0, min_el_rad, &pass);

    if (found) {
        /* Verify AOS is at the threshold */
        sgp4_look_angles_t aos_angles;
        bool success = calculate_look_angles_at_time(&state, &observer, pass.aos_jd, &aos_angles);
        TEST_ASSERT(success, "AOS calculation");

        /* AOS elevation should be very close to min_el_rad (within 0.1 degrees) */
        double el_diff = fabs(aos_angles.elevation_rad - min_el_rad);
        TEST_ASSERT(el_diff < 0.1 * SGP4_DEG_TO_RAD, "AOS at threshold (within 0.1 deg)");

        printf("  AOS elevation: %.3f deg (threshold: %.1f deg)\n",
               aos_angles.elevation_rad * SGP4_RAD_TO_DEG,
               min_el_rad * SGP4_RAD_TO_DEG);

        /* Verify LOS is at the threshold */
        sgp4_look_angles_t los_angles;
        success = calculate_look_angles_at_time(&state, &observer, pass.los_jd, &los_angles);
        TEST_ASSERT(success, "LOS calculation");

        el_diff = fabs(los_angles.elevation_rad - min_el_rad);
        TEST_ASSERT(el_diff < 0.1 * SGP4_DEG_TO_RAD, "LOS at threshold (within 0.1 deg)");

        printf("  LOS elevation: %.3f deg (threshold: %.1f deg)\n",
               los_angles.elevation_rad * SGP4_RAD_TO_DEG,
               min_el_rad * SGP4_RAD_TO_DEG);
    }

    printf("  PASSED\n");
    return 0;
}

/* Test max elevation calculation */
static int test_max_elevation_calculation(void)
{
    printf("test_max_elevation_calculation...\n");

    const char *line1 = "1 25544U 98067A   26001.50000000  .00016717  00000-0  10270-3 0  9999";
    const char *line2 = "2 25544  51.6400 247.4627 0006703 130.5360 325.0288 15.72125391999993";

    sgp4_state_t state;
    sgp4_tle_t tle;
    int ret = init_sgp4_from_tle(line1, line2, &state, &tle);
    TEST_ASSERT_EQ(0, ret, "TLE init");

    sgp4_geodetic_t observer = {
        35.6762 * SGP4_DEG_TO_RAD,
        139.6503 * SGP4_DEG_TO_RAD,
        0.040
    };

    double epoch_jd = state.jdsatepoch + state.jdsatepochF;
    double min_el_rad = 5.0 * SGP4_DEG_TO_RAD;

    pass_info_t pass;
    bool found = find_next_pass(&state, tle.norad_id, &observer,
                                 epoch_jd, epoch_jd + 1.0, min_el_rad, &pass);

    if (found) {
        /* Sample elevation throughout pass */
        double max_sampled = -100.0;
        int sample_count = 100;

        for (int i = 0; i <= sample_count; i++) {
            double t = pass.aos_jd + (pass.los_jd - pass.aos_jd) * i / sample_count;
            sgp4_look_angles_t angles;
            if (calculate_look_angles_at_time(&state, &observer, t, &angles)) {
                if (angles.elevation_rad > max_sampled) {
                    max_sampled = angles.elevation_rad;
                }
            }
        }

        /* Reported max should be close to sampled max */
        double diff = fabs(pass.max_elevation_rad - max_sampled);
        TEST_ASSERT(diff < 0.5 * SGP4_DEG_TO_RAD, "Max elevation accurate (within 0.5 deg)");

        printf("  Reported max: %.2f deg, Sampled max: %.2f deg\n",
               pass.max_elevation_rad * SGP4_RAD_TO_DEG,
               max_sampled * SGP4_RAD_TO_DEG);
    }

    printf("  PASSED\n");
    return 0;
}

/* ============================================================================
 * Main
 * ============================================================================ */

int main(void)
{
    printf("=== Pass Calculation Unit Tests ===\n\n");

    int failed = 0;

    /* Look angles tests */
    failed += test_look_angles_at_time();
    failed += test_elevation_varies_with_time();

    /* Pass finding tests */
    failed += test_find_iss_pass();
    failed += test_all_passes_reasonable_duration();
    failed += test_satellite_already_visible();

    /* Refinement tests */
    failed += test_refinement_precision();
    failed += test_max_elevation_calculation();

    printf("\n=== Results ===\n");
    printf("Tests run: %d\n", tests_run);
    printf("Tests passed: %d\n", tests_passed);
    printf("Tests failed: %d\n", tests_run - tests_passed);

    if (failed > 0) {
        printf("\nFAILED: %d test function(s) had failures\n", failed);
        return 1;
    }

    printf("\nAll tests PASSED!\n");
    return 0;
}
