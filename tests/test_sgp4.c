/*
 * SGP4 Unit Tests
 *
 * Host-compilable tests for SGP4 propagation and coordinate transforms.
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "sgp4.h"

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
        printf("  FAIL: %s - expected %.8f, got %.8f (diff=%.8f, tol=%.8f) (line %d)\n", \
               msg, (expected), (actual), fabs((expected)-(actual)), (tolerance), __LINE__); \
        return 1; \
    } else { \
        tests_passed++; \
    } \
} while(0)

/* ============================================================================
 * Time Conversion Tests
 * ============================================================================ */

/* Test Unix to Julian Date conversion */
static int test_unix_to_jd(void)
{
    printf("test_unix_to_jd...\n");

    /* Unix epoch (1970-01-01 00:00:00 UTC) = JD 2440587.5 */
    double jd = sgp4_unix_to_jd(0.0);
    TEST_ASSERT_DOUBLE_NEAR(2440587.5, jd, 0.0001, "Unix epoch to JD");

    /* 2000-01-01 12:00:00 UTC = JD 2451545.0 (J2000 epoch) */
    /* Unix time for J2000: (2451545.0 - 2440587.5) * 86400 = 946728000 */
    jd = sgp4_unix_to_jd(946728000.0);
    TEST_ASSERT_DOUBLE_NEAR(2451545.0, jd, 0.0001, "J2000 epoch to JD");

    /* 2026-01-05 12:00:00 UTC */
    /* Let's verify: 2026-01-05 00:00:00 UTC is JD 2460680.5 (midnight) */
    /* 2026-01-05 12:00:00 UTC is JD 2460681.0 (noon) */
    /* Unix time for 2026-01-05 12:00:00 = days_since_1970 * 86400 */
    /* Days from 1970-01-01 to 2026-01-05 = 56 years ~= 20454 days + some */
    /* Let's compute: (2460681.0 - 2440587.5) * 86400 = 1736078400 */
    jd = sgp4_unix_to_jd(1736078400.0);
    TEST_ASSERT_DOUBLE_NEAR(2460681.0, jd, 0.0001, "2026-01-05 12:00:00 UTC to JD");

    printf("  PASSED\n");
    return 0;
}

/* Test Julian Date to Unix conversion */
static int test_jd_to_unix(void)
{
    printf("test_jd_to_unix...\n");

    /* Unix epoch */
    double unix_time = sgp4_jd_to_unix(2440587.5);
    TEST_ASSERT_DOUBLE_NEAR(0.0, unix_time, 0.1, "JD to Unix epoch");

    /* J2000 */
    unix_time = sgp4_jd_to_unix(2451545.0);
    TEST_ASSERT_DOUBLE_NEAR(946728000.0, unix_time, 0.1, "JD J2000 to Unix");

    printf("  PASSED\n");
    return 0;
}

/* Test round-trip conversion */
static int test_time_round_trip(void)
{
    printf("test_time_round_trip...\n");

    double test_times[] = {0.0, 946728000.0, 1736078400.0, 1000000000.0};

    for (int i = 0; i < 4; i++) {
        double original = test_times[i];
        double jd = sgp4_unix_to_jd(original);
        double result = sgp4_jd_to_unix(jd);
        TEST_ASSERT_DOUBLE_NEAR(original, result, 0.001, "Time round-trip");
    }

    printf("  PASSED\n");
    return 0;
}

/* ============================================================================
 * Coordinate Transform Tests
 * ============================================================================ */

/* Test geodetic to ECEF conversion */
static int test_geodetic_to_ecef(void)
{
    printf("test_geodetic_to_ecef...\n");

    /* Test: Equator at prime meridian, sea level */
    sgp4_geodetic_t geo1 = {0.0, 0.0, 0.0};  /* lat=0, lon=0, alt=0 */
    sgp4_vec3_t ecef1;
    sgp4_geodetic_to_ecef(&geo1, &ecef1);

    /* At equator on prime meridian, X should equal Earth radius, Y and Z should be 0 */
    TEST_ASSERT_DOUBLE_NEAR(SGP4_WGS84_A, ecef1.x, 0.001, "Equator X");
    TEST_ASSERT_DOUBLE_NEAR(0.0, ecef1.y, 0.001, "Equator Y");
    TEST_ASSERT_DOUBLE_NEAR(0.0, ecef1.z, 0.001, "Equator Z");

    /* Test: North pole, sea level */
    sgp4_geodetic_t geo2 = {SGP4_PI / 2.0, 0.0, 0.0};  /* lat=90deg, lon=0, alt=0 */
    sgp4_vec3_t ecef2;
    sgp4_geodetic_to_ecef(&geo2, &ecef2);

    /* At north pole, Z should be polar radius, X and Y should be ~0 */
    double polar_radius = SGP4_WGS84_A * (1.0 - SGP4_WGS84_F);
    TEST_ASSERT_DOUBLE_NEAR(0.0, ecef2.x, 0.001, "North pole X");
    TEST_ASSERT_DOUBLE_NEAR(0.0, ecef2.y, 0.001, "North pole Y");
    TEST_ASSERT_DOUBLE_NEAR(polar_radius, ecef2.z, 0.1, "North pole Z");

    /* Test: Tokyo, Japan (35.6762 N, 139.6503 E, 40m) */
    sgp4_geodetic_t geo3 = {
        35.6762 * SGP4_DEG_TO_RAD,
        139.6503 * SGP4_DEG_TO_RAD,
        0.040  /* 40 meters in km */
    };
    sgp4_vec3_t ecef3;
    sgp4_geodetic_to_ecef(&geo3, &ecef3);

    /* Verify reasonable values for Tokyo */
    /* Expected approximately: X=-3953km, Y=3354km, Z=3700km */
    TEST_ASSERT_DOUBLE_NEAR(-3954.0, ecef3.x, 5.0, "Tokyo X");
    TEST_ASSERT_DOUBLE_NEAR(3354.0, ecef3.y, 5.0, "Tokyo Y");
    TEST_ASSERT_DOUBLE_NEAR(3700.0, ecef3.z, 5.0, "Tokyo Z");

    printf("  PASSED\n");
    return 0;
}

/* Test ECEF to geodetic conversion (round trip) */
static int test_ecef_geodetic_round_trip(void)
{
    printf("test_ecef_geodetic_round_trip...\n");

    /* Test various locations */
    sgp4_geodetic_t test_locs[] = {
        {0.0, 0.0, 0.0},                                  /* Equator, prime meridian */
        {45.0 * SGP4_DEG_TO_RAD, 45.0 * SGP4_DEG_TO_RAD, 0.0},   /* Mid-latitude */
        {35.6762 * SGP4_DEG_TO_RAD, 139.6503 * SGP4_DEG_TO_RAD, 0.040},  /* Tokyo */
        {-33.9 * SGP4_DEG_TO_RAD, 151.2 * SGP4_DEG_TO_RAD, 0.058},  /* Sydney */
    };

    for (int i = 0; i < 4; i++) {
        sgp4_vec3_t ecef;
        sgp4_geodetic_to_ecef(&test_locs[i], &ecef);

        sgp4_geodetic_t result;
        sgp4_ecef_to_geodetic(&ecef, &result);

        TEST_ASSERT_DOUBLE_NEAR(test_locs[i].lat_rad, result.lat_rad, 1e-6, "Lat round-trip");
        TEST_ASSERT_DOUBLE_NEAR(test_locs[i].lon_rad, result.lon_rad, 1e-6, "Lon round-trip");
        TEST_ASSERT_DOUBLE_NEAR(test_locs[i].alt_km, result.alt_km, 0.001, "Alt round-trip");
    }

    printf("  PASSED\n");
    return 0;
}

/* Test ECEF to ENU conversion */
static int test_ecef_to_enu(void)
{
    printf("test_ecef_to_enu...\n");

    /* Observer at Tokyo */
    sgp4_geodetic_t observer = {
        35.6762 * SGP4_DEG_TO_RAD,
        139.6503 * SGP4_DEG_TO_RAD,
        0.040
    };

    /* Target directly above observer at 400km altitude (like ISS) */
    sgp4_geodetic_t target_geo = {
        35.6762 * SGP4_DEG_TO_RAD,
        139.6503 * SGP4_DEG_TO_RAD,
        400.0
    };
    sgp4_vec3_t target_ecef;
    sgp4_geodetic_to_ecef(&target_geo, &target_ecef);

    sgp4_vec3_t enu;
    sgp4_ecef_to_enu(&target_ecef, &observer, &enu);

    /* When target is directly overhead, E and N should be ~0, U should be altitude difference */
    TEST_ASSERT_DOUBLE_NEAR(0.0, enu.x, 1.0, "Overhead target: East");
    TEST_ASSERT_DOUBLE_NEAR(0.0, enu.y, 1.0, "Overhead target: North");
    TEST_ASSERT_DOUBLE_NEAR(400.0 - 0.040, enu.z, 1.0, "Overhead target: Up");

    printf("  PASSED\n");
    return 0;
}

/* Test look angles calculation */
static int test_look_angles(void)
{
    printf("test_look_angles...\n");

    /* Observer at Tokyo */
    sgp4_geodetic_t observer = {
        35.6762 * SGP4_DEG_TO_RAD,
        139.6503 * SGP4_DEG_TO_RAD,
        0.040
    };

    /* Target directly overhead at 400km */
    sgp4_geodetic_t overhead_geo = {
        35.6762 * SGP4_DEG_TO_RAD,
        139.6503 * SGP4_DEG_TO_RAD,
        400.0
    };
    sgp4_vec3_t overhead_ecef;
    sgp4_geodetic_to_ecef(&overhead_geo, &overhead_ecef);

    sgp4_look_angles_t angles;
    sgp4_look_angles(&overhead_ecef, &observer, &angles);

    /* Elevation should be 90 degrees (pi/2 radians) for overhead target */
    TEST_ASSERT_DOUBLE_NEAR(SGP4_PI / 2.0, angles.elevation_rad, 0.01, "Overhead elevation");

    /* Range should be approximately 400 km */
    TEST_ASSERT_DOUBLE_NEAR(400.0, angles.range_km, 1.0, "Overhead range");

    /* Target on the horizon to the North (same longitude, higher latitude) */
    /* At ~35.7 deg, a target at about 38 deg would be roughly at horizon */
    sgp4_geodetic_t north_geo = {
        (35.6762 + 2.0) * SGP4_DEG_TO_RAD,  /* ~2 degrees north */
        139.6503 * SGP4_DEG_TO_RAD,
        400.0
    };
    sgp4_vec3_t north_ecef;
    sgp4_geodetic_to_ecef(&north_geo, &north_ecef);

    sgp4_look_angles(&north_ecef, &observer, &angles);

    /* Azimuth should be approximately 0 (North) */
    TEST_ASSERT_DOUBLE_NEAR(0.0, angles.azimuth_rad, 0.1, "North azimuth");

    /* Elevation should be positive but less than 90 */
    TEST_ASSERT(angles.elevation_rad > 0.0, "North elevation positive");
    TEST_ASSERT(angles.elevation_rad < SGP4_PI / 2.0, "North elevation less than zenith");

    printf("  PASSED\n");
    return 0;
}

/* Test look angles for target below horizon */
static int test_look_angles_below_horizon(void)
{
    printf("test_look_angles_below_horizon...\n");

    /* Observer at Tokyo */
    sgp4_geodetic_t observer = {
        35.6762 * SGP4_DEG_TO_RAD,
        139.6503 * SGP4_DEG_TO_RAD,
        0.040
    };

    /* Target on opposite side of Earth (through the planet) */
    sgp4_geodetic_t opposite_geo = {
        -35.6762 * SGP4_DEG_TO_RAD,  /* Southern hemisphere */
        (139.6503 - 180.0) * SGP4_DEG_TO_RAD,  /* Opposite longitude */
        400.0
    };
    sgp4_vec3_t opposite_ecef;
    sgp4_geodetic_to_ecef(&opposite_geo, &opposite_ecef);

    sgp4_look_angles_t angles;
    sgp4_look_angles(&opposite_ecef, &observer, &angles);

    /* Elevation should be negative (below horizon) */
    TEST_ASSERT(angles.elevation_rad < 0.0, "Opposite side elevation negative");

    printf("  PASSED\n");
    return 0;
}

/* ============================================================================
 * TLE Parsing Tests
 * ============================================================================ */

/* Test ISS TLE parsing */
static int test_tle_parsing(void)
{
    printf("test_tle_parsing...\n");

    /* ISS TLE with valid checksums */
    /* Line format: 69 characters total, checksum at position 69 */
    const char *line1 = "1 25544U 98067A   26001.50000000  .00016717  00000-0  10270-3 0  9999";
    const char *line2 = "2 25544  51.6400 247.4627 0006703 130.5360 325.0288 15.72125391999993";

    sgp4_tle_t tle;
    sgp4_error_t err = sgp4_parse_tle_2line(line1, line2, &tle);
    TEST_ASSERT_EQ(SGP4_SUCCESS, err, "TLE parse success");

    TEST_ASSERT_EQ(25544, tle.norad_id, "NORAD ID");
    TEST_ASSERT_DOUBLE_NEAR(51.64, tle.inclination, 0.01, "Inclination");
    TEST_ASSERT_DOUBLE_NEAR(15.72125391, tle.mean_motion, 0.0001, "Mean motion");

    printf("  PASSED\n");
    return 0;
}

/* Test TLE to elements conversion */
static int test_tle_to_elements(void)
{
    printf("test_tle_to_elements...\n");

    const char *line1 = "1 25544U 98067A   26001.50000000  .00016717  00000-0  10270-3 0  9999";
    const char *line2 = "2 25544  51.6400 247.4627 0006703 130.5360 325.0288 15.72125391999993";

    sgp4_tle_t tle;
    sgp4_error_t err = sgp4_parse_tle_2line(line1, line2, &tle);
    TEST_ASSERT_EQ(SGP4_SUCCESS, err, "TLE parse");

    sgp4_elements_t elements;
    err = sgp4_tle_to_elements(&tle, &elements);
    TEST_ASSERT_EQ(SGP4_SUCCESS, err, "TLE to elements");

    /* Inclination should be in radians */
    TEST_ASSERT_DOUBLE_NEAR(51.64 * SGP4_DEG_TO_RAD, elements.inclination, 0.001, "Inclination in radians");

    /* Mean motion should be in rad/min (converted from rev/day) */
    /* 15.72125391 rev/day * 2*pi / 1440 = ~0.0686 rad/min */
    double expected_mm = 15.72125391 * SGP4_TWO_PI / 1440.0;
    TEST_ASSERT_DOUBLE_NEAR(expected_mm, elements.mean_motion, 0.0001, "Mean motion rad/min");

    printf("  PASSED\n");
    return 0;
}

/* ============================================================================
 * SGP4 Propagation Tests
 * ============================================================================ */

/* Test SGP4 initialization and basic propagation */
static int test_sgp4_propagation(void)
{
    printf("test_sgp4_propagation...\n");

    const char *line1 = "1 25544U 98067A   26001.50000000  .00016717  00000-0  10270-3 0  9999";
    const char *line2 = "2 25544  51.6400 247.4627 0006703 130.5360 325.0288 15.72125391999993";

    sgp4_tle_t tle;
    sgp4_error_t err = sgp4_parse_tle_2line(line1, line2, &tle);
    TEST_ASSERT_EQ(SGP4_SUCCESS, err, "TLE parse");

    sgp4_elements_t elements;
    err = sgp4_tle_to_elements(&tle, &elements);
    TEST_ASSERT_EQ(SGP4_SUCCESS, err, "TLE to elements");

    sgp4_state_t state;
    err = sgp4_init(&state, &elements);
    TEST_ASSERT_EQ(SGP4_SUCCESS, err, "SGP4 init");

    /* Propagate at epoch (tsince = 0) */
    sgp4_result_t result;
    err = sgp4_propagate(&state, 0.0, &result);
    TEST_ASSERT_EQ(SGP4_SUCCESS, err, "Propagate at epoch");

    /* ISS should be at roughly 400-420 km altitude */
    /* Position magnitude should be Earth radius + altitude = ~6778 km */
    double r_mag = sqrt(result.r[0]*result.r[0] + result.r[1]*result.r[1] + result.r[2]*result.r[2]);
    TEST_ASSERT(r_mag > 6700.0, "Altitude check - too low");
    TEST_ASSERT(r_mag < 6850.0, "Altitude check - too high");

    /* Velocity should be roughly 7.5-7.8 km/s for ISS */
    double v_mag = sqrt(result.v[0]*result.v[0] + result.v[1]*result.v[1] + result.v[2]*result.v[2]);
    TEST_ASSERT(v_mag > 7.0, "Velocity check - too low");
    TEST_ASSERT(v_mag < 8.0, "Velocity check - too high");

    /* Propagate forward 90 minutes (roughly one orbit) */
    err = sgp4_propagate(&state, 90.0, &result);
    TEST_ASSERT_EQ(SGP4_SUCCESS, err, "Propagate +90 min");

    /* Should still be at similar altitude */
    r_mag = sqrt(result.r[0]*result.r[0] + result.r[1]*result.r[1] + result.r[2]*result.r[2]);
    TEST_ASSERT(r_mag > 6700.0, "90min altitude - too low");
    TEST_ASSERT(r_mag < 6850.0, "90min altitude - too high");

    printf("  PASSED\n");
    return 0;
}

/* Test that ISS orbital period is approximately 90-93 minutes */
static int test_iss_orbital_period(void)
{
    printf("test_iss_orbital_period...\n");

    /* ISS mean motion is about 15.72 rev/day */
    /* Period = 1440 min/day / 15.72 rev/day = ~91.6 minutes */
    double mean_motion_rev_day = 15.72125391;
    double period_min = 1440.0 / mean_motion_rev_day;

    TEST_ASSERT(period_min > 90.0, "ISS period > 90 min");
    TEST_ASSERT(period_min < 93.0, "ISS period < 93 min");

    printf("  PASSED\n");
    return 0;
}

/* ============================================================================
 * Vector Operations Tests
 * ============================================================================ */

static int test_vector_operations(void)
{
    printf("test_vector_operations...\n");

    sgp4_vec3_t a = {3.0, 4.0, 0.0};
    sgp4_vec3_t b = {1.0, 0.0, 0.0};

    /* Magnitude */
    double mag = sgp4_vec3_magnitude(&a);
    TEST_ASSERT_DOUBLE_NEAR(5.0, mag, 0.0001, "Magnitude");

    /* Dot product */
    double dot = sgp4_vec3_dot(&a, &b);
    TEST_ASSERT_DOUBLE_NEAR(3.0, dot, 0.0001, "Dot product");

    /* Cross product */
    sgp4_vec3_t cross;
    sgp4_vec3_cross(&a, &b, &cross);
    TEST_ASSERT_DOUBLE_NEAR(0.0, cross.x, 0.0001, "Cross X");
    TEST_ASSERT_DOUBLE_NEAR(0.0, cross.y, 0.0001, "Cross Y");
    TEST_ASSERT_DOUBLE_NEAR(-4.0, cross.z, 0.0001, "Cross Z");

    /* Normalize */
    sgp4_vec3_t norm;
    sgp4_vec3_normalize(&a, &norm);
    TEST_ASSERT_DOUBLE_NEAR(0.6, norm.x, 0.0001, "Norm X");
    TEST_ASSERT_DOUBLE_NEAR(0.8, norm.y, 0.0001, "Norm Y");
    TEST_ASSERT_DOUBLE_NEAR(0.0, norm.z, 0.0001, "Norm Z");

    printf("  PASSED\n");
    return 0;
}

/* ============================================================================
 * Greenwich Sidereal Time Tests
 * ============================================================================ */

static int test_gstime(void)
{
    printf("test_gstime...\n");

    /* At J2000.0 epoch, GST should be approximately 280.46 degrees = 4.894 radians */
    double gst = sgp4_gstime(2451545.0);

    /* GST should be between 0 and 2*pi */
    TEST_ASSERT(gst >= 0.0, "GST >= 0");
    TEST_ASSERT(gst < SGP4_TWO_PI, "GST < 2*pi");

    printf("  PASSED\n");
    return 0;
}

/* ============================================================================
 * Main
 * ============================================================================ */

int main(void)
{
    printf("=== SGP4 Unit Tests ===\n\n");

    int failed = 0;

    /* Time tests */
    failed += test_unix_to_jd();
    failed += test_jd_to_unix();
    failed += test_time_round_trip();

    /* Coordinate tests */
    failed += test_geodetic_to_ecef();
    failed += test_ecef_geodetic_round_trip();
    failed += test_ecef_to_enu();
    failed += test_look_angles();
    failed += test_look_angles_below_horizon();

    /* TLE tests */
    failed += test_tle_parsing();
    failed += test_tle_to_elements();

    /* SGP4 propagation tests */
    failed += test_sgp4_propagation();
    failed += test_iss_orbital_period();

    /* Vector tests */
    failed += test_vector_operations();

    /* GST tests */
    failed += test_gstime();

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
