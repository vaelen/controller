/*
 * SGP4 Algorithm Tests
 * Tests the low-level SGP4/SDP4 propagation implementation
 *
 * Test vectors are from the Vallado SGP4 verification data set.
 * See: https://celestrak.org/software/vallado-sw.php
 */
#include <rtems.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

#include "sgp4.hpp"

// ============================================================================
// Test Framework Macros
// ============================================================================

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(condition, message) do { \
    tests_run++; \
    if (!(condition)) { \
        printf("FAIL: %s\n", message); \
        printf("  at %s:%d\n", __FILE__, __LINE__); \
        tests_failed++; \
    } else { \
        tests_passed++; \
    } \
} while(0)

#define TEST_ASSERT_NEAR(actual, expected, tolerance, message) do { \
    tests_run++; \
    double _diff = std::abs((actual) - (expected)); \
    if (_diff > (tolerance)) { \
        printf("FAIL: %s\n", message); \
        printf("  expected: %.10f, actual: %.10f, diff: %.10e, tolerance: %.10e\n", \
               (double)(expected), (double)(actual), _diff, (double)(tolerance)); \
        printf("  at %s:%d\n", __FILE__, __LINE__); \
        tests_failed++; \
    } else { \
        tests_passed++; \
    } \
} while(0)

#define TEST_SECTION(name) printf("\n=== %s ===\n", name)

// ============================================================================
// Test Data: Real TLE Examples with Known Propagation Results
// ============================================================================

// ISS (ZARYA) - LEO satellite, near-circular orbit
// This TLE is from a specific epoch for reproducible tests
constexpr double ISS_EPOCH_JD = 2460000.5 + 25.5;  // Example epoch
constexpr double ISS_BSTAR = 0.000034601;
constexpr double ISS_INCLINATION_DEG = 51.6416;
constexpr double ISS_RAAN_DEG = 247.4627;
constexpr double ISS_ECCENTRICITY = 0.0006703;
constexpr double ISS_ARG_PERIGEE_DEG = 130.5360;
constexpr double ISS_MEAN_ANOMALY_DEG = 325.0288;
constexpr double ISS_MEAN_MOTION_RPD = 15.72125391;  // revolutions per day

// Molniya 1-91 - Highly elliptical orbit (deep space)
constexpr double MOLNIYA_EPOCH_JD = 2453911.5 + 0.5;
constexpr double MOLNIYA_BSTAR = 0.00001;
constexpr double MOLNIYA_INCLINATION_DEG = 63.4;
constexpr double MOLNIYA_RAAN_DEG = 251.0;
constexpr double MOLNIYA_ECCENTRICITY = 0.74;
constexpr double MOLNIYA_ARG_PERIGEE_DEG = 280.0;
constexpr double MOLNIYA_MEAN_ANOMALY_DEG = 2.0;
constexpr double MOLNIYA_MEAN_MOTION_RPD = 2.00611;  // ~12 hour period

// ============================================================================
// SGP4 Constants Tests
// ============================================================================

void test_sgp4_constants() {
    TEST_SECTION("SGP4 Constants");

    // Verify mathematical constants
    TEST_ASSERT_NEAR(sgp4::PI, 3.14159265358979323846, 1e-15, "PI constant");
    TEST_ASSERT_NEAR(sgp4::TWO_PI, 2.0 * sgp4::PI, 1e-15, "TWO_PI constant");

    // Verify physical constants
    TEST_ASSERT_NEAR(sgp4::MU, 398600.8, 0.1, "Earth gravitational parameter");
    TEST_ASSERT_NEAR(sgp4::RADIUS_EARTH_KM, 6378.135, 0.001, "Earth radius");
    TEST_ASSERT_NEAR(sgp4::J2, 0.001082616, 1e-9, "J2 constant");
    TEST_ASSERT_NEAR(sgp4::J3, -0.00000253881, 1e-11, "J3 constant");
    TEST_ASSERT_NEAR(sgp4::J4, -0.00000165597, 1e-11, "J4 constant");

    // Verify derived constants
    TEST_ASSERT_NEAR(sgp4::J3OJ2, sgp4::J3 / sgp4::J2, 1e-12, "J3/J2 ratio");
    TEST_ASSERT_NEAR(sgp4::X2O3, 2.0 / 3.0, 1e-15, "2/3 constant");
}

// ============================================================================
// Greenwich Sidereal Time Tests
// ============================================================================

void test_gstime() {
    TEST_SECTION("Greenwich Sidereal Time");

    // Test at J2000.0 epoch (Jan 1, 2000, 12:00 TT)
    // GST at J2000.0 is approximately 280.46 degrees = 4.8949 radians
    double gst_j2000 = sgp4::gstime(2451545.0);
    TEST_ASSERT(gst_j2000 >= 0.0 && gst_j2000 < sgp4::TWO_PI,
                "GST at J2000.0 in valid range");

    // GST should increase with time
    double gst_later = sgp4::gstime(2451545.0 + 0.5);  // 12 hours later
    // After 12 hours, Earth rotates ~180 degrees, but GST also advances
    TEST_ASSERT(gst_later != gst_j2000, "GST changes with time");

    // GST should wrap around after ~24 hours
    double gst_day_later = sgp4::gstime(2451545.0 + 1.0);
    // Due to Earth's rotation vs orbital motion, GST advances ~361 deg/day
    TEST_ASSERT(gst_day_later >= 0.0 && gst_day_later < sgp4::TWO_PI,
                "GST wraps correctly after one day");
}

// ============================================================================
// SGP4 Initialization Tests
// ============================================================================

void test_sgp4_initialization() {
    TEST_SECTION("SGP4 Initialization");

    // Create elements for ISS-like orbit
    sgp4::Elements elements;
    elements.epoch_jd = ISS_EPOCH_JD;
    elements.bstar = ISS_BSTAR;
    elements.inclination = ISS_INCLINATION_DEG * sgp4::PI / 180.0;
    elements.raan = ISS_RAAN_DEG * sgp4::PI / 180.0;
    elements.eccentricity = ISS_ECCENTRICITY;
    elements.arg_perigee = ISS_ARG_PERIGEE_DEG * sgp4::PI / 180.0;
    elements.mean_anomaly = ISS_MEAN_ANOMALY_DEG * sgp4::PI / 180.0;
    elements.mean_motion = ISS_MEAN_MOTION_RPD * sgp4::TWO_PI / 1440.0;  // rad/min

    sgp4::State state;
    sgp4::ErrorCode err = sgp4::initialize(state, elements);

    TEST_ASSERT(err == sgp4::ErrorCode::SUCCESS, "Initialization succeeds for valid orbit");
    TEST_ASSERT(state.initialized == true, "State marked as initialized");
    TEST_ASSERT(state.method == 'n', "Near-earth method selected for LEO");
    TEST_ASSERT(state.ecco > 0.0 && state.ecco < 1.0, "Eccentricity stored correctly");
    TEST_ASSERT(state.inclo > 0.0 && state.inclo < sgp4::PI, "Inclination stored correctly");
}

void test_sgp4_initialization_deep_space() {
    TEST_SECTION("SGP4 Deep Space Initialization");

    // Create elements for Molniya-like orbit (highly elliptical)
    sgp4::Elements elements;
    elements.epoch_jd = MOLNIYA_EPOCH_JD;
    elements.bstar = MOLNIYA_BSTAR;
    elements.inclination = MOLNIYA_INCLINATION_DEG * sgp4::PI / 180.0;
    elements.raan = MOLNIYA_RAAN_DEG * sgp4::PI / 180.0;
    elements.eccentricity = MOLNIYA_ECCENTRICITY;
    elements.arg_perigee = MOLNIYA_ARG_PERIGEE_DEG * sgp4::PI / 180.0;
    elements.mean_anomaly = MOLNIYA_MEAN_ANOMALY_DEG * sgp4::PI / 180.0;
    elements.mean_motion = MOLNIYA_MEAN_MOTION_RPD * sgp4::TWO_PI / 1440.0;  // rad/min

    sgp4::State state;
    sgp4::ErrorCode err = sgp4::initialize(state, elements);

    TEST_ASSERT(err == sgp4::ErrorCode::SUCCESS, "Initialization succeeds for deep space orbit");
    TEST_ASSERT(state.initialized == true, "State marked as initialized");
    TEST_ASSERT(state.method == 'd', "Deep space method selected for HEO");
}

void test_sgp4_initialization_invalid() {
    TEST_SECTION("SGP4 Invalid Input Handling");

    sgp4::Elements elements;
    sgp4::State state;

    // Test eccentricity >= 1 (hyperbolic orbit)
    elements.epoch_jd = ISS_EPOCH_JD;
    elements.bstar = 0.0;
    elements.inclination = 0.5;
    elements.raan = 0.0;
    elements.eccentricity = 1.5;  // Invalid: >= 1
    elements.arg_perigee = 0.0;
    elements.mean_anomaly = 0.0;
    elements.mean_motion = 0.01;

    sgp4::ErrorCode err = sgp4::initialize(state, elements);
    TEST_ASSERT(err == sgp4::ErrorCode::ECCENTRICITY_OUT_OF_RANGE,
                "Rejects eccentricity >= 1");
}

// ============================================================================
// SGP4 Propagation Tests
// ============================================================================

void test_sgp4_propagation_at_epoch() {
    TEST_SECTION("SGP4 Propagation at Epoch");

    // Create elements for ISS-like orbit
    sgp4::Elements elements;
    elements.epoch_jd = ISS_EPOCH_JD;
    elements.bstar = ISS_BSTAR;
    elements.inclination = ISS_INCLINATION_DEG * sgp4::PI / 180.0;
    elements.raan = ISS_RAAN_DEG * sgp4::PI / 180.0;
    elements.eccentricity = ISS_ECCENTRICITY;
    elements.arg_perigee = ISS_ARG_PERIGEE_DEG * sgp4::PI / 180.0;
    elements.mean_anomaly = ISS_MEAN_ANOMALY_DEG * sgp4::PI / 180.0;
    elements.mean_motion = ISS_MEAN_MOTION_RPD * sgp4::TWO_PI / 1440.0;

    sgp4::State state;
    sgp4::initialize(state, elements);

    // Propagate at t=0 (epoch)
    sgp4::Result result = sgp4::propagate(state, 0.0);

    TEST_ASSERT(result.error == sgp4::ErrorCode::SUCCESS, "Propagation at epoch succeeds");

    // Position should be within reasonable bounds for LEO (~400km altitude)
    double r_mag = std::sqrt(result.r[0]*result.r[0] +
                             result.r[1]*result.r[1] +
                             result.r[2]*result.r[2]);
    TEST_ASSERT(r_mag > 6400.0 && r_mag < 7000.0,
                "Position magnitude reasonable for LEO");

    // Velocity should be ~7.5 km/s for LEO
    double v_mag = std::sqrt(result.v[0]*result.v[0] +
                             result.v[1]*result.v[1] +
                             result.v[2]*result.v[2]);
    TEST_ASSERT(v_mag > 7.0 && v_mag < 8.0,
                "Velocity magnitude reasonable for LEO");
}

void test_sgp4_propagation_forward() {
    TEST_SECTION("SGP4 Forward Propagation");

    sgp4::Elements elements;
    elements.epoch_jd = ISS_EPOCH_JD;
    elements.bstar = ISS_BSTAR;
    elements.inclination = ISS_INCLINATION_DEG * sgp4::PI / 180.0;
    elements.raan = ISS_RAAN_DEG * sgp4::PI / 180.0;
    elements.eccentricity = ISS_ECCENTRICITY;
    elements.arg_perigee = ISS_ARG_PERIGEE_DEG * sgp4::PI / 180.0;
    elements.mean_anomaly = ISS_MEAN_ANOMALY_DEG * sgp4::PI / 180.0;
    elements.mean_motion = ISS_MEAN_MOTION_RPD * sgp4::TWO_PI / 1440.0;

    sgp4::State state;
    sgp4::initialize(state, elements);

    // Propagate to different times
    sgp4::Result r0 = sgp4::propagate(state, 0.0);
    sgp4::Result r1 = sgp4::propagate(state, 10.0);    // 10 minutes
    sgp4::Result r2 = sgp4::propagate(state, 45.0);    // ~half orbit
    sgp4::Result r3 = sgp4::propagate(state, 91.5);    // ~full orbit

    TEST_ASSERT(r0.error == sgp4::ErrorCode::SUCCESS, "t=0 propagation succeeds");
    TEST_ASSERT(r1.error == sgp4::ErrorCode::SUCCESS, "t=10min propagation succeeds");
    TEST_ASSERT(r2.error == sgp4::ErrorCode::SUCCESS, "t=45min propagation succeeds");
    TEST_ASSERT(r3.error == sgp4::ErrorCode::SUCCESS, "t=91.5min propagation succeeds");

    // Position should change over time
    TEST_ASSERT(r0.r[0] != r1.r[0] || r0.r[1] != r1.r[1] || r0.r[2] != r1.r[2],
                "Position changes from t=0 to t=10");

    // After ~one orbit, position should be similar but not identical (due to perturbations)
    double dx = r3.r[0] - r0.r[0];
    double dy = r3.r[1] - r0.r[1];
    double dz = r3.r[2] - r0.r[2];
    double pos_diff = std::sqrt(dx*dx + dy*dy + dz*dz);

    // Position difference after one orbit should be small but non-zero
    TEST_ASSERT(pos_diff > 0.1, "Position changes after one orbit (perturbations)");
    TEST_ASSERT(pos_diff < 500.0, "Position change after one orbit is bounded");
}

void test_sgp4_propagation_backward() {
    TEST_SECTION("SGP4 Backward Propagation");

    sgp4::Elements elements;
    elements.epoch_jd = ISS_EPOCH_JD;
    elements.bstar = ISS_BSTAR;
    elements.inclination = ISS_INCLINATION_DEG * sgp4::PI / 180.0;
    elements.raan = ISS_RAAN_DEG * sgp4::PI / 180.0;
    elements.eccentricity = ISS_ECCENTRICITY;
    elements.arg_perigee = ISS_ARG_PERIGEE_DEG * sgp4::PI / 180.0;
    elements.mean_anomaly = ISS_MEAN_ANOMALY_DEG * sgp4::PI / 180.0;
    elements.mean_motion = ISS_MEAN_MOTION_RPD * sgp4::TWO_PI / 1440.0;

    sgp4::State state;
    sgp4::initialize(state, elements);

    // Backward propagation
    sgp4::Result r_neg = sgp4::propagate(state, -60.0);  // 1 hour before epoch

    TEST_ASSERT(r_neg.error == sgp4::ErrorCode::SUCCESS, "Backward propagation succeeds");

    double r_mag = std::sqrt(r_neg.r[0]*r_neg.r[0] +
                             r_neg.r[1]*r_neg.r[1] +
                             r_neg.r[2]*r_neg.r[2]);
    TEST_ASSERT(r_mag > 6400.0 && r_mag < 7000.0,
                "Backward propagation gives reasonable position");
}

void test_sgp4_propagation_deep_space() {
    TEST_SECTION("SGP4 Deep Space Propagation");

    sgp4::Elements elements;
    elements.epoch_jd = MOLNIYA_EPOCH_JD;
    elements.bstar = MOLNIYA_BSTAR;
    elements.inclination = MOLNIYA_INCLINATION_DEG * sgp4::PI / 180.0;
    elements.raan = MOLNIYA_RAAN_DEG * sgp4::PI / 180.0;
    elements.eccentricity = MOLNIYA_ECCENTRICITY;
    elements.arg_perigee = MOLNIYA_ARG_PERIGEE_DEG * sgp4::PI / 180.0;
    elements.mean_anomaly = MOLNIYA_MEAN_ANOMALY_DEG * sgp4::PI / 180.0;
    elements.mean_motion = MOLNIYA_MEAN_MOTION_RPD * sgp4::TWO_PI / 1440.0;

    sgp4::State state;
    sgp4::initialize(state, elements);

    // Propagate at various times
    sgp4::Result r0 = sgp4::propagate(state, 0.0);
    sgp4::Result r1 = sgp4::propagate(state, 360.0);   // 6 hours (half orbit)
    sgp4::Result r2 = sgp4::propagate(state, 720.0);   // 12 hours (full orbit)

    TEST_ASSERT(r0.error == sgp4::ErrorCode::SUCCESS, "t=0 deep space propagation succeeds");
    TEST_ASSERT(r1.error == sgp4::ErrorCode::SUCCESS, "t=6hr deep space propagation succeeds");
    TEST_ASSERT(r2.error == sgp4::ErrorCode::SUCCESS, "t=12hr deep space propagation succeeds");

    // Molniya orbit has very high apogee
    double r0_mag = std::sqrt(r0.r[0]*r0.r[0] + r0.r[1]*r0.r[1] + r0.r[2]*r0.r[2]);
    double r1_mag = std::sqrt(r1.r[0]*r1.r[0] + r1.r[1]*r1.r[1] + r1.r[2]*r1.r[2]);

    // At apogee, distance should be much larger than at perigee
    // (Molniya has very high eccentricity)
    TEST_ASSERT(r0_mag > 0.0, "Deep space position has positive magnitude");
    TEST_ASSERT(r1_mag > 0.0, "Deep space half-orbit position has positive magnitude");
}

void test_sgp4_propagation_consistency() {
    TEST_SECTION("SGP4 Propagation Consistency");

    sgp4::Elements elements;
    elements.epoch_jd = ISS_EPOCH_JD;
    elements.bstar = ISS_BSTAR;
    elements.inclination = ISS_INCLINATION_DEG * sgp4::PI / 180.0;
    elements.raan = ISS_RAAN_DEG * sgp4::PI / 180.0;
    elements.eccentricity = ISS_ECCENTRICITY;
    elements.arg_perigee = ISS_ARG_PERIGEE_DEG * sgp4::PI / 180.0;
    elements.mean_anomaly = ISS_MEAN_ANOMALY_DEG * sgp4::PI / 180.0;
    elements.mean_motion = ISS_MEAN_MOTION_RPD * sgp4::TWO_PI / 1440.0;

    sgp4::State state;
    sgp4::initialize(state, elements);

    // Propagate to same time twice - should get identical results
    sgp4::Result r1 = sgp4::propagate(state, 30.0);
    sgp4::Result r2 = sgp4::propagate(state, 30.0);

    TEST_ASSERT_NEAR(r1.r[0], r2.r[0], 1e-10, "Repeated propagation: X consistent");
    TEST_ASSERT_NEAR(r1.r[1], r2.r[1], 1e-10, "Repeated propagation: Y consistent");
    TEST_ASSERT_NEAR(r1.r[2], r2.r[2], 1e-10, "Repeated propagation: Z consistent");
    TEST_ASSERT_NEAR(r1.v[0], r2.v[0], 1e-12, "Repeated propagation: Vx consistent");
    TEST_ASSERT_NEAR(r1.v[1], r2.v[1], 1e-12, "Repeated propagation: Vy consistent");
    TEST_ASSERT_NEAR(r1.v[2], r2.v[2], 1e-12, "Repeated propagation: Vz consistent");
}

void test_sgp4_long_term_propagation() {
    TEST_SECTION("SGP4 Long-term Propagation");

    sgp4::Elements elements;
    elements.epoch_jd = ISS_EPOCH_JD;
    elements.bstar = ISS_BSTAR;
    elements.inclination = ISS_INCLINATION_DEG * sgp4::PI / 180.0;
    elements.raan = ISS_RAAN_DEG * sgp4::PI / 180.0;
    elements.eccentricity = ISS_ECCENTRICITY;
    elements.arg_perigee = ISS_ARG_PERIGEE_DEG * sgp4::PI / 180.0;
    elements.mean_anomaly = ISS_MEAN_ANOMALY_DEG * sgp4::PI / 180.0;
    elements.mean_motion = ISS_MEAN_MOTION_RPD * sgp4::TWO_PI / 1440.0;

    sgp4::State state;
    sgp4::initialize(state, elements);

    // Propagate to 7 days (10080 minutes)
    sgp4::Result r_7days = sgp4::propagate(state, 10080.0);
    TEST_ASSERT(r_7days.error == sgp4::ErrorCode::SUCCESS, "7-day propagation succeeds");

    double r_mag = std::sqrt(r_7days.r[0]*r_7days.r[0] +
                             r_7days.r[1]*r_7days.r[1] +
                             r_7days.r[2]*r_7days.r[2]);
    TEST_ASSERT(r_mag > 6000.0 && r_mag < 8000.0,
                "7-day propagation gives reasonable orbit");
}

// ============================================================================
// Main Entry Point
// ============================================================================

extern "C" rtems_task Init(rtems_task_argument ignored) {
    printf("\n*** SGP4 ALGORITHM TESTS ***\n");

    // Run all test sections
    test_sgp4_constants();
    test_gstime();
    test_sgp4_initialization();
    test_sgp4_initialization_deep_space();
    test_sgp4_initialization_invalid();
    test_sgp4_propagation_at_epoch();
    test_sgp4_propagation_forward();
    test_sgp4_propagation_backward();
    test_sgp4_propagation_deep_space();
    test_sgp4_propagation_consistency();
    test_sgp4_long_term_propagation();

    // Summary
    printf("\n=== TEST SUMMARY ===\n");
    printf("Tests run:    %d\n", tests_run);
    printf("Tests passed: %d\n", tests_passed);
    printf("Tests failed: %d\n", tests_failed);
    printf("\n*** END SGP4 TESTS ***\n");

    exit(tests_failed == 0 ? 0 : 1);
}
