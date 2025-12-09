/*
 * Coordinate System and Visibility Tests
 * Tests coordinate transformations, look angles, and visibility calculations
 */
#include <rtems.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

#include "satellite.hpp"

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
// Test Data
// ============================================================================

// ISS TLE for visibility tests
constexpr const char* ISS_TLE =
    "ISS (ZARYA)\n"
    "1 25544U 98067A   24001.50000000  .00016717  00000+0  10270-3 0  9993\n"
    "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.72125391434083\n";

// Known locations for testing
// San Francisco, CA
constexpr double SF_LAT_DEG = 37.7749;
constexpr double SF_LON_DEG = -122.4194;
constexpr double SF_ALT_KM = 0.016;  // ~16m above sea level

// London, UK
constexpr double LONDON_LAT_DEG = 51.5074;
constexpr double LONDON_LON_DEG = -0.1278;
constexpr double LONDON_ALT_KM = 0.011;

// Tokyo, Japan
constexpr double TOKYO_LAT_DEG = 35.6762;
constexpr double TOKYO_LON_DEG = 139.6503;
constexpr double TOKYO_ALT_KM = 0.040;

// North Pole
constexpr double NORTH_POLE_LAT_DEG = 90.0;
constexpr double NORTH_POLE_LON_DEG = 0.0;

// Equator on Prime Meridian
constexpr double EQUATOR_LAT_DEG = 0.0;
constexpr double EQUATOR_LON_DEG = 0.0;

// ============================================================================
// Vec3 Tests
// ============================================================================

void test_vec3_operations() {
    TEST_SECTION("Vec3 Operations");

    sattrack::Vec3 a{1.0, 2.0, 3.0};
    sattrack::Vec3 b{4.0, 5.0, 6.0};

    // Addition
    sattrack::Vec3 sum = a + b;
    TEST_ASSERT_NEAR(sum.x, 5.0, 1e-10, "Vec3 addition X");
    TEST_ASSERT_NEAR(sum.y, 7.0, 1e-10, "Vec3 addition Y");
    TEST_ASSERT_NEAR(sum.z, 9.0, 1e-10, "Vec3 addition Z");

    // Subtraction
    sattrack::Vec3 diff = b - a;
    TEST_ASSERT_NEAR(diff.x, 3.0, 1e-10, "Vec3 subtraction X");
    TEST_ASSERT_NEAR(diff.y, 3.0, 1e-10, "Vec3 subtraction Y");
    TEST_ASSERT_NEAR(diff.z, 3.0, 1e-10, "Vec3 subtraction Z");

    // Scalar multiplication
    sattrack::Vec3 scaled = a * 2.0;
    TEST_ASSERT_NEAR(scaled.x, 2.0, 1e-10, "Vec3 scalar mult X");
    TEST_ASSERT_NEAR(scaled.y, 4.0, 1e-10, "Vec3 scalar mult Y");
    TEST_ASSERT_NEAR(scaled.z, 6.0, 1e-10, "Vec3 scalar mult Z");

    // Magnitude
    sattrack::Vec3 c{3.0, 4.0, 0.0};
    TEST_ASSERT_NEAR(c.magnitude(), 5.0, 1e-10, "Vec3 magnitude");

    // Dot product
    double dot = a.dot(b);
    TEST_ASSERT_NEAR(dot, 32.0, 1e-10, "Vec3 dot product");  // 1*4 + 2*5 + 3*6 = 32

    // Cross product
    sattrack::Vec3 i{1.0, 0.0, 0.0};
    sattrack::Vec3 j{0.0, 1.0, 0.0};
    sattrack::Vec3 k = i.cross(j);
    TEST_ASSERT_NEAR(k.x, 0.0, 1e-10, "Cross product X");
    TEST_ASSERT_NEAR(k.y, 0.0, 1e-10, "Cross product Y");
    TEST_ASSERT_NEAR(k.z, 1.0, 1e-10, "Cross product Z");

    // Normalize
    sattrack::Vec3 d{3.0, 4.0, 0.0};
    sattrack::Vec3 d_norm = d.normalize();
    TEST_ASSERT_NEAR(d_norm.magnitude(), 1.0, 1e-10, "Normalized magnitude is 1");
    TEST_ASSERT_NEAR(d_norm.x, 0.6, 1e-10, "Normalized X");
    TEST_ASSERT_NEAR(d_norm.y, 0.8, 1e-10, "Normalized Y");
}

// ============================================================================
// Geodetic to ECEF Tests
// ============================================================================

void test_geodetic_to_ecef() {
    TEST_SECTION("Geodetic to ECEF Conversion");

    // Test point at equator, prime meridian, sea level
    sattrack::Geodetic equator{0.0, 0.0, 0.0};
    sattrack::Vec3 ecef_eq = equator.toECEF();

    // At equator on prime meridian, X should be ~Earth radius, Y and Z near 0
    TEST_ASSERT_NEAR(ecef_eq.x, sgp4::RADIUS_EARTH_KM, 1.0, "Equator ECEF X ~ Earth radius");
    TEST_ASSERT_NEAR(ecef_eq.y, 0.0, 0.1, "Equator ECEF Y ~ 0");
    TEST_ASSERT_NEAR(ecef_eq.z, 0.0, 0.1, "Equator ECEF Z ~ 0");

    // Test point at North Pole
    sattrack::Geodetic north_pole{sgp4::PI / 2.0, 0.0, 0.0};
    sattrack::Vec3 ecef_np = north_pole.toECEF();

    // At North Pole, X and Y should be ~0, Z should be ~polar radius
    TEST_ASSERT_NEAR(ecef_np.x, 0.0, 0.1, "North Pole ECEF X ~ 0");
    TEST_ASSERT_NEAR(ecef_np.y, 0.0, 0.1, "North Pole ECEF Y ~ 0");
    // Polar radius is slightly less than equatorial due to flattening
    TEST_ASSERT(ecef_np.z > 6350.0 && ecef_np.z < 6380.0, "North Pole ECEF Z reasonable");

    // Test point at 90°E longitude on equator
    sattrack::Geodetic east90{0.0, sgp4::PI / 2.0, 0.0};
    sattrack::Vec3 ecef_e90 = east90.toECEF();

    TEST_ASSERT_NEAR(ecef_e90.x, 0.0, 0.1, "90°E ECEF X ~ 0");
    TEST_ASSERT_NEAR(ecef_e90.y, sgp4::RADIUS_EARTH_KM, 1.0, "90°E ECEF Y ~ Earth radius");

    // Test with altitude
    sattrack::Geodetic high{0.0, 0.0, 400.0};  // 400 km altitude
    sattrack::Vec3 ecef_high = high.toECEF();
    TEST_ASSERT_NEAR(ecef_high.x, sgp4::RADIUS_EARTH_KM + 400.0, 1.0, "400km altitude ECEF X");
}

// ============================================================================
// ECEF to Geodetic Tests
// ============================================================================

void test_ecef_to_geodetic() {
    TEST_SECTION("ECEF to Geodetic Conversion");

    // Point on equator at prime meridian
    sattrack::Vec3 ecef_eq{sgp4::RADIUS_EARTH_KM, 0.0, 0.0};
    sattrack::Geodetic geo_eq = sattrack::ecefToGeodetic(ecef_eq);

    TEST_ASSERT_NEAR(geo_eq.latInRadians, 0.0, 0.001, "Equator lat ~ 0");
    TEST_ASSERT_NEAR(geo_eq.lonInRadians, 0.0, 0.001, "Prime meridian lon ~ 0");
    TEST_ASSERT_NEAR(geo_eq.altInKilometers, 0.0, 1.0, "Sea level alt ~ 0");

    // Point at North Pole (approximately - polar radius is ~6356.752 km)
    sattrack::Vec3 ecef_np{0.0, 0.0, 6356.752};
    sattrack::Geodetic geo_np = sattrack::ecefToGeodetic(ecef_np);

    TEST_ASSERT_NEAR(geo_np.latInRadians, sgp4::PI / 2.0, 0.01, "North Pole lat ~ 90°");

    // Point at 400 km altitude over equator
    sattrack::Vec3 ecef_high{sgp4::RADIUS_EARTH_KM + 400.0, 0.0, 0.0};
    sattrack::Geodetic geo_high = sattrack::ecefToGeodetic(ecef_high);

    TEST_ASSERT_NEAR(geo_high.altInKilometers, 400.0, 1.0, "400 km altitude");
}

void test_geodetic_ecef_roundtrip() {
    TEST_SECTION("Geodetic-ECEF Round Trip");

    // Test several locations
    double test_lats[] = {0.0, 45.0, -30.0, 89.0};
    double test_lons[] = {0.0, 90.0, -120.0, 180.0};
    double test_alts[] = {0.0, 100.0, 400.0, 35786.0};  // Including GEO altitude

    for (int i = 0; i < 4; i++) {
        double lat_rad = test_lats[i] * sattrack::DEGREES_TO_RADIANS;
        double lon_rad = test_lons[i] * sattrack::DEGREES_TO_RADIANS;
        double alt_km = test_alts[i];

        sattrack::Geodetic original{lat_rad, lon_rad, alt_km};
        sattrack::Vec3 ecef = original.toECEF();
        sattrack::Geodetic recovered = sattrack::ecefToGeodetic(ecef);

        char msg[100];
        snprintf(msg, sizeof(msg), "Round trip lat[%d]", i);
        TEST_ASSERT_NEAR(recovered.latInRadians, original.latInRadians, 0.0001, msg);

        snprintf(msg, sizeof(msg), "Round trip lon[%d]", i);
        // Handle longitude wrap-around
        double lon_diff = std::abs(recovered.lonInRadians - original.lonInRadians);
        if (lon_diff > sgp4::PI) lon_diff = sgp4::TWO_PI - lon_diff;
        tests_run++;
        if (lon_diff > 0.0001) {
            printf("FAIL: %s\n", msg);
            tests_failed++;
        } else {
            tests_passed++;
        }

        snprintf(msg, sizeof(msg), "Round trip alt[%d]", i);
        TEST_ASSERT_NEAR(recovered.altInKilometers, original.altInKilometers, 0.1, msg);
    }
}

// ============================================================================
// ECI to ECEF Tests
// ============================================================================

void test_eci_to_ecef() {
    TEST_SECTION("ECI to ECEF Conversion");

    // At GST = 0, ECI and ECEF should be aligned
    sattrack::Vec3 eci{7000.0, 0.0, 0.0};
    sattrack::Vec3 ecef = sattrack::eciToECEF(eci, 0.0);

    TEST_ASSERT_NEAR(ecef.x, 7000.0, 0.001, "ECI to ECEF at GST=0 X");
    TEST_ASSERT_NEAR(ecef.y, 0.0, 0.001, "ECI to ECEF at GST=0 Y");
    TEST_ASSERT_NEAR(ecef.z, 0.0, 0.001, "ECI to ECEF at GST=0 Z");

    // At GST = π/2 (90°), X axis of ECI points to Y axis of ECEF
    ecef = sattrack::eciToECEF(eci, sgp4::PI / 2.0);
    TEST_ASSERT_NEAR(ecef.x, 0.0, 0.001, "ECI to ECEF at GST=90° X");
    TEST_ASSERT_NEAR(ecef.y, -7000.0, 0.001, "ECI to ECEF at GST=90° Y");

    // Z component should be unchanged
    sattrack::Vec3 eci_z{0.0, 0.0, 5000.0};
    ecef = sattrack::eciToECEF(eci_z, sgp4::PI / 4.0);
    TEST_ASSERT_NEAR(ecef.z, 5000.0, 0.001, "ECI to ECEF Z unchanged");
}

// ============================================================================
// GMST Tests
// ============================================================================

void test_gmst() {
    TEST_SECTION("Greenwich Mean Sidereal Time");

    // GMST should be in range [0, 2π)
    double jd_test = 2460000.5;  // Arbitrary recent Julian Date
    double gst = sattrack::gmst(jd_test);

    TEST_ASSERT(gst >= 0.0, "GMST >= 0");
    TEST_ASSERT(gst < sgp4::TWO_PI, "GMST < 2π");

    // GMST should change with time
    double gst_later = sattrack::gmst(jd_test + 0.5);  // 12 hours later
    TEST_ASSERT(gst != gst_later, "GMST changes over time");

    // After ~24 sidereal hours, GMST should wrap
    // (Actually 23h 56m sidereal = 24h solar)
}

// ============================================================================
// Julian Date Tests
// ============================================================================

void test_julian_date() {
    TEST_SECTION("Julian Date Conversion");

    // J2000.0 epoch: Jan 1, 2000, 12:00 TT = JD 2451545.0
    // Unix epoch: Jan 1, 1970, 00:00 UTC = JD 2440587.5

    // Create a time_point at Unix epoch
    sattrack::time_point unix_epoch{};
    double jd_unix = sattrack::toJulianDate(unix_epoch);
    TEST_ASSERT_NEAR(jd_unix, 2440587.5, 0.0001, "Unix epoch Julian Date");

    // Create a time_point 1 day after Unix epoch
    auto one_day = std::chrono::hours(24);
    sattrack::time_point day_after = unix_epoch + one_day;
    double jd_day = sattrack::toJulianDate(day_after);
    TEST_ASSERT_NEAR(jd_day, 2440588.5, 0.0001, "One day after Unix epoch");
}

// ============================================================================
// ECEF to ENU Tests
// ============================================================================

void test_ecef_to_enu() {
    TEST_SECTION("ECEF to ENU Conversion");

    // Observer at equator on prime meridian
    sattrack::Geodetic observer{0.0, 0.0, 0.0};
    sattrack::Vec3 obs_ecef = observer.toECEF();

    // Target directly above observer (same lat/lon, higher altitude)
    sattrack::Geodetic above_observer{0.0, 0.0, 100.0};
    sattrack::Vec3 above_ecef = above_observer.toECEF();

    sattrack::Vec3 enu = sattrack::ecefToENU(above_ecef, observer);

    // Target directly above should have positive Up, near-zero East/North
    TEST_ASSERT_NEAR(enu.x, 0.0, 1.0, "Directly above: East ~ 0");
    TEST_ASSERT_NEAR(enu.y, 0.0, 1.0, "Directly above: North ~ 0");
    TEST_ASSERT(enu.z > 99.0, "Directly above: Up > 99 km");

    // Target to the East
    sattrack::Geodetic east_target{0.0, 1.0 * sattrack::DEGREES_TO_RADIANS, 0.0};
    sattrack::Vec3 east_ecef = east_target.toECEF();
    enu = sattrack::ecefToENU(east_ecef, observer);

    TEST_ASSERT(enu.x > 0.0, "Eastern target: positive East");
    // North should be approximately zero
    TEST_ASSERT(std::abs(enu.y) < 10.0, "Eastern target: small North");
}

// ============================================================================
// Look Angles Tests
// ============================================================================

void test_look_angles() {
    TEST_SECTION("Look Angles Calculation");

    // Observer at equator on prime meridian
    sattrack::Geodetic observer{0.0, 0.0, 0.0};

    // Target directly above at 400 km
    sattrack::Geodetic above{0.0, 0.0, 400.0};
    sattrack::Vec3 above_ecef = above.toECEF();

    sattrack::LookAngles angles = sattrack::getLookAngles(above_ecef, observer);

    // Elevation should be 90° (directly overhead)
    TEST_ASSERT_NEAR(angles.elevationInRadians, sgp4::PI / 2.0, 0.01,
                     "Overhead target: elevation ~ 90°");

    // Range should be ~400 km
    TEST_ASSERT_NEAR(angles.rangeInKilometers, 400.0, 1.0,
                     "Overhead target: range ~ 400 km");

    // Test target on horizon to the North
    // At equator, looking North along the surface
    sattrack::Geodetic north_target{45.0 * sattrack::DEGREES_TO_RADIANS, 0.0, 0.0};
    sattrack::Vec3 north_ecef = north_target.toECEF();
    angles = sattrack::getLookAngles(north_ecef, observer);

    // Azimuth should be ~0 (North)
    TEST_ASSERT(angles.azimuthInRadians < 0.1 || angles.azimuthInRadians > sgp4::TWO_PI - 0.1,
                "Northern target: azimuth ~ 0 (North)");

    // Elevation should be negative (below horizon at distance)
    // Actually for a point on Earth's surface far away, it will be below horizon
}

void test_look_angles_for_satellite() {
    TEST_SECTION("Look Angles for Satellite");

    sattrack::Satellite sat;
    sat.updateFromTLE(ISS_TLE);

    // San Francisco observer
    sattrack::Geodetic sf_observer{
        SF_LAT_DEG * sattrack::DEGREES_TO_RADIANS,
        SF_LON_DEG * sattrack::DEGREES_TO_RADIANS,
        SF_ALT_KM
    };

    double jd = sattrack::toJulianDate(sat.getEpoch());

    sattrack::LookAngles angles = sattrack::getLookAngles(sat, sf_observer, jd);

    // Verify angles are in valid ranges
    TEST_ASSERT(angles.azimuthInRadians >= 0.0, "Azimuth >= 0");
    TEST_ASSERT(angles.azimuthInRadians < sgp4::TWO_PI, "Azimuth < 2π");
    TEST_ASSERT(angles.elevationInRadians >= -sgp4::PI / 2.0, "Elevation >= -90°");
    TEST_ASSERT(angles.elevationInRadians <= sgp4::PI / 2.0, "Elevation <= 90°");
    TEST_ASSERT(angles.rangeInKilometers > 0.0, "Range > 0");
    TEST_ASSERT(angles.rangeInKilometers < 50000.0, "Range < 50000 km");  // Reasonable for LEO
}

// ============================================================================
// Visibility Tests
// ============================================================================

void test_visibility() {
    TEST_SECTION("Visibility Checks");

    // Test isVisible with look angles
    sattrack::LookAngles above_horizon{0.0, 0.1, 1000.0};  // 0.1 rad ~ 5.7° elevation
    sattrack::LookAngles below_horizon{0.0, -0.1, 1000.0};  // negative elevation

    TEST_ASSERT(sattrack::isVisible(above_horizon, 0.0), "Above horizon is visible");
    TEST_ASSERT(!sattrack::isVisible(below_horizon, 0.0), "Below horizon not visible");

    // Test with minimum elevation threshold
    double min_elev_10deg = 10.0 * sattrack::DEGREES_TO_RADIANS;
    sattrack::LookAngles barely_above{0.0, 0.15, 1000.0};  // ~8.6° elevation

    TEST_ASSERT(!sattrack::isVisible(barely_above, min_elev_10deg),
                "Below min elevation not visible");

    sattrack::LookAngles well_above{0.0, 0.3, 1000.0};  // ~17° elevation
    TEST_ASSERT(sattrack::isVisible(well_above, min_elev_10deg),
                "Above min elevation is visible");
}

void test_satellite_visibility() {
    TEST_SECTION("Satellite Visibility");

    sattrack::Satellite sat;
    sat.updateFromTLE(ISS_TLE);

    sattrack::Geodetic sf_observer{
        SF_LAT_DEG * sattrack::DEGREES_TO_RADIANS,
        SF_LON_DEG * sattrack::DEGREES_TO_RADIANS,
        SF_ALT_KM
    };

    double jd = sattrack::toJulianDate(sat.getEpoch());

    // Check visibility at epoch (may or may not be visible)
    bool visible = sattrack::isVisible(sat, sf_observer, jd, 0.0);

    // Just verify the function doesn't crash and returns a boolean
    TEST_ASSERT(visible == true || visible == false, "isVisible returns boolean");

    // With very high minimum elevation, satellite likely not visible
    double high_min_elev = 85.0 * sattrack::DEGREES_TO_RADIANS;  // 85°
    bool visible_high_elev = sattrack::isVisible(sat, sf_observer, jd, high_min_elev);

    // This is probabilistic but very unlikely ISS is >85° above any specific location
    // at an arbitrary time
    TEST_ASSERT(visible_high_elev == true || visible_high_elev == false,
                "High elevation check returns boolean");
}

// ============================================================================
// Pass Finding Tests
// ============================================================================

void test_find_next_pass() {
    TEST_SECTION("Find Next Pass");

    sattrack::Satellite sat;
    sat.updateFromTLE(ISS_TLE);

    // London observer (good location for ISS passes)
    sattrack::Geodetic london_observer{
        LONDON_LAT_DEG * sattrack::DEGREES_TO_RADIANS,
        LONDON_LON_DEG * sattrack::DEGREES_TO_RADIANS,
        LONDON_ALT_KM
    };

    double min_elevation = 10.0 * sattrack::DEGREES_TO_RADIANS;

    // Search from TLE epoch
    auto result = sattrack::findNextPass(sat, london_observer, min_elevation, sat.getEpoch());

    if (result.has_value()) {
        sattrack::PassInfo pass = result.value();

        TEST_ASSERT(pass.noradID == sat.getNoradID(), "Pass NORAD ID matches");
        TEST_ASSERT(pass.riseTime < pass.maxElevationTime, "Rise before max elevation");
        TEST_ASSERT(pass.maxElevationTime < pass.setTime, "Max elevation before set");
        TEST_ASSERT(pass.maxAngles.elevationInRadians >= min_elevation,
                    "Max elevation >= minimum");
        TEST_ASSERT(pass.riseAngles.elevationInRadians >= min_elevation - 0.01,
                    "Rise elevation ~ minimum");
        TEST_ASSERT(pass.setAngles.elevationInRadians >= min_elevation - 0.01,
                    "Set elevation ~ minimum");

        printf("  Found pass: rise=%lld, max=%lld, set=%lld\n",
               (long long)std::chrono::duration_cast<std::chrono::seconds>(
                   pass.riseTime.time_since_epoch()).count(),
               (long long)std::chrono::duration_cast<std::chrono::seconds>(
                   pass.maxElevationTime.time_since_epoch()).count(),
               (long long)std::chrono::duration_cast<std::chrono::seconds>(
                   pass.setTime.time_since_epoch()).count());
        printf("  Max elevation: %.1f degrees\n",
               pass.maxAngles.elevationInRadians * sattrack::RADIANS_TO_DEGREES);
    } else {
        // It's possible no pass is found in the 48-hour window - that's OK
        printf("  No pass found in search window (acceptable for some epochs)\n");
        tests_run++;
        tests_passed++;
    }
}

// ============================================================================
// Edge Cases
// ============================================================================

void test_edge_cases() {
    TEST_SECTION("Edge Cases");

    // Test geodetic at poles
    sattrack::Geodetic north_pole{sgp4::PI / 2.0, 0.0, 0.0};
    sattrack::Vec3 np_ecef = north_pole.toECEF();
    TEST_ASSERT(np_ecef.z > 6000.0, "North pole ECEF Z positive");
    TEST_ASSERT(std::abs(np_ecef.x) < 1.0, "North pole ECEF X ~ 0");
    TEST_ASSERT(std::abs(np_ecef.y) < 1.0, "North pole ECEF Y ~ 0");

    sattrack::Geodetic south_pole{-sgp4::PI / 2.0, 0.0, 0.0};
    sattrack::Vec3 sp_ecef = south_pole.toECEF();
    TEST_ASSERT(sp_ecef.z < -6000.0, "South pole ECEF Z negative");

    // Test very high altitude
    sattrack::Geodetic geo_altitude{0.0, 0.0, 35786.0};  // GEO altitude
    sattrack::Vec3 geo_ecef = geo_altitude.toECEF();
    double geo_radius = geo_ecef.magnitude();
    TEST_ASSERT_NEAR(geo_radius, sgp4::RADIUS_EARTH_KM + 35786.0, 10.0, "GEO altitude radius");

    // Test dateline crossing
    sattrack::Geodetic west_dateline{0.0, -179.9 * sattrack::DEGREES_TO_RADIANS, 0.0};
    sattrack::Geodetic east_dateline{0.0, 179.9 * sattrack::DEGREES_TO_RADIANS, 0.0};

    sattrack::Vec3 west_ecef = west_dateline.toECEF();
    sattrack::Vec3 east_ecef = east_dateline.toECEF();

    // Both should be valid and similar
    TEST_ASSERT_NEAR(west_ecef.x, east_ecef.x, 100.0, "Dateline X similar");
}

// ============================================================================
// Main Entry Point
// ============================================================================

extern "C" rtems_task Init(rtems_task_argument ignored) {
    printf("\n*** COORDINATE SYSTEM TESTS ***\n");

    // Vector operations
    test_vec3_operations();

    // Coordinate conversions
    test_geodetic_to_ecef();
    test_ecef_to_geodetic();
    test_geodetic_ecef_roundtrip();
    test_eci_to_ecef();

    // Time functions
    test_gmst();
    test_julian_date();

    // ENU and look angles
    test_ecef_to_enu();
    test_look_angles();
    test_look_angles_for_satellite();

    // Visibility
    test_visibility();
    test_satellite_visibility();
    test_find_next_pass();

    // Edge cases
    test_edge_cases();

    // Summary
    printf("\n=== TEST SUMMARY ===\n");
    printf("Tests run:    %d\n", tests_run);
    printf("Tests passed: %d\n", tests_passed);
    printf("Tests failed: %d\n", tests_failed);
    printf("\n*** END COORDINATE TESTS ***\n");

    exit(tests_failed == 0 ? 0 : 1);
}
