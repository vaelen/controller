/*
 * Satellite Class Tests
 * Tests TLE parsing, round-trip serialization, and high-level satellite tracking
 */
#include <rtems.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <sstream>

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

#define TEST_ASSERT_STR_EQ(actual, expected, message) do { \
    tests_run++; \
    if (std::strcmp((actual), (expected)) != 0) { \
        printf("FAIL: %s\n", message); \
        printf("  expected: \"%s\"\n  actual:   \"%s\"\n", (expected), (actual)); \
        printf("  at %s:%d\n", __FILE__, __LINE__); \
        tests_failed++; \
    } else { \
        tests_passed++; \
    } \
} while(0)

#define TEST_SECTION(name) printf("\n=== %s ===\n", name)

// ============================================================================
// Real TLE Test Data
// ============================================================================

// ISS (ZARYA) - Real TLE from CelesTrak (Dec 2025)
// Checksum calculation: sum all digits, '-' counts as 1, mod 10
constexpr const char* ISS_TLE_3LINE =
    "ISS (ZARYA)\n"
    "1 25544U 98067A   25342.86487849  .00012323  00000+0  22804-3 0  9999\n"
    "2 25544  51.6308 161.6653 0003381 221.9847 138.0883 15.49428834542258\n";

// ISS without name line (2-line format)
constexpr const char* ISS_TLE_2LINE =
    "1 25544U 98067A   25342.86487849  .00012323  00000+0  22804-3 0  9999\n"
    "2 25544  51.6308 161.6653 0003381 221.9847 138.0883 15.49428834542258\n";

// NOAA 19 - Polar orbiting weather satellite
constexpr const char* NOAA19_TLE =
    "NOAA 19\n"
    "1 33591U 09005A   24001.51819444  .00000119  00000+0  77800-4 0  9995\n"
    "2 33591  99.1933 336.5612 0013750 157.3549 202.8298 14.12540753770123\n";

// GOES 16 - Geostationary satellite
constexpr const char* GOES16_TLE =
    "GOES 16\n"
    "1 41866U 16071A   24001.50000000 -.00000097  00000+0  00000+0 0  9998\n"
    "2 41866   0.0415 273.6438 0001295 115.7568 322.5471  1.00271447 26197\n";

// GPS satellite (MEO)
constexpr const char* GPS_TLE =
    "GPS BIIR-2  (PRN 13)\n"
    "1 24876U 97035A   24001.12345678  .00000003  00000+0  00000+0 0  9999\n"
    "2 24876  55.7219  58.8468 0054812 119.9854 240.6230  2.00560527195678\n";

// Starlink satellite - Very low drag
constexpr const char* STARLINK_TLE =
    "STARLINK-1007\n"
    "1 44713U 19074A   24001.50000000  .00001234  00000+0  12345-4 0  9991\n"
    "2 44713  53.0000 100.0000 0001234  90.0000 270.0000 15.05000000 12345\n";

// ============================================================================
// TLE Parsing Tests
// ============================================================================

void test_tle_parsing_3line() {
    TEST_SECTION("TLE Parsing - 3-Line Format");

    sattrack::Satellite sat;
    sat.updateFromTLE(ISS_TLE_3LINE);

    TEST_ASSERT_STR_EQ(sat.getName(), "ISS (ZARYA)", "Name parsed correctly");
    TEST_ASSERT(sat.getNoradID() == 25544, "NORAD ID parsed correctly");
    TEST_ASSERT(sat.getClassification() == 'U', "Classification parsed correctly");
    TEST_ASSERT_STR_EQ(sat.getDesignator(), "98067A  ", "Designator parsed correctly");

    TEST_ASSERT_NEAR(sat.getInclination(), 51.6308, 0.0001, "Inclination parsed correctly");
    TEST_ASSERT_NEAR(sat.getRightAscensionOfAscendingNode(), 161.6653, 0.0001, "RAAN parsed correctly");
    TEST_ASSERT_NEAR(sat.getEccentricity(), 0.0003381, 0.0000001, "Eccentricity parsed correctly");
    TEST_ASSERT_NEAR(sat.getArgumentOfPerigee(), 221.9847, 0.0001, "Arg of Perigee parsed correctly");
    TEST_ASSERT_NEAR(sat.getMeanAnomaly(), 138.0883, 0.0001, "Mean Anomaly parsed correctly");
    TEST_ASSERT_NEAR(sat.getMeanMotion(), 15.49428834, 0.00000001, "Mean Motion parsed correctly");

    TEST_ASSERT(sat.getRevolutionNumberAtEpoch() == 54225, "Rev number parsed correctly");
    TEST_ASSERT(sat.getElementSetNumber() == 999, "Element set number parsed correctly");
}

void test_tle_parsing_2line() {
    TEST_SECTION("TLE Parsing - 2-Line Format");

    sattrack::Satellite sat;
    sat.updateFromTLE(ISS_TLE_2LINE);

    TEST_ASSERT(sat.getNoradID() == 25544, "NORAD ID from 2-line TLE");
    TEST_ASSERT_NEAR(sat.getInclination(), 51.6308, 0.0001, "Inclination from 2-line TLE");
    TEST_ASSERT_NEAR(sat.getEccentricity(), 0.0003381, 0.0000001, "Eccentricity from 2-line TLE");
}

void test_tle_parsing_with_separate_name() {
    TEST_SECTION("TLE Parsing - Separate Name");

    sattrack::Satellite sat;
    sat.updateFromTLE("MY CUSTOM NAME", ISS_TLE_2LINE);

    TEST_ASSERT_STR_EQ(sat.getName(), "MY CUSTOM NAME", "Custom name used");
    TEST_ASSERT(sat.getNoradID() == 25544, "NORAD ID still correct");
}

void test_tle_parsing_exponential_values() {
    TEST_SECTION("TLE Parsing - Exponential Values");

    sattrack::Satellite sat;
    sat.updateFromTLE(ISS_TLE_3LINE);

    // First derivative: .00012323
    TEST_ASSERT_NEAR(sat.getFirstDerivativeMeanMotion(), 0.00012323, 0.00000001,
                     "First derivative parsed");

    // Second derivative: 00000+0 = 0.0
    TEST_ASSERT_NEAR(sat.getSecondDerivativeMeanMotion(), 0.0, 1e-10,
                     "Second derivative (zero) parsed");

    // BSTAR: 22804-3 = 0.00022804
    TEST_ASSERT_NEAR(sat.getBstarDragTerm(), 0.00022804, 0.00000001,
                     "BSTAR parsed correctly");
}

void test_tle_parsing_negative_values() {
    TEST_SECTION("TLE Parsing - Negative Exponential Values");

    sattrack::Satellite sat;
    sat.updateFromTLE(GOES16_TLE);

    // GOES 16 has negative first derivative: -.00000097
    TEST_ASSERT(sat.getFirstDerivativeMeanMotion() < 0,
                "Negative first derivative");
}

void test_tle_parsing_various_satellites() {
    TEST_SECTION("TLE Parsing - Various Satellite Types");

    // NOAA 19 - Polar orbit
    sattrack::Satellite noaa;
    noaa.updateFromTLE(NOAA19_TLE);
    TEST_ASSERT(noaa.getNoradID() == 33591, "NOAA 19 NORAD ID");
    TEST_ASSERT_NEAR(noaa.getInclination(), 99.1933, 0.0001, "NOAA 19 high inclination");

    // GOES 16 - Geostationary
    sattrack::Satellite goes;
    goes.updateFromTLE(GOES16_TLE);
    TEST_ASSERT(goes.getNoradID() == 41866, "GOES 16 NORAD ID");
    TEST_ASSERT_NEAR(goes.getInclination(), 0.0415, 0.001, "GOES 16 near-zero inclination");
    TEST_ASSERT_NEAR(goes.getMeanMotion(), 1.00271447, 0.00000001, "GOES 16 ~1 rev/day");

    // GPS - MEO
    sattrack::Satellite gps;
    gps.updateFromTLE(GPS_TLE);
    TEST_ASSERT(gps.getNoradID() == 24876, "GPS NORAD ID");
    TEST_ASSERT_NEAR(gps.getMeanMotion(), 2.00560527, 0.00000001, "GPS ~2 rev/day");
}

// ============================================================================
// TLE Round-Trip Tests
// ============================================================================

// Helper to compare TLE lines character by character, reporting first difference
static bool compareTLELines(const char* line1, const char* line2, int lineNum) {
    size_t len1 = std::strlen(line1);
    size_t len2 = std::strlen(line2);

    // Find first newline to get line length
    const char* nl1 = std::strchr(line1, '\n');
    const char* nl2 = std::strchr(line2, '\n');
    size_t lineLen1 = nl1 ? (size_t)(nl1 - line1) : len1;
    size_t lineLen2 = nl2 ? (size_t)(nl2 - line2) : len2;

    if (lineLen1 != lineLen2) {
        printf("  Line %d length mismatch: %zu vs %zu\n", lineNum, lineLen1, lineLen2);
        return false;
    }

    for (size_t i = 0; i < lineLen1; i++) {
        if (line1[i] != line2[i]) {
            printf("  Line %d char %zu: '%c' (0x%02x) vs '%c' (0x%02x)\n",
                   lineNum, i, line1[i], (unsigned char)line1[i],
                   line2[i], (unsigned char)line2[i]);
            return false;
        }
    }
    return true;
}

void test_tle_roundtrip_iss() {
    TEST_SECTION("TLE Round-Trip - ISS");

    sattrack::Satellite sat;
    sat.updateFromTLE(ISS_TLE_3LINE);

    sattrack::TLEString output = sat.getTLE();

    // Compare line by line
    const char* original = ISS_TLE_3LINE;
    const char* generated = output.data;

    // Parse into lines for comparison
    bool match = true;

    // Line 0 (name)
    const char* orig_line0 = original;
    const char* gen_line0 = generated;
    if (!compareTLELines(orig_line0, gen_line0, 0)) {
        printf("  Original: %.69s\n", orig_line0);
        printf("  Generated: %.69s\n", gen_line0);
        match = false;
    }

    TEST_ASSERT(match, "ISS TLE round-trip name line matches");

    // Move to line 1
    const char* orig_line1 = std::strchr(original, '\n');
    const char* gen_line1 = std::strchr(generated, '\n');
    if (orig_line1 && gen_line1) {
        orig_line1++;
        gen_line1++;
        match = compareTLELines(orig_line1, gen_line1, 1);
        if (!match) {
            // Find the next newline to print just line 1
            const char* orig_end = std::strchr(orig_line1, '\n');
            const char* gen_end = std::strchr(gen_line1, '\n');
            size_t orig_len = orig_end ? (size_t)(orig_end - orig_line1) : std::strlen(orig_line1);
            size_t gen_len = gen_end ? (size_t)(gen_end - gen_line1) : std::strlen(gen_line1);
            printf("  Original L1:  %.*s\n", (int)orig_len, orig_line1);
            printf("  Generated L1: %.*s\n", (int)gen_len, gen_line1);
        }
    }
    TEST_ASSERT(match, "ISS TLE round-trip line 1 matches");

    // Move to line 2
    if (orig_line1 && gen_line1) {
        const char* orig_line2 = std::strchr(orig_line1, '\n');
        const char* gen_line2 = std::strchr(gen_line1, '\n');
        if (orig_line2 && gen_line2) {
            orig_line2++;
            gen_line2++;
            match = compareTLELines(orig_line2, gen_line2, 2);
            if (!match) {
                const char* orig_end = std::strchr(orig_line2, '\n');
                const char* gen_end = std::strchr(gen_line2, '\n');
                size_t orig_len = orig_end ? (size_t)(orig_end - orig_line2) : std::strlen(orig_line2);
                size_t gen_len = gen_end ? (size_t)(gen_end - gen_line2) : std::strlen(gen_line2);
                printf("  Original L2:  %.*s\n", (int)orig_len, orig_line2);
                printf("  Generated L2: %.*s\n", (int)gen_len, gen_line2);
            }
        }
    }
    TEST_ASSERT(match, "ISS TLE round-trip line 2 matches");
}

void test_tle_roundtrip_noaa() {
    TEST_SECTION("TLE Round-Trip - NOAA 19");

    sattrack::Satellite sat;
    sat.updateFromTLE(NOAA19_TLE);

    sattrack::TLEString output = sat.getTLE();

    // Verify key values are preserved
    sattrack::Satellite sat2;
    sat2.updateFromTLE(std::string_view(output.data));

    TEST_ASSERT(sat2.getNoradID() == sat.getNoradID(), "NORAD ID preserved");
    TEST_ASSERT_NEAR(sat2.getInclination(), sat.getInclination(), 0.0001,
                     "Inclination preserved");
    TEST_ASSERT_NEAR(sat2.getEccentricity(), sat.getEccentricity(), 0.0000001,
                     "Eccentricity preserved");
    TEST_ASSERT_NEAR(sat2.getMeanMotion(), sat.getMeanMotion(), 0.00000001,
                     "Mean motion preserved");
}

void test_tle_roundtrip_geostationary() {
    TEST_SECTION("TLE Round-Trip - Geostationary (GOES 16)");

    sattrack::Satellite sat;
    sat.updateFromTLE(GOES16_TLE);

    sattrack::TLEString output = sat.getTLE();

    sattrack::Satellite sat2;
    sat2.updateFromTLE(std::string_view(output.data));

    TEST_ASSERT(sat2.getNoradID() == sat.getNoradID(), "GEO NORAD ID preserved");
    TEST_ASSERT_NEAR(sat2.getInclination(), sat.getInclination(), 0.001,
                     "GEO inclination preserved");
    TEST_ASSERT_NEAR(sat2.getMeanMotion(), sat.getMeanMotion(), 0.00000001,
                     "GEO mean motion preserved");
}

// ============================================================================
// Satellite Position/Velocity Tests
// ============================================================================

void test_satellite_position() {
    TEST_SECTION("Satellite Position Calculation");

    sattrack::Satellite sat;
    sat.updateFromTLE(ISS_TLE_3LINE);

    // Get epoch Julian Date
    double jd = sattrack::toJulianDate(sat.getEpoch());

    // Get position at epoch
    sattrack::Vec3 pos = sat.getECI(jd);

    // ISS should be at ~400km altitude, so ~6778 km from Earth center
    double r_mag = pos.magnitude();
    TEST_ASSERT(r_mag > 6600.0 && r_mag < 6900.0, "ISS position magnitude at epoch");

    // Position components should all be non-zero (unless at specific orbital positions)
    TEST_ASSERT(pos.x != 0.0 || pos.y != 0.0, "Position has non-zero XY component");
}

void test_satellite_velocity() {
    TEST_SECTION("Satellite Velocity Calculation");

    sattrack::Satellite sat;
    sat.updateFromTLE(ISS_TLE_3LINE);

    double jd = sattrack::toJulianDate(sat.getEpoch());
    sattrack::Vec3 vel = sat.getVelocity(jd);

    // ISS velocity should be ~7.66 km/s
    double v_mag = vel.magnitude();
    TEST_ASSERT(v_mag > 7.0 && v_mag < 8.0, "ISS velocity magnitude");
}

void test_satellite_geodetic_location() {
    TEST_SECTION("Satellite Geodetic Location");

    sattrack::Satellite sat;
    sat.updateFromTLE(ISS_TLE_3LINE);

    double jd = sattrack::toJulianDate(sat.getEpoch());
    sattrack::Geodetic geo = sat.getGeodeticLocationAtTime(jd);

    // Latitude should be within ISS inclination bounds (~51.6 degrees)
    double lat_deg = geo.latInRadians * sattrack::RADIANS_TO_DEGREES;
    TEST_ASSERT(lat_deg >= -52.0 && lat_deg <= 52.0,
                "ISS latitude within inclination bounds");

    // Longitude should be valid
    double lon_deg = geo.lonInRadians * sattrack::RADIANS_TO_DEGREES;
    TEST_ASSERT(lon_deg >= -180.0 && lon_deg <= 180.0,
                "Longitude in valid range");

    // Altitude should be ~400 km
    TEST_ASSERT(geo.altInKilometers > 350.0 && geo.altInKilometers < 450.0,
                "ISS altitude reasonable");
}

void test_satellite_propagation_over_time() {
    TEST_SECTION("Satellite Propagation Over Time");

    sattrack::Satellite sat;
    sat.updateFromTLE(ISS_TLE_3LINE);

    double jd0 = sattrack::toJulianDate(sat.getEpoch());

    sattrack::Vec3 pos0 = sat.getECI(jd0);
    sattrack::Vec3 pos1 = sat.getECI(jd0 + 1.0/1440.0);  // 1 minute later
    sattrack::Vec3 pos2 = sat.getECI(jd0 + 0.5/24.0);    // 30 minutes later

    // Position should change
    double delta1 = (pos1 - pos0).magnitude();
    double delta2 = (pos2 - pos0).magnitude();

    TEST_ASSERT(delta1 > 100.0, "Position changes after 1 minute");
    TEST_ASSERT(delta2 > delta1, "Position change increases with time");
}

// ============================================================================
// Satellite Getter Tests
// ============================================================================

void test_satellite_all_getters() {
    TEST_SECTION("Satellite All Getters");

    sattrack::Satellite sat;
    sat.updateFromTLE(ISS_TLE_3LINE);

    // Test all getters return sensible values
    TEST_ASSERT(std::strlen(sat.getName()) > 0, "getName returns non-empty string");
    TEST_ASSERT(sat.getNoradID() > 0, "getNoradID returns positive value");
    TEST_ASSERT(sat.getClassification() != '\0', "getClassification returns character");
    TEST_ASSERT(std::strlen(sat.getDesignator()) > 0, "getDesignator returns non-empty");

    // Epoch should be after 1970
    auto epoch = sat.getEpoch();
    auto epoch_since_1970 = epoch.time_since_epoch();
    TEST_ASSERT(epoch_since_1970.count() > 0, "Epoch is after 1970");

    TEST_ASSERT(sat.getFirstDerivativeMeanMotion() != -999.0, "First derivative accessible");
    TEST_ASSERT(sat.getSecondDerivativeMeanMotion() != -999.0, "Second derivative accessible");
    TEST_ASSERT(sat.getBstarDragTerm() != -999.0, "BSTAR accessible");
    TEST_ASSERT(sat.getElementSetNumber() >= 0, "Element set number accessible");

    TEST_ASSERT(sat.getInclination() >= 0.0 && sat.getInclination() <= 180.0,
                "Inclination in valid range");
    TEST_ASSERT(sat.getRightAscensionOfAscendingNode() >= 0.0 &&
                sat.getRightAscensionOfAscendingNode() < 360.0,
                "RAAN in valid range");
    TEST_ASSERT(sat.getEccentricity() >= 0.0 && sat.getEccentricity() < 1.0,
                "Eccentricity in valid range");
    TEST_ASSERT(sat.getArgumentOfPerigee() >= 0.0 &&
                sat.getArgumentOfPerigee() < 360.0,
                "Arg of Perigee in valid range");
    TEST_ASSERT(sat.getMeanAnomaly() >= 0.0 && sat.getMeanAnomaly() < 360.0,
                "Mean Anomaly in valid range");
    TEST_ASSERT(sat.getMeanMotion() > 0.0, "Mean Motion positive");
    TEST_ASSERT(sat.getRevolutionNumberAtEpoch() >= 0, "Rev number non-negative");
}

// ============================================================================
// Print Info Test
// ============================================================================

void test_satellite_print_info() {
    TEST_SECTION("Satellite Print Info");

    sattrack::Satellite sat;
    sat.updateFromTLE(ISS_TLE_3LINE);

    std::ostringstream oss;
    sat.printInfo(oss);

    std::string output = oss.str();
    TEST_ASSERT(output.find("ISS") != std::string::npos, "Print contains name");
    TEST_ASSERT(output.find("25544") != std::string::npos, "Print contains NORAD ID");
    TEST_ASSERT(output.find("51.6308") != std::string::npos, "Print contains inclination");
}

// ============================================================================
// TLE Checksum Tests
// ============================================================================

void test_tle_checksum() {
    TEST_SECTION("TLE Checksum Calculation");

    // ISS Line 1 (68 chars without the checksum digit at end)
    // Checksum: sum all digits, '-' counts as 1, mod 10
    const char* line1 = "1 25544U 98067A   25342.86487849  .00012323  00000+0  22804-3 0  999";
    int checksum1 = sattrack::calculateChecksum(line1);
    TEST_ASSERT(checksum1 == 9, "Line 1 checksum correct");

    // ISS Line 2 (68 chars without the checksum digit at end)
    const char* line2 = "2 25544  51.6308 161.6653 0003381 221.9847 138.0883 15.4942883454225";
    int checksum2 = sattrack::calculateChecksum(line2);
    TEST_ASSERT(checksum2 == 8, "Line 2 checksum correct");
}

// ============================================================================
// TLE Exponential Format Tests
// ============================================================================

void test_tle_exponential_format() {
    TEST_SECTION("TLE Exponential Format");

    char buffer[9];

    // Test zero
    sattrack::toTLEExponential(0.0, buffer);
    buffer[8] = '\0';
    TEST_ASSERT_STR_EQ(buffer, " 00000+0", "Zero formatted correctly");

    // Test positive small value (like BSTAR)
    sattrack::toTLEExponential(0.00010270, buffer);
    buffer[8] = '\0';
    // Should be something like " 10270-3"
    TEST_ASSERT(buffer[0] == ' ', "Positive value has space sign");
    TEST_ASSERT(buffer[6] == '-' || buffer[6] == '+', "Has exponent sign");

    // Test negative value
    sattrack::toTLEExponential(-0.00012345, buffer);
    buffer[8] = '\0';
    TEST_ASSERT(buffer[0] == '-', "Negative value has minus sign");
}

void test_tle_first_derivative_format() {
    TEST_SECTION("TLE First Derivative Format");

    char buffer[11];

    // Test positive value
    sattrack::formatFirstDerivative(0.00016717, buffer);
    buffer[10] = '\0';
    TEST_ASSERT(buffer[0] == ' ', "Positive has space sign");
    TEST_ASSERT(buffer[1] == '.', "Has decimal point");

    // Test negative value
    sattrack::formatFirstDerivative(-0.00000097, buffer);
    buffer[10] = '\0';
    TEST_ASSERT(buffer[0] == '-', "Negative has minus sign");
}

// ============================================================================
// Main Entry Point
// ============================================================================

extern "C" rtems_task Init(rtems_task_argument ignored) {
    printf("\n*** SATELLITE CLASS TESTS ***\n");

    // TLE Parsing tests
    test_tle_parsing_3line();
    test_tle_parsing_2line();
    test_tle_parsing_with_separate_name();
    test_tle_parsing_exponential_values();
    test_tle_parsing_negative_values();
    test_tle_parsing_various_satellites();

    // TLE Round-trip tests
    test_tle_roundtrip_iss();
    test_tle_roundtrip_noaa();
    test_tle_roundtrip_geostationary();

    // Position/Velocity tests
    test_satellite_position();
    test_satellite_velocity();
    test_satellite_geodetic_location();
    test_satellite_propagation_over_time();

    // Getter tests
    test_satellite_all_getters();
    test_satellite_print_info();

    // TLE formatting tests
    test_tle_checksum();
    test_tle_exponential_format();
    test_tle_first_derivative_format();

    // Summary
    printf("\n=== TEST SUMMARY ===\n");
    printf("Tests run:    %d\n", tests_run);
    printf("Tests passed: %d\n", tests_passed);
    printf("Tests failed: %d\n", tests_failed);
    printf("\n*** END SATELLITE TESTS ***\n");

    exit(tests_failed == 0 ? 0 : 1);
}
