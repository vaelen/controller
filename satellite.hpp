/*
 * Copyright (c) 2025 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 *
 * SGP4 algorithm based on the Vallado reference implementation.
 * See: https://celestrak.org/software/vallado-sw.php
 */

#ifndef __SATTRACK_SATELLITE_HPP
#define __SATTRACK_SATELLITE_HPP

#include <sgp4.hpp>

#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <string_view>
#include <iostream>
#include <map>

namespace sattrack {

using time_point = std::chrono::system_clock::time_point;

// Re-export SGP4 error codes
using ErrorCode = sgp4::ErrorCode;

// Astronomical constants
constexpr double J2000_JD = 2451545.0;                      // Julian Date of J2000.0 epoch
constexpr double DAYS_PER_JULIAN_CENTURY = 36525.0;         // Days in a Julian century
constexpr double GMST_AT_J2000 = 280.46061837;              // GMST at J2000.0 epoch (degrees)
constexpr double EARTH_SIDEREAL_RATE = 360.98564736629;     // Earth's rotation rate (deg/day)

// IAU polynomial correction coefficients for long-term variations in Earth's rotation
constexpr double GMST_T2_COEFF = 0.000387933;   // Quadratic correction for precession (T² term)
constexpr double GMST_T3_DIVISOR = 38710000.0;  // Cubic correction divisor (T³ term)

// Degree-radian conversion factors
constexpr double DEGREES_TO_RADIANS = M_PI / 180.0;
constexpr double RADIANS_TO_DEGREES = 180.0 / M_PI;

// ============================================================================
// Basic Data Types
// ============================================================================

/**
 * 3D vector in Cartesian coordinates.
 */
struct Vec3 {
    double x, y, z;

    Vec3 operator+(const Vec3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vec3 operator-(const Vec3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    Vec3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    double magnitude() const {
        return std::sqrt(x*x + y*y + z*z);
    }

    double dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    Vec3 cross(const Vec3& other) const {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }

    /**
     * Returns a unit vector (magnitude = 1) in the same direction as this vector.
     */
    Vec3 normalize() const {
        double mag = magnitude();
        return {x / mag, y / mag, z / mag};
    }
};

/**
 * Geodetic coordinates representing a position on or above Earth's surface.
 */
struct Geodetic {
    double latInRadians;      ///< Geodetic latitude (-π/2 to +π/2, positive = North)
    double lonInRadians;      ///< Longitude (-π to +π, positive = East)
    double altInKilometers;   ///< Altitude above the WGS84 ellipsoid surface

    Vec3 toECEF() const;
};

/**
 * Look angles from an observer to a target (typically a satellite).
 */
struct LookAngles {
    double azimuthInRadians;      ///< Compass direction (0 = North, π/2 = East, etc.)
    double elevationInRadians;    ///< Angle above horizon (0 = horizon, π/2 = overhead)
    double rangeInKilometers;     ///< Slant range (straight-line distance) to the target
};

/**
 * Information about a single satellite pass over a ground station.
 */
struct PassInfo {
    int noradID;                  ///< NORAD Catalog ID of the satellite
    std::string name;             ///< Name of the satellite
    time_point riseTime;          ///< When the satellite rises above the elevation threshold
    time_point maxElevationTime;  ///< When the satellite reaches its highest point
    time_point setTime;           ///< When the satellite sets below the elevation threshold
    LookAngles riseAngles;        ///< Azimuth/elevation at rise
    LookAngles maxAngles;         ///< Azimuth/elevation at maximum elevation
    LookAngles setAngles;         ///< Azimuth/elevation at set
};

// ============================================================================
// Satellite Class with SGP4 Propagation
// ============================================================================

/**
 * Represents a satellite orbit and provides SGP4/SDP4 propagation.
 *
 * The SGP4 algorithm is used for near-Earth satellites (orbital period < 225 min)
 * and SDP4 is used for deep-space satellites (orbital period >= 225 min).
 *
 * Usage:
 *   Satellite sat;
 *   sat.updateFromTLE(tleString);
 *   Vec3 position = sat.getECI(julianDate);
 *
 * Note: This class is not thread-safe for the same instance. The mutable SGP4
 * state is initialized lazily on first propagation.
 */
class Satellite {
public:
    Satellite() = default;
    ~Satellite() = default;

    /**
     * Update orbital elements from a TLE string with a separate name.
     */
    void updateFromTLE(const std::string_view &name, const std::string_view &tle);

    /**
     * Update orbital elements from a TLE string (name from TLE line 0).
     */
    void updateFromTLE(const std::string_view &tle);

    // Accessors for TLE data
    std::string getName() const;
    int getNoradID() const;
    char getClassification() const;
    std::string getDesignator() const;
    time_point getEpoch() const;
    double getFirstDerivativeMeanMotion() const;
    double getSecondDerivativeMeanMotion() const;
    double getBstarDragTerm() const;
    int getElementSetNumber() const;

    // Accessors for orbital elements
    double getInclination() const;
    double getRightAscensionOfAscendingNode() const;
    double getEccentricity() const;
    double getArgumentOfPerigee() const;
    double getMeanAnomaly() const;
    double getMeanMotion() const;
    int getRevolutionNumberAtEpoch() const;

    /**
     * Get position in Earth-Centered Inertial (ECI/TEME) coordinates.
     *
     * Uses SGP4/SDP4 propagation to compute the satellite position at the
     * specified Julian Date. The returned coordinates are in the True Equator
     * Mean Equinox (TEME) reference frame, in kilometers.
     *
     * @param julianDate The time for which to compute position
     * @return Position vector in TEME coordinates (km), or zero vector on error
     */
    Vec3 getECI(double julianDate) const;

    /**
     * Get velocity in Earth-Centered Inertial (ECI/TEME) coordinates.
     *
     * @param julianDate The time for which to compute velocity
     * @return Velocity vector in TEME coordinates (km/s)
     */
    Vec3 getVelocity(double julianDate) const;

    /**
     * Get geodetic location (latitude, longitude, altitude) at a given time.
     */
    Geodetic getGeodeticLocationAtTime(const time_point tp) const;
    Geodetic getGeodeticLocationAtTime(const double julianDate) const;

    /**
     * Print orbital element information to a stream.
     */
    void printInfo(std::ostream &os) const;

    /**
     * Get TLE string representation.
     */
    std::string getTLE() const;

private:
    // ========================================================================
    // TLE Data
    // ========================================================================

    // Satellite Identification
    std::string name;

    // First Line - Satellite Identification
    int noradID = 0;
    char classification = 'U';
    std::string designator;
    time_point epoch;
    double firstDerivativeMeanMotion = 0.0;
    double secondDerivativeMeanMotion = 0.0;
    double bstarDragTerm = 0.0;
    int elementSetNumber = 0;

    // Second Line - Orbital Elements (in degrees, except eccentricity)
    double inclination = 0.0;
    double rightAscensionOfAscendingNode = 0.0;
    double eccentricity = 0.0;
    double argumentOfPerigee = 0.0;
    double meanAnomaly = 0.0;
    double meanMotion = 0.0;  // revolutions per day
    int revolutionNumberAtEpoch = 0;

    // SGP4 State 
    mutable sgp4::State sgp4State_;

    /**
     * Ensure SGP4 state is initialized before propagation.
     * Called lazily on first propagation.
     */
    void ensureSGP4Initialized() const;
};

// ============================================================================
// TLE Database Functions
// ============================================================================

void loadTLEDatabase(std::istream &s, std::map<int, Satellite> &database, std::ostream &logStream = std::cerr);
void loadTLEDatabase(const std::string &filepath, std::map<int, Satellite> &database, std::ostream &logStream = std::cerr);
void saveTLEDatabase(std::ostream &s, const std::map<int, Satellite> &database, std::ostream &logStream = std::cerr);
void saveTLEDatabase(const std::string &filepath, const std::map<int, Satellite> &database, std::ostream &logStream = std::cerr);

// TLE formatting utilities
int calculateChecksum(const std::string& line);
std::string toTLEExponential(double value);
std::string formatFirstDerivative(double value);

// ============================================================================
// Coordinate System Transformations and Time Functions
// ============================================================================

/**
 * Converts a time_point to Julian Date.
 */
double toJulianDate(time_point tp);

/**
 * Computes Greenwich Mean Sidereal Time (GMST) for a given Julian Date.
 * @return GMST in radians, normalized to [0, 2π)
 */
double gmst(double julianDate);

/**
 * Converts Earth-Centered Inertial (ECI) to Earth-Centered Earth-Fixed (ECEF).
 */
Vec3 eciToECEF(const Vec3 &eci, double gst);

/**
 * Converts ECEF coordinates to geodetic.
 */
Geodetic ecefToGeodetic(const Vec3 &ecef);

// ============================================================================
// Visibility and Look Angle Functions
// ============================================================================

/**
 * Transforms a position from ECEF to ENU (East-North-Up) coordinates.
 */
Vec3 ecefToENU(const Vec3& targetECEF, const Geodetic& observer);

/**
 * Computes look angles from an observer to a target given in ECEF coordinates.
 */
LookAngles getLookAngles(const Vec3& satECEF, const Geodetic& observer);

/**
 * Computes look angles from an observer to a satellite at a specific time.
 */
LookAngles getLookAngles(const Satellite& satellite, const Geodetic& observer, double julianDate);

/**
 * Checks if a satellite is visible based on its look angles.
 */
bool isVisible(const LookAngles& angles, double minElevationInRadians);

/**
 * Checks if a satellite is visible from an observer at a specific time.
 */
bool isVisible(const Satellite& satellite, const Geodetic& observer,
               double julianDate, double minElevationInRadians);

/**
 * Finds the next satellite pass over an observer's location.
 */
std::optional<PassInfo> findNextPass(
    const Satellite& satellite,
    const Geodetic& observer,
    double minElevationInRadians,
    time_point startTime);

}

#endif
