/*
 * Copyright (c) 2025 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 *
 * SGP4/SDP4 Satellite Propagation Module
 * Based on the Vallado reference implementation from CelesTrak.
 * See: https://celestrak.org/software/vallado-sw.php
 */

#ifndef __SATTRACK_SGP4_HPP
#define __SATTRACK_SGP4_HPP

#include <cmath>
#include <string>

namespace sgp4 {

// ============================================================================
// SGP4 Constants
// ============================================================================

// Mathematical constants (defined here for portability - some toolchains don't provide M_PI)
constexpr double PI = 3.14159265358979323846;

// WGS84 / EGM-96 Constants
constexpr double MU = 398600.8;                    // Earth gravitational parameter (km^3/s^2)
constexpr double RADIUS_EARTH_KM = 6378.135;       // Earth equatorial radius (km)
constexpr double J2 = 0.001082616;                 // Second gravitational zonal harmonic
constexpr double J3 = -0.00000253881;              // Third gravitational zonal harmonic
constexpr double J4 = -0.00000165597;              // Fourth gravitational zonal harmonic
constexpr double J3OJ2 = J3 / J2;
constexpr double XKE = 0.0743669161331734132;      // sqrt(GM) in Earth radii^1.5/min
constexpr double TUMIN = 13.44683969695931;        // Minutes per time unit
constexpr double VKMPERSEC = 7.905366149846074;    // km/s per velocity unit
constexpr double TWO_PI = 2.0 * PI;
constexpr double X2O3 = 2.0 / 3.0;

// ============================================================================
// SGP4 Error Codes
// ============================================================================

enum class ErrorCode {
    SUCCESS = 0,
    SATELLITE_DECAYED,
    INVALID_ORBIT,
    ECCENTRICITY_OUT_OF_RANGE,
    NEGATIVE_SEMI_LATUS_RECTUM
};

// ============================================================================
// SGP4 Data Structures
// ============================================================================

/**
 * SGP4/SDP4 state variables computed during initialization.
 * This struct holds all the precomputed coefficients needed for propagation.
 */
struct State {
    bool initialized = false;

    // Epoch in Julian Date (split for precision)
    double jdsatepoch = 0.0;      // Integer part
    double jdsatepochF = 0.0;     // Fractional part

    // Method flag: 'n' = near-earth (SGP4), 'd' = deep-space (SDP4)
    char method = 'n';

    // Initialization flags
    bool isimp = false;           // Simple drag flag
    int irez = 0;                 // Resonance flag (0=none, 1=1-day, 2=0.5-day)

    // Common orbital parameters
    double a = 0.0;               // Semi-major axis (Earth radii)
    double alta = 0.0;            // Altitude at apogee
    double altp = 0.0;            // Altitude at perigee
    double argpo = 0.0;           // Argument of perigee (rad)
    double bstar = 0.0;           // Drag term
    double ecco = 0.0;            // Eccentricity
    double inclo = 0.0;           // Inclination (rad)
    double mo = 0.0;              // Mean anomaly (rad)
    double no_kozai = 0.0;        // Mean motion (Kozai, rad/min)
    double no_unkozai = 0.0;      // Mean motion (un-Kozai'd, rad/min)
    double nodeo = 0.0;           // Right ascension (rad)
    double gsto = 0.0;            // Greenwich sidereal time at epoch
    double cosio2 = 0.0;          // cosine of inclination squared
    double eccsq = 0.0;           // Eccentricity squared

    // Near-earth coefficients
    double aycof = 0.0;
    double con41 = 0.0;
    double cc1 = 0.0, cc4 = 0.0, cc5 = 0.0;
    double d2 = 0.0, d3 = 0.0, d4 = 0.0;
    double delmo = 0.0;
    double eta = 0.0;
    double argpdot = 0.0;
    double omgcof = 0.0;
    double sinmao = 0.0;
    double t2cof = 0.0, t3cof = 0.0, t4cof = 0.0, t5cof = 0.0;
    double x1mth2 = 0.0;
    double x7thm1 = 0.0;
    double mdot = 0.0;
    double nodedot = 0.0;
    double xlcof = 0.0;
    double xmcof = 0.0;
    double nodecf = 0.0;

    // Deep space coefficients
    double e3 = 0.0, ee2 = 0.0;
    double peo = 0.0, pgho = 0.0, pho = 0.0, pinco = 0.0, plo = 0.0;
    double se2 = 0.0, se3 = 0.0, sgh2 = 0.0, sgh3 = 0.0, sgh4 = 0.0;
    double sh2 = 0.0, sh3 = 0.0, si2 = 0.0, si3 = 0.0, sl2 = 0.0;
    double sl3 = 0.0, sl4 = 0.0;
    double xgh2 = 0.0, xgh3 = 0.0, xgh4 = 0.0;
    double xh2 = 0.0, xh3 = 0.0;
    double xi2 = 0.0, xi3 = 0.0;
    double xl2 = 0.0, xl3 = 0.0, xl4 = 0.0;
    double xlamo = 0.0;
    double zmol = 0.0, zmos = 0.0;
    double atime = 0.0;
    double xli = 0.0, xni = 0.0;

    // Resonance coefficients
    double d2201 = 0.0, d2211 = 0.0;
    double d3210 = 0.0, d3222 = 0.0;
    double d4410 = 0.0, d4422 = 0.0;
    double d5220 = 0.0, d5232 = 0.0, d5421 = 0.0, d5433 = 0.0;
    double del1 = 0.0, del2 = 0.0, del3 = 0.0;
    double dedt = 0.0, didt = 0.0, dmdt = 0.0;
    double dnodt = 0.0, domdt = 0.0;
};

/**
 * Input elements from TLE data for SGP4 initialization.
 */
struct Elements {
    double epoch_jd;           // Epoch as Julian Date
    double bstar;              // BSTAR drag term
    double inclination;        // Inclination (radians)
    double raan;               // Right ascension of ascending node (radians)
    double eccentricity;       // Eccentricity
    double arg_perigee;        // Argument of perigee (radians)
    double mean_anomaly;       // Mean anomaly (radians)
    double mean_motion;        // Mean motion (rad/min)
};

/**
 * Output from SGP4 propagation.
 */
struct Result {
    double r[3];  // Position (km) in TEME frame
    double v[3];  // Velocity (km/s) in TEME frame

    // Updated resonance state (for deep-space satellites)
    // For sequential propagations, copy these back to State for efficiency.
    double atime;  // Time of last resonance integration
    double xli;    // Mean longitude for resonance
    double xni;    // Mean motion for resonance

    // Error code from propagation
    ErrorCode error = ErrorCode::SUCCESS;
};

// ============================================================================
// SGP4 Public API
// ============================================================================

/**
 * Initialize SGP4 state from orbital elements.
 * Must be called before propagate().
 *
 * @param state Output state structure to initialize
 * @param elements Input orbital elements from TLE
 * @return ErrorCode::SUCCESS on success, or an error code on failure
 */
ErrorCode initialize(State& state, const Elements& elements);

/**
 * Propagate to time since epoch.
 *
 * @param state The initialized SGP4 state (from initialize())
 * @param tsince Minutes since epoch
 * @return Result containing position, velocity, updated resonance state, and error code
 */
Result propagate(const State& state, double tsince);

// ============================================================================
// Deep Space Helper Functions (Public for testing/advanced use)
// ============================================================================

/**
 * Initialize deep space (SDP4) coefficients.
 * Called by initialize() for deep-space satellites.
 */
void initializeDeepSpace(
    State& state,
    double tc,
    double& snodm, double& cnodm, double& sinim, double& cosim,
    double& sinomm, double& cosomm, double& day, double& em, double& emsq,
    double& gam, double& rtemsq, double& s1, double& s2, double& s3,
    double& s4, double& s5, double& s6, double& s7, double& ss1,
    double& ss2, double& ss3, double& ss4, double& ss5, double& ss6,
    double& ss7, double& sz1, double& sz2, double& sz3, double& sz11,
    double& sz12, double& sz13, double& sz21, double& sz22, double& sz23,
    double& sz31, double& sz32, double& sz33, double& nm, double& z1,
    double& z2, double& z3, double& z11, double& z12, double& z13,
    double& z21, double& z22, double& z23, double& z31, double& z32,
    double& z33);

/**
 * Apply deep space secular effects.
 * Called by propagate() for deep-space satellites.
 *
 * @param state The SGP4 state
 * @param t Time since epoch (minutes)
 * @param em Eccentricity (modified)
 * @param argpm Argument of perigee (modified)
 * @param inclm Inclination (modified)
 * @param nodem RAAN (modified)
 * @param mm Mean anomaly (modified)
 * @param nm Mean motion (modified)
 * @param atime Updated resonance time (output)
 * @param xli Updated resonance longitude (output)
 * @param xni Updated resonance mean motion (output)
 */
void deepSpaceSecular(
    const State& state,
    double t, double& em, double& argpm, double& inclm,
    double& nodem, double& mm, double& nm,
    double& atime, double& xli, double& xni);

/**
 * Apply deep space periodic effects.
 * Called by propagate() for deep-space satellites.
 */
void deepSpacePeriodic(
    const State& state,
    double t, double& em, double& inclm, double& nodem,
    double& argpm, double& mm);

/**
 * Compute Greenwich Sidereal Time at a given Julian Date.
 * Used internally by SGP4 initialization.
 */
double gstime(double jdut1);

} // namespace sattrack::sgp4

#endif // __SATTRACK_SGP4_HPP
