/*
 * Copyright (c) 2025 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 *
 * SGP4/SDP4 Satellite Propagation Implementation
 * Based on the Vallado reference implementation from CelesTrak.
 * See: https://celestrak.org/software/vallado-sw.php
 */

#include "sgp4.hpp"

#include <cmath>
#include <cstdio>

namespace sgp4 {

// Compute Greenwich Sidereal Time at epoch for SGP4
double gstime(double jdut1) {
    double tut1 = (jdut1 - 2451545.0) / 36525.0;
    double temp = -6.2e-6 * tut1 * tut1 * tut1
                  + 0.093104 * tut1 * tut1
                  + (876600.0 * 3600 + 8640184.812866) * tut1
                  + 67310.54841;
    constexpr double DEG_TO_RAD = PI / 180.0;
    temp = std::fmod(temp * DEG_TO_RAD / 240.0, TWO_PI);
    if (temp < 0.0) temp += TWO_PI;
    return temp;
}

// Initialize SGP4 state from orbital elements
ErrorCode initialize(State& state, const Elements& elements) {
    // Convert input elements to internal units
    state.ecco = elements.eccentricity;
    state.inclo = elements.inclination;
    state.nodeo = elements.raan;
    state.argpo = elements.arg_perigee;
    state.mo = elements.mean_anomaly;
    state.bstar = elements.bstar;
    state.no_kozai = elements.mean_motion;

    // Compute epoch Julian Date (split for precision)
    state.jdsatepoch = std::floor(elements.epoch_jd);
    state.jdsatepochF = elements.epoch_jd - state.jdsatepoch;

    // Compute Greenwich sidereal time at epoch
    state.gsto = gstime(elements.epoch_jd);

    // WGS-72 Earth constants
    constexpr double radiusearthkm = RADIUS_EARTH_KM;
    constexpr double xke = XKE;
    constexpr double j2 = J2;
    constexpr double j3oj2 = J3OJ2;
    constexpr double j4 = J4;

    // Recover original mean motion (no_unkozai) and semimajor axis from input
    double a1 = std::pow(xke / state.no_kozai, X2O3);
    double cosio = std::cos(state.inclo);
    double cosio2 = cosio * cosio;
    double eccsq = state.ecco * state.ecco;
    double omeosq = 1.0 - eccsq;
    double rteosq = std::sqrt(omeosq);
    double d1 = 0.75 * j2 * (3.0 * cosio2 - 1.0) / (rteosq * omeosq);
    double del_ = d1 / (a1 * a1);
    double ao = a1 * (1.0 - del_ * (1.0/3.0 + del_ * (1.0 + 134.0/81.0 * del_)));
    double delo = d1 / (ao * ao);
    state.no_unkozai = state.no_kozai / (1.0 + delo);
    ao = a1 * (1.0 - del_ * (1.0/3.0 + del_ * (1.0 + 134.0/81.0 * del_)));

    // Compute semi-major axis
    state.a = std::pow(xke / state.no_unkozai, X2O3);

    // Compute perigee and apogee altitudes
    state.altp = state.a * (1.0 - state.ecco) - 1.0;
    state.alta = state.a * (1.0 + state.ecco) - 1.0;

    // Determine if deep space (period >= 225 min)
    double periodearthradii = TWO_PI / state.no_unkozai;
    if (periodearthradii >= 225.0) {
        state.method = 'd';  // deep space
    } else {
        state.method = 'n';  // near earth
    }

    // SGP4 initialization
    double ss = 78.0 / radiusearthkm + 1.0;
    double qzms2t = std::pow((120.0 - 78.0) / radiusearthkm, 4);

    double sinio = std::sin(state.inclo);
    double x1mth2 = 1.0 - cosio2;
    state.x1mth2 = x1mth2;
    state.cosio2 = cosio2;
    state.eccsq = eccsq;

    // Check for eccentricity out of range
    if (state.ecco >= 1.0 || state.ecco < -0.001) {
        // TODO: Address exception handling
        printf("SGP4 Error: Eccentricity out of range: %f\n", state.ecco);
        return ErrorCode::ECCENTRICITY_OUT_OF_RANGE;
    }
    if (state.ecco < 1.0e-10) {
        state.ecco = 1.0e-10;  // Clamp to avoid division by zero
    }

    // Compute perigee
    double rp = state.a * (1.0 - state.ecco);

    // Check if satellite has decayed
    if (rp < 1.0) {
        // TODO: Address exception handling
        printf("SGP4 Error: Satellite has decayed\n");
        return ErrorCode::SATELLITE_DECAYED;
    }

    double sfour = ss;
    double qzms24 = qzms2t;
    double perige = (rp - 1.0) * radiusearthkm;

    // For perigees below 156 km, adjust s and qoms2t
    if (perige < 156.0) {
        sfour = perige - 78.0;
        if (perige < 98.0) {
            sfour = 20.0;
        }
        qzms24 = std::pow((120.0 - sfour) / radiusearthkm, 4);
        sfour = sfour / radiusearthkm + 1.0;
    }

    double pinvsq = 1.0 / (state.a * state.a * omeosq * omeosq);
    double tsi = 1.0 / (state.a - sfour);
    state.eta = state.a * state.ecco * tsi;
    double etasq = state.eta * state.eta;
    double eeta = state.ecco * state.eta;
    double psisq = std::abs(1.0 - etasq);
    double coef = qzms24 * std::pow(tsi, 4);
    double coef1 = coef / std::pow(psisq, 3.5);
    double cc2 = coef1 * state.no_unkozai * (state.a * (1.0 + 1.5 * etasq + eeta * (4.0 + etasq))
                 + 0.375 * j2 * tsi / psisq * (3.0 * (3.0 * cosio2 - 1.0)
                 * (1.0 + 1.5 * etasq) - 0.5 * (3.0 - 7.0 * cosio2) * eeta * std::cos(2.0 * state.argpo)));
    state.cc1 = state.bstar * cc2;
    double cc3 = 0.0;
    if (state.ecco > 1.0e-4) {
        cc3 = -2.0 * coef * tsi * j3oj2 * state.no_unkozai * sinio / state.ecco;
    }
    state.x7thm1 = 7.0 * cosio2 - 1.0;
    state.cc4 = 2.0 * state.no_unkozai * coef1 * state.a * omeosq
           * (state.eta * (2.0 + 0.5 * etasq) + state.ecco * (0.5 + 2.0 * etasq)
           - j2 * tsi / (state.a * psisq) * (-3.0 * (3.0 * (3.0 * cosio2 - 1.0) - 7.0 * state.x7thm1)
           * (1.0 + 1.5 * etasq) / 6.0 + 0.25 * (3.0 - 7.0 * cosio2) * (2.0 * etasq - eeta * (1.0 + etasq)) * std::cos(2.0 * state.argpo)));
    state.cc5 = 2.0 * coef1 * state.a * omeosq * (1.0 + 2.75 * (etasq + eeta) + eeta * etasq);

    double cosio4 = cosio2 * cosio2;
    double temp1 = 1.5 * j2 * pinvsq * state.no_unkozai;
    double temp2 = 0.5 * temp1 * j2 * pinvsq;
    double temp3 = -0.46875 * j4 * pinvsq * pinvsq * state.no_unkozai;
    state.mdot = state.no_unkozai + 0.5 * temp1 * rteosq * (3.0 * cosio2 - 1.0)
            + 0.0625 * temp2 * rteosq * (13.0 - 78.0 * cosio2 + 137.0 * cosio4);
    state.argpdot = -0.5 * temp1 * (1.0 - 5.0 * cosio2)
               + 0.0625 * temp2 * (7.0 - 114.0 * cosio2 + 395.0 * cosio4)
               + temp3 * (3.0 - 36.0 * cosio2 + 49.0 * cosio4);
    double xhdot1 = -temp1 * cosio;
    state.nodedot = xhdot1 + (0.5 * temp2 * (4.0 - 19.0 * cosio2) + 2.0 * temp3 * (3.0 - 7.0 * cosio2)) * cosio;
    state.omgcof = state.bstar * cc3 * std::cos(state.argpo);
    state.xmcof = 0.0;
    if (state.ecco > 1.0e-4) {
        state.xmcof = -X2O3 * coef * state.bstar / eeta;
    }
    state.nodecf = 3.5 * omeosq * xhdot1 * state.cc1;
    state.t2cof = 1.5 * state.cc1;

    // Set xlcof and aycof
    if (std::abs(cosio + 1.0) > 1.5e-12) {
        state.xlcof = -0.25 * j3oj2 * sinio * (3.0 + 5.0 * cosio) / (1.0 + cosio);
    } else {
        state.xlcof = -0.25 * j3oj2 * sinio * (3.0 + 5.0 * cosio) / 1.5e-12;
    }
    state.aycof = -0.5 * j3oj2 * sinio;

    // For SGP4, initialize additional terms
    state.delmo = std::pow(1.0 + state.eta * std::cos(state.mo), 3);
    state.sinmao = std::sin(state.mo);

    // Compute con41 for later use
    state.con41 = 3.0 * cosio2 - 1.0;

    // Set isimp flag for very high drag satellites
    state.isimp = false;
    if ((omeosq >= 0.0) || (state.no_unkozai >= 0.0)) {
        if ((rp < (220.0 / radiusearthkm + 1.0))) {
            state.isimp = true;
        }
    }

    // Initialize d2, d3, d4, t3cof, t4cof, t5cof (for non-simple satellites)
    state.d2 = 0.0;
    state.d3 = 0.0;
    state.d4 = 0.0;
    state.t3cof = 0.0;
    state.t4cof = 0.0;
    state.t5cof = 0.0;

    if (!state.isimp) {
        double c1sq = state.cc1 * state.cc1;
        state.d2 = 4.0 * state.a * tsi * c1sq;
        double temp = state.d2 * tsi * state.cc1 / 3.0;
        state.d3 = (17.0 * state.a + sfour) * temp;
        state.d4 = 0.5 * temp * state.a * tsi * (221.0 * state.a + 31.0 * sfour) * state.cc1;
        state.t3cof = state.d2 + 2.0 * c1sq;
        state.t4cof = 0.25 * (3.0 * state.d3 + state.cc1 * (12.0 * state.d2 + 10.0 * c1sq));
        state.t5cof = 0.2 * (3.0 * state.d4 + 12.0 * state.cc1 * state.d3 + 6.0 * state.d2 * state.d2 + 15.0 * c1sq * (2.0 * state.d2 + c1sq));
    }

    // Deep space initialization if needed
    if (state.method == 'd') {
        state.irez = 0;
        if ((state.no_unkozai < 0.0052359877) && (state.no_unkozai > 0.0034906585)) {
            state.irez = 1;  // Synchronous resonance
        }
        if ((state.no_unkozai >= 8.26e-3) && (state.no_unkozai <= 9.24e-3) && (state.ecco >= 0.5)) {
            state.irez = 2;  // Half-day resonance
        }

        // Initialize deep space terms
        double tc = 0.0;
        double snodm, cnodm, sinim, cosim, sinomm, cosomm;
        double day, em, emsq, gam, rtemsq;
        double s1, s2, s3, s4, s5, s6, s7;
        double ss1, ss2, ss3, ss4, ss5, ss6, ss7;
        double sz1, sz2, sz3, sz11, sz12, sz13, sz21, sz22, sz23, sz31, sz32, sz33;
        double nm, z1, z2, z3, z11, z12, z13, z21, z22, z23, z31, z32, z33;

        initializeDeepSpace(state, tc, snodm, cnodm, sinim, cosim, sinomm, cosomm,
                           day, em, emsq, gam, rtemsq, s1, s2, s3, s4, s5, s6, s7,
                           ss1, ss2, ss3, ss4, ss5, ss6, ss7,
                           sz1, sz2, sz3, sz11, sz12, sz13, sz21, sz22, sz23, sz31, sz32, sz33,
                           nm, z1, z2, z3, z11, z12, z13, z21, z22, z23, z31, z32, z33);
    }

    state.initialized = true;
    return ErrorCode::SUCCESS;
}

// Initialize deep space (SDP4) coefficients
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
    double& z33) {

    // Solar and lunar constants
    constexpr double zes = 0.01675;
    constexpr double zel = 0.05490;
    constexpr double c1ss = 2.9864797e-6;
    constexpr double c1l = 4.7968065e-7;
    constexpr double zsinis = 0.39785416;
    constexpr double zcosis = 0.91744867;
    constexpr double zcosgs = 0.1945905;
    constexpr double zsings = -0.98088458;

    nm = state.no_unkozai;
    em = state.ecco;
    snodm = std::sin(state.nodeo);
    cnodm = std::cos(state.nodeo);
    sinomm = std::sin(state.argpo);
    cosomm = std::cos(state.argpo);
    sinim = std::sin(state.inclo);
    cosim = std::cos(state.inclo);
    emsq = em * em;
    double betasq = 1.0 - emsq;
    rtemsq = std::sqrt(betasq);

    day = state.jdsatepoch + state.jdsatepochF - 2433281.5 + tc / 1440.0;
    double xnodce = std::fmod(4.5236020 - 9.2422029e-4 * day, TWO_PI);
    double stem = std::sin(xnodce);
    double ctem = std::cos(xnodce);
    double zcosil = 0.91375164 - 0.03568096 * ctem;
    double zsinil = std::sqrt(1.0 - zcosil * zcosil);
    double zsinhl = 0.089683511 * stem / zsinil;
    double zcoshl = std::sqrt(1.0 - zsinhl * zsinhl);
    gam = 5.8351514 + 0.0019443680 * day;
    double zx = 0.39785416 * stem / zsinil;
    double zy = zcoshl * ctem + 0.91744867 * zsinhl * stem;
    zx = std::atan2(zx, zy);
    zx = gam + zx - xnodce;
    double zcosgl = std::cos(zx);
    double zsingl = std::sin(zx);

    // Solar terms
    double zcosg = zcosgs;
    double zsing = zsings;
    double zcosi = zcosis;
    double zsini = zsinis;
    double zcosh = cnodm;
    double zsinh = snodm;
    double cc = c1ss;
    double xnoi = 1.0 / nm;

    for (int lsflg = 1; lsflg <= 2; lsflg++) {
        double a1 = zcosg * zcosh + zsing * zcosi * zsinh;
        double a3 = -zsing * zcosh + zcosg * zcosi * zsinh;
        double a7 = -zcosg * zsinh + zsing * zcosi * zcosh;
        double a8 = zsing * zsini;
        double a9 = zsing * zsinh + zcosg * zcosi * zcosh;
        double a10 = zcosg * zsini;
        double a2 = cosim * a7 + sinim * a8;
        double a4 = cosim * a9 + sinim * a10;
        double a5 = -sinim * a7 + cosim * a8;
        double a6 = -sinim * a9 + cosim * a10;

        double x1 = a1 * cosomm + a2 * sinomm;
        double x2 = a3 * cosomm + a4 * sinomm;
        double x3 = -a1 * sinomm + a2 * cosomm;
        double x4 = -a3 * sinomm + a4 * cosomm;
        double x5 = a5 * sinomm;
        double x6 = a6 * sinomm;
        double x7 = a5 * cosomm;
        double x8 = a6 * cosomm;

        z31 = 12.0 * x1 * x1 - 3.0 * x3 * x3;
        z32 = 24.0 * x1 * x2 - 6.0 * x3 * x4;
        z33 = 12.0 * x2 * x2 - 3.0 * x4 * x4;
        z1 = 3.0 * (a1 * a1 + a2 * a2) + z31 * emsq;
        z2 = 6.0 * (a1 * a3 + a2 * a4) + z32 * emsq;
        z3 = 3.0 * (a3 * a3 + a4 * a4) + z33 * emsq;
        z11 = -6.0 * a1 * a5 + emsq * (-24.0 * x1 * x7 - 6.0 * x3 * x5);
        z12 = -6.0 * (a1 * a6 + a3 * a5) + emsq * (-24.0 * (x2 * x7 + x1 * x8) - 6.0 * (x3 * x6 + x4 * x5));
        z13 = -6.0 * a3 * a6 + emsq * (-24.0 * x2 * x8 - 6.0 * x4 * x6);
        z21 = 6.0 * a2 * a5 + emsq * (24.0 * x1 * x5 - 6.0 * x3 * x7);
        z22 = 6.0 * (a4 * a5 + a2 * a6) + emsq * (24.0 * (x2 * x5 + x1 * x6) - 6.0 * (x4 * x7 + x3 * x8));
        z23 = 6.0 * a4 * a6 + emsq * (24.0 * x2 * x6 - 6.0 * x4 * x8);
        z1 = z1 + z1 + betasq * z31;
        z2 = z2 + z2 + betasq * z32;
        z3 = z3 + z3 + betasq * z33;
        s3 = cc * xnoi;
        s2 = -0.5 * s3 / rtemsq;
        s4 = s3 * rtemsq;
        s1 = -15.0 * em * s4;
        s5 = x1 * x3 + x2 * x4;
        s6 = x2 * x3 + x1 * x4;
        s7 = x2 * x4 - x1 * x3;

        if (lsflg == 1) {
            ss1 = s1;
            ss2 = s2;
            ss3 = s3;
            ss4 = s4;
            ss5 = s5;
            ss6 = s6;
            ss7 = s7;
            sz1 = z1;
            sz2 = z2;
            sz3 = z3;
            sz11 = z11;
            sz12 = z12;
            sz13 = z13;
            sz21 = z21;
            sz22 = z22;
            sz23 = z23;
            sz31 = z31;
            sz32 = z32;
            sz33 = z33;
            zcosg = zcosgl;
            zsing = zsingl;
            zcosi = zcosil;
            zsini = zsinil;
            zcosh = zcoshl * cnodm + zsinhl * snodm;
            zsinh = snodm * zcoshl - cnodm * zsinhl;
            cc = c1l;
        }
    }

    state.zmol = std::fmod(4.7199672 + 0.22997150 * day - gam, TWO_PI);
    state.zmos = std::fmod(6.2565837 + 0.017201977 * day, TWO_PI);

    // Solar terms
    state.se2 = 2.0 * ss1 * ss6;
    state.se3 = 2.0 * ss1 * ss7;
    state.si2 = 2.0 * ss2 * sz12;
    state.si3 = 2.0 * ss2 * (sz13 - sz11);
    state.sl2 = -2.0 * ss3 * sz2;
    state.sl3 = -2.0 * ss3 * (sz3 - sz1);
    state.sl4 = -2.0 * ss3 * (-21.0 - 9.0 * emsq) * zes;
    state.sgh2 = 2.0 * ss4 * sz32;
    state.sgh3 = 2.0 * ss4 * (sz33 - sz31);
    state.sgh4 = -18.0 * ss4 * zes;
    state.sh2 = -2.0 * ss2 * sz22;
    state.sh3 = -2.0 * ss2 * (sz23 - sz21);

    // Lunar terms
    state.ee2 = 2.0 * s1 * s6;
    state.e3 = 2.0 * s1 * s7;
    state.xi2 = 2.0 * s2 * z12;
    state.xi3 = 2.0 * s2 * (z13 - z11);
    state.xl2 = -2.0 * s3 * z2;
    state.xl3 = -2.0 * s3 * (z3 - z1);
    state.xl4 = -2.0 * s3 * (-21.0 - 9.0 * emsq) * zel;
    state.xgh2 = 2.0 * s4 * z32;
    state.xgh3 = 2.0 * s4 * (z33 - z31);
    state.xgh4 = -18.0 * s4 * zel;
    state.xh2 = -2.0 * s2 * z22;
    state.xh3 = -2.0 * s2 * (z23 - z21);

    // Apply deep space long period periodics
    double f2 = 0.5 * sinomm * sinomm - 0.25;
    double f3 = -0.5 * sinomm * cosomm;
    double ses = state.se2 * f2 + state.se3 * f3;
    double sis = state.si2 * f2 + state.si3 * f3;
    double sls = state.sl2 * f2 + state.sl3 * f3 + state.sl4 * sinomm;
    double sghs = state.sgh2 * f2 + state.sgh3 * f3 + state.sgh4 * sinomm;
    double shs = state.sh2 * f2 + state.sh3 * f3;

    if (state.inclo < 5.2359877e-2 || state.inclo > PI - 5.2359877e-2) {
        shs = 0.0;
    }
    if (sinim != 0.0) {
        shs = shs / sinim;
    }

    double sel = state.ee2 * f2 + state.e3 * f3;
    double sil = state.xi2 * f2 + state.xi3 * f3;
    double sll = state.xl2 * f2 + state.xl3 * f3 + state.xl4 * sinomm;
    double sghl = state.xgh2 * f2 + state.xgh3 * f3 + state.xgh4 * sinomm;
    double shll = state.xh2 * f2 + state.xh3 * f3;

    if (state.inclo < 5.2359877e-2 || state.inclo > PI - 5.2359877e-2) {
        shll = 0.0;
    }

    state.peo = ses + sel;
    state.pinco = sis + sil;
    state.plo = sls + sll;
    state.pgho = sghs + sghl;
    state.pho = shs + shll;

    // Initialize resonance terms
    if (state.irez != 0) {
        double aonv = std::pow(nm / XKE, X2O3);

        // Compute the resonance terms
        // Geopotential resonance terms
        double cosisq = cosim * cosim;
        double emo = em;
        em = state.ecco;
        double emsqo = emsq;
        emsq = state.eccsq;
        emsq = state.ecco * state.ecco;
        double eoc = em * emsq;
        double g201 = -0.306 - (em - 0.64) * 0.440;
        double g211, g310, g322, g410, g422, g520, g521, g532, g533;

        if (em <= 0.65) {
            g211 = 3.616 - 13.2470 * em + 16.2900 * emsq;
            g310 = -19.302 + 117.3900 * em - 228.4190 * emsq + 156.5910 * eoc;
            g322 = -18.9068 + 109.7927 * em - 214.6334 * emsq + 146.5816 * eoc;
            g410 = -41.122 + 242.6940 * em - 471.0940 * emsq + 313.9530 * eoc;
            g422 = -146.407 + 841.8800 * em - 1629.014 * emsq + 1083.4350 * eoc;
            g520 = -532.114 + 3017.977 * em - 5740.032 * emsq + 3708.2760 * eoc;
        } else {
            g211 = -72.099 + 331.819 * em - 508.738 * emsq + 266.724 * eoc;
            g310 = -346.844 + 1582.851 * em - 2415.925 * emsq + 1246.113 * eoc;
            g322 = -342.585 + 1554.908 * em - 2366.899 * emsq + 1215.972 * eoc;
            g410 = -1052.797 + 4758.686 * em - 7193.992 * emsq + 3651.957 * eoc;
            g422 = -3581.690 + 16178.11 * em - 24462.77 * emsq + 12422.52 * eoc;
            if (em > 0.715) {
                g520 = -5149.66 + 29936.92 * em - 54087.36 * emsq + 31324.56 * eoc;
            } else {
                g520 = 1464.74 - 4664.75 * em + 3763.64 * emsq;
            }
        }

        if (em < 0.7) {
            g533 = -919.22770 + 4988.6100 * em - 9064.7700 * emsq + 5542.210 * eoc;
            g521 = -822.71072 + 4568.6173 * em - 8491.4146 * emsq + 5337.524 * eoc;
            g532 = -853.66600 + 4690.2500 * em - 8624.7700 * emsq + 5341.400 * eoc;
        } else {
            g533 = -37995.780 + 161616.52 * em - 229838.20 * emsq + 109377.94 * eoc;
            g521 = -51752.104 + 218913.95 * em - 309468.16 * emsq + 146349.42 * eoc;
            g532 = -40023.880 + 170470.89 * em - 242699.48 * emsq + 115605.82 * eoc;
        }

        double sini2 = sinim * sinim;
        double f220 = 0.75 * (1.0 + 2.0 * cosim + cosisq);
        double f221 = 1.5 * sini2;
        double f321 = 1.875 * sinim * (1.0 - 2.0 * cosim - 3.0 * cosisq);
        double f322 = -1.875 * sinim * (1.0 + 2.0 * cosim - 3.0 * cosisq);
        double f441 = 35.0 * sini2 * f220;
        double f442 = 39.375 * sini2 * sini2;
        double f522 = 9.84375 * sinim * (sini2 * (1.0 - 2.0 * cosim - 5.0 * cosisq) +
                      0.33333333 * (-2.0 + 4.0 * cosim + 6.0 * cosisq));
        double f523 = sinim * (4.92187512 * sini2 * (-2.0 - 4.0 * cosim + 10.0 * cosisq) +
                      6.56250012 * (1.0 + 2.0 * cosim - 3.0 * cosisq));
        double f542 = 29.53125 * sinim * (2.0 - 8.0 * cosim + cosisq * (-12.0 + 8.0 * cosim + 10.0 * cosisq));
        double f543 = 29.53125 * sinim * (-2.0 - 8.0 * cosim + cosisq * (12.0 + 8.0 * cosim - 10.0 * cosisq));

        double xno2 = nm * nm;
        double ainv2 = aonv * aonv;
        double temp1_ds = 3.0 * xno2 * ainv2;
        double temp_ds = temp1_ds * J2 / 2.0;

        if (state.irez == 2) {
            // Half-day resonance terms
            double theta = std::fmod(state.gsto + tc * 7.29211514668855e-5, TWO_PI);
            state.d2201 = temp_ds * f220 * g201;
            state.d2211 = temp_ds * f221 * g211;
            state.d3210 = temp1_ds * 1.5 * J2 * f321 * g310 * ainv2;
            state.d3222 = temp1_ds * 1.5 * J2 * f322 * g322 * ainv2;
            state.d4410 = temp1_ds * temp_ds * 2.0 * f441 * g410 * ainv2;
            state.d4422 = temp1_ds * temp_ds * 2.0 * f442 * g422 * ainv2;
            state.d5220 = temp1_ds * temp_ds * f522 * g520 * ainv2 * ainv2;
            state.d5232 = temp1_ds * temp_ds * f523 * g532 * ainv2 * ainv2;
            state.d5421 = temp1_ds * temp_ds * f542 * g521 * ainv2 * ainv2;
            state.d5433 = temp1_ds * temp_ds * f543 * g533 * ainv2 * ainv2;
            state.xlamo = std::fmod(state.mo + state.nodeo + state.nodeo - theta - theta, TWO_PI);
            double xfact = state.mdot + state.dmdt + 2.0 * (state.nodedot + state.dnodt - 7.29211514668855e-5) - state.no_unkozai;
            em = emo;
            emsq = emsqo;
        }

        if (state.irez == 1) {
            // Synchronous resonance terms
            double g200 = 1.0 + emsq * (-2.5 + 0.8125 * emsq);
            double g310_s = 1.0 + 2.0 * emsq;
            double g300 = 1.0 + emsq * (-6.0 + 6.60937 * emsq);
            double f220_s = 0.75 * (1.0 + cosim) * (1.0 + cosim);
            double f311 = 0.9375 * sinim * sinim * (1.0 + 3.0 * cosim) - 0.75 * (1.0 + cosim);
            double f330 = 1.0 + cosim;
            f330 = 1.875 * f330 * f330 * f330;

            state.del1 = 3.0 * nm * nm * aonv * aonv;
            state.del2 = 2.0 * state.del1 * f220_s * g200 * J2;
            state.del3 = 3.0 * state.del1 * f330 * g300 * J3OJ2 * aonv;
            state.del1 = state.del1 * f311 * g310_s * J2 * aonv;

            double theta_s = std::fmod(state.gsto + tc * 7.29211514668855e-5, TWO_PI);
            state.xlamo = std::fmod(state.mo + state.nodeo + state.argpo - theta_s, TWO_PI);
            double xfact = state.mdot + state.argpdot + state.nodedot - 7.29211514668855e-5 - state.no_unkozai;
        }

        // Initialize integrator
        state.xli = state.xlamo;
        state.xni = state.no_unkozai;
        state.atime = 0.0;
    }
}

// Apply deep space secular effects
void deepSpaceSecular(
    const State& state,
    double t, double& em, double& argpm, double& inclm,
    double& nodem, double& mm, double& nm,
    double& atime, double& xli, double& xni) {

    constexpr double step = 720.0;
    constexpr double step2 = step * step / 2.0;

    // Initialize from state
    atime = state.atime;
    xli = state.xli;
    xni = state.xni;

    // Apply lunar-solar periodics
    double zm = state.zmos + 0.017201977 * t;
    double zf = zm + 2.0 * 0.01675 * std::sin(zm);
    double sinzf = std::sin(zf);
    double f2 = 0.5 * sinzf * sinzf - 0.25;
    double f3 = -0.5 * sinzf * std::cos(zf);

    double ses = state.se2 * f2 + state.se3 * f3;
    double sis = state.si2 * f2 + state.si3 * f3;
    double sls = state.sl2 * f2 + state.sl3 * f3 + state.sl4 * sinzf;
    double sghs = state.sgh2 * f2 + state.sgh3 * f3 + state.sgh4 * sinzf;
    double shs = state.sh2 * f2 + state.sh3 * f3;

    zm = state.zmol + 0.22997150 * t;
    zf = zm + 2.0 * 0.05490 * std::sin(zm);
    sinzf = std::sin(zf);
    f2 = 0.5 * sinzf * sinzf - 0.25;
    f3 = -0.5 * sinzf * std::cos(zf);

    double sel = state.ee2 * f2 + state.e3 * f3;
    double sil = state.xi2 * f2 + state.xi3 * f3;
    double sll = state.xl2 * f2 + state.xl3 * f3 + state.xl4 * sinzf;
    double sghl = state.xgh2 * f2 + state.xgh3 * f3 + state.xgh4 * sinzf;
    double shll = state.xh2 * f2 + state.xh3 * f3;

    double pe = ses + sel;
    double pinc = sis + sil;
    double pl = sls + sll;
    double pgh = sghs + sghl;
    double ph = shs + shll;

    if (std::abs(state.inclo) >= 0.2) {
        ph /= std::sin(state.inclo);
        inclm += pinc;
        nodem += ph;
        argpm -= pgh;
    } else {
        // Apply Lyddane modification
        double siniq = std::sin(state.inclo);
        double cosiq = std::cos(state.inclo);
        double temp_mod = ph * cosiq;

        inclm += pinc;
        nodem += ph / siniq;
        argpm -= temp_mod / siniq;
    }

    em += pe;
    mm += pl;

    // Handle resonance effects
    if (state.irez != 0) {
        double ft = 0.0;
        constexpr double earthRotRate = 7.29211514668855e-5;  // Earth rotation rate rad/min

        // Synchronous resonance terms
        if (state.irez == 1) {
            double xndt;
            double xfact = state.mdot + state.argpdot + state.nodedot - earthRotRate - state.no_unkozai;
            double xldot = xni + xfact;

            // Calculate resonance effects
            double theta = std::fmod(state.gsto + t * earthRotRate, TWO_PI);
            xndt = state.del1 * std::sin(state.xlamo - 2.0 * (state.nodeo + state.argpo) + theta) +
                   state.del2 * std::sin(2.0 * (state.xlamo - state.nodeo - state.argpo)) +
                   state.del3 * std::sin(3.0 * state.xlamo - state.nodeo - state.argpo + theta);

            if (std::abs(t - atime) >= step) {
                // Integrate using predictor-corrector
                double stepp = step;
                if (t < atime) {
                    stepp = -step;
                }

                while (std::abs(t - atime) >= step) {
                    xldot = xni + xfact;
                    xli += xldot * stepp + xndt * step2;
                    xni += xndt * stepp;
                    atime += stepp;

                    theta = std::fmod(state.gsto + atime * earthRotRate, TWO_PI);
                    xndt = state.del1 * std::sin(xli - 2.0 * (state.nodeo + state.argpo) + theta) +
                           state.del2 * std::sin(2.0 * (xli - state.nodeo - state.argpo)) +
                           state.del3 * std::sin(3.0 * xli - state.nodeo - state.argpo + theta);
                }
            }

            ft = t - atime;
            xldot = xni + xfact;
            nm = xni + xndt * ft;
            double xl = xli + xldot * ft + xndt * ft * ft * 0.5;
            mm = xl - 2.0 * nodem + 2.0 * theta;
        }

        // Half-day resonance terms
        if (state.irez == 2) {
            double xndt, xnddt, xomi;
            double xfact = state.mdot + state.dmdt + 2.0 * (state.nodedot + state.dnodt - earthRotRate) - state.no_unkozai;
            double xldot = xni + xfact;
            constexpr double g22 = 5.7686396;
            constexpr double g32 = 0.95240898;
            constexpr double g44 = 1.8014998;
            constexpr double g52 = 1.0508330;
            constexpr double g54 = 4.4108898;

            double theta = std::fmod(state.gsto + t * earthRotRate, TWO_PI);

            xomi = state.argpo + state.argpdot * atime;
            double x2omi = xomi + xomi;
            double x2li = xli + xli;

            xndt = state.d2201 * std::sin(x2omi + xli - g22) +
                   state.d2211 * std::sin(xli - g22) +
                   state.d3210 * std::sin(xomi + xli - g32) +
                   state.d3222 * std::sin(-xomi + xli - g32) +
                   state.d4410 * std::sin(x2omi + x2li - g44) +
                   state.d4422 * std::sin(x2li - g44) +
                   state.d5220 * std::sin(xomi + xli - g52) +
                   state.d5232 * std::sin(-xomi + xli - g52) +
                   state.d5421 * std::sin(xomi + x2li - g54) +
                   state.d5433 * std::sin(-xomi + x2li - g54);

            xnddt = state.d2201 * std::cos(x2omi + xli - g22) +
                    state.d2211 * std::cos(xli - g22) +
                    state.d3210 * std::cos(xomi + xli - g32) +
                    state.d3222 * std::cos(-xomi + xli - g32) +
                    state.d5220 * std::cos(xomi + xli - g52) +
                    state.d5232 * std::cos(-xomi + xli - g52) +
                    2.0 * (state.d4410 * std::cos(x2omi + x2li - g44) +
                           state.d4422 * std::cos(x2li - g44) +
                           state.d5421 * std::cos(xomi + x2li - g54) +
                           state.d5433 * std::cos(-xomi + x2li - g54));
            xnddt *= xldot;

            if (std::abs(t - atime) >= step) {
                double stepp = step;
                if (t < atime) {
                    stepp = -step;
                }

                while (std::abs(t - atime) >= step) {
                    xldot = xni + xfact;
                    xli += xldot * stepp + xndt * step2;
                    xni += xndt * stepp;
                    atime += stepp;

                    xomi = state.argpo + state.argpdot * atime;
                    x2omi = xomi + xomi;
                    x2li = xli + xli;

                    xndt = state.d2201 * std::sin(x2omi + xli - g22) +
                           state.d2211 * std::sin(xli - g22) +
                           state.d3210 * std::sin(xomi + xli - g32) +
                           state.d3222 * std::sin(-xomi + xli - g32) +
                           state.d4410 * std::sin(x2omi + x2li - g44) +
                           state.d4422 * std::sin(x2li - g44) +
                           state.d5220 * std::sin(xomi + xli - g52) +
                           state.d5232 * std::sin(-xomi + xli - g52) +
                           state.d5421 * std::sin(xomi + x2li - g54) +
                           state.d5433 * std::sin(-xomi + x2li - g54);

                    xnddt = state.d2201 * std::cos(x2omi + xli - g22) +
                            state.d2211 * std::cos(xli - g22) +
                            state.d3210 * std::cos(xomi + xli - g32) +
                            state.d3222 * std::cos(-xomi + xli - g32) +
                            state.d5220 * std::cos(xomi + xli - g52) +
                            state.d5232 * std::cos(-xomi + xli - g52) +
                            2.0 * (state.d4410 * std::cos(x2omi + x2li - g44) +
                                   state.d4422 * std::cos(x2li - g44) +
                                   state.d5421 * std::cos(xomi + x2li - g54) +
                                   state.d5433 * std::cos(-xomi + x2li - g54));
                    xnddt *= xldot;
                }
            }

            ft = t - atime;
            xldot = xni + xfact;
            nm = xni + xndt * ft + xnddt * ft * ft * 0.5;
            double xl = xli + xldot * ft + xndt * ft * ft * 0.5;
            double temp_mm = -nodem - nodem + theta + theta;
            mm = xl - xomi + temp_mm;
        }
    }
}

// Apply deep space periodic effects
void deepSpacePeriodic(
    const State& state,
    double t, double& em, double& inclm, double& nodem,
    double& argpm, double& mm) {

    // Calculate lunar-solar periodics
    double zm = state.zmos + 0.017201977 * t;
    double zf = zm + 2.0 * 0.01675 * std::sin(zm);
    double sinzf = std::sin(zf);
    double f2 = 0.5 * sinzf * sinzf - 0.25;
    double f3 = -0.5 * sinzf * std::cos(zf);

    double ses = state.se2 * f2 + state.se3 * f3;
    double sis = state.si2 * f2 + state.si3 * f3;
    double sls = state.sl2 * f2 + state.sl3 * f3 + state.sl4 * sinzf;
    double sghs = state.sgh2 * f2 + state.sgh3 * f3 + state.sgh4 * sinzf;
    double shs = state.sh2 * f2 + state.sh3 * f3;

    zm = state.zmol + 0.22997150 * t;
    zf = zm + 2.0 * 0.05490 * std::sin(zm);
    sinzf = std::sin(zf);
    f2 = 0.5 * sinzf * sinzf - 0.25;
    f3 = -0.5 * sinzf * std::cos(zf);

    double sel = state.ee2 * f2 + state.e3 * f3;
    double sil = state.xi2 * f2 + state.xi3 * f3;
    double sll = state.xl2 * f2 + state.xl3 * f3 + state.xl4 * sinzf;
    double sghl = state.xgh2 * f2 + state.xgh3 * f3 + state.xgh4 * sinzf;
    double shll = state.xh2 * f2 + state.xh3 * f3;

    double pe = ses + sel - state.peo;
    double pinc = sis + sil - state.pinco;
    double pl = sls + sll - state.plo;
    double pgh = sghs + sghl - state.pgho;
    double ph = shs + shll - state.pho;

    if (std::abs(state.inclo) >= 0.2) {
        ph /= std::sin(state.inclo);
        inclm += pinc;
        em += pe;
        nodem += ph;
        argpm -= pgh;
        mm += pl;
    } else {
        // Apply Lyddane modification
        double siniq = std::sin(state.inclo);
        double cosiq = std::cos(state.inclo);

        inclm += pinc;
        em += pe;

        double sinis = std::sin(inclm);
        double cosis = std::cos(inclm);

        if (std::abs(inclm) >= 0.2) {
            double temp_per = ph / sinis;
            nodem += temp_per;
            argpm -= pgh - cosiq * temp_per;
            mm += pl;
        } else {
            // Near-equatorial orbit, Lyddane modification
            double temp_per = ph * cosiq;
            nodem += ph / siniq;
            argpm -= temp_per / siniq;
            mm += pl;
        }
    }
}

// Core SGP4/SDP4 propagation
Result propagate(const State& state, double tsince) {
    constexpr double radiusearthkm = RADIUS_EARTH_KM;
    constexpr double xke = XKE;
    constexpr double j2 = J2;
    constexpr double j3oj2 = J3OJ2;
    constexpr double vkmpersec = VKMPERSEC;

    Result result;

    double cosio = std::cos(state.inclo);
    double sinio = std::sin(state.inclo);

    // Update for secular gravity and atmospheric drag
    double xmdf = state.mo + state.mdot * tsince;
    double argpdf = state.argpo + state.argpdot * tsince;
    double nodedf = state.nodeo + state.nodedot * tsince;
    double argpm = argpdf;
    double mm = xmdf;
    double t2 = tsince * tsince;
    double nodem = nodedf + state.nodecf * t2;
    double tempa = 1.0 - state.cc1 * tsince;
    double tempe = state.bstar * state.cc4 * tsince;
    double templ = state.t2cof * t2;

    if (!state.isimp) {
        double delomg = state.omgcof * tsince;
        double delm = state.xmcof * (std::pow(1.0 + state.eta * std::cos(xmdf), 3) - state.delmo);
        double temp_sgp = delomg + delm;
        mm = xmdf + temp_sgp;
        argpm = argpdf - temp_sgp;
        double t3 = t2 * tsince;
        double t4 = t3 * tsince;
        tempa = tempa - state.d2 * t2 - state.d3 * t3 - state.d4 * t4;
        tempe = tempe + state.bstar * state.cc5 * (std::sin(mm) - state.sinmao);
        templ = templ + state.t3cof * t3 + t4 * (state.t4cof + tsince * state.t5cof);
    }

    double nm = state.no_unkozai;
    double em = state.ecco;
    double inclm = state.inclo;

    // Initialize resonance state from input
    double atime = state.atime;
    double xli = state.xli;
    double xni = state.xni;

    // Handle deep space satellites
    if (state.method == 'd') {
        deepSpaceSecular(state, tsince, em, argpm, inclm, nodem, mm, nm, atime, xli, xni);
    }

    double am = std::pow(xke / nm, X2O3) * tempa * tempa;
    nm = xke / std::pow(am, 1.5);
    em = em - tempe;

    // Check for eccentricity out of range
    if (em >= 1.0 || em < -0.001) {
        // TODO: Address exception handling
        printf("SGP4 Error: Eccentricity out of range during propagation: %f\n", em);
        result.error = ErrorCode::ECCENTRICITY_OUT_OF_RANGE;
        return result;
    }
    if (em < 1.0e-6) {
        em = 1.0e-6;
    }

    mm = mm + state.no_unkozai * templ;
    double xlm = mm + argpm + nodem;

    nodem = std::fmod(nodem, TWO_PI);
    argpm = std::fmod(argpm, TWO_PI);
    xlm = std::fmod(xlm, TWO_PI);
    mm = std::fmod(xlm - argpm - nodem, TWO_PI);

    // Apply deep space periodic effects
    if (state.method == 'd') {
        deepSpacePeriodic(state, tsince, em, inclm, nodem, argpm, mm);
    }

    // Re-compute sini/cosi if inclination changed
    if (inclm != state.inclo) {
        sinio = std::sin(inclm);
        cosio = std::cos(inclm);
    }

    // Check for eccentricity out of range (after deep space effects)
    if (em < 0.0) {
        em = 1.0e-6;
    }

    double sinim = std::sin(inclm);
    double cosim = std::cos(inclm);

    // Add lunar-solar periodics
    double ep = em;
    double xincp = inclm;
    double argpp = argpm;
    double nodep = nodem;
    double mp = mm;

    // Check if satellite decayed
    if (nm <= 0.0) {
        // TODO: Address exception handling
        printf("SGP4 Error: Satellite has decayed\n");
        result.error = ErrorCode::SATELLITE_DECAYED;
        return result;
    }

    double eccsq = ep * ep;
    double omeosq = 1.0 - eccsq;

    if (omeosq <= 0.0) {
        // TODO: Address exception handling
        printf("SGP4 Error: Semi-latus rectum is negative\n");
        result.error = ErrorCode::NEGATIVE_SEMI_LATUS_RECTUM;
        return result;
    }

    double rteosq = std::sqrt(omeosq);

    // Long period periodics
    cosio = cosim;
    sinio = sinim;
    double cosio2 = cosio * cosio;

    double axnl = ep * std::cos(argpp);
    double temp_lp = 1.0 / (am * omeosq);
    double aynl = ep * std::sin(argpp) + temp_lp * state.aycof;
    double xl = mp + argpp + nodep + temp_lp * state.xlcof * axnl;

    // Solve Kepler's equation
    double u = std::fmod(xl - nodep, TWO_PI);
    double eo1 = u;
    double tem5 = 9999.9;
    int ktr = 1;
    double sineo1, coseo1;

    while ((std::abs(tem5) >= 1.0e-12) && (ktr <= 10)) {
        sineo1 = std::sin(eo1);
        coseo1 = std::cos(eo1);
        tem5 = 1.0 - coseo1 * axnl - sineo1 * aynl;
        tem5 = (u - aynl * coseo1 + axnl * sineo1 - eo1) / tem5;
        if (std::abs(tem5) >= 0.95) {
            tem5 = tem5 > 0.0 ? 0.95 : -0.95;
        }
        eo1 = eo1 + tem5;
        ktr++;
    }

    // Short period preliminary quantities
    double ecose = axnl * coseo1 + aynl * sineo1;
    double esine = axnl * sineo1 - aynl * coseo1;
    double el2 = axnl * axnl + aynl * aynl;
    double pl = am * (1.0 - el2);

    if (pl < 0.0) {
        // TODO: Address exception handling
        printf("SGP4 Error: Semi-latus rectum is negative\n");
        result.error = ErrorCode::NEGATIVE_SEMI_LATUS_RECTUM;
        return result;
    }

    double rl = am * (1.0 - ecose);
    double rdotl = std::sqrt(am) * esine / rl;
    double rvdotl = std::sqrt(pl) / rl;
    double betal = std::sqrt(1.0 - el2);
    double temp_sp = esine / (1.0 + betal);
    double sinu = am / rl * (sineo1 - aynl - axnl * temp_sp);
    double cosu = am / rl * (coseo1 - axnl + aynl * temp_sp);
    double su = std::atan2(sinu, cosu);
    double sin2u = (cosu + cosu) * sinu;
    double cos2u = 1.0 - 2.0 * sinu * sinu;
    double temp_sp2 = 1.0 / pl;
    double temp1_sp = 0.5 * j2 * temp_sp2;
    double temp2_sp = temp1_sp * temp_sp2;

    // Update for short period periodics
    double con41 = 3.0 * cosio2 - 1.0;
    double x1mth2 = 1.0 - cosio2;
    double x7thm1 = 7.0 * cosio2 - 1.0;

    double mrt = rl * (1.0 - 1.5 * temp2_sp * betal * con41) + 0.5 * temp1_sp * x1mth2 * cos2u;
    su = su - 0.25 * temp2_sp * x7thm1 * sin2u;
    double xnode = nodep + 1.5 * temp2_sp * cosio * sin2u;
    double xinc = xincp + 1.5 * temp2_sp * cosio * sinio * cos2u;
    double mvt = rdotl - nm * temp1_sp * x1mth2 * sin2u / xke;
    double rvdot = rvdotl + nm * temp1_sp * (x1mth2 * cos2u + 1.5 * con41) / xke;

    // Orientation vectors
    double sinsu = std::sin(su);
    double cossu = std::cos(su);
    double snod = std::sin(xnode);
    double cnod = std::cos(xnode);
    double sini = std::sin(xinc);
    double cosi = std::cos(xinc);
    double xmx = -snod * cosi;
    double xmy = cnod * cosi;
    double ux = xmx * sinsu + cnod * cossu;
    double uy = xmy * sinsu + snod * cossu;
    double uz = sini * sinsu;
    double vx = xmx * cossu - cnod * sinsu;
    double vy = xmy * cossu - snod * sinsu;
    double vz = sini * cossu;

    // Position and velocity (in km and km/s)
    result.r[0] = mrt * ux * radiusearthkm;
    result.r[1] = mrt * uy * radiusearthkm;
    result.r[2] = mrt * uz * radiusearthkm;
    result.v[0] = (mvt * ux + rvdot * vx) * vkmpersec;
    result.v[1] = (mvt * uy + rvdot * vy) * vkmpersec;
    result.v[2] = (mvt * uz + rvdot * vz) * vkmpersec;

    // Store updated resonance state
    result.atime = atime;
    result.xli = xli;
    result.xni = xni;

    // Check if satellite decayed
    if (mrt < 1.0) {
        // TODO: Address exception handling
        printf("SGP4 Error: Satellite has decayed\n");
        result.error = ErrorCode::SATELLITE_DECAYED;
        return result;
    }

    return result;
}

} // namespace sattrack::sgp4
