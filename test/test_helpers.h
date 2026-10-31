// Shared helpers for the Quaternion test suite.
//
// The reference implementation throughout is boost::math::quaternion<double>.
// Our Quaternion stores (a, b, c, d) meaning a + b*i + c*j + d*k, which is the
// same component order Boost uses (R_component_1 .. R_component_4), so the
// conversions below are straight copies.
//
// Everything the library under test computes is single precision, so the
// reference values are computed in double from the *same float inputs* and the
// comparison tolerances are expressed in units of float epsilon.
#ifndef QUATERNION_TEST_HELPERS_H
#define QUATERNION_TEST_HELPERS_H

#include <Quaternion.h>

#include <boost/math/quaternion.hpp>
#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <iomanip>
#include <limits>
#include <ostream>
#include <random>

typedef boost::math::quaternion<double> BoostQuat;

const double kPi = 3.14159265358979323846;
const float kPiF = static_cast<float>(kPi);

// kOpTol: a single arithmetic operation (a product, a sum, a norm). The float
//         Hamilton product measures about 1.4 ulp away from the double result.
// kChainTol: a handful of chained operations (trig + products + normalize).
// Both are absolute tolerances for unit-magnitude results; use scaled() for
// results whose magnitude grows with the operands.
const double kOpTol = 8.0 * FLT_EPSILON;      // ~9.5e-7
const double kChainTol = 32.0 * FLT_EPSILON;  // ~3.8e-6

inline double scaled(double tol, double magnitude) {
    return tol * std::max(1.0, magnitude);
}

// ---------------------------------------------------------------- conversions

inline BoostQuat toBoost(const Quaternion &q) {
    return BoostQuat(q.a, q.b, q.c, q.d);
}

inline BoostQuat toBoost(const BoostQuat &q) {
    return q;
}

inline Quaternion fromBoost(const BoostQuat &q) {
    return Quaternion(static_cast<float>(q.R_component_1()),
                      static_cast<float>(q.R_component_2()),
                      static_cast<float>(q.R_component_3()),
                      static_cast<float>(q.R_component_4()));
}

inline std::ostream &operator<<(std::ostream &os, const Quaternion &q) {
    return os << "(" << q.a << "," << q.b << "," << q.c << "," << q.d << ")";
}

// ------------------------------------------------------------------- 3-vectors

struct Vec3 {
    double x;
    double y;
    double z;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

inline Vec3 operator+(const Vec3 &u, const Vec3 &v) {
    return Vec3(u.x + v.x, u.y + v.y, u.z + v.z);
}

inline Vec3 operator-(const Vec3 &u, const Vec3 &v) {
    return Vec3(u.x - v.x, u.y - v.y, u.z - v.z);
}

inline Vec3 operator*(double s, const Vec3 &v) {
    return Vec3(s * v.x, s * v.y, s * v.z);
}

inline double dot(const Vec3 &u, const Vec3 &v) {
    return u.x * v.x + u.y * v.y + u.z * v.z;
}

inline Vec3 cross(const Vec3 &u, const Vec3 &v) {
    return Vec3(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x);
}

inline double length(const Vec3 &v) {
    return std::sqrt(dot(v, v));
}

inline double distance(const Vec3 &u, const Vec3 &v) {
    return length(u - v);
}

inline Vec3 normalized(const Vec3 &v) {
    return (1.0 / length(v)) * v;
}

// Round each component to the nearest float, so a reference computed in double
// sees exactly the values the float library was given.
inline Vec3 asFloat(const Vec3 &v) {
    return Vec3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

inline Quaternion pure(const Vec3 &v) {
    return Quaternion(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

inline BoostQuat pureBoost(const Vec3 &v) {
    return BoostQuat(0.0, v.x, v.y, v.z);
}

inline Vec3 vectorPart(const Quaternion &q) {
    return Vec3(q.b, q.c, q.d);
}

inline Vec3 vectorPart(const BoostQuat &q) {
    return Vec3(q.R_component_2(), q.R_component_3(), q.R_component_4());
}

// Rodrigues' rotation formula: rotate v by `angle` radians about the unit
// vector `axis`, right-hand rule. This is an independent reference for what a
// rotation *means*; it uses no quaternion arithmetic at all.
inline Vec3 rodrigues(const Vec3 &axis, double angle, const Vec3 &v) {
    double c = std::cos(angle);
    double s = std::sin(angle);
    return c * v + s * cross(axis, v) + ((1.0 - c) * dot(axis, v)) * axis;
}

// ------------------------------------------------------- reference quaternions

// Axis-angle construction in double via Boost: exp of the pure quaternion v/2
// is cos(|v|/2) + (v/|v|) sin(|v|/2), i.e. the rotation by |v| radians about v.
inline BoostQuat axisAngleReference(const Vec3 &v) {
    return exp(BoostQuat(0.0, v.x / 2.0, v.y / 2.0, v.z / 2.0));
}

// Yaw-pitch-roll (Tait-Bryan ZYX): roll about x first, then pitch about y,
// then yaw about z, all about the fixed frame axes.
inline BoostQuat eulerReference(double x, double y, double z) {
    return axisAngleReference(Vec3(0.0, 0.0, z)) * axisAngleReference(Vec3(0.0, y, 0.0)) *
           axisAngleReference(Vec3(x, 0.0, 0.0));
}

// ------------------------------------------------------------------ comparisons

// A NaN anywhere counts as infinitely far apart. std::max() silently drops a
// NaN operand, which would let a NaN result pass a closeness check.
inline double maxComponentDiff(const BoostQuat &x, const BoostQuat &y) {
    const double diffs[4] = {std::fabs(x.R_component_1() - y.R_component_1()),
                             std::fabs(x.R_component_2() - y.R_component_2()),
                             std::fabs(x.R_component_3() - y.R_component_3()),
                             std::fabs(x.R_component_4() - y.R_component_4())};
    double worst = 0.0;
    for (int i = 0; i < 4; i++) {
        if (std::isnan(diffs[i])) {
            return std::numeric_limits<double>::infinity();
        }
        worst = std::max(worst, diffs[i]);
    }
    return worst;
}

// Rotation angle in [0, pi] represented by a (not necessarily unit) quaternion,
// treating q and -q as the same rotation. NaN components give infinity so that
// a NaN can never look like a small angle.
inline double rotationAngle(const BoostQuat &q) {
    double angle = 2.0 * std::atan2(length(vectorPart(q)), std::fabs(q.R_component_1()));
    return std::isnan(angle) ? std::numeric_limits<double>::infinity() : angle;
}

// Angle of the rotation taking rotation x to rotation y: zero when they are
// the same rotation, including when y == -x.
inline double rotationDistance(const BoostQuat &x, const BoostQuat &y) {
    return rotationAngle(conj(x) * y);
}

// Every component of ACTUAL is within TOL of EXPECTED. Either argument may be
// a Quaternion or a BoostQuat.
#define CHECK_QUAT_CLOSE(ACTUAL, EXPECTED, TOL)                                        \
    do {                                                                               \
        const BoostQuat actual_ = toBoost(ACTUAL);                                     \
        const BoostQuat expected_ = toBoost(EXPECTED);                                 \
        const double tol_ = (TOL);                                                     \
        BOOST_TEST_CONTEXT(std::setprecision(9) << "actual = " << actual_              \
                           << ", expected = " << expected_ << ", tolerance = " << tol_) { \
            BOOST_TEST(maxComponentDiff(actual_, expected_) <= tol_);                  \
        }                                                                              \
    } while (0)

// ACTUAL and EXPECTED describe the same rotation to within TOL radians.
#define CHECK_SAME_ROTATION(ACTUAL, EXPECTED, TOL)                                     \
    do {                                                                               \
        const BoostQuat actual_ = toBoost(ACTUAL);                                     \
        const BoostQuat expected_ = toBoost(EXPECTED);                                 \
        const double tol_ = (TOL);                                                     \
        BOOST_TEST_CONTEXT(std::setprecision(9) << "actual = " << actual_              \
                           << ", expected = " << expected_ << ", tolerance = " << tol_) { \
            BOOST_TEST(rotationDistance(actual_, expected_) <= tol_);                  \
        }                                                                              \
    } while (0)

// ----------------------------------------------------------------- random data

// Deterministic pseudo-random inputs so a failure can be reproduced. Every
// value handed out is exactly representable as a float, so the double
// reference and the float library see identical inputs.
class Random {
public:
    explicit Random(unsigned seed) : rng_(seed) {}

    double uniform(double lo, double hi) {
        return roundToFloat(std::uniform_real_distribution<double>(lo, hi)(rng_));
    }

    // Components uniform in [-magnitude, magnitude].
    Vec3 vector(double magnitude) {
        return Vec3(uniform(-magnitude, magnitude), uniform(-magnitude, magnitude),
                    uniform(-magnitude, magnitude));
    }

    // Uniformly distributed direction (unit to within float rounding).
    Vec3 unitVector() {
        std::normal_distribution<double> gauss;
        for (;;) {
            Vec3 v(gauss(rng_), gauss(rng_), gauss(rng_));
            if (length(v) > 0.1) {
                return asFloat(normalized(v));
            }
        }
    }

    // axis * angle with the angle uniform in (0, maxAngle].
    Vec3 rotationVector(double maxAngle) {
        double angle = std::max(std::uniform_real_distribution<double>(0.0, maxAngle)(rng_), 1e-6);
        return asFloat(angle * unitVector());
    }

    // Arbitrary quaternion with components uniform in [-magnitude, magnitude].
    Quaternion quaternion(double magnitude) {
        return Quaternion(static_cast<float>(uniform(-magnitude, magnitude)),
                          static_cast<float>(uniform(-magnitude, magnitude)),
                          static_cast<float>(uniform(-magnitude, magnitude)),
                          static_cast<float>(uniform(-magnitude, magnitude)));
    }

    // Uniformly distributed rotation (unit to within float rounding).
    Quaternion unitQuaternion() {
        std::normal_distribution<double> gauss;
        for (;;) {
            BoostQuat q(gauss(rng_), gauss(rng_), gauss(rng_), gauss(rng_));
            if (abs(q) > 0.1) {
                return fromBoost(q / abs(q));
            }
        }
    }

private:
    static double roundToFloat(double x) {
        return static_cast<float>(x);
    }

    std::mt19937 rng_;
};

#endif
