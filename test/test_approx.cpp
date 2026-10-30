// The small-angle approximations (from_*_approx) and fractional() are
// deliberately inexact, so these tests bound their error instead of expecting
// equality. The bounds come from the Taylor remainders of the approximations
// and from measuring the implementation in double precision.
#include "test_helpers.h"

namespace {

Quaternion axisAngleApprox(const Vec3 &v) {
    return Quaternion::from_axis_angle_approx(static_cast<float>(v.x), static_cast<float>(v.y),
                                              static_cast<float>(v.z));
}

Quaternion eulerApprox(double x, double y, double z) {
    return Quaternion::from_euler_rotation_approx(static_cast<float>(x), static_cast<float>(y),
                                                  static_cast<float>(z));
}

}  // namespace

BOOST_AUTO_TEST_SUITE(Approximations)

// from_axis_angle_approx replaces cos(t/2) by 1 - t^2/8 and sin(t/2) by t/2.
//   cos(t/2) = 1 - t^2/8 + t^4/384 - ...   so  |error in w|  <= t^4/384
//   sin(t/2) = t/2 - t^3/48 + ...          so  |error in xyz| <= t^3/48
// (alternating series with shrinking terms for t < 2). The vector error is
// along the unit axis, so the same bound holds for its length.
BOOST_AUTO_TEST_CASE(AxisAngleApproxErrorIsWithinTheTaylorRemainder) {
    Random rnd(30);
    for (int n = 0; n < 3000; n++) {
        Vec3 v = rnd.rotationVector(kPi / 4);
        double t = length(v);
        Quaternion q = axisAngleApprox(v);
        BoostQuat exact = axisAngleReference(v);

        double wBound = std::pow(t, 4) / 384.0 + 4 * FLT_EPSILON;
        double vBound = std::pow(t, 3) / 48.0 + 4 * FLT_EPSILON;
        BOOST_TEST_CONTEXT(std::setprecision(9) << "angle " << t << " approx " << q << " exact " << exact) {
            BOOST_TEST(std::fabs(q.a - real(exact)) <= wBound);
            BOOST_TEST(distance(vectorPart(q), vectorPart(exact)) <= vBound);
        }
    }
}

BOOST_AUTO_TEST_CASE(AxisAngleApproxIsExactForNoRotation) {
    // Unlike from_axis_angle(), there is no division by the angle here.
    CHECK_QUAT_CLOSE(Quaternion::from_axis_angle_approx(0, 0, 0), Quaternion(), 0.0);
}

BOOST_AUTO_TEST_CASE(AxisAngleApproxConvergesAtThirdOrder) {
    // Halving the angle must cut the error by close to 8x, as it does for an
    // error that scales with t^3. Stop before float rounding dominates.
    const Vec3 dir = normalized(Vec3(0.3, -0.5, 0.8));
    double previous = 0.0;
    for (double t = kPi / 4; t > kPi / 64; t /= 2) {
        Vec3 v = asFloat(t * dir);
        double err = maxComponentDiff(toBoost(axisAngleApprox(v)), axisAngleReference(v));
        BOOST_TEST_CONTEXT("angle " << t << " error " << err << " previous " << previous) {
            if (previous > 0.0) {
                BOOST_TEST(previous / err >= 6.0);
                BOOST_TEST(previous / err <= 10.0);
            }
        }
        previous = err;
    }
}

BOOST_AUTO_TEST_CASE(EulerApproxIsCloseToExactWithin45Degrees) {
    // Documented domain: each angle below pi/4. Measured worst case over this
    // grid is 0.0149 per component and 2.3 degrees of rotation error, at the
    // corners where all three angles are 45 degrees.
    const int steps = 8;
    const double limit = kPi / 4;
    double maxComponent = 0.0, maxAngle = 0.0;
    for (int i = -steps; i <= steps; i++) {
        for (int j = -steps; j <= steps; j++) {
            for (int k = -steps; k <= steps; k++) {
                Vec3 e = asFloat(Vec3(i * limit / steps, j * limit / steps, k * limit / steps));
                BoostQuat approx = toBoost(eulerApprox(e.x, e.y, e.z));
                BoostQuat exact = eulerReference(e.x, e.y, e.z);
                maxComponent = std::max(maxComponent, maxComponentDiff(approx, exact));
                maxAngle = std::max(maxAngle, rotationDistance(approx, exact));
            }
        }
    }
    BOOST_TEST(maxComponent <= 0.016);
    BOOST_TEST(maxAngle <= 0.045);  // radians, about 2.6 degrees

    // The error shrinks with the cube of the angle, so a 5 degree step (the
    // sort of per-sample gyro increment this exists for) is far better.
    const double small = 5.0 * kPi / 180.0;
    double maxSmallAngle = 0.0;
    for (int i = -steps; i <= steps; i++) {
        for (int j = -steps; j <= steps; j++) {
            for (int k = -steps; k <= steps; k++) {
                Vec3 e = asFloat(Vec3(i * small / steps, j * small / steps, k * small / steps));
                maxSmallAngle = std::max(maxSmallAngle, rotationDistance(toBoost(eulerApprox(e.x, e.y, e.z)),
                                                                         eulerReference(e.x, e.y, e.z)));
            }
        }
    }
    BOOST_TEST(maxSmallAngle <= 0.045 * std::pow(small / limit, 3) * 2.0);  // ~1.2e-4 rad
}

BOOST_AUTO_TEST_CASE(EulerApproxConvergesAtThirdOrder) {
    double previous = 0.0;
    for (double t = kPi / 4; t > kPi / 64; t /= 2) {
        Vec3 e = asFloat(Vec3(t, 0.7 * t, -0.4 * t));
        double err = maxComponentDiff(toBoost(eulerApprox(e.x, e.y, e.z)), eulerReference(e.x, e.y, e.z));
        BOOST_TEST_CONTEXT("angle " << t << " error " << err << " previous " << previous) {
            if (previous > 0.0) {
                BOOST_TEST(previous / err >= 6.0);
                BOOST_TEST(previous / err <= 10.0);
            }
        }
        previous = err;
    }
}

BOOST_AUTO_TEST_CASE(EulerApproxMatchesAxisAngleApproxForASingleAxis) {
    // With two angles zero both approximations reduce to (1 - t^2/8, t/2).
    Random rnd(31);
    for (int n = 0; n < 1000; n++) {
        double t = rnd.uniform(-kPi / 4, kPi / 4);
        CHECK_QUAT_CLOSE(eulerApprox(t, 0, 0), axisAngleApprox(Vec3(t, 0, 0)), kOpTol);
        CHECK_QUAT_CLOSE(eulerApprox(0, t, 0), axisAngleApprox(Vec3(0, t, 0)), kOpTol);
        CHECK_QUAT_CLOSE(eulerApprox(0, 0, t), axisAngleApprox(Vec3(0, 0, t)), kOpTol);
    }
}

BOOST_AUTO_TEST_CASE(ApproximationsAreNearlyUnit) {
    // Neither approximation normalizes. Within the documented domain the norm
    // drifts by under 1%, so code integrating many steps should normalize.
    Random rnd(32);
    for (int n = 0; n < 2000; n++) {
        Quaternion a = axisAngleApprox(rnd.rotationVector(kPi / 4));
        BOOST_TEST(std::fabs(a.norm() - 1.0) <= 0.01);

        Quaternion e = eulerApprox(rnd.uniform(-kPi / 4, kPi / 4), rnd.uniform(-kPi / 4, kPi / 4),
                                   rnd.uniform(-kPi / 4, kPi / 4));
        BOOST_TEST(std::fabs(e.norm() - 1.0) <= 0.01);
    }
}

BOOST_AUTO_TEST_SUITE_END()

// fractional(f) is a normalized linear interpolation (nlerp) from the identity
// towards this rotation. It keeps the axis exactly and is exact at f = 0, 0.5
// and 1. In between, the angle deviates from f * theta by an amount that grows
// with theta: measured 0.9 degrees at theta = 90 degrees, 8 degrees at 180.
BOOST_AUTO_TEST_SUITE(Fractional)

BOOST_AUTO_TEST_CASE(ZeroGivesTheIdentity) {
    Random rnd(40);
    for (int n = 0; n < 500; n++) {
        Quaternion q = rnd.unitQuaternion();
        q.fractional(0.0f);
        CHECK_QUAT_CLOSE(q, Quaternion(), 0.0);
    }
}

BOOST_AUTO_TEST_CASE(OneGivesTheSameRotation) {
    Random rnd(41);
    for (int n = 0; n < 500; n++) {
        Quaternion q = rnd.unitQuaternion();
        Quaternion before = q;
        q.fractional(1.0f);
        CHECK_QUAT_CLOSE(q, before, kOpTol);
    }
}

BOOST_AUTO_TEST_CASE(HalfGivesExactlyHalfTheAngle) {
    Random rnd(42);
    for (int n = 0; n < 2000; n++) {
        Vec3 w = rnd.rotationVector(0.95 * kPi);
        Quaternion q = Quaternion::from_axis_angle(static_cast<float>(w.x), static_cast<float>(w.y),
                                                   static_cast<float>(w.z));
        q.fractional(0.5f);
        CHECK_QUAT_CLOSE(q, axisAngleReference(0.5 * w), kChainTol);
    }
}

BOOST_AUTO_TEST_CASE(KeepsTheAxisAndScalesTheAngleApproximately) {
    Random rnd(43);
    for (int n = 0; n < 2000; n++) {
        Vec3 w = rnd.rotationVector(kPi / 2);
        double f = rnd.uniform(0.05, 1.0);
        Quaternion q = Quaternion::from_axis_angle(static_cast<float>(w.x), static_cast<float>(w.y),
                                                   static_cast<float>(w.z));
        Quaternion r = q;
        r.fractional(static_cast<float>(f));

        BOOST_TEST_CONTEXT(std::setprecision(9) << "rotation " << q << " fraction " << f << " result " << r) {
            BOOST_TEST(std::fabs(r.norm() - 1.0) <= kChainTol);
            // The vector part is only rescaled, so the axis is unchanged.
            BOOST_TEST(distance(normalized(vectorPart(r)), normalized(vectorPart(q))) <= kChainTol);
            // nlerp error measured at most 1.02% of theta for theta <= 90 degrees.
            double theta = length(w);
            BOOST_TEST(std::fabs(rotationAngle(toBoost(r)) - f * theta) <= 0.015 * theta + kChainTol);
        }
    }
}

BOOST_AUTO_TEST_CASE(AngleGrowsMonotonicallyWithTheFraction) {
    Quaternion q = Quaternion::from_axis_angle(0, 0, 2 * kPiF / 3);  // 120 degrees
    double previous = -1.0;
    for (int i = 0; i <= 20; i++) {
        Quaternion r = q;
        r.fractional(i / 20.0f);
        double angle = rotationAngle(toBoost(r));
        BOOST_TEST(angle >= previous);
        previous = angle;
    }
    BOOST_TEST(std::fabs(previous - 2 * kPi / 3) <= kChainTol);
}

BOOST_AUTO_TEST_CASE(ReturnsAReferenceToItself) {
    Quaternion q = Quaternion::from_axis_angle(1, 0, 0);
    Quaternion &ret = q.fractional(0.3f);
    BOOST_TEST(&ret == &q);
}

BOOST_AUTO_TEST_SUITE_END()
