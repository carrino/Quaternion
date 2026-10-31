// Rotation semantics: the axis-angle and Euler constructors, rotate() and
// rotation_between_vectors(). Checked against Boost's quaternion exponential
// (the textbook axis-angle construction) and against Rodrigues' rotation
// formula, which does not use quaternions at all and so pins down what the
// quaternions are supposed to *mean*.
//
// Conventions these tests lock in:
//   * Rotations are active and follow the right-hand rule: +90 deg about z
//     takes +x to +y.
//   * q.rotate(v) == q * v * conj(q).
//   * (p * q).rotate(v) == p.rotate(q.rotate(v)): the right-hand factor is
//     applied first.
//   * from_euler_rotation(x, y, z) == Qz(z) * Qy(y) * Qx(x): roll about x is
//     applied first, then pitch about y, then yaw about z, each about the fixed
//     frame axes (aerospace yaw-pitch-roll, Tait-Bryan ZYX).
//
// Known limitations that are deliberately not asserted here (they return NaN):
//   * rotation_between_vectors() of exactly anti-parallel vectors.
//   * normalize() of the zero quaternion.
#include "test_helpers.h"

namespace {

const Vec3 kX(1, 0, 0);
const Vec3 kY(0, 1, 0);
const Vec3 kZ(0, 0, 1);

Quaternion axisAngle(const Vec3 &v) {
    return Quaternion::from_axis_angle(static_cast<float>(v.x), static_cast<float>(v.y),
                                       static_cast<float>(v.z));
}

}  // namespace

BOOST_AUTO_TEST_SUITE(Rotation)

BOOST_AUTO_TEST_CASE(FromAxisAngleMatchesBoostExp) {
    Random rnd(10);
    for (int n = 0; n < 2000; n++) {
        Vec3 v = rnd.rotationVector(2.0 * kPi);
        CHECK_QUAT_CLOSE(axisAngle(v), axisAngleReference(v), kChainTol);
    }
    // Tiny angles must not lose the axis direction.
    for (double angle = 1e-1; angle > 1e-6; angle /= 10.0) {
        Vec3 v = asFloat(angle * normalized(Vec3(1, -2, 2)));
        CHECK_QUAT_CLOSE(axisAngle(v), axisAngleReference(v), kChainTol);
    }
    // Boost's exp handles the zero vector (its sinc is 1 there), so we must too.
    CHECK_QUAT_CLOSE(axisAngle(Vec3(0, 0, 0)), axisAngleReference(Vec3(0, 0, 0)), 0.0);
}

BOOST_AUTO_TEST_CASE(FromAxisAngleOfZeroVectorIsTheIdentity) {
    // No rotation is exactly the identity, not NaN. This matters in practice:
    // a single NaN quaternion poisons every product that follows it, so an
    // orientation estimate fed one zero gyro sample would never recover.
    CHECK_QUAT_CLOSE(Quaternion::from_axis_angle(0, 0, 0), Quaternion(), 0.0);
    CHECK_QUAT_CLOSE(Quaternion::from_axis_angle(-0.0f, -0.0f, -0.0f), Quaternion(), 0.0);

    // Components so small that their squares underflow to zero are also no
    // rotation to within float precision; they used to come out as NaN too.
    CHECK_QUAT_CLOSE(Quaternion::from_axis_angle(1e-23f, -1e-23f, 1e-23f), Quaternion(), 0.0);

    // The identity composes and rotates like one.
    Quaternion none = Quaternion::from_axis_angle(0, 0, 0);
    Quaternion q = Quaternion::from_axis_angle(0.3f, -0.4f, 1.2f);
    CHECK_QUAT_CLOSE(q * none, q, 0.0);
    CHECK_QUAT_CLOSE(none * q, q, 0.0);
    CHECK_QUAT_CLOSE(none.rotate(pure(Vec3(1.5, -2.5, 3.5))), pure(Vec3(1.5, -2.5, 3.5)), 0.0);
}

BOOST_AUTO_TEST_CASE(FromAxisAngleExplicitCases) {
    const double h = std::sqrt(0.5);  // cos(pi/4) == sin(pi/4)
    CHECK_QUAT_CLOSE(Quaternion::from_axis_angle(kPiF / 2, 0, 0), BoostQuat(h, h, 0, 0), kChainTol);
    CHECK_QUAT_CLOSE(Quaternion::from_axis_angle(0, kPiF / 2, 0), BoostQuat(h, 0, h, 0), kChainTol);
    CHECK_QUAT_CLOSE(Quaternion::from_axis_angle(0, 0, kPiF / 2), BoostQuat(h, 0, 0, h), kChainTol);
    CHECK_QUAT_CLOSE(Quaternion::from_axis_angle(-kPiF / 2, 0, 0), BoostQuat(h, -h, 0, 0), kChainTol);

    // 180 degrees about y is the pure quaternion j.
    CHECK_QUAT_CLOSE(Quaternion::from_axis_angle(0, kPiF, 0), BoostQuat(0, 0, 1, 0), kChainTol);
    // A full turn is the identity rotation, represented by -1.
    CHECK_QUAT_CLOSE(Quaternion::from_axis_angle(2 * kPiF, 0, 0), BoostQuat(-1, 0, 0, 0), kChainTol);

    // The axis is the direction of the input and the angle is its length.
    const float x = 0.3f, y = -0.4f, z = 1.2f;  // length 1.3
    double angle = std::sqrt(double(x) * x + double(y) * y + double(z) * z);
    double s = std::sin(angle / 2) / angle;
    CHECK_QUAT_CLOSE(Quaternion::from_axis_angle(x, y, z),
                     BoostQuat(std::cos(angle / 2), x * s, y * s, z * s), kChainTol);
}

BOOST_AUTO_TEST_CASE(FromAxisAngleIsUnit) {
    Random rnd(11);
    for (int n = 0; n < 2000; n++) {
        BOOST_TEST(std::fabs(axisAngle(rnd.rotationVector(2.0 * kPi)).norm() - 1.0) <= kChainTol);
    }
}

BOOST_AUTO_TEST_CASE(FromAxisAngleOfNegatedVectorIsTheConjugate) {
    Random rnd(12);
    for (int n = 0; n < 1000; n++) {
        Vec3 v = rnd.rotationVector(2.0 * kPi);
        CHECK_QUAT_CLOSE(axisAngle(-1.0 * v), axisAngle(v).conj(), kChainTol);
    }
}

BOOST_AUTO_TEST_CASE(RotateMatchesBoostSandwichProduct) {
    Random rnd(13);
    for (int n = 0; n < 2000; n++) {
        Quaternion q = rnd.unitQuaternion();
        Vec3 v = rnd.vector(10.0);
        BoostQuat expected = toBoost(q) * pureBoost(v) * conj(toBoost(q));
        CHECK_QUAT_CLOSE(q.rotate(pure(v)), expected, scaled(kChainTol, length(v)));
    }
}

BOOST_AUTO_TEST_CASE(RotateFollowsTheRightHandRule) {
    Quaternion aboutX = Quaternion::from_axis_angle(kPiF / 2, 0, 0);
    Quaternion aboutY = Quaternion::from_axis_angle(0, kPiF / 2, 0);
    Quaternion aboutZ = Quaternion::from_axis_angle(0, 0, kPiF / 2);

    CHECK_QUAT_CLOSE(aboutZ.rotate(pure(kX)), pure(kY), kChainTol);
    CHECK_QUAT_CLOSE(aboutZ.rotate(pure(kY)), pure(-1.0 * kX), kChainTol);
    CHECK_QUAT_CLOSE(aboutZ.rotate(pure(kZ)), pure(kZ), kChainTol);

    CHECK_QUAT_CLOSE(aboutX.rotate(pure(kY)), pure(kZ), kChainTol);
    CHECK_QUAT_CLOSE(aboutX.rotate(pure(kZ)), pure(-1.0 * kY), kChainTol);
    CHECK_QUAT_CLOSE(aboutX.rotate(pure(kX)), pure(kX), kChainTol);

    CHECK_QUAT_CLOSE(aboutY.rotate(pure(kZ)), pure(kX), kChainTol);
    CHECK_QUAT_CLOSE(aboutY.rotate(pure(kX)), pure(-1.0 * kZ), kChainTol);
    CHECK_QUAT_CLOSE(aboutY.rotate(pure(kY)), pure(kY), kChainTol);
}

BOOST_AUTO_TEST_CASE(RotateMatchesRodriguesFormula) {
    Random rnd(14);
    for (int n = 0; n < 2000; n++) {
        Vec3 w = rnd.rotationVector(2.0 * kPi);
        Vec3 v = rnd.vector(5.0);
        Quaternion rotated = axisAngle(w).rotate(pure(v));
        Vec3 expected = rodrigues(normalized(w), length(w), v);

        BOOST_TEST_CONTEXT("rotation vector " << w.x << "," << w.y << "," << w.z) {
            BOOST_TEST(distance(vectorPart(rotated), expected) <= scaled(kChainTol, length(v)));
            // The result of rotating a vector is still a vector.
            BOOST_TEST(std::fabs(rotated.a) <= scaled(kChainTol, length(v)));
        }
    }
}

BOOST_AUTO_TEST_CASE(RotateIsAProperRigidMotion) {
    Random rnd(15);
    for (int n = 0; n < 2000; n++) {
        Quaternion q = rnd.unitQuaternion();
        Vec3 u = rnd.vector(1.0);
        Vec3 v = rnd.vector(1.0);
        Vec3 ru = vectorPart(q.rotate(pure(u)));
        Vec3 rv = vectorPart(q.rotate(pure(v)));

        BOOST_TEST(std::fabs(length(ru) - length(u)) <= kChainTol);
        BOOST_TEST(std::fabs(dot(ru, rv) - dot(u, v)) <= kChainTol);
        // Cross products are preserved by rotations and flipped by reflections.
        BOOST_TEST(distance(vectorPart(q.rotate(pure(cross(u, v)))), cross(ru, rv)) <= kChainTol);
    }
}

BOOST_AUTO_TEST_CASE(ProductComposesRotationsRightToLeft) {
    Random rnd(16);
    for (int n = 0; n < 2000; n++) {
        Quaternion p = rnd.unitQuaternion();
        Quaternion q = rnd.unitQuaternion();
        Vec3 v = rnd.vector(1.0);
        CHECK_QUAT_CLOSE((p * q).rotate(pure(v)), p.rotate(q.rotate(pure(v))), kChainTol);
    }

    // Spelled out: rotate +x about z first (to +y), then about x (to +z).
    Quaternion aboutX = Quaternion::from_axis_angle(kPiF / 2, 0, 0);
    Quaternion aboutZ = Quaternion::from_axis_angle(0, 0, kPiF / 2);
    CHECK_QUAT_CLOSE((aboutX * aboutZ).rotate(pure(kX)), pure(kZ), kChainTol);
    // The other order rotates +x about x first (no change), then about z (to +y).
    CHECK_QUAT_CLOSE((aboutZ * aboutX).rotate(pure(kX)), pure(kY), kChainTol);
}

BOOST_AUTO_TEST_CASE(ConjugateIsTheInverseRotation) {
    Random rnd(17);
    for (int n = 0; n < 2000; n++) {
        Quaternion q = rnd.unitQuaternion();
        Vec3 v = rnd.vector(1.0);
        CHECK_QUAT_CLOSE(q.conj().rotate(q.rotate(pure(v))), pure(v), kChainTol);
        CHECK_QUAT_CLOSE(q * q.conj(), Quaternion(), kChainTol);
        CHECK_QUAT_CLOSE(q.conj() * q, Quaternion(), kChainTol);
    }
}

BOOST_AUTO_TEST_CASE(IdentityRotationIsANoOp) {
    Quaternion v(1.5f, -2.5f, 3.5f);
    CHECK_QUAT_CLOSE(Quaternion().rotate(v), v, 0.0);
}

BOOST_AUTO_TEST_CASE(FromEulerRotationIsYawPitchRollZYX) {
    Random rnd(18);
    for (int n = 0; n < 2000; n++) {
        double x = rnd.uniform(-kPi, kPi);
        double y = rnd.uniform(-kPi, kPi);
        double z = rnd.uniform(-kPi, kPi);
        Quaternion q = Quaternion::from_euler_rotation(static_cast<float>(x), static_cast<float>(y),
                                                       static_cast<float>(z));

        // Boost: exp(k z/2) * exp(j y/2) * exp(i x/2).
        CHECK_QUAT_CLOSE(q, eulerReference(x, y, z), kChainTol);

        // Rodrigues: roll about world x, then pitch about world y, then yaw
        // about world z.
        Vec3 v = rnd.vector(1.0);
        Vec3 expected = rodrigues(kZ, z, rodrigues(kY, y, rodrigues(kX, x, v)));
        BOOST_TEST_CONTEXT("euler " << x << "," << y << "," << z) {
            BOOST_TEST(distance(vectorPart(q.rotate(pure(v))), expected) <= kChainTol);
        }
    }
}

BOOST_AUTO_TEST_CASE(FromEulerRotationExplicitCases) {
    // No rotation is exactly the identity.
    CHECK_QUAT_CLOSE(Quaternion::from_euler_rotation(0, 0, 0), Quaternion(), 0.0);

    const double h = std::sqrt(0.5);
    CHECK_QUAT_CLOSE(Quaternion::from_euler_rotation(kPiF / 2, 0, 0), BoostQuat(h, h, 0, 0), kChainTol);
    CHECK_QUAT_CLOSE(Quaternion::from_euler_rotation(0, kPiF / 2, 0), BoostQuat(h, 0, h, 0), kChainTol);
    CHECK_QUAT_CLOSE(Quaternion::from_euler_rotation(0, 0, kPiF / 2), BoostQuat(h, 0, 0, h), kChainTol);
    CHECK_QUAT_CLOSE(Quaternion::from_euler_rotation(-kPiF / 2, 0, 0), BoostQuat(h, -h, 0, 0), kChainTol);

    // Gimbal-lock configuration: 90 degrees on every axis collapses to a pure
    // 90 degree rotation about y.
    CHECK_QUAT_CLOSE(Quaternion::from_euler_rotation(kPiF / 2, kPiF / 2, kPiF / 2),
                     BoostQuat(h, 0, h, 0), kChainTol);

    // Single-axis Euler rotations are axis-angle rotations.
    Random rnd(19);
    for (int n = 0; n < 500; n++) {
        float t = static_cast<float>(rnd.uniform(-kPi, kPi));
        CHECK_QUAT_CLOSE(Quaternion::from_euler_rotation(t, 0, 0), Quaternion::from_axis_angle(t, 0, 0), kChainTol);
        CHECK_QUAT_CLOSE(Quaternion::from_euler_rotation(0, t, 0), Quaternion::from_axis_angle(0, t, 0), kChainTol);
        CHECK_QUAT_CLOSE(Quaternion::from_euler_rotation(0, 0, t), Quaternion::from_axis_angle(0, 0, t), kChainTol);
    }
}

BOOST_AUTO_TEST_CASE(FromEulerRotationIsUnit) {
    Random rnd(20);
    for (int n = 0; n < 2000; n++) {
        Quaternion q = Quaternion::from_euler_rotation(static_cast<float>(rnd.uniform(-kPi, kPi)),
                                                       static_cast<float>(rnd.uniform(-kPi, kPi)),
                                                       static_cast<float>(rnd.uniform(-kPi, kPi)));
        BOOST_TEST(std::fabs(q.norm() - 1.0) <= kChainTol);
    }
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(RotationBetweenVectors)

namespace {

// Two unit vectors no more than maxAngle apart. The library loses precision as
// the vectors approach anti-parallel (1 + u.v cancels), and is undefined at
// exactly 180 degrees.
void randomPair(Random &rnd, double maxAngle, Vec3 &u, Vec3 &v) {
    for (;;) {
        u = rnd.unitVector();
        v = rnd.unitVector();
        if (dot(u, v) >= std::cos(maxAngle)) {
            return;
        }
    }
}

}  // namespace

BOOST_AUTO_TEST_CASE(RotatesTheFirstVectorOntoTheSecond) {
    Random rnd(21);
    for (int n = 0; n < 2000; n++) {
        Vec3 u(0, 0, 0), v(0, 0, 0);
        randomPair(rnd, 160.0 * kPi / 180.0, u, v);
        Quaternion q = pure(u).rotation_between_vectors(pure(v));

        BOOST_TEST(std::fabs(q.norm() - 1.0) <= kChainTol);
        CHECK_QUAT_CLOSE(q.rotate(pure(u)), pure(v), kChainTol);
    }
}

BOOST_AUTO_TEST_CASE(IsTheShortestArcAboutTheCrossProduct) {
    Random rnd(22);
    for (int n = 0; n < 2000; n++) {
        Vec3 u(0, 0, 0), v(0, 0, 0);
        randomPair(rnd, 160.0 * kPi / 180.0, u, v);
        Quaternion q = pure(u).rotation_between_vectors(pure(v));

        double angle = std::acos(std::max(-1.0, std::min(1.0, dot(u, v))));
        Vec3 axis = normalized(cross(u, v));
        BoostQuat expected(std::cos(angle / 2), 0, 0, 0);
        expected += pureBoost(std::sin(angle / 2) * axis);
        // Same sign as well: the scalar part is positive for the short arc.
        CHECK_QUAT_CLOSE(q, expected, kChainTol);
    }
}

BOOST_AUTO_TEST_CASE(ExplicitAxisCases) {
    const double h = std::sqrt(0.5);
    // x -> y is +90 degrees about z, and so on around the right-handed frame.
    CHECK_QUAT_CLOSE(pure(kX).rotation_between_vectors(pure(kY)), BoostQuat(h, 0, 0, h), kChainTol);
    CHECK_QUAT_CLOSE(pure(kY).rotation_between_vectors(pure(kZ)), BoostQuat(h, h, 0, 0), kChainTol);
    CHECK_QUAT_CLOSE(pure(kZ).rotation_between_vectors(pure(kX)), BoostQuat(h, 0, h, 0), kChainTol);
    // Reversing the arguments inverts the rotation.
    CHECK_QUAT_CLOSE(pure(kY).rotation_between_vectors(pure(kX)), BoostQuat(h, 0, 0, -h), kChainTol);
}

BOOST_AUTO_TEST_CASE(ParallelVectorsGiveTheIdentity) {
    Random rnd(23);
    for (int n = 0; n < 500; n++) {
        Vec3 u = rnd.unitVector();
        CHECK_QUAT_CLOSE(pure(u).rotation_between_vectors(pure(u)), Quaternion(), kChainTol);
    }
}

BOOST_AUTO_TEST_SUITE_END()
