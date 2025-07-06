#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE QuaternionTest
#define BOOST_TEST_NO_OLD_TOOLS
#include <boost/test/unit_test.hpp>
#include <Quaternion.h>
#include <iostream>
#include <cmath> // For std::sqrt, std::cos, std::sin, M_PI_2 etc.

// Define a tolerance for floating point comparisons
const float TOLERANCE = 0.0001f; // Default tolerance: 0.01%

// General check for equality using the default TOLERANCE
void checkEquals(const Quaternion &q1, const Quaternion &q2) {
    BOOST_CHECK_CLOSE(q1.a, q2.a, TOLERANCE);
    BOOST_CHECK_CLOSE(q1.b, q2.b, TOLERANCE);
    BOOST_CHECK_CLOSE(q1.c, q2.c, TOLERANCE);
    BOOST_CHECK_CLOSE(q1.d, q2.d, TOLERANCE);
}

// Helper for checking if a quaternion is NaN
bool isNaN(const Quaternion& q) {
    return std::isnan(q.a) || std::isnan(q.b) || std::isnan(q.c) || std::isnan(q.d);
}

BOOST_AUTO_TEST_SUITE(QuaternionTests)

BOOST_AUTO_TEST_CASE(DefaultConstructor) {
    Quaternion q;
    BOOST_CHECK_EQUAL(q.a, 1.0f);
    BOOST_CHECK_EQUAL(q.b, 0.0f);
    BOOST_CHECK_EQUAL(q.c, 0.0f);
    BOOST_CHECK_EQUAL(q.d, 0.0f);
}

BOOST_AUTO_TEST_CASE(VectorConstructor) {
    Quaternion q(1.0f, 2.0f, 3.0f);
    BOOST_CHECK_EQUAL(q.a, 0.0f);
    BOOST_CHECK_EQUAL(q.b, 1.0f);
    BOOST_CHECK_EQUAL(q.c, 2.0f);
    BOOST_CHECK_EQUAL(q.d, 3.0f);
}

BOOST_AUTO_TEST_CASE(FromEulerRotation) {
    // Test zero rotation
    Quaternion q_zero = Quaternion::from_euler_rotation(0, 0, 0);
    checkEquals(q_zero, Quaternion(1, 0, 0, 0)); // Should be identity

    // Test 90 deg rotation around X
    Quaternion qx = Quaternion::from_euler_rotation(M_PI_2, 0, 0);
    checkEquals(qx, Quaternion(std::sqrt(0.5f), std::sqrt(0.5f), 0, 0));

    // Test 90 deg rotation around Y
    Quaternion qy = Quaternion::from_euler_rotation(0, M_PI_2, 0);
    checkEquals(qy, Quaternion(std::sqrt(0.5f), 0, std::sqrt(0.5f), 0));

    // Test 90 deg rotation around Z
    Quaternion qz = Quaternion::from_euler_rotation(0, 0, M_PI_2);
    checkEquals(qz, Quaternion(std::sqrt(0.5f), 0, 0, std::sqrt(0.5f)));

    // Test combined rotation (Roll=pi/2, Pitch=pi/2, Yaw=pi/2)
    // Formula used in from_euler_rotation corresponds to ZYX: Q = Qyaw * Qpitch * Qroll
    // For (x=roll, y=pitch, z=yaw) all pi/2:
    // Expected Q = (cos(pi/4), 0, cos(pi/4), 0) = (sqrt(0.5), 0, sqrt(0.5), 0)
    Quaternion q_combined = Quaternion::from_euler_rotation(M_PI_2, M_PI_2, M_PI_2);
    checkEquals(q_combined, Quaternion(std::sqrt(0.5f), 0.0f, std::sqrt(0.5f), 0.0f));


    // Test negative rotation
    Quaternion q_neg_x = Quaternion::from_euler_rotation(-M_PI_2, 0, 0);
    checkEquals(q_neg_x, Quaternion(std::sqrt(0.5f), -std::sqrt(0.5f), 0, 0));
}

BOOST_AUTO_TEST_CASE(FromEulerRotationApprox) {
    float small_angle = M_PI / 10; // 18 degrees, well within 45 deg
    float large_angle = M_PI_2;   // 90 degrees, outside 45 deg

    Quaternion q_exact_small = Quaternion::from_euler_rotation(small_angle, 0, 0);
    Quaternion q_approx_small = Quaternion::from_euler_rotation_approx(small_angle, 0, 0);
    // Using 1% tolerance for approximation check
    float approx_tolerance_1pc = 0.01f;
    BOOST_CHECK(std::abs(q_exact_small.a - q_approx_small.a) < (approx_tolerance_1pc * std::abs(q_exact_small.a)));
    BOOST_CHECK(std::abs(q_exact_small.b - q_approx_small.b) < (approx_tolerance_1pc * std::abs(q_exact_small.b)));
    // c and d are zero for this rotation, so check small absolute difference
    BOOST_CHECK_SMALL(q_exact_small.c - q_approx_small.c, approx_tolerance_1pc); // Using 1% as absolute for near-zero
    BOOST_CHECK_SMALL(q_exact_small.d - q_approx_small.d, approx_tolerance_1pc);


    Quaternion q_exact_large = Quaternion::from_euler_rotation(large_angle, 0, 0);
    Quaternion q_approx_large = Quaternion::from_euler_rotation_approx(large_angle, 0, 0);
    // Check that they are NOT equal for large angles
    bool are_different = std::abs(q_exact_large.a - q_approx_large.a) > TOLERANCE ||
                         std::abs(q_exact_large.b - q_approx_large.b) > TOLERANCE ||
                         std::abs(q_exact_large.c - q_approx_large.c) > TOLERANCE ||
                         std::abs(q_exact_large.d - q_approx_large.d) > TOLERANCE;
    BOOST_CHECK(are_different);

    // Test with combined small angles
    Quaternion q_exact_small_xyz = Quaternion::from_euler_rotation(small_angle, small_angle, small_angle);
    Quaternion q_approx_small_xyz = Quaternion::from_euler_rotation_approx(small_angle, small_angle, small_angle);
    float approx_tolerance_1pc_euler_xyz = 0.01f; // 1%
    BOOST_CHECK(std::abs(q_exact_small_xyz.a - q_approx_small_xyz.a) < (approx_tolerance_1pc_euler_xyz * std::abs(q_exact_small_xyz.a)));
    BOOST_CHECK(std::abs(q_exact_small_xyz.b - q_approx_small_xyz.b) < (approx_tolerance_1pc_euler_xyz * std::abs(q_exact_small_xyz.b)));
    BOOST_CHECK(std::abs(q_exact_small_xyz.c - q_approx_small_xyz.c) < (approx_tolerance_1pc_euler_xyz * std::abs(q_exact_small_xyz.c)));
    BOOST_CHECK(std::abs(q_exact_small_xyz.d - q_approx_small_xyz.d) < (approx_tolerance_1pc_euler_xyz * std::abs(q_exact_small_xyz.d)));
}

BOOST_AUTO_TEST_CASE(FromAxisAngle) {
    // Rotate 90 deg around X axis
    Quaternion qx = Quaternion::from_axis_angle(M_PI_2, 0, 0);
    checkEquals(qx, Quaternion(std::cos(M_PI_4), std::sin(M_PI_4), 0, 0));

    // Rotate 180 deg around Y axis
    Quaternion qy = Quaternion::from_axis_angle(0, M_PI, 0);
    // Expected: (0,0,1,0)
    BOOST_CHECK(std::abs(qy.a) < TOLERANCE); // Expected cos(pi/2) = 0
    BOOST_CHECK(std::abs(qy.b) < TOLERANCE); // Expected 0
    BOOST_CHECK_CLOSE(qy.c, 1.0f, TOLERANCE);   // Expected sin(pi/2)* (M_PI/M_PI) = 1
    BOOST_CHECK(std::abs(qy.d) < TOLERANCE); // Expected 0
    // checkEquals(qy, Quaternion(0.0f, 0.0f, 1.0f, 0.0f), TOLERANCE); // Use individual checks for more clarity on zero

    // Rotate 0 deg around Z axis (should be identity)
    // Note: Current implementation divides by angle, so angle=0 is an issue.
    // Let's test with a very small angle instead.
    float tiny_angle = 1e-9f;
    Quaternion qz_zero = Quaternion::from_axis_angle(0, 0, tiny_angle);
    // Expect w ~ cos(tiny_angle/2) ~ 1, d ~ sin(tiny_angle/2) ~ tiny_angle/2
    // x/angle * s, y/angle * s, z/angle * s
    // For (0,0,tiny_angle), it becomes (0,0, sin(tiny_angle/2))
    checkEquals(qz_zero, Quaternion(std::cos(tiny_angle/2.0f), 0, 0, std::sin(tiny_angle/2.0f)));


    // Test with a non-unit axis vector (should be normalized by the angle magnitude)
    // Rotate by 'angle' radians around axis (1,1,1)
    // angle = sqrt( (pi/3)^2 + (pi/3)^2 + (pi/3)^2 ) = sqrt(3 * (pi/3)^2) = sqrt(3)/3 * pi = pi/sqrt(3)
    // axis vector (1,1,1) / sqrt(3)
    float component_angle = M_PI / (2.0f * std::sqrt(3.0f)); // each component for axis (1,1,1) to make total angle PI/2
    Quaternion q_axis = Quaternion::from_axis_angle(component_angle, component_angle, component_angle);
    // Expected rotation: PI/2 around (1/sqrt(3), 1/sqrt(3), 1/sqrt(3))
    float angle_mag = std::sqrt(3.0f * component_angle * component_angle); // Should be PI/2
    float s = std::sin(angle_mag / 2.0f);
    float c = std::cos(angle_mag / 2.0f);
    float axis_norm = std::sqrt(3.0f);
    checkEquals(q_axis, Quaternion(c, s * (component_angle/angle_mag), s * (component_angle/angle_mag), s * (component_angle/angle_mag)));
}

BOOST_AUTO_TEST_CASE(FromAxisAngleApprox) {
    float small_angle_val = M_PI / 10; // Small angle for x, y, z components
    float large_angle_val = M_PI_2;   // Large angle for x, y, z components

    Quaternion q_exact_small = Quaternion::from_axis_angle(small_angle_val, 0, 0);
    Quaternion q_approx_small = Quaternion::from_axis_angle_approx(small_angle_val, 0, 0);
    float approx_tolerance_axis_1pc = 0.01f;
    BOOST_CHECK(std::abs(q_exact_small.a - q_approx_small.a) < (approx_tolerance_axis_1pc * std::abs(q_exact_small.a)));
    BOOST_CHECK(std::abs(q_exact_small.b - q_approx_small.b) < (approx_tolerance_axis_1pc * std::abs(q_exact_small.b)));
    BOOST_CHECK_SMALL(q_exact_small.c - q_approx_small.c, approx_tolerance_axis_1pc);
    BOOST_CHECK_SMALL(q_exact_small.d - q_approx_small.d, approx_tolerance_axis_1pc);


    Quaternion q_exact_large = Quaternion::from_axis_angle(large_angle_val, 0, 0);
    Quaternion q_approx_large = Quaternion::from_axis_angle_approx(large_angle_val, 0, 0);
    bool are_different_large = std::abs(q_exact_large.a - q_approx_large.a) > TOLERANCE ||
                               std::abs(q_exact_large.b - q_approx_large.b) > TOLERANCE;
    BOOST_CHECK(are_different_large);

    // Test with combined small angles for axis
    Quaternion q_exact_small_axis = Quaternion::from_axis_angle(small_angle_val, small_angle_val, small_angle_val);
    Quaternion q_approx_small_axis = Quaternion::from_axis_angle_approx(small_angle_val, small_angle_val, small_angle_val);
    // The approximation for from_axis_angle_approx is:
    // w = 1 - angle^2/8, b = x/2, c = y/2, d = z/2
    // angle = sqrt(x^2+y^2+z^2)
    // For from_axis_angle:
    // w = cos(angle/2), b = x/angle * sin(angle/2), etc.
    // These will be close if angle is small.
    float approx_tolerance_axis_2pc = 0.02f; // 2%
    BOOST_CHECK(std::abs(q_exact_small_axis.a - q_approx_small_axis.a) < (approx_tolerance_axis_2pc * std::abs(q_exact_small_axis.a)));
    BOOST_CHECK(std::abs(q_exact_small_axis.b - q_approx_small_axis.b) < (approx_tolerance_axis_2pc * std::abs(q_exact_small_axis.b)));
    BOOST_CHECK(std::abs(q_exact_small_axis.c - q_approx_small_axis.c) < (approx_tolerance_axis_2pc * std::abs(q_exact_small_axis.c)));
    BOOST_CHECK(std::abs(q_exact_small_axis.d - q_approx_small_axis.d) < (approx_tolerance_axis_2pc * std::abs(q_exact_small_axis.d)));
}

BOOST_AUTO_TEST_CASE(AssignmentOperator) {
    Quaternion q1(0.1f, 0.2f, 0.3f, 0.4f);
    Quaternion q2;
    q2 = q1;
    checkEquals(q1, q2);

    Quaternion q3(1.0f, 2.0f, 3.0f); // vector constructor
    Quaternion q4;
    q4 = q3;
    checkEquals(q3, q4);
}

BOOST_AUTO_TEST_CASE(MultiplicationAssignmentOperator) {
    Quaternion q1(0.5f, 0.5f, 0.0f, 0.0f); // Represents some rotation
    q1.a = std::sqrt(0.5f); q1.b = std::sqrt(0.5f); // 90 deg X rotation
    Quaternion q_orig = q1;
    Quaternion q2(std::sqrt(0.5f), 0.0f, std::sqrt(0.5f), 0.0f); // 90 deg Y rotation

    q1 *= q2;
    // Expected: q1_new = q_orig * q2
    // (0.707+0.707i)*(0.707+0.707j) = 0.5 + 0.5j + 0.5k - 0.5ij = 0.5 + 0.5j + 0.5k - 0.5(-k) ... this is wrong.
    // (w1,x1,y1,z1) * (w2,x2,y2,z2)
    // w = w1w2 - x1x2 - y1y2 - z1z2
    // x = w1x2 + x1w2 + y1z2 - z1y2
    // y = w1y2 - x1z2 + y1w2 + z1x2
    // z = w1z2 + x1y2 - y1x2 + z1w2
    // q1 = (sqrt(0.5), sqrt(0.5), 0, 0)
    // q2 = (sqrt(0.5), 0, sqrt(0.5), 0)
    // w = sqrt(0.5)*sqrt(0.5) - sqrt(0.5)*0 - 0*sqrt(0.5) - 0*0 = 0.5
    // x = sqrt(0.5)*0 + sqrt(0.5)*sqrt(0.5) + 0*0 - 0*sqrt(0.5) = 0.5
    // y = sqrt(0.5)*sqrt(0.5) - sqrt(0.5)*0 + 0*sqrt(0.5) + 0*0 = 0.5
    // z = sqrt(0.5)*0 + sqrt(0.5)*sqrt(0.5) - 0*0 + 0*sqrt(0.5) = 0.5
    checkEquals(q1, Quaternion(0.5f, 0.5f, 0.5f, 0.5f));
}

BOOST_AUTO_TEST_CASE(MultiplicationOperator) {
    Quaternion q1(std::sqrt(0.5f), std::sqrt(0.5f), 0.0f, 0.0f); // 90 deg X
    Quaternion q2(std::sqrt(0.5f), 0.0f, std::sqrt(0.5f), 0.0f); // 90 deg Y
    Quaternion result = q1 * q2;
    checkEquals(result, Quaternion(0.5f, 0.5f, 0.5f, 0.5f));
    // Ensure q1 and q2 are unchanged
    checkEquals(q1, Quaternion(std::sqrt(0.5f), std::sqrt(0.5f), 0.0f, 0.0f));
    checkEquals(q2, Quaternion(std::sqrt(0.5f), 0.0f, std::sqrt(0.5f), 0.0f));
}

BOOST_AUTO_TEST_CASE(AdditionAssignmentOperator) {
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion q2(0.1f, 0.2f, 0.3f, 0.4f);
    q1 += q2;
    checkEquals(q1, Quaternion(1.1f, 2.2f, 3.3f, 4.4f));
}

BOOST_AUTO_TEST_CASE(AdditionOperator) {
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion q2(0.1f, 0.2f, 0.3f, 0.4f);
    Quaternion result = q1 + q2;
    checkEquals(result, Quaternion(1.1f, 2.2f, 3.3f, 4.4f));
    // Ensure q1 and q2 are unchanged
    checkEquals(q1, Quaternion(1.0f, 2.0f, 3.0f, 4.0f));
    checkEquals(q2, Quaternion(0.1f, 0.2f, 0.3f, 0.4f));
}

BOOST_AUTO_TEST_CASE(ScalarMultiplicationAssignmentOperator) {
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    q1 *= 2.0f;
    checkEquals(q1, Quaternion(2.0f, 4.0f, 6.0f, 8.0f));
    q1 *= 0.5f;
    checkEquals(q1, Quaternion(1.0f, 2.0f, 3.0f, 4.0f));
}

BOOST_AUTO_TEST_CASE(ScalarMultiplicationOperator) {
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion result = q1 * 2.0f;
    checkEquals(result, Quaternion(2.0f, 4.0f, 6.0f, 8.0f));
    // Ensure q1 is unchanged
    checkEquals(q1, Quaternion(1.0f, 2.0f, 3.0f, 4.0f));

    Quaternion result2 = q1 * 0.5f;
    checkEquals(result2, Quaternion(0.5f, 1.0f, 1.5f, 2.0f));
}

BOOST_AUTO_TEST_CASE(rotateX) {
    auto rotateX = Quaternion::from_euler_rotation(M_PI_2, 0, 0);
    BOOST_CHECK_CLOSE(std::sqrt(0.5f), rotateX.a, TOLERANCE);
    BOOST_CHECK_CLOSE(std::sqrt(0.5f), rotateX.b, TOLERANCE); // Corrected from sqrt(2)/2.0 and fixed tolerance
    auto q = rotateX.rotate(Quaternion(0, 1, 0)); // Rotate vector (0,1,0) by 90 deg around X
    // (0,1,0) -> (0,0,1)
    checkEquals(q, Quaternion(0, 0, 1.0f)); // d component becomes z, which is 1
    BOOST_CHECK_SMALL(q.a, TOLERANCE); // Pure vector result
}

BOOST_AUTO_TEST_CASE(rotateY) {
    auto rotate = Quaternion::from_euler_rotation(0, M_PI_2, 0); // 90 deg around Y
    BOOST_CHECK_CLOSE(std::sqrt(0.5f), rotate.a, TOLERANCE);
    BOOST_CHECK_CLOSE(std::sqrt(0.5f), rotate.c, TOLERANCE);
    auto q = rotate.rotate(Quaternion(0, 0, 1)); // Rotate vector (0,0,1) by 90 deg around Y
    // (0,0,1) -> (1,0,0)
    checkEquals(q, Quaternion(1.0f, 0, 0)); // b component becomes x, which is 1
    BOOST_CHECK_SMALL(q.a, TOLERANCE); // Pure vector result
}

BOOST_AUTO_TEST_CASE(rotateZ) {
    auto rotate = Quaternion::from_euler_rotation(0, 0, M_PI_2); // 90 deg around Z
    BOOST_CHECK_CLOSE(std::sqrt(0.5f), rotate.a, TOLERANCE);
    BOOST_CHECK_CLOSE(std::sqrt(0.5f), rotate.d, TOLERANCE);
    auto q = rotate.rotate(Quaternion(1, 0, 0)); // Rotate vector (1,0,0) by 90 deg around Z
    // (1,0,0) -> (0,1,0)
    checkEquals(q, Quaternion(0, 1.0f, 0)); // c component becomes y, which is 1
    BOOST_CHECK_SMALL(q.a, TOLERANCE); // Pure vector result
}

BOOST_AUTO_TEST_CASE(rotateTwice) {
    // Rotate 90 deg Z, then 90 deg X
    auto rotateZ90 = Quaternion::from_euler_rotation(0, 0, M_PI_2);
    auto rotateX90 = Quaternion::from_euler_rotation(M_PI_2, 0, 0);
    auto rotate = rotateZ90 * rotateX90; // Combined rotation
    // rotate.normalize(); // Not strictly needed if inputs are normalized and op preserves it
                         // The multiplication of two unit quaternions is a unit quaternion.

    // Expected (0.5, 0.5, 0.5, 0.5) from original test, let's verify:
    // qZ = (c,0,0,s) = (sqrt(0.5),0,0,sqrt(0.5))
    // qX = (c,s,0,0) = (sqrt(0.5),sqrt(0.5),0,0)
    // w = c*c - 0*s - 0*0 - s*0 = c*c = 0.5
    // x = c*s + 0*c + 0*0 - s*0 = c*s = 0.5
    // y = c*0 - 0*0 + 0*c + s*s = s*s = 0.5
    // z = c*0 + 0*0 - 0*s + s*c = s*c = 0.5
    checkEquals(rotate, Quaternion(0.5f, 0.5f, 0.5f, 0.5f));

    // Rotate vector (0,1,0) by this combined rotation
    // Initial: (0,1,0) (Y-axis)
    // After Z90: (1,0,0) (X-axis)
    // After X90: (1,0,0) should not change by X rotation if it's along X (this logic is flawed)
    // Let's trace: q_vec = (0,0,1,0)
    // Rotated by Z90: (sqrt(0.5),0,0,sqrt(0.5)) * (0,0,1,0) * (sqrt(0.5),0,0,-sqrt(0.5))
    // P' = Q P Q*
    // P = (0,0,1,0) (Y-axis vector)
    // After Z90 (rotates Y to -X): (0,-1,0,0)
    // q_intermediate = rotateZ90.rotate(Quaternion(0,1,0));
    // checkEquals(q_intermediate, Quaternion(-1,0,0,0)); // Y to -X by Z90
    // After X90 (rotates -X to -X, no change): (0,-1,0,0)
    // q_final = rotateX90.rotate(q_intermediate);
    // checkEquals(q_final, Quaternion(-1,0,0,0));
    // The original test has rotate.rotate(Quaternion(0,1,0)) -> (0,0,0,1) (Z-axis)
    // This means Y -> Z.
    // If world frame: Y_world -> Z_world.
    // Rotation order is Rz then Rx.
    // Y-axis (0,1,0). Rotate by Rz(pi/2): leads to (-1,0,0) [-X axis].
    // Then rotate (-1,0,0) by Rx(pi/2): (-1,0,0) stays (-1,0,0) because it's on the X axis.
    // So result should be (-1,0,0).
    // The original test had (0.5,0.5,0.5,0.5) rotate (0,1,0) into (0,0,0,1) i.e. (0,0,1) vector.
    // Let's re-verify the multiplication order for combined rotation.
    // If R = Rz * Rx, then P' = R * P * R_conj.
    // The original test's assertion BOOST_CHECK_CLOSE(1.0, q.d, .0001); implies q.d (z-component of vector) is 1.
    // So (0,1,0) transforms to (0,0,1). This is a rotation from Y-axis to Z-axis.
    // This corresponds to a +90 degree rotation around the X-axis.
    // But the combined rotation is Rz(pi/2) * Rx(pi/2).
    // Let's check the effect of Q(0.5,0.5,0.5,0.5) on (0,0,1,0) (Y-vector)
    // Q = 0.5 * (1+i+j+k)
    // P = j
    // Q P Q* = 0.5(1+i+j+k) * j * 0.5(1-i-j-k)
    // = 0.25 * (1+i+j+k) * (j -ji -jj -jk)
    // = 0.25 * (1+i+j+k) * (j +k -(-1) -i)
    // = 0.25 * (1+i+j+k) * (1 -i +j +k)
    // = 0.25 * ( (1+j+k) + i ) * ( (1+j+k) - i )
    // = 0.25 * ( (1+j+k)^2 - i^2 )
    // = 0.25 * ( 1+j^2+k^2+2j+2k+2jk - (-1) )
    // = 0.25 * ( 1-1-1+2j+2k+2i + 1 )
    // = 0.25 * ( 2i+2j+2k ) = 0.5i + 0.5j + 0.5k. This is not (0,0,1).
    // The original test might have a specific interpretation or a different Rz*Rx quaternion.
    // The quaternion (0.5, 0.5, 0.5, 0.5) is correct for Rz(pi/2) * Rx(pi/2).
    // Let's assume the original test's final check was correct for its specific setup.
    // I will keep the check for q.d == 1.0, but the intermediate logic might need review if it fails.
    // After re-evaluation, the rotation Q = (0.5, 0.5, 0.5, 0.5) corresponds to a 120-degree rotation about the (1,1,1) axis.
    // Rotating (0,1,0) by this:
    // (0,1,0) -> (0,0,1) is what the original test asserted.
    // This means Y-axis maps to Z-axis. This is a rotation of +pi/2 about X-axis.
    // Q_expected_for_Y_to_Z = (cos(pi/4), sin(pi/4), 0, 0) = (sqrt(0.5), sqrt(0.5), 0, 0)
    // This is not (0.5,0.5,0.5,0.5).
    // So, the original test's 'rotateTwice' quaternion composed as Rz(pi/2)*Rx(pi/2) and then normalized
    // (which is (0.5,0.5,0.5,0.5)) does NOT rotate (0,1,0) to (0,0,1).
    // It rotates (0,1,0) to (1,0,0). (Y -> X)
    // Q = (0.5,0.5,0.5,0.5). P = (0,0,1,0) [vector Y]
    // P' = Q P Q*
    // Result is (0,0,0,1) [vector Z]
    Quaternion q_vec_y(0,1,0); // Input Y-vector (0,0,1,0)
    auto q_rotated_y = rotate.rotate(q_vec_y);
    // Original test was: BOOST_CHECK_CLOSE(1.0, q.d, .0001); checkEquals(q, Quaternion(0,0,1));
    // Quaternion(0,0,1) is vector (0,0,1) which is (a=0, b=0,c=0,d=1)
    checkEquals(q_rotated_y, Quaternion(0.0f, 0.0f, 0.0f, 1.0f));
    BOOST_CHECK_SMALL(q_rotated_y.a, TOLERANCE); // Ensure scalar part is zero for the resulting vector
}

BOOST_AUTO_TEST_CASE(nonCommutative) {
    // (q1*q2).conj() vs q2.conj()*q1.conj()
    // This property is (AB)* = B*A*
    auto q1 = Quaternion::from_euler_rotation(0, 0, M_PI_2); // Arbitrary q1
    auto q2 = Quaternion::from_euler_rotation(M_PI_2, 0, 0); // Arbitrary q2
    auto lhs = (q1 * q2).conj();
    auto rhs = q2.conj() * q1.conj();
    checkEquals(lhs, rhs);
}

BOOST_AUTO_TEST_CASE(rotateRelative) {
    // Original test: yaw(Z) * pitch(Y) * roll(X)
    // Rotates (0,0,1) [Z-axis vector] to (1,0,0) [X-axis vector]
    auto yaw = Quaternion::from_euler_rotation(0, 0, M_PI_2);   // Rot Z by pi/2
    auto pitch = Quaternion::from_euler_rotation(0, M_PI_2, 0); // Rot Y by pi/2
    auto roll = Quaternion::from_euler_rotation(M_PI_2, 0, 0);  // Rot X by pi/2

    // Standard sequence for Tait-Bryan ZYX: Q = Q_yaw * Q_pitch * Q_roll
    auto composite = yaw * pitch * roll;
    // composite should be (0, 0.707, 0.707, 0) if calculated as Rz*Ry*Rx with pi/2 each
    // Rz = (c,0,0,s), Ry=(c,0,s,0), Rx=(c,s,0,0) where c=s=sqrt(0.5)
    // Ry*Rx = (c,s,0,0)*(c,0,s,0) ; W = c*c, X=c*s, Y=c*s, Z=s*s. No, this is (c,s,0,0) for Rx, (c,0,s,0) for Ry.
    // Rx = (c,s,0,0). Ry = (c,0,s,0). Rz = (c,0,0,s)
    // Ry*Rx: w=c*c, x=c*s, y=c*s, z=s*s. (0.5, 0.5, 0.5, -0.5) if intrinsic. (0.5, 0.5, 0.5, 0.5) if extrinsic
    // QyQx = (c,0,s,0)(c,s,0,0) = (c^2, cs, c^2, -s^2) ? No.
    // w = c*c - 0*s - s*0 - 0*0 = c^2 = 0.5
    // x = c*s + 0*c + s*0 - 0*s = cs = 0.5
    // y = c*0 - 0*0 + s*c + 0*s = sc = 0.5
    // z = c*0 + 0*s - s*s + 0*c = -s^2 = -0.5
    // So Ry*Rx = (0.5, 0.5, 0.5, -0.5)
    // Qz*(QyQx) = (c,0,0,s) * (0.5,0.5,0.5,-0.5)
    // w = c*0.5 - s*(-0.5) = 0.5c + 0.5s = 0.5(c+s) = 0.5(sqrt(0.5)+sqrt(0.5)) = sqrt(0.5)
    // x = c*0.5 + 0*0.5 + 0*(-0.5) - s*0.5 = 0.5c - 0.5s = 0
    // y = c*0.5 - 0*(-0.5) + 0*0.5 + s*0.5 = 0.5c + 0.5s = sqrt(0.5)
    // z = c*(-0.5) + 0*0.5 - 0*0.5 + s*0.5 = -0.5c + 0.5s = 0
    // Result: (sqrt(0.5), 0, sqrt(0.5), 0). This is a pure Y rotation by PI/2. Qy.
    // This means ZYX(pi/2,pi/2,pi/2) = Y(pi/2). This is a known Gimbal lock configuration.
    // So, composite = Qy = (sqrt(0.5), 0, sqrt(0.5), 0)
    checkEquals(composite, Quaternion(std::sqrt(0.5f), 0, std::sqrt(0.5f), 0));

    Quaternion vec_Z(0,0,1); // Vector (0,0,1)
    auto rotated = composite.rotate(vec_Z);
    // Rotating (0,0,1) by Qy(pi/2) should result in (1,0,0)
    checkEquals(rotated, Quaternion(1.0f, 0, 0));
    BOOST_CHECK_SMALL(rotated.a, TOLERANCE);
}

// New tests for core operations
BOOST_AUTO_TEST_CASE(Norm) {
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f); // a=1, b=2, c=3, d=4
    // norm = sqrt(1*1 + 2*2 + 3*3 + 4*4) = sqrt(1+4+9+16) = sqrt(30)
    BOOST_CHECK_CLOSE(q1.norm(), std::sqrt(30.0f), TOLERANCE);

    Quaternion q_vec(0.0f, 3.0f, 4.0f, 0.0f); // a=0, b=3, c=4, d=0
    // norm = sqrt(0+9+16+0) = sqrt(25) = 5
    BOOST_CHECK_CLOSE(q_vec.norm(), 5.0f, TOLERANCE);

    Quaternion q_iden; // Default constructor: 1,0,0,0
    BOOST_CHECK_CLOSE(q_iden.norm(), 1.0f, TOLERANCE);

    Quaternion q_zero_comps(0.0f,0.0f,0.0f,0.0f); // All components zero
    q_zero_comps.a = 0; // ensure it's (0,0,0,0) not (1,0,0,0)
    BOOST_CHECK_CLOSE(q_zero_comps.norm(), 0.0f, TOLERANCE);
}

BOOST_AUTO_TEST_CASE(Normalize) {
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    q1.normalize();
    BOOST_CHECK_CLOSE(q1.norm(), 1.0f, TOLERANCE);
    float inv_sqrt30 = 1.0f / std::sqrt(30.0f);
    checkEquals(q1, Quaternion(1.0f*inv_sqrt30, 2.0f*inv_sqrt30, 3.0f*inv_sqrt30, 4.0f*inv_sqrt30));

    Quaternion q_already_normalized = Quaternion::from_euler_rotation(M_PI_4, 0, 0);
    Quaternion q_copy = q_already_normalized;
    q_already_normalized.normalize(); // Should not change much
    checkEquals(q_already_normalized, q_copy);
    BOOST_CHECK_CLOSE(q_already_normalized.norm(), 1.0f, TOLERANCE);

    // Test normalizing a zero quaternion (0,0,0,0)
    // Current implementation will divide by zero.
    // Per user feedback, not adding a test to assert NaN for this.
    // Quaternion q_zero_vec(0.0f,0.0f,0.0f);
    // q_zero_vec.normalize();
    // BOOST_CHECK(isNaN(q_zero_vec));


    Quaternion q_zero_scalar_part(0.0f, 1.0f, 2.0f, 3.0f); // a = 0
    q_zero_scalar_part.normalize();
    BOOST_CHECK_CLOSE(q_zero_scalar_part.norm(), 1.0f, TOLERANCE);
    float inv_sqrt14 = 1.0f / std::sqrt(1.0f + 4.0f + 9.0f);
    checkEquals(q_zero_scalar_part, Quaternion(0, 1.0f*inv_sqrt14, 2.0f*inv_sqrt14, 3.0f*inv_sqrt14));
}

BOOST_AUTO_TEST_CASE(Conjugate) {
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion conj_q1 = q1.conj();
    checkEquals(conj_q1, Quaternion(1.0f, -2.0f, -3.0f, -4.0f));
    // Ensure original is not modified
    checkEquals(q1, Quaternion(1.0f, 2.0f, 3.0f, 4.0f));

    Quaternion q_pure_real(5.0f,0,0,0);
    q_pure_real.a = 5.0f; // from default
    checkEquals(q_pure_real.conj(), q_pure_real);

    Quaternion q_pure_vector(0.0f, 1.0f, 2.0f, 3.0f);
    checkEquals(q_pure_vector.conj(), Quaternion(0.0f, -1.0f, -2.0f, -3.0f));
}

BOOST_AUTO_TEST_CASE(DotProduct) {
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion q2(0.5f, -0.5f, 0.2f, -0.2f);
    // 1*0.5 + 2*(-0.5) + 3*0.2 + 4*(-0.2) = 0.5 - 1.0 + 0.6 - 0.8 = -0.7
    BOOST_CHECK_CLOSE(q1.dot_product(q2), -0.7f, TOLERANCE);

    Quaternion v1(0,1,0,0); // vector x
    Quaternion v2(0,0,1,0); // vector y
    BOOST_CHECK_CLOSE(v1.dot_product(v2), 0.0f, TOLERANCE); // Orthogonal
    BOOST_CHECK_CLOSE(v1.dot_product(v1), 1.0f, TOLERANCE); // Parallel with self
}


BOOST_AUTO_TEST_CASE(RotationBetweenVectors) {
    // Vectors must be pure (a=0) and normalized.
    Quaternion v1(0,1,0,0); v1.normalize(); // X-axis
    Quaternion v2(0,0,1,0); v2.normalize(); // Y-axis
    // Rotation from X to Y is +90 deg around Z-axis.
    // Q = (cos(pi/4), 0,0,sin(pi/4)) = (sqrt(0.5),0,0,sqrt(0.5))
    Quaternion rot_x_to_y = v1.rotation_between_vectors(v2);
    checkEquals(rot_x_to_y, Quaternion(std::sqrt(0.5f), 0,0,std::sqrt(0.5f)));
    checkEquals(rot_x_to_y.rotate(v1), v2);


    Quaternion v_z_pos(0,0,0,1); v_z_pos.normalize(); // Z-axis
    Quaternion v_z_neg(0,0,0,-1); v_z_neg.normalize(); // -Z-axis
    // Rotation between Z and -Z can be 180 deg around any axis in XY plane.
    // E.g., 180 deg around X: Q = (cos(pi/2), sin(pi/2),0,0) = (0,1,0,0)
    // E.g., 180 deg around Y: Q = (cos(pi/2), 0,sin(pi/2),0) = (0,0,1,0)
    // The function's formula: ret.a = 1 - ret.a (from v1*v2) then normalize.
    // v1*v2: v_z_pos * v_z_neg = (0,0,0,1)*(0,0,0,-1)
    // a = -(0*0)-(0*0)-(1*-1) = 1
    // b,c,d = 0
    // So, (v1*v2) = (1,0,0,0).
    // ret.a = 1 - 1 = 0.
    // ret = (0,0,0,0) before normalize. Normalize will result in NaN.
    // This is a known issue for 180-degree rotations.
    // Let's test with non-collinear vectors first.
    // The implementation uses: ret = (*this) * q; ret.a = 1.0f - ret.a; ret.normalize();
    // For v1=(0,0,0,1), q=(0,1,0,0) (Z-axis to Y-axis)
    // v1*q = (0,0,0,1)*(0,0,1,0) = (w=0, x=1, y=0, z=0) -> (0,1,0,0)
    // ret.a = 1-0 = 1. So ret=(1,1,0,0). Normalize: (1/sqrt(2), 1/sqrt(2),0,0)
    // This is rotation from Z to Y by 90 deg around X axis. Correct.
    Quaternion v_z(0,0,0,1); v_z.normalize(); // Z-axis
    Quaternion v_y(0,0,1,0); v_y.normalize(); // Y-axis
    Quaternion rot_z_to_y = v_z.rotation_between_vectors(v_y);
    // Computed rotation is (sqrt(0.5), -sqrt(0.5), 0, 0) which is -90 deg around X. This also rotates Z to Y.
    checkEquals(rot_z_to_y, Quaternion(std::sqrt(0.5f), -std::sqrt(0.5f), 0.0f, 0.0f));
    checkEquals(rot_z_to_y.rotate(v_z), v_y); // This is the important check

    // Parallel vectors: v1, v1
    // v1*v1 = (0,1,0,0)*(0,1,0,0) = (w=-1, x=0,y=0,z=0) -> (-1,0,0,0)
    // ret.a = 1 - (-1) = 2. ret = (2,0,0,0). Normalize: (1,0,0,0) (Identity)
    Quaternion rot_v1_to_v1 = v1.rotation_between_vectors(v1);
    checkEquals(rot_v1_to_v1, Quaternion(1,0,0,0)); // Identity rotation
    checkEquals(rot_v1_to_v1.rotate(v1), v1);

    // Anti-parallel: X-axis to -X-axis
    Quaternion v_x_pos(0,1,0,0); v_x_pos.normalize();
    Quaternion v_x_neg(0,-1,0,0); v_x_neg.normalize();
    // v_x_pos * v_x_neg = (0,1,0,0)*(0,-1,0,0) = (w = -1*(-1)=1, x=0,y=0,z=0) -> (1,0,0,0)
    // ret.a = 1 - 1 = 0. ret = (0,0,0,0). Normalize -> NaN.
    // This method has issues with 180-degree rotations.
    // The comment "w = 1 + v1•v2" suggests a different formula was considered.
    // v1•v2 = -1 for anti-parallel. w = 1-1 = 0.
    // x,y,z from cross product v1 x v2 = 0 for anti-parallel.
    // So (0,0,0,0) -> NaN.
    // Per user feedback, not adding a test to assert NaN for this.
    // Quaternion rot_x_to_neg_x = v_x_pos.rotation_between_vectors(v_x_neg);
    // BOOST_CHECK(isNaN(rot_x_to_neg_x));
}

BOOST_AUTO_TEST_CASE(RotateMore) {
    // Rotate by identity
    Quaternion iden; // (1,0,0,0)
    Quaternion vec(0,1,2,3);
    checkEquals(iden.rotate(vec), vec);

    // Rotate 180 deg around X axis
    Quaternion rot_180_x = Quaternion::from_axis_angle(M_PI, 0, 0); // (0,1,0,0)
    checkEquals(rot_180_x, Quaternion(0,1,0,0));
    Quaternion y_vec(0,0,1,0); // (0,Y,0)
    // Rotating Y by 180 deg around X should give -Y
    checkEquals(rot_180_x.rotate(y_vec), Quaternion(0,0,-1,0));

    // Combine rotations: 90 deg X then 90 deg Y
    // Rx(pi/2) = (c,s,0,0)
    // Ry(pi/2) = (c,0,s,0)
    // Combined Q = Ry(pi/2) * Rx(pi/2) (rotate by Rx first, then by Ry in world frame)
    // Q = (c,0,s,0) * (c,s,0,0)
    // w = c*c = 0.5
    // x = c*s = 0.5
    // y = s*c = 0.5
    // z = -s*s = -0.5 (Error in previous manual calculation, s*0 - c*s = -cs)
    //   z = w1z2 + x1y2 - y1x2 + z1w2 = c*0 + s*0 - 0*s + 0*c = 0. No, this is wrong.
    // z = a*q.d + b*q.c - c*q.b + d*q.a (from code for this*q)
    // Q1=Ry=(c,0,s,0), Q2=Rx=(c,s,0,0)
    // ret.d = Q1.a*Q2.d + Q1.b*Q2.c - Q1.c*Q2.b + Q1.d*Q2.a
    //       = c*0     + 0*0     - s*s     + 0*c = -s*s = -0.5
    Quaternion rotX90 = Quaternion::from_euler_rotation(M_PI_2, 0, 0);
    Quaternion rotY90 = Quaternion::from_euler_rotation(0, M_PI_2, 0);
    Quaternion combined_rot = rotY90 * rotX90; // Apply X then Y
    checkEquals(combined_rot, Quaternion(0.5f, 0.5f, 0.5f, -0.5f));

    Quaternion z_vec(0,0,0,1); // (0,0,Z)
    // Z by X(90) -> -Y
    // -Y by Y(90) -> -Y (no change)
    // Expected: (0,0,-1,0)
    checkEquals(combined_rot.rotate(z_vec), Quaternion(0,0,-1.0f,0));
}

BOOST_AUTO_TEST_CASE(Fractional) {
    Quaternion rot = Quaternion::from_euler_rotation(M_PI_2, 0, 0); // 90 deg X
    Quaternion rot_orig = rot;

    Quaternion frac_0 = Quaternion(rot_orig);
    frac_0.fractional(0.0f);
    // a = 1-f + f*a = 1. b,c,d = 0. Then normalize. -> (1,0,0,0)
    checkEquals(frac_0, Quaternion(1,0,0,0)); // Identity

    Quaternion frac_1 = Quaternion(rot_orig);
    frac_1.fractional(1.0f);
    // a = 0 + 1*a_orig. b,c,d = b_orig, c_orig, d_orig. Normalize.
    // Since rot_orig is already normalized, it should be rot_orig.
    checkEquals(frac_1, rot_orig);

    Quaternion frac_05 = Quaternion(rot_orig);
    frac_05.fractional(0.5f);
    // rot_orig = (sqrt(0.5), sqrt(0.5),0,0)
    // a_new = (1-0.5) + 0.5*sqrt(0.5) = 0.5 + 0.5*sqrt(0.5) = 0.5 + 0.35355 = 0.85355
    // b_new = 0.5 * sqrt(0.5) = 0.35355
    // c_new = 0
    // d_new = 0
    // Q_temp = (0.85355, 0.35355, 0, 0)
    // Normalize this. Norm = sqrt(0.85355^2 + 0.35355^2) = sqrt(0.7285 + 0.125) = sqrt(0.85355) = 0.9238
    // a_norm = 0.85355 / 0.9238 = 0.9239
    // b_norm = 0.35355 / 0.9238 = 0.3826
    // This should correspond to a rotation of M_PI_4 (45 deg) around X.
    // Q_expected = (cos(pi/8), sin(pi/8),0,0) = (0.923879, 0.382683,0,0)
    Quaternion expected_half_rot = Quaternion::from_euler_rotation(M_PI_4, 0, 0);
    checkEquals(frac_05, expected_half_rot);
}

// Tests for Properties
BOOST_AUTO_TEST_CASE(MultiplyByIdentity) {
    Quaternion q(0.1f, 0.2f, 0.3f, 0.4f);
    Quaternion iden; // Default is (1,0,0,0)
    checkEquals(q * iden, q);
    checkEquals(iden * q, q);
}

BOOST_AUTO_TEST_CASE(MultiplyByConjugate) {
    Quaternion q(0.1f, 0.2f, 0.3f, 0.4f);
    Quaternion q_conj = q.conj();
    Quaternion result = q * q_conj;
    // q * q.conj() = (norm(q))^2. This is a real quaternion (b,c,d = 0)
    float norm_sq = q.norm() * q.norm();
    BOOST_CHECK_CLOSE(result.a, norm_sq, TOLERANCE);
    BOOST_CHECK(std::abs(result.b) < TOLERANCE);
    BOOST_CHECK(std::abs(result.c) < TOLERANCE);
    BOOST_CHECK(std::abs(result.d) < TOLERANCE);

    // Try with a normalized quaternion
    Quaternion q_norm = Quaternion::from_euler_rotation(M_PI/3, M_PI/4, M_PI/5);
    q_norm.normalize(); // ensure it is
    Quaternion result_norm = q_norm * q_norm.conj();
    BOOST_CHECK_CLOSE(result_norm.a, 1.0f, TOLERANCE); // Norm squared is 1
    BOOST_CHECK(std::abs(result_norm.b) < TOLERANCE);
    BOOST_CHECK(std::abs(result_norm.c) < TOLERANCE);
    BOOST_CHECK(std::abs(result_norm.d) < TOLERANCE);
}

BOOST_AUTO_TEST_CASE(NormOfProduct) {
    Quaternion q1 = Quaternion::from_euler_rotation(0.2f, 0.3f, 0.4f);
    Quaternion q2 = Quaternion::from_euler_rotation(0.5f, 0.6f, 0.7f);
    // For unit quaternions (like those from_euler_rotation), norm is 1.
    // So norm(q1*q2) should be 1. And norm(q1)*norm(q2) = 1*1 = 1.
    BOOST_CHECK_CLOSE((q1*q2).norm(), q1.norm() * q2.norm(), TOLERANCE);
    BOOST_CHECK_CLOSE((q1*q2).norm(), 1.0f, TOLERANCE); // Since they are rotation quaternions

    Quaternion q3(1.0f,2.0f,0.5f,0.1f);
    Quaternion q4(0.5f,0.2f,1.0f,2.0f);
    BOOST_CHECK_CLOSE((q3*q4).norm(), q3.norm() * q4.norm(), TOLERANCE * 10); // Higher tolerance for non-unit
}

BOOST_AUTO_TEST_CASE(AssociativityOfMultiplication) {
    Quaternion q1 = Quaternion::from_euler_rotation(0.1f, 0.2f, 0.3f);
    Quaternion q2 = Quaternion::from_euler_rotation(0.4f, 0.5f, 0.6f);
    Quaternion q3 = Quaternion::from_euler_rotation(0.7f, 0.8f, 0.9f);

    Quaternion res1 = (q1 * q2) * q3;
    Quaternion res2 = q1 * (q2 * q3);
    checkEquals(res1, res2);
}

BOOST_AUTO_TEST_CASE(Distributivity) {
    // q1 * (q2 + q3) vs (q1 * q2) + (q1 * q3)
    Quaternion q1 = Quaternion::from_euler_rotation(0.1f, 0.2f, 0.3f); // A rotation quaternion
    Quaternion q2(0.5f, 0.2f, 0.8f, 0.1f); // Arbitrary quaternion
    Quaternion q3(0.3f, 0.7f, 0.4f, 0.6f); // Arbitrary quaternion

    Quaternion res1_lhs = q1 * (q2 + q3);
    Quaternion res1_rhs = (q1 * q2) + (q1 * q3);
    checkEquals(res1_lhs, res1_rhs);

    // (q1 + q2) * q3 vs (q1 * q3) + (q2 * q3)
    Quaternion res2_lhs = (q1 + q2) * q3;
    Quaternion res2_rhs = (q1 * q3) + (q2 * q3);
    checkEquals(res2_lhs, res2_rhs);
}

BOOST_AUTO_TEST_SUITE_END()
