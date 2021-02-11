#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE QuaternionTest
#define BOOST_TEST_NO_OLD_TOOLS
#include <boost/test/unit_test.hpp>
#include <Quaternion.h>
#include <iostream>

void checkEquals(const Quaternion &q1, const Quaternion &q2) {
    BOOST_CHECK_SMALL(q1.a - q2.a, .0001);
    BOOST_CHECK_SMALL(q1.b - q2.b, .0001);
    BOOST_CHECK_SMALL(q1.c - q2.c, .0001);
    BOOST_CHECK_SMALL(q1.d - q2.d, .0001);
}

BOOST_AUTO_TEST_CASE(rotateX) {
    auto rotateX = Quaternion::from_euler_rotation(M_PI_2, 0, 0);
    BOOST_CHECK_CLOSE(sqrt(2)/2.0, rotateX.a, .00001);
    BOOST_CHECK_CLOSE(sqrt(2)/2.0, rotateX.b, .00001);
    auto q = rotateX.rotate(Quaternion(0, 1, 0));
    BOOST_CHECK_CLOSE(1.0, q.d, .00001);
    checkEquals(q, Quaternion(0, 0, 1));
}

BOOST_AUTO_TEST_CASE(rotateY) {
    auto rotate = Quaternion::from_euler_rotation(0, M_PI_2, 0);
    BOOST_CHECK_CLOSE(sqrt(2)/2.0, rotate.a, .00001);
    BOOST_CHECK_CLOSE(sqrt(2)/2.0, rotate.c, .00001);
    auto q = rotate.rotate(Quaternion(0, 0, 1));
    BOOST_CHECK_CLOSE(1.0, q.b, .00001);
}

BOOST_AUTO_TEST_CASE(rotateZ) {
    auto rotate = Quaternion::from_euler_rotation(0, 0, M_PI_2);
    BOOST_CHECK_CLOSE(sqrt(2)/2.0, rotate.a, .00001);
    BOOST_CHECK_CLOSE(sqrt(2)/2.0, rotate.d, .00001);
    auto q = rotate.rotate(Quaternion(1, 0, 0));
    BOOST_CHECK_CLOSE(1.0, q.c, .00001);
}

BOOST_AUTO_TEST_CASE(rotateTwice) {
    auto rotate = Quaternion::from_euler_rotation(0, 0, M_PI_2) *
            Quaternion::from_euler_rotation(M_PI_2, 0, 0);
    rotate.normalize();
    BOOST_CHECK_CLOSE(.5, rotate.a, .0001);
    BOOST_CHECK_CLOSE(.5, rotate.b, .0001);
    BOOST_CHECK_CLOSE(.5, rotate.c, .0001);
    BOOST_CHECK_CLOSE(.5, rotate.d, .0001);

    auto q = rotate.rotate(Quaternion(0, 1, 0));
    BOOST_CHECK_CLOSE(1.0, q.d, .0001);
}

BOOST_AUTO_TEST_CASE(nonCommutative) {
    auto q1 = Quaternion::from_euler_rotation(0, 0, M_PI_2);
    auto q2 = Quaternion::from_euler_rotation(M_PI_2, 0, 0);
    auto rotate1 = (q1 * q2).conj();
    auto rotate2 = q2.conj() * q1.conj();
    checkEquals(rotate1, rotate2);
}

BOOST_AUTO_TEST_CASE(rotateRelative) {
    auto yaw = Quaternion::from_euler_rotation(0, 0, M_PI_2);
    auto pitch = Quaternion::from_euler_rotation(0, M_PI_2, 0);
    auto roll = Quaternion::from_euler_rotation(M_PI_2, 0, 0);
    auto composite = yaw * pitch * roll;
    auto rotated = composite.rotate(Quaternion(0, 0, 1));
    checkEquals(rotated, Quaternion(1, 0, 0));
}

