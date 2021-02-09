#define BOOST_TEST_DYN_LINK
//#define BOOST_TEST_MAIN  // in only one cpp file
#define BOOST_TEST_MODULE QuaternionTest
#include <boost/test/unit_test.hpp>
#include <Quaternion.h>
#include <iostream>

//using namespace std;
using std::cout;

BOOST_AUTO_TEST_CASE(rotateX) {
    auto rotateX = Quaternion::from_euler_rotation(M_PI / 2, 0, 0);
    BOOST_CHECK_CLOSE(sqrt(2)/2.0, rotateX.a, .00001);
    BOOST_CHECK_CLOSE(sqrt(2)/2.0, rotateX.b, .00001);
    auto q = rotateX.rotate(Quaternion(0, 1, 0));
    BOOST_CHECK_CLOSE(1.0, q.d, .00001);
}
