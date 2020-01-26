#include "Quaternion.h"
#include "Arduino.h"

// http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm
// 800B
Quaternion & Quaternion::operator*=(const Quaternion &q) {
    Quaternion ret;
    ret.a = a*q.a - b*q.b - c*q.c - d*q.d;
    ret.b = b*q.a + a*q.b + c*q.d - d*q.c;
    ret.c = a*q.c - b*q.d + c*q.a + d*q.b;
    ret.d = a*q.d + b*q.c - c*q.b + d*q.a;
    return (*this = ret);
}

Quaternion & Quaternion::operator+=(const Quaternion &q) {
    a += q.a;
    b += q.b;
    c += q.c;
    d += q.d;
    return *this;
}

Quaternion & Quaternion::operator*=(float scale) {
    a *= scale;
    b *= scale;
    c *= scale;
    d *= scale;
    return *this;
}

float Quaternion::norm() const {
    float norm2 = a*a + b*b + c*c + d*d;
    return sqrt(norm2);
}

// 400B
Quaternion & Quaternion::normalize() {
    float n = norm();
    a /= n;
    b /= n;
    c /= n;
    d /= n;
    return *this;
}

// this method returns a euler rotation
void Quaternion::to_euler_rotation(float *x, float *y, float *z) {
    double sqw = _w*_w;
    double sqx = _x*_x;
    double sqy = _y*_y;
    double sqz = _z*_z;

    x() = atan2(2.0*(_x*_y+_z*_w),(sqx-sqy-sqz+sqw));
    y() = asin(-2.0*(_x*_z-_y*_w)/(sqx+sqy+sqz+sqw));
    z() = atan2(2.0*(_y*_z+_x*_w),(-sqx-sqy+sqz+sqw));
}

// This method takes an euler rotation in rad and converts it to an equivilent
// Quaternion rotation.
// 800B
const Quaternion Quaternion::from_euler_rotation(float x, float y, float z) {
    float c1 = cos(y/2);
    float c2 = cos(z/2);
    float c3 = cos(x/2);

    float s1 = sin(y/2);
    float s2 = sin(z/2);
    float s3 = sin(x/2);
    Quaternion ret;
    ret.a = c1 * c2 * c3 - s1 * s2 * s3;
    ret.b = s1 * s2 * c3 + c1 * c2 * s3;
    ret.c = s1 * c2 * c3 + c1 * s2 * s3;
    ret.d = c1 * s2 * c3 - s1 * c2 * s3;
    return ret;
}

const Quaternion Quaternion::from_euler_rotation_approx(float x, float y, float z) {
    // approximage cos(theta) as 1 - theta^2 / 2
    float c1 = 1 - (y * y / 8);
    float c2 = 1 - (z * z / 8);
    float c3 = 1 - (x * x / 8);

    // appromixate sin(theta) as theta
    float s1 = y/2;
    float s2 = z/2;
    float s3 = x/2;
    Quaternion ret;
    ret.a = c1 * c2 * c3 - s1 * s2 * s3;
    ret.b = s1 * s2 * c3 + c1 * c2 * s3;
    ret.c = s1 * c2 * c3 + c1 * s2 * s3;
    ret.d = c1 * s2 * c3 - s1 * c2 * s3;
    return ret;
}

const Quaternion Quaternion::conj() const {
    Quaternion ret(*this);
    ret.b *= -1;
    ret.c *= -1;
    ret.d *= -1;
    return ret;
}

// This method takes two vectors and computes the rotation vector between them.
// Both the left and right hand sides must be pure vectors (a == 0)
// Both the left and right hand sides must normalized already.
// This computes the rotation that will tranform this to q.
// 500B
const Quaternion Quaternion::rotation_between_vectors(const Quaternion& q) const {
    // http://www.euclideanspace.com/maths/algebra/vectors/angleBetween/
    // We want to compute the below values.
    // w = 1 + v1•v2
    // x = (v1 x v2).x
    // y = (v1 x v2).y
    // z = (v1 x v2).z

    // Instead of writing the below code direclty, we reduce code size by
    // just using multiplication to implement it.
    //Quaternion ret;
    //ret.a = 1 + b * q.b + c * q.c + d * q.d;
    //ret.b = c * q.d - d * q.c;
    //ret.c = d * q.b - b * q.d;
    //ret.d = b * q.c - c * q.b;
    //ret.normalize();
    //return ret;

    // From wikipedia https://en.wikipedia.org/wiki/Quaternion#Quaternions_and_the_geometry_of_R3
    // The cross product p x q is just the vector part of multiplying p * q
    Quaternion ret = (*this) * q;
    ret.a = 1 - ret.a;
    ret.normalize();
    return ret;
}

float Quaternion::dot_product(const Quaternion& q) const {
    return a * q.a + b * q.b + c * q.c + d * q.d;
}

// This will roate the input vector by this normalized rotation quaternion.
const Quaternion Quaternion::rotate(const Quaternion& q) const {
    return (*this) * q * conj();
}

// This modifies this normalized rotation quaternion and makes it
// rotate between 0-1 as much as it would normally rotate.
// The math here is pretty sloppy but should work for
// most cases.
Quaternion & Quaternion::fractional(float f) {
    a = 1-f + f*a;
    b *= f;
    c *= f;
    d *= f;
    return normalize();
}
