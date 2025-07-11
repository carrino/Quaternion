#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE QuaternionTest
#define BOOST_TEST_NO_OLD_TOOLS
#include <boost/test/unit_test.hpp>
#include <boost/math/quaternion.hpp> // For Boost Quaternion
#include <Quaternion.h>
#include <iostream>
#include <cmath> // For std::sqrt, std::cos, std::sin, M_PI_2 etc.

// Define a tolerance for floating point comparisons
const float TOLERANCE = 0.0001f; // Default tolerance: 0.01%
const float BOOST_CMP_TOLERANCE = 0.001f; // Slightly higher tolerance for cross-library comparison

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
    Quaternion q_zero = Quaternion::from_euler_rotation(0, 0, 0);
    checkEquals(q_zero, Quaternion(1, 0, 0, 0));
    Quaternion qx = Quaternion::from_euler_rotation(M_PI_2, 0, 0);
    checkEquals(qx, Quaternion(std::sqrt(0.5f), std::sqrt(0.5f), 0, 0));
    Quaternion qy = Quaternion::from_euler_rotation(0, M_PI_2, 0);
    checkEquals(qy, Quaternion(std::sqrt(0.5f), 0, std::sqrt(0.5f), 0));
    Quaternion qz = Quaternion::from_euler_rotation(0, 0, M_PI_2);
    checkEquals(qz, Quaternion(std::sqrt(0.5f), 0, 0, std::sqrt(0.5f)));
    Quaternion q_combined = Quaternion::from_euler_rotation(M_PI_2, M_PI_2, M_PI_2);
    checkEquals(q_combined, Quaternion(std::sqrt(0.5f), 0.0f, std::sqrt(0.5f), 0.0f));
    Quaternion q_neg_x = Quaternion::from_euler_rotation(-M_PI_2, 0, 0);
    checkEquals(q_neg_x, Quaternion(std::sqrt(0.5f), -std::sqrt(0.5f), 0, 0));
}

BOOST_AUTO_TEST_CASE(FromEulerRotationApprox) {
    float small_angle = M_PI / 10.0f;
    float large_angle = M_PI_2;
    Quaternion q_exact_small = Quaternion::from_euler_rotation(small_angle, 0, 0);
    Quaternion q_approx_small = Quaternion::from_euler_rotation_approx(small_angle, 0, 0);
    float approx_tolerance_1pc = 0.01f;
    BOOST_CHECK(std::abs(q_exact_small.a - q_approx_small.a) < (approx_tolerance_1pc * std::abs(q_exact_small.a)));
    BOOST_CHECK(std::abs(q_exact_small.b - q_approx_small.b) < (approx_tolerance_1pc * std::abs(q_exact_small.b)));
    BOOST_CHECK_SMALL(q_exact_small.c - q_approx_small.c, approx_tolerance_1pc);
    BOOST_CHECK_SMALL(q_exact_small.d - q_approx_small.d, approx_tolerance_1pc);
    Quaternion q_exact_large = Quaternion::from_euler_rotation(large_angle, 0, 0);
    Quaternion q_approx_large = Quaternion::from_euler_rotation_approx(large_angle, 0, 0);
    bool are_different = std::abs(q_exact_large.a - q_approx_large.a) > TOLERANCE ||
                         std::abs(q_exact_large.b - q_approx_large.b) > TOLERANCE ||
                         std::abs(q_exact_large.c - q_approx_large.c) > TOLERANCE ||
                         std::abs(q_exact_large.d - q_approx_large.d) > TOLERANCE;
    BOOST_CHECK(are_different);
    Quaternion q_exact_small_xyz = Quaternion::from_euler_rotation(small_angle, small_angle, small_angle);
    Quaternion q_approx_small_xyz = Quaternion::from_euler_rotation_approx(small_angle, small_angle, small_angle);
    float approx_tolerance_1pc_euler_xyz = 0.01f;
    BOOST_CHECK(std::abs(q_exact_small_xyz.a - q_approx_small_xyz.a) < (approx_tolerance_1pc_euler_xyz * std::abs(q_exact_small_xyz.a)));
    BOOST_CHECK(std::abs(q_exact_small_xyz.b - q_approx_small_xyz.b) < (approx_tolerance_1pc_euler_xyz * std::abs(q_exact_small_xyz.b)));
    BOOST_CHECK(std::abs(q_exact_small_xyz.c - q_approx_small_xyz.c) < (approx_tolerance_1pc_euler_xyz * std::abs(q_exact_small_xyz.c)));
    BOOST_CHECK(std::abs(q_exact_small_xyz.d - q_approx_small_xyz.d) < (approx_tolerance_1pc_euler_xyz * std::abs(q_exact_small_xyz.d)));
}

BOOST_AUTO_TEST_CASE(FromAxisAngle) {
    Quaternion qx = Quaternion::from_axis_angle(M_PI_2, 0, 0);
    checkEquals(qx, Quaternion(std::cos(static_cast<float>(M_PI_4)), std::sin(static_cast<float>(M_PI_4)), 0, 0));
    Quaternion qy = Quaternion::from_axis_angle(0, M_PI, 0);
    BOOST_CHECK(std::abs(qy.a) < TOLERANCE);
    BOOST_CHECK(std::abs(qy.b) < TOLERANCE);
    BOOST_CHECK_CLOSE(qy.c, 1.0f, TOLERANCE);
    BOOST_CHECK(std::abs(qy.d) < TOLERANCE);
    Quaternion qz_zero = Quaternion::from_axis_angle(0.0f, 0.0f, 0.0f); // Test zero angle
    checkEquals(qz_zero, Quaternion(1.0f, 0.0f, 0.0f, 0.0f));      // Should be identity
    float component_angle = static_cast<float>(M_PI / (2.0 * std::sqrt(3.0)));
    Quaternion q_axis = Quaternion::from_axis_angle(component_angle, component_angle, component_angle);
    float angle_mag = std::sqrt(3.0f * component_angle * component_angle);
    float s = std::sin(angle_mag / 2.0f);
    float c = std::cos(angle_mag / 2.0f);
    checkEquals(q_axis, Quaternion(c, s * (component_angle/angle_mag), s * (component_angle/angle_mag), s * (component_angle/angle_mag)));
}

BOOST_AUTO_TEST_CASE(FromAxisAngleApprox) {
    float small_angle_val = static_cast<float>(M_PI / 10.0);
    float large_angle_val = M_PI_2; // Define large_angle_val here
    Quaternion q_exact_small = Quaternion::from_axis_angle(small_angle_val, 0, 0);
    Quaternion q_approx_small = Quaternion::from_axis_angle_approx(small_angle_val, 0, 0);
    // For 0.1 rad (approx M_PI/30, not M_PI/10), error in vector comp is ~0.04%. M_PI/10 is ~0.314 rad.
    // Error for vector component ~ angle^2/24 = (0.314)^2/24 ~ 0.0041 or 0.41% for angle = M_PI/10.
    // Scalar component error is much smaller.
    // Use a tolerance of 0.5% for BOOST_CHECK_CLOSE. (0.5% = 0.5 in Boost percentage terms)
    float approx_tolerance_check = 0.5f;
    BOOST_CHECK_CLOSE(q_exact_small.a, q_approx_small.a, approx_tolerance_check);
    BOOST_CHECK_CLOSE(q_exact_small.b, q_approx_small.b, approx_tolerance_check);
    BOOST_CHECK_CLOSE(q_exact_small.c, q_approx_small.c, approx_tolerance_check);
    BOOST_CHECK_CLOSE(q_exact_small.d, q_approx_small.d, approx_tolerance_check);

    // For large angles, the approximation is poor, so we just check they are different
    // enough (more different than TOLERANCE which is 0.01%).
    // Note: The original FromAxisAngleApprox test had more complex logic here from before the full overwrite.
    // This simplified check below is from the previous version of FromEulerRotationApprox.
    // The original test for FromAxisAngleApprox checked that for large_angle_val, q_exact_large and q_approx_large *were different*.
    Quaternion q_exact_large_val = Quaternion::from_axis_angle(large_angle_val, 0, 0);
    Quaternion q_approx_large_val = Quaternion::from_axis_angle_approx(large_angle_val, 0, 0);
    bool are_different_for_large_angle =
        (std::abs(q_exact_large_val.a - q_approx_large_val.a) > TOLERANCE) ||
        (std::abs(q_exact_large_val.b - q_approx_large_val.b) > TOLERANCE) ||
        (std::abs(q_exact_large_val.c - q_approx_large_val.c) > TOLERANCE) || // c and d are zero for this rotation
        (std::abs(q_exact_large_val.d - q_approx_large_val.d) > TOLERANCE);   // but check anyway
    BOOST_CHECK(are_different_for_large_angle);
}

BOOST_AUTO_TEST_CASE(AssignmentOperator) { Quaternion q1(0.1f,0.2f,0.3f,0.4f); Quaternion q2; q2=q1; checkEquals(q1,q2); Quaternion q3(1.f,2.f,3.f); Quaternion q4; q4=q3; checkEquals(q3,q4); }
BOOST_AUTO_TEST_CASE(MultiplicationAssignmentOperator) { Quaternion q1(sqrtf(0.5f),sqrtf(0.5f),0,0); Quaternion q2(sqrtf(0.5f),0,sqrtf(0.5f),0); q1*=q2; checkEquals(q1,Quaternion(0.5f,0.5f,0.5f,0.5f));}
BOOST_AUTO_TEST_CASE(MultiplicationOperator) { Quaternion q1(sqrtf(0.5f),sqrtf(0.5f),0,0); Quaternion q2(sqrtf(0.5f),0,sqrtf(0.5f),0); auto r=q1*q2; checkEquals(r,Quaternion(0.5f,0.5f,0.5f,0.5f));}
BOOST_AUTO_TEST_CASE(AdditionAssignmentOperator) { Quaternion q1(1,2,3,4); Quaternion q2(0.1f,0.2f,0.3f,0.4f); q1+=q2; checkEquals(q1,Quaternion(1.1f,2.2f,3.3f,4.4f));}
BOOST_AUTO_TEST_CASE(AdditionOperator) { Quaternion q1(1,2,3,4); Quaternion q2(0.1f,0.2f,0.3f,0.4f); auto r=q1+q2; checkEquals(r,Quaternion(1.1f,2.2f,3.3f,4.4f));}
BOOST_AUTO_TEST_CASE(ScalarMultiplicationAssignmentOperator) { Quaternion q1(1,2,3,4); q1*=2.f; checkEquals(q1,Quaternion(2,4,6,8)); q1*=0.5f; checkEquals(q1,Quaternion(1,2,3,4));}
BOOST_AUTO_TEST_CASE(ScalarMultiplicationOperator) { Quaternion q1(1,2,3,4); auto r=q1*2.f; checkEquals(r,Quaternion(2,4,6,8)); auto r2=q1*0.5f; checkEquals(r2,Quaternion(0.5f,1,1.5f,2));}
BOOST_AUTO_TEST_CASE(rotateX) {auto rX=Quaternion::from_euler_rotation(M_PI_2,0,0); auto q=rX.rotate(Quaternion(0,1,0)); checkEquals(q,Quaternion(0,0,1.f));}
BOOST_AUTO_TEST_CASE(rotateY) {auto rY=Quaternion::from_euler_rotation(0,M_PI_2,0); auto q=rY.rotate(Quaternion(0,0,1)); checkEquals(q,Quaternion(1.f,0,0));}
BOOST_AUTO_TEST_CASE(rotateZ) {auto rZ=Quaternion::from_euler_rotation(0,0,M_PI_2); auto q=rZ.rotate(Quaternion(1,0,0)); checkEquals(q,Quaternion(0,1.f,0));}
BOOST_AUTO_TEST_CASE(rotateTwice) {auto rZ=Quaternion::from_euler_rotation(0,0,M_PI_2); auto rX=Quaternion::from_euler_rotation(M_PI_2,0,0); auto r=rZ*rX; checkEquals(r,Quaternion(0.5f,0.5f,0.5f,0.5f)); auto q=r.rotate(Quaternion(0,1,0)); checkEquals(q,Quaternion(0,0,0,1.f));}
BOOST_AUTO_TEST_CASE(nonCommutative) {auto q1=Quaternion::from_euler_rotation(0,0,M_PI_2); auto q2=Quaternion::from_euler_rotation(M_PI_2,0,0); checkEquals((q1*q2).conj(), q2.conj()*q1.conj());}
BOOST_AUTO_TEST_CASE(rotateRelative) {auto y=Quaternion::from_euler_rotation(0,0,M_PI_2); auto p=Quaternion::from_euler_rotation(0,M_PI_2,0); auto r=Quaternion::from_euler_rotation(M_PI_2,0,0); auto comp=y*p*r; checkEquals(comp, Quaternion(sqrtf(0.5f),0,sqrtf(0.5f),0)); auto rot=comp.rotate(Quaternion(0,0,1)); checkEquals(rot,Quaternion(1.f,0,0));}
BOOST_AUTO_TEST_CASE(Norm) {Quaternion q1(1,2,3,4); BOOST_CHECK_CLOSE(q1.norm(),sqrtf(30.f),TOLERANCE); Quaternion qv(0,3,4,0); BOOST_CHECK_CLOSE(qv.norm(),5.f,TOLERANCE); Quaternion qi; BOOST_CHECK_CLOSE(qi.norm(),1.f,TOLERANCE); Quaternion q0(0,0,0,0); q0.a=0; BOOST_CHECK_CLOSE(q0.norm(),0.f,TOLERANCE);}
BOOST_AUTO_TEST_CASE(Normalize) {Quaternion q1(1,2,3,4); q1.normalize(); BOOST_CHECK_CLOSE(q1.norm(),1.f,TOLERANCE); float invs30=1.f/sqrtf(30.f); checkEquals(q1,Quaternion(1*invs30,2*invs30,3*invs30,4*invs30)); Quaternion q_an=Quaternion::from_euler_rotation(M_PI_4,0,0); Quaternion qc=q_an;q_an.normalize();checkEquals(q_an,qc);BOOST_CHECK_CLOSE(q_an.norm(),1.f,TOLERANCE); Quaternion q0sp(0,1,2,3);q0sp.normalize();BOOST_CHECK_CLOSE(q0sp.norm(),1.f,TOLERANCE);float invs14=1.f/sqrtf(14.f);checkEquals(q0sp,Quaternion(0,1*invs14,2*invs14,3*invs14));}
BOOST_AUTO_TEST_CASE(Conjugate) {Quaternion q1(1,2,3,4); checkEquals(q1.conj(),Quaternion(1,-2,-3,-4)); Quaternion qr(5,0,0,0); qr.a=5; checkEquals(qr.conj(),qr); Quaternion qv(0,1,2,3); checkEquals(qv.conj(),Quaternion(0,-1,-2,-3));}
BOOST_AUTO_TEST_CASE(DotProduct) {Quaternion q1(1,2,3,4);Quaternion q2(0.5f,-0.5f,0.2f,-0.2f);BOOST_CHECK_CLOSE(q1.dot_product(q2),-0.7f,TOLERANCE);Quaternion v1(0,1,0,0);Quaternion v2(0,0,1,0);BOOST_CHECK_CLOSE(v1.dot_product(v2),0.f,TOLERANCE);BOOST_CHECK_CLOSE(v1.dot_product(v1),1.f,TOLERANCE);}
BOOST_AUTO_TEST_CASE(RotationBetweenVectors) {Quaternion v1(0,1,0,0);v1.normalize();Quaternion v2(0,0,1,0);v2.normalize();Quaternion rxy=v1.rotation_between_vectors(v2);checkEquals(rxy,Quaternion(sqrtf(0.5f),0,0,sqrtf(0.5f)));checkEquals(rxy.rotate(v1),v2); Quaternion vz(0,0,0,1);vz.normalize();Quaternion vy(0,0,1,0);vy.normalize();Quaternion r_zy=vz.rotation_between_vectors(vy);checkEquals(r_zy,Quaternion(sqrtf(0.5f),-sqrtf(0.5f),0,0));checkEquals(r_zy.rotate(vz),vy);Quaternion r_v1v1=v1.rotation_between_vectors(v1);checkEquals(r_v1v1,Quaternion(1,0,0,0));checkEquals(r_v1v1.rotate(v1),v1);}
BOOST_AUTO_TEST_CASE(RotateMore) {Quaternion id;Quaternion v(0,1,2,3);checkEquals(id.rotate(v),v);Quaternion r180x=Quaternion::from_axis_angle(M_PI,0,0);checkEquals(r180x,Quaternion(0,1,0,0));Quaternion yv(0,0,1,0);checkEquals(r180x.rotate(yv),Quaternion(0,0,-1,0));Quaternion rX90=Quaternion::from_euler_rotation(M_PI_2,0,0);Quaternion rY90=Quaternion::from_euler_rotation(0,M_PI_2,0);Quaternion cr=rY90*rX90;checkEquals(cr,Quaternion(0.5f,0.5f,0.5f,-0.5f));Quaternion zv(0,0,0,1);checkEquals(cr.rotate(zv),Quaternion(0,0,-1,0));}
BOOST_AUTO_TEST_CASE(Fractional) {Quaternion r=Quaternion::from_euler_rotation(M_PI_2,0,0);Quaternion r0=r;r0.fractional(0.f);checkEquals(r0,Quaternion(1,0,0,0));Quaternion r1=r;r1.fractional(1.f);checkEquals(r1,r);Quaternion r05=r;r05.fractional(0.5f);Quaternion ehr=Quaternion::from_euler_rotation(M_PI_4,0,0);checkEquals(r05,ehr);}
BOOST_AUTO_TEST_CASE(MultiplyByIdentity) {Quaternion q(0.1f,0.2f,0.3f,0.4f);Quaternion id;checkEquals(q*id,q);checkEquals(id*q,q);}
BOOST_AUTO_TEST_CASE(MultiplyByConjugate) {
    Quaternion q(0.1f, 0.2f, 0.3f, 0.4f);
    float n2 = q.norm() * q.norm();
    Quaternion r = q * q.conj();
    BOOST_CHECK_CLOSE(r.a, n2, TOLERANCE);
    BOOST_CHECK(std::abs(r.b) < 1e-5f);
    BOOST_CHECK(std::abs(r.c) < 1e-5f);
    BOOST_CHECK(std::abs(r.d) < 1e-5f);

    Quaternion qn = Quaternion::from_euler_rotation(static_cast<float>(M_PI/3.0), static_cast<float>(M_PI/4.0), static_cast<float>(M_PI/5.0));
    qn.normalize();
    Quaternion rn = qn * qn.conj();
    BOOST_CHECK_CLOSE(rn.a, 1.0f, TOLERANCE);
    BOOST_CHECK(std::abs(rn.b) < 1e-5f);
    BOOST_CHECK(std::abs(rn.c) < 1e-5f);
    BOOST_CHECK(std::abs(rn.d) < 1e-5f);
}
BOOST_AUTO_TEST_CASE(NormOfProduct) {Quaternion q1=Quaternion::from_euler_rotation(0.2f,0.3f,0.4f);Quaternion q2=Quaternion::from_euler_rotation(0.5f,0.6f,0.7f);BOOST_CHECK_CLOSE((q1*q2).norm(),q1.norm()*q2.norm(),TOLERANCE);BOOST_CHECK_CLOSE((q1*q2).norm(),1.f,TOLERANCE);Quaternion q3(1,2,0.5f,0.1f);Quaternion q4(0.5f,0.2f,1,2);BOOST_CHECK_CLOSE((q3*q4).norm(),q3.norm()*q4.norm(),TOLERANCE*100);} // Increased tolerance for non-unit
BOOST_AUTO_TEST_CASE(AssociativityOfMultiplication) {Quaternion q1=Quaternion::from_euler_rotation(0.1f,0.2f,0.3f);Quaternion q2=Quaternion::from_euler_rotation(0.4f,0.5f,0.6f);Quaternion q3=Quaternion::from_euler_rotation(0.7f,0.8f,0.9f);checkEquals((q1*q2)*q3,q1*(q2*q3));}
BOOST_AUTO_TEST_CASE(Distributivity) {Quaternion q1=Quaternion::from_euler_rotation(0.1f,0.2f,0.3f);Quaternion q2(0.5f,0.2f,0.8f,0.1f);Quaternion q3(0.3f,0.7f,0.4f,0.6f);checkEquals(q1*(q2+q3),(q1*q2)+(q1*q3));checkEquals((q1+q2)*q3,(q1*q3)+(q2*q3));}
BOOST_AUTO_TEST_SUITE_END()


// Helper to compare our Quaternion with boost::math::quaternion<float>
const float ACTUAL_BOOST_CMP_TOLERANCE = 0.001f; // Renamed to avoid conflict
void checkBoostEquals(const Quaternion& our_q, const boost::math::quaternion<float>& boost_q, float tolerance = ACTUAL_BOOST_CMP_TOLERANCE) {
    BOOST_CHECK_CLOSE(our_q.a, boost_q.R_component_1(), tolerance);
    BOOST_CHECK_CLOSE(our_q.b, boost_q.R_component_2(), tolerance);
    BOOST_CHECK_CLOSE(our_q.c, boost_q.R_component_3(), tolerance);
    BOOST_CHECK_CLOSE(our_q.d, boost_q.R_component_4(), tolerance);
}
boost::math::quaternion<float> toBoostQuaternion(const Quaternion& q) {
    return boost::math::quaternion<float>(q.a, q.b, q.c, q.d);
}
Quaternion fromBoostQuaternion(const boost::math::quaternion<float>& bq) {
    return Quaternion(bq.R_component_1(), bq.R_component_2(), bq.R_component_3(), bq.R_component_4());
}

BOOST_AUTO_TEST_SUITE(BoostQuaternionComparisonTests) // Float vs Float
BOOST_AUTO_TEST_CASE(BoostMultiplication) {
    Quaternion q1_our(std::sqrt(0.5f), std::sqrt(0.5f),0,0);
    Quaternion q2_our(std::sqrt(0.5f),0,std::sqrt(0.5f),0);
    auto r_our = q1_our*q2_our;
    auto q1_b = toBoostQuaternion(q1_our);
    auto q2_b = toBoostQuaternion(q2_our);
    auto r_b = q1_b*q2_b;
    checkBoostEquals(r_our, r_b);
    Quaternion q3_our(0.1f,0.2f,0.3f,0.4f); q3_our.normalize();
    Quaternion q4_our(0.5f,-0.1f,-0.2f,0.3f); q4_our.normalize();
    auto q3_b = toBoostQuaternion(q3_our); auto q4_b = toBoostQuaternion(q4_our);
    checkBoostEquals(q3_our*q4_our, q3_b*q4_b);
}
BOOST_AUTO_TEST_CASE(BoostFromAxisAngle) {
    float angle_x=M_PI_2; float ax=1,ay=0,az=0;
    auto qx_our=Quaternion::from_axis_angle(ax*angle_x,ay*angle_x,az*angle_x);
    float chx=std::cos(angle_x/2.f); float shx=std::sin(angle_x/2.f);
    boost::math::quaternion<float> qx_b(chx,shx*ax,shx*ay,shx*az);
    checkBoostEquals(qx_our,qx_b);
    // Add more variations like in FloatToDoubleFromAxisAngle if desired
    Quaternion q_zero_our = Quaternion::from_axis_angle(0.0f, 0.0f, 0.0f);
    boost::math::quaternion<float> q_zero_boost(1.0f, 0.0f, 0.0f, 0.0f);
    checkBoostEquals(q_zero_our, q_zero_boost);
}
BOOST_AUTO_TEST_CASE(BoostRotate) {
    Quaternion r_q_our=Quaternion::from_euler_rotation(M_PI_2,0,0);
    auto r_q_b=toBoostQuaternion(r_q_our);
    auto r_q_b_c=conj(r_q_b);
    Quaternion vec_our(0,0,1.f,0);
    auto vec_b = boost::math::quaternion<float>(0,vec_our.b,vec_our.c,vec_our.d);
    auto rot_vec_our=r_q_our.rotate(vec_our);
    auto rot_vec_b=r_q_b*vec_b*r_q_b_c;
    checkBoostEquals(rot_vec_our,rot_vec_b);
    // Add more variations
}
BOOST_AUTO_TEST_SUITE_END()


// --- BoostDoubleComparisonTests Suite (Float vs Double) ---
// Helper to compare our float Quaternion with boost::math::quaternion<double>
const double PPM_PERCENT_TOLERANCE = 0.0001; // 1 PPM = 1e-6 = 0.0001%
void checkBoostDoubleEquals(const Quaternion& our_q_float, const boost::math::quaternion<double>& boost_q_double, double tolerance_percentage = PPM_PERCENT_TOLERANCE) {
    BOOST_CHECK_CLOSE(static_cast<double>(our_q_float.a), boost_q_double.R_component_1(), tolerance_percentage);
    BOOST_CHECK_CLOSE(static_cast<double>(our_q_float.b), boost_q_double.R_component_2(), tolerance_percentage);
    BOOST_CHECK_CLOSE(static_cast<double>(our_q_float.c), boost_q_double.R_component_3(), tolerance_percentage);
    BOOST_CHECK_CLOSE(static_cast<double>(our_q_float.d), boost_q_double.R_component_4(), tolerance_percentage);
}
boost::math::quaternion<double> toBoostQuaternionDouble(const Quaternion& q_float) {
    return boost::math::quaternion<double>(static_cast<double>(q_float.a), static_cast<double>(q_float.b), static_cast<double>(q_float.c), static_cast<double>(q_float.d));
}

// Test data (using double for Boost side)
const double M_PI_D = M_PI;
const double M_PI_2_D = M_PI_D / 2.0;
const double M_PI_4_D = M_PI_D / 4.0;
const double SQRT_0_5_D = std::sqrt(0.5);

// Helper struct for basic 3D vector operations for test setup
template<typename T>
struct TestVec3 {
    T x, y, z;
    TestVec3(T x_ = 0, T y_ = 0, T z_ = 0) : x(x_), y(y_), z(z_) {}
    TestVec3 normalize() { T mag = std::sqrt(x*x + y*y + z*z); if (mag == 0) return TestVec3(0,0,0); return TestVec3(x/mag, y/mag, z/mag); }
    T dot(const TestVec3& other) const { return x*other.x + y*other.y + z*other.z; }
    TestVec3 cross(const TestVec3& other) const { return TestVec3( y*other.z - z*other.y, z*other.x - x*other.z, x*other.y - y*other.x ); }
};

BOOST_AUTO_TEST_SUITE(BoostDoubleComparisonTests)

BOOST_AUTO_TEST_CASE(FloatToDoubleMultiplication) {
    Quaternion q1_our_f(std::sqrt(0.5f), std::sqrt(0.5f), 0.0f, 0.0f);
    Quaternion q2_our_f(std::sqrt(0.5f), 0.0f, std::sqrt(0.5f), 0.0f);
    boost::math::quaternion<double> q1_boost_d(SQRT_0_5_D, SQRT_0_5_D, 0.0, 0.0);
    boost::math::quaternion<double> q2_boost_d(SQRT_0_5_D, 0.0, SQRT_0_5_D, 0.0);
    Quaternion result_our_f = q1_our_f * q2_our_f;
    boost::math::quaternion<double> result_boost_d = q1_boost_d * q2_boost_d;
    checkBoostDoubleEquals(result_our_f, result_boost_d);

    Quaternion q3_our_f(0.1f, 0.2f, 0.3f, 0.4f); q3_our_f.normalize();
    Quaternion q4_our_f(0.5f, -0.1f, -0.2f, 0.3f); q4_our_f.normalize();
    boost::math::quaternion<double> q3_boost_d_orig(0.1, 0.2, 0.3, 0.4);
    double q3_norm_d = boost::math::abs(q3_boost_d_orig);
    boost::math::quaternion<double> q3_boost_d = (q3_norm_d == 0.0) ? q3_boost_d_orig : q3_boost_d_orig / q3_norm_d;
    boost::math::quaternion<double> q4_boost_d_orig(0.5, -0.1, -0.2, 0.3);
    double q4_norm_d = boost::math::abs(q4_boost_d_orig);
    boost::math::quaternion<double> q4_boost_d = (q4_norm_d == 0.0) ? q4_boost_d_orig : q4_boost_d_orig / q4_norm_d;
    checkBoostDoubleEquals(q3_our_f * q4_our_f, q3_boost_d * q4_boost_d);
}

BOOST_AUTO_TEST_CASE(FloatToDoubleFromAxisAngle) {
    float angle_x_f = M_PI_2; double angle_x_d = M_PI_2_D;
    float axis_x_x_f = 1.0f, axis_x_y_f = 0.0f, axis_x_z_f = 0.0f;
    double axis_x_x_d = 1.0, axis_x_y_d = 0.0, axis_x_z_d = 0.0;
    Quaternion qx_our_f = Quaternion::from_axis_angle(axis_x_x_f * angle_x_f, axis_x_y_f * angle_x_f, axis_x_z_f * angle_x_f);
    double c_half_x_d = std::cos(angle_x_d / 2.0); double s_half_x_d = std::sin(angle_x_d / 2.0);
    boost::math::quaternion<double> qx_boost_d(c_half_x_d, s_half_x_d * axis_x_x_d, s_half_x_d * axis_x_y_d, s_half_x_d * axis_x_z_d);
    checkBoostDoubleEquals(qx_our_f, qx_boost_d);

    float angle_xyz_f = static_cast<float>(M_PI / 6.0); double angle_xyz_d = M_PI_D / 6.0;
    double axis_norm_d = std::sqrt(3.0); // Use double for intermediate axis component calculation for float version
    float comp_x_f = static_cast<float>(angle_xyz_d * (1.0/axis_norm_d)); // Create float inputs from double reference
    float comp_y_f = static_cast<float>(angle_xyz_d * (1.0/axis_norm_d));
    float comp_z_f = static_cast<float>(angle_xyz_d * (1.0/axis_norm_d));
    Quaternion qxyz_our_f = Quaternion::from_axis_angle(comp_x_f, comp_y_f, comp_z_f);
    double c_half_xyz_d = std::cos(angle_xyz_d / 2.0); double s_half_xyz_d = std::sin(angle_xyz_d / 2.0);
    boost::math::quaternion<double> qxyz_boost_d(c_half_xyz_d, (1.0/axis_norm_d) * s_half_xyz_d, (1.0/axis_norm_d) * s_half_xyz_d, (1.0/axis_norm_d) * s_half_xyz_d);
    checkBoostDoubleEquals(qxyz_our_f, qxyz_boost_d);

    Quaternion q_zero_our_f = Quaternion::from_axis_angle(0.0f, 0.0f, 0.0f);
    boost::math::quaternion<double> q_zero_boost_d(1.0, 0.0, 0.0, 0.0);
    checkBoostDoubleEquals(q_zero_our_f, q_zero_boost_d);
}

BOOST_AUTO_TEST_CASE(FloatToDoubleRotate) {
    Quaternion rot_q_our_f = Quaternion::from_euler_rotation(M_PI_2, 0.0f, 0.0f);
    boost::math::quaternion<double> rot_q_boost_d(std::cos(M_PI_4_D), std::sin(M_PI_4_D), 0.0, 0.0);
    boost::math::quaternion<double> rot_q_boost_d_conj = conj(rot_q_boost_d);
    Quaternion vec_our_f(0.0f, 0.0f, 1.0f, 0.0f);
    boost::math::quaternion<double> vec_boost_d(0.0, 0.0, 1.0, 0.0);
    Quaternion rotated_vec_our_f = rot_q_our_f.rotate(vec_our_f);
    boost::math::quaternion<double> rotated_vec_boost_d = rot_q_boost_d * vec_boost_d * rot_q_boost_d_conj;
    checkBoostDoubleEquals(rotated_vec_our_f, rotated_vec_boost_d);

    Quaternion id_rot_our_f;
    boost::math::quaternion<double> id_rot_boost_d(1.0,0.0,0.0,0.0);
    Quaternion vec3_our_f(0.0f, 0.5f, -0.3f, 1.2f);
    boost::math::quaternion<double> vec3_boost_d(0.0, 0.5, -0.3, 1.2);
    Quaternion rotated_by_id_our_f = id_rot_our_f.rotate(vec3_our_f);
    boost::math::quaternion<double> rotated_by_id_boost_d = id_rot_boost_d * vec3_boost_d * conj(id_rot_boost_d);
    checkBoostDoubleEquals(rotated_by_id_our_f, rotated_by_id_boost_d);
    checkBoostDoubleEquals(rotated_by_id_our_f, toBoostQuaternionDouble(vec3_our_f));
}

BOOST_AUTO_TEST_CASE(FloatToDoubleFromAxisAngleApprox) {
    double axis_d_x = 1.0 / std::sqrt(3.0); double axis_d_y = 1.0 / std::sqrt(3.0); double axis_d_z = 1.0 / std::sqrt(3.0);
    float axis_f_x = static_cast<float>(axis_d_x); float axis_f_y = static_cast<float>(axis_d_y); float axis_f_z = static_cast<float>(axis_d_z);
    float total_angle_small_f = 0.1f; double total_angle_small_d = 0.1;
    Quaternion q_approx_small_f = Quaternion::from_axis_angle_approx(total_angle_small_f * axis_f_x, total_angle_small_f * axis_f_y, total_angle_small_f * axis_f_z);
    double c_half_small_d = std::cos(total_angle_small_d / 2.0); double s_half_small_d = std::sin(total_angle_small_d / 2.0);
    boost::math::quaternion<double> q_exact_small_d(c_half_small_d, s_half_small_d * axis_d_x, s_half_small_d * axis_d_y, s_half_small_d * axis_d_z);
    checkBoostDoubleEquals(q_approx_small_f, q_exact_small_d, 0.05);

    float total_angle_large_f = 0.5f; double total_angle_large_d = 0.5;
    Quaternion q_approx_large_f = Quaternion::from_axis_angle_approx(total_angle_large_f * axis_f_x, total_angle_large_f * axis_f_y, total_angle_large_f * axis_f_z);
    double c_half_large_d = std::cos(total_angle_large_d / 2.0); double s_half_large_d = std::sin(total_angle_large_d / 2.0);
    boost::math::quaternion<double> q_exact_large_d(c_half_large_d, s_half_large_d * axis_d_x, s_half_large_d * axis_d_y, s_half_large_d * axis_d_z);
    checkBoostDoubleEquals(q_approx_large_f, q_exact_large_d, 2.0);
}

BOOST_AUTO_TEST_CASE(FloatToDoubleRotationBetweenVectors) {
    TestVec3<float> v1_f_s1(1.0f, 0.0f, 0.0f); TestVec3<float> v2_f_s1(0.0f, 1.0f, 0.0f);
    Quaternion q_v1_f_s1(0.0f, v1_f_s1.x, v1_f_s1.y, v1_f_s1.z); Quaternion q_v2_f_s1(0.0f, v2_f_s1.x, v2_f_s1.y, v2_f_s1.z);
    Quaternion rot_our_f_s1 = q_v1_f_s1.rotation_between_vectors(q_v2_f_s1);
    TestVec3<double> v1_d_s1(1.0, 0.0, 0.0); TestVec3<double> v2_d_s1(0.0, 1.0, 0.0);
    double dot_s1_d = v1_d_s1.dot(v2_d_s1); TestVec3<double> cross_s1_d = v1_d_s1.cross(v2_d_s1);
    boost::math::quaternion<double> q_ref_unnorm_s1_d(1.0 + dot_s1_d, cross_s1_d.x, cross_s1_d.y, cross_s1_d.z);
    double abs_s1_d = boost::math::abs(q_ref_unnorm_s1_d);
    boost::math::quaternion<double> rot_boost_d_s1 = (abs_s1_d == 0.0) ? q_ref_unnorm_s1_d : q_ref_unnorm_s1_d / abs_s1_d;
    checkBoostDoubleEquals(rot_our_f_s1, rot_boost_d_s1);

    TestVec3<float> v1_f_s2(1.0f, 0.0f, 0.0f); TestVec3<float> v2_f_s2(std::cos(static_cast<float>(M_PI_4)), std::sin(static_cast<float>(M_PI_4)), 0.0f);
    v2_f_s2 = v2_f_s2.normalize();
    Quaternion q_v1_f_s2(0.0f, v1_f_s2.x, v1_f_s2.y, v1_f_s2.z); Quaternion q_v2_f_s2(0.0f, v2_f_s2.x, v2_f_s2.y, v2_f_s2.z);
    Quaternion rot_our_f_s2 = q_v1_f_s2.rotation_between_vectors(q_v2_f_s2);
    TestVec3<double> v1_d_s2(1.0, 0.0, 0.0); TestVec3<double> v2_d_s2(std::cos(M_PI_4_D), std::sin(M_PI_4_D), 0.0);
    v2_d_s2 = v2_d_s2.normalize();
    double dot_s2_d = v1_d_s2.dot(v2_d_s2); TestVec3<double> cross_s2_d = v1_d_s2.cross(v2_d_s2);
    boost::math::quaternion<double> q_ref_unnorm_s2_d(1.0 + dot_s2_d, cross_s2_d.x, cross_s2_d.y, cross_s2_d.z);
    double abs_s2_d = boost::math::abs(q_ref_unnorm_s2_d);
    boost::math::quaternion<double> rot_boost_d_s2 = (abs_s2_d == 0.0) ? q_ref_unnorm_s2_d : q_ref_unnorm_s2_d / abs_s2_d;
    checkBoostDoubleEquals(rot_our_f_s2, rot_boost_d_s2);

    TestVec3<float> v1_f_s3(1.0f, 0.0f, 0.0f);
    Quaternion q_v1_f_s3(0.0f, v1_f_s3.x, v1_f_s3.y, v1_f_s3.z);
    Quaternion rot_our_f_s3 = q_v1_f_s3.rotation_between_vectors(q_v1_f_s3);
    TestVec3<double> v1_d_s3(1.0, 0.0, 0.0);
    double dot_s3_d = v1_d_s3.dot(v1_d_s3); TestVec3<double> cross_s3_d = v1_d_s3.cross(v1_d_s3);
    boost::math::quaternion<double> q_ref_unnorm_s3_d(1.0 + dot_s3_d, cross_s3_d.x, cross_s3_d.y, cross_s3_d.z);
    double abs_s3_d = boost::math::abs(q_ref_unnorm_s3_d);
    boost::math::quaternion<double> rot_boost_d_s3 = (abs_s3_d == 0.0) ? q_ref_unnorm_s3_d : q_ref_unnorm_s3_d / abs_s3_d;
    checkBoostDoubleEquals(rot_our_f_s3, rot_boost_d_s3);
}

BOOST_AUTO_TEST_SUITE_END()
