// Quaternion algebra: construction, assignment and the arithmetic operators,
// checked component-wise against boost::math::quaternion<double>.
#include "test_helpers.h"

BOOST_AUTO_TEST_SUITE(Algebra)

BOOST_AUTO_TEST_CASE(DefaultConstructorIsIdentity) {
    Quaternion q;
    BOOST_TEST(q.a == 1.0f);
    BOOST_TEST(q.b == 0.0f);
    BOOST_TEST(q.c == 0.0f);
    BOOST_TEST(q.d == 0.0f);
}

BOOST_AUTO_TEST_CASE(VectorConstructorIsPureQuaternion) {
    Quaternion v(1.5f, -2.0f, 3.25f);
    BOOST_TEST(v.a == 0.0f);
    BOOST_TEST(v.b == 1.5f);
    BOOST_TEST(v.c == -2.0f);
    BOOST_TEST(v.d == 3.25f);
}

BOOST_AUTO_TEST_CASE(FullConstructorUsesBoostComponentOrder) {
    // (a, b, c, d) is a + b*i + c*j + d*k, exactly Boost's R_component_1..4.
    Quaternion q(0.5f, -1.0f, 2.0f, -4.0f);
    BoostQuat b = toBoost(q);
    BOOST_TEST(b.R_component_1() == 0.5);
    BOOST_TEST(b.R_component_2() == -1.0);
    BOOST_TEST(b.R_component_3() == 2.0);
    BOOST_TEST(b.R_component_4() == -4.0);
    BOOST_TEST(real(b) == 0.5);
}

BOOST_AUTO_TEST_CASE(CopyAndAssignment) {
    Quaternion q(0.1f, 0.2f, 0.3f, 0.4f);

    Quaternion copy(q);
    CHECK_QUAT_CLOSE(copy, q, 0.0);

    Quaternion assigned;
    assigned = q;
    CHECK_QUAT_CLOSE(assigned, q, 0.0);

    Quaternion &alias = assigned;
    assigned = alias;  // self-assignment
    CHECK_QUAT_CLOSE(assigned, q, 0.0);

    // Assignment is chainable and returns the left-hand side.
    Quaternion x, y;
    Quaternion &ret = (x = y = q);
    BOOST_TEST(&ret == &x);
    CHECK_QUAT_CLOSE(x, q, 0.0);
    CHECK_QUAT_CLOSE(y, q, 0.0);
}

BOOST_AUTO_TEST_CASE(BasisProductsFollowHamiltonsRules) {
    // i*i = j*j = k*k = i*j*k = -1; i*j = k, j*k = i, k*i = j; and the basis
    // elements anti-commute. These products are exact in float.
    const Quaternion one(1, 0, 0, 0), i(0, 1, 0, 0), j(0, 0, 1, 0), k(0, 0, 0, 1);
    const Quaternion minusOne = one * -1.0f;

    CHECK_QUAT_CLOSE(i * i, minusOne, 0.0);
    CHECK_QUAT_CLOSE(j * j, minusOne, 0.0);
    CHECK_QUAT_CLOSE(k * k, minusOne, 0.0);
    CHECK_QUAT_CLOSE(i * j * k, minusOne, 0.0);

    CHECK_QUAT_CLOSE(i * j, k, 0.0);
    CHECK_QUAT_CLOSE(j * k, i, 0.0);
    CHECK_QUAT_CLOSE(k * i, j, 0.0);

    CHECK_QUAT_CLOSE(j * i, k * -1.0f, 0.0);
    CHECK_QUAT_CLOSE(k * j, i * -1.0f, 0.0);
    CHECK_QUAT_CLOSE(i * k, j * -1.0f, 0.0);
}

BOOST_AUTO_TEST_CASE(ProductMatchesBoost) {
    Random rnd(1);
    for (int n = 0; n < 2000; n++) {
        Quaternion p = rnd.quaternion(10.0);
        Quaternion q = rnd.quaternion(10.0);
        BoostQuat expected = toBoost(p) * toBoost(q);
        double tol = scaled(kOpTol, abs(toBoost(p)) * abs(toBoost(q)));

        CHECK_QUAT_CLOSE(p * q, expected, tol);

        Quaternion inPlace = p;
        Quaternion &ret = (inPlace *= q);
        BOOST_TEST(&ret == &inPlace);
        CHECK_QUAT_CLOSE(inPlace, expected, tol);
    }
}

BOOST_AUTO_TEST_CASE(ProductDoesNotModifyOperands) {
    const Quaternion p(1, 2, 3, 4), q(5, 6, 7, 8);
    Quaternion r = p * q;
    CHECK_QUAT_CLOSE(r, toBoost(p) * toBoost(q), 0.0);
    CHECK_QUAT_CLOSE(p, Quaternion(1, 2, 3, 4), 0.0);
    CHECK_QUAT_CLOSE(q, Quaternion(5, 6, 7, 8), 0.0);
}

BOOST_AUTO_TEST_CASE(ProductWithItselfInPlace) {
    // q *= q must read every component of q before overwriting any of them.
    Quaternion q(1, 2, 3, 4);
    BoostQuat expected = toBoost(q) * toBoost(q);
    q *= q;
    CHECK_QUAT_CLOSE(q, expected, 0.0);
}

BOOST_AUTO_TEST_CASE(ProductIsNotCommutative) {
    const Quaternion i(0, 1, 0, 0), j(0, 0, 1, 0);
    // i*j = k but j*i = -k.
    BOOST_TEST(maxComponentDiff(toBoost(i * j), toBoost(j * i)) == 2.0);
}

BOOST_AUTO_TEST_CASE(SumMatchesBoost) {
    Random rnd(2);
    for (int n = 0; n < 2000; n++) {
        Quaternion p = rnd.quaternion(10.0);
        Quaternion q = rnd.quaternion(10.0);
        BoostQuat expected = toBoost(p) + toBoost(q);
        double tol = scaled(kOpTol, abs(toBoost(p)) + abs(toBoost(q)));

        CHECK_QUAT_CLOSE(p + q, expected, tol);
        CHECK_QUAT_CLOSE(q + p, expected, tol);

        Quaternion inPlace = p;
        Quaternion &ret = (inPlace += q);
        BOOST_TEST(&ret == &inPlace);
        CHECK_QUAT_CLOSE(inPlace, expected, tol);
    }
}

BOOST_AUTO_TEST_CASE(SumDoesNotModifyOperands) {
    const Quaternion p(1, 2, 3, 4), q(5, 6, 7, 8);
    Quaternion r = p + q;
    CHECK_QUAT_CLOSE(r, Quaternion(6, 8, 10, 12), 0.0);
    CHECK_QUAT_CLOSE(p, Quaternion(1, 2, 3, 4), 0.0);
    CHECK_QUAT_CLOSE(q, Quaternion(5, 6, 7, 8), 0.0);
}

BOOST_AUTO_TEST_CASE(ScalarProductMatchesBoost) {
    Random rnd(3);
    for (int n = 0; n < 2000; n++) {
        Quaternion q = rnd.quaternion(10.0);
        double s = rnd.uniform(-10.0, 10.0);
        BoostQuat expected = toBoost(q) * s;
        double tol = scaled(kOpTol, abs(toBoost(q)) * std::fabs(s));

        CHECK_QUAT_CLOSE(q * static_cast<float>(s), expected, tol);

        Quaternion inPlace = q;
        Quaternion &ret = (inPlace *= static_cast<float>(s));
        BOOST_TEST(&ret == &inPlace);
        CHECK_QUAT_CLOSE(inPlace, expected, tol);
    }

    const Quaternion q(1, -2, 3, -4);
    CHECK_QUAT_CLOSE(q * 0.0f, Quaternion(0, 0, 0, 0), 0.0);
    CHECK_QUAT_CLOSE(q * 1.0f, q, 0.0);
    CHECK_QUAT_CLOSE(q * -1.0f, Quaternion(-1, 2, -3, 4), 0.0);
    CHECK_QUAT_CLOSE(q * 2.0f, Quaternion(2, -4, 6, -8), 0.0);
    CHECK_QUAT_CLOSE(q, Quaternion(1, -2, 3, -4), 0.0);  // operand untouched
}

BOOST_AUTO_TEST_CASE(ConjugateMatchesBoost) {
    Random rnd(4);
    for (int n = 0; n < 2000; n++) {
        Quaternion q = rnd.quaternion(10.0);
        Quaternion before = q;
        // Negation is exact, so no tolerance is needed.
        CHECK_QUAT_CLOSE(q.conj(), conj(toBoost(q)), 0.0);
        CHECK_QUAT_CLOSE(q, before, 0.0);
    }
    CHECK_QUAT_CLOSE(Quaternion(1, 2, 3, 4).conj(), Quaternion(1, -2, -3, -4), 0.0);
    CHECK_QUAT_CLOSE(Quaternion(5, 0, 0, 0).conj(), Quaternion(5, 0, 0, 0), 0.0);
}

BOOST_AUTO_TEST_CASE(NormIsTheEuclideanMagnitude) {
    // Beware the naming: Boost's norm() is the Cayley norm (the *squared*
    // magnitude). Our norm() is the Euclidean magnitude, which Boost calls abs().
    BOOST_TEST(Quaternion(0, 3, 4, 0).norm() == 5.0f);
    BOOST_TEST(Quaternion(2, 0, 0, 0).norm() == 2.0f);
    BOOST_TEST(Quaternion().norm() == 1.0f);
    BOOST_TEST(Quaternion(0, 0, 0, 0).norm() == 0.0f);
    BOOST_TEST(Quaternion(1, 2, 3, 4).norm() == std::sqrt(30.0f));

    Random rnd(5);
    for (int n = 0; n < 2000; n++) {
        Quaternion q = rnd.quaternion(10.0);
        double magnitude = abs(toBoost(q));
        BOOST_TEST(std::fabs(q.norm() - magnitude) <= scaled(kOpTol, magnitude));
        BOOST_TEST(std::fabs(q.norm() * q.norm() - norm(toBoost(q))) <= scaled(kOpTol, magnitude * magnitude));
    }
}

BOOST_AUTO_TEST_CASE(NormalizeMatchesBoost) {
    Random rnd(6);
    for (int n = 0; n < 2000; n++) {
        Quaternion q = rnd.quaternion(10.0);
        if (abs(toBoost(q)) < 0.5) {
            continue;  // normalize() of (nearly) zero is out of contract
        }
        BoostQuat expected = toBoost(q) / abs(toBoost(q));

        Quaternion &ret = q.normalize();
        BOOST_TEST(&ret == &q);
        CHECK_QUAT_CLOSE(q, expected, kOpTol);
        BOOST_TEST(std::fabs(q.norm() - 1.0) <= kOpTol);

        // Normalizing again changes nothing beyond rounding.
        Quaternion again = q;
        again.normalize();
        CHECK_QUAT_CLOSE(again, q, kOpTol);
    }

    Quaternion q(1, 2, 3, 4);
    q.normalize();
    float inv = 1.0f / std::sqrt(30.0f);
    CHECK_QUAT_CLOSE(q, Quaternion(1 * inv, 2 * inv, 3 * inv, 4 * inv), kOpTol);
}

BOOST_AUTO_TEST_CASE(DotProductIsTheFourDimensionalDotProduct) {
    BOOST_TEST(Quaternion(1, 2, 3, 4).dot_product(Quaternion(5, 6, 7, 8)) == 70.0f);
    BOOST_TEST(Quaternion(0, 1, 0, 0).dot_product(Quaternion(0, 0, 1, 0)) == 0.0f);
    BOOST_TEST(Quaternion(0, 1, 0, 0).dot_product(Quaternion(0, 1, 0, 0)) == 1.0f);

    Random rnd(7);
    for (int n = 0; n < 2000; n++) {
        Quaternion p = rnd.quaternion(10.0);
        Quaternion q = rnd.quaternion(10.0);
        // In Boost terms the 4-D dot product is the real part of p * conj(q).
        double expected = real(toBoost(p) * conj(toBoost(q)));
        double tol = scaled(kOpTol, abs(toBoost(p)) * abs(toBoost(q)));
        BOOST_TEST(std::fabs(p.dot_product(q) - expected) <= tol);
        BOOST_TEST(p.dot_product(q) == q.dot_product(p));
        BOOST_TEST(std::fabs(p.dot_product(p) - norm(toBoost(p))) <= scaled(kOpTol, norm(toBoost(p))));
    }
}

BOOST_AUTO_TEST_CASE(AlgebraicIdentitiesHold) {
    Random rnd(8);
    const Quaternion one;
    for (int n = 0; n < 1000; n++) {
        Quaternion p = rnd.quaternion(2.0);
        Quaternion q = rnd.quaternion(2.0);
        Quaternion r = rnd.quaternion(2.0);
        double ap = abs(toBoost(p)), aq = abs(toBoost(q)), ar = abs(toBoost(r));

        CHECK_QUAT_CLOSE(p * one, p, 0.0);
        CHECK_QUAT_CLOSE(one * p, p, 0.0);
        CHECK_QUAT_CLOSE((p * q) * r, p * (q * r), scaled(kChainTol, ap * aq * ar));
        CHECK_QUAT_CLOSE(p * (q + r), p * q + p * r, scaled(kChainTol, ap * (aq + ar)));
        CHECK_QUAT_CLOSE((p + q) * r, p * r + q * r, scaled(kChainTol, (ap + aq) * ar));
        CHECK_QUAT_CLOSE((p * q).conj(), q.conj() * p.conj(), scaled(kChainTol, ap * aq));
        BOOST_TEST(std::fabs((p * q).norm() - p.norm() * q.norm()) <= scaled(kChainTol, ap * aq));
        // p * conj(p) is the real number |p|^2.
        CHECK_QUAT_CLOSE(p * p.conj(), BoostQuat(norm(toBoost(p)), 0, 0, 0), scaled(kChainTol, ap * ap));
    }
}

BOOST_AUTO_TEST_SUITE_END()
