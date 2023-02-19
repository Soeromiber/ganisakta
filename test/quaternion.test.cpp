#include <gtest/gtest.h>
#include "ganisakta/quaternion/quaternion.hpp"
#include "ganisakta/angle/angle.hpp"

using namespace ganisakta::quaternion;
using namespace ganisakta::angle;

TEST(QuaternionTest, ToEulerAngles)
{
    // create a quaternion with roll=0, pitch=0, yaw=0
    Quaternion q1(0.0, 0.0, 0.0, 1.0);
    Angle expected(0.0, 0.0, 0.0);
    Angle result = Quaternion::toEulerAngles(q1);
    EXPECT_NEAR(expected.roll, result.roll, 0.001);
    EXPECT_NEAR(expected.pitch, result.pitch, 0.001);
    EXPECT_NEAR(expected.yaw, result.yaw, 0.001);

    // create a quaternion with roll=pi/4, pitch=pi/4, yaw=pi/4
    Quaternion q2(0.5, 0.5, 0.5, 0.5);
    expected = Angle(1.5708, 0.0, 1.5708);
    result = Quaternion::toEulerAngles(q2);
    EXPECT_NEAR(expected.roll, result.roll, 0.001);
    EXPECT_NEAR(expected.pitch, result.pitch, 0.001);
    EXPECT_NEAR(expected.yaw, result.yaw, 0.001);

    // create a quaternion with roll=0, pitch=pi/2, yaw=0
    Quaternion q3(0.707, 0.0, 0.0, 0.707);
    expected = Angle(1.5708, 0.0, 0.0);
    result = Quaternion::toEulerAngles(q3);
    EXPECT_NEAR(expected.roll, result.roll, 0.001);
    EXPECT_NEAR(expected.pitch, result.pitch, 0.001);
    EXPECT_NEAR(expected.yaw, result.yaw, 0.001);
}

TEST(QuaternionTest, ArithmeticOperators)
{
    Quaternion q1(1.0, 2.0, 3.0, 4.0);
    Quaternion q2(2.0, 3.0, 4.0, 5.0);

    // Addition
    Quaternion expected = Quaternion(3.0, 5.0, 7.0, 9.0);
    Quaternion result = q1 + q2;
    ASSERT_EQ(expected, result);

    // Subtraction
    expected = Quaternion(-1.0, -1.0, -1.0, -1.0);
    result = q1 - q2;
    ASSERT_EQ(expected, result);

    // Scalar multiplication
    expected = Quaternion(2.0, 4.0, 6.0, 8.0);
    result = q1 * 2.0;
    ASSERT_EQ(expected, result);

    // Scalar division
    expected = Quaternion(0.5, 1.0, 1.5, 2.0);
    result = q1 / 2.0;

    // Compound addition
    expected = Quaternion(3.0, 5.0, 7.0, 9.0);
    q1 += q2;
    ASSERT_EQ(expected, q1);

    // Compound subtraction
    expected = Quaternion(1.0, 2.0, 3.0, 4.0);
    q1 -= q2;
    ASSERT_EQ(expected, q1);

    // Compound scalar multiplication
    expected = Quaternion(2.0, 4.0, 6.0, 8.0);
    q1 *= 2.0;
    ASSERT_EQ(expected, q1);

    // Compound scalar division
    expected = Quaternion(1.0, 2.0, 3.0, 4.0);
    q1 /= 2.0;
    ASSERT_EQ(expected, q1);
}

TEST(QuaternionTest, ComparisonOperators)
{
    Quaternion q1(1.0, 2.0, 3.0, 4.0);
    Quaternion q2(1.0, 2.0, 3.0, 4.0);
    Quaternion q3(5.0, 6.0, 7.0, 8.0);

    // Equality
    ASSERT_TRUE(q1 == q2);
    ASSERT_FALSE(q1 == q3);

    // Inequality
    ASSERT_FALSE(q1 != q2);
    ASSERT_TRUE(q1 != q3);
}

TEST(QuaternionTest, Conjugate)
{
    Quaternion q1(1.0, 2.0, 3.0, 4.0);
    Quaternion expected(-1.0, -2.0, -3.0, 4.0);
    Quaternion result = q1.conjugate();
    ASSERT_EQ(expected, result);
}

TEST(QuaternionTest, IndexOperator)
{
    Quaternion q1(1.0, 2.0, 3.0, 4.0);
    ASSERT_EQ(1.0, q1[0]);
    ASSERT_EQ(2.0, q1[1]);
    ASSERT_EQ(3.0, q1[2]);
    ASSERT_EQ(4.0, q1[3]);
}