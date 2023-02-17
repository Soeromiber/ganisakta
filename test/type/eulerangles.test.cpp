#include "gtest/gtest.h"
#include "ganisakta/type/eulerangles.hpp"
#include "ganisakta/type/quaternion.hpp"

using namespace ganisakta::type;

TEST(EulerAnglesTest, FromQuaternionTest)
{
    Quaternion q(0, 0, 0.707, 0.707);
    EulerAngles ypr = EulerAngles::fromQuaternion(q);
    EXPECT_NEAR(ypr.roll, 0, 0.001);
    EXPECT_NEAR(ypr.pitch, 0, 0.001);
    EXPECT_NEAR(ypr.yaw, M_PI_2, 0.001);

    Quaternion q2(0.5, 0.5, 0.5, 0.5);
    EulerAngles ypr2 = EulerAngles::fromQuaternion(q2);
    EXPECT_NEAR(ypr2.roll, M_PI_2, 0.001);
    EXPECT_NEAR(ypr2.pitch, 0, 0.001);
    EXPECT_NEAR(ypr2.yaw, M_PI_2, 0.001);

    Quaternion q3(0, 0.707, 0.707, 0);
    EulerAngles ypr3 = EulerAngles::fromQuaternion(q3);
    EXPECT_NEAR(ypr3.roll, M_PI_2, 0.001);
    EXPECT_NEAR(ypr3.pitch, 0, 0.001);
    EXPECT_NEAR(ypr3.yaw, M_PI, 0.001);

    // Quaternion q3(0.707, 0, 0.707, 0);
    // EulerAngles ypr3 = EulerAngles::fromQuaternion(q3);
    // std::cerr << ypr3.roll << " " << ypr3.pitch << " " << ypr3.yaw << std::endl;
    // EXPECT_NEAR(ypr3.roll, 0, 0.001);
    // EXPECT_NEAR(ypr3.pitch, -M_PI_2, 0.001);
    // EXPECT_NEAR(ypr3.yaw, 0, 0.001);
}

TEST(EulerAnglesTest, ToEulerAnglesDegreeTest)
{
    EulerAngles ypr(0, M_PI / 4, M_PI / 2);
    EulerAngles ypr_deg = EulerAngles::toEulerAnglesDegree(ypr);
    EXPECT_DOUBLE_EQ(ypr_deg.roll, 0);
    EXPECT_DOUBLE_EQ(ypr_deg.pitch, 45);
    EXPECT_DOUBLE_EQ(ypr_deg.yaw, 90);
}

TEST(EulerAnglesTest, OperatorTest)
{
    EulerAngles ypr1(0, 1, 2);
    EulerAngles ypr2(3, 4, 5);
    EulerAngles ypr_sum = ypr1 + ypr2;
    EXPECT_DOUBLE_EQ(ypr_sum.roll, 3);
    EXPECT_DOUBLE_EQ(ypr_sum.pitch, 5);
    EXPECT_DOUBLE_EQ(ypr_sum.yaw, 7);
    
    EulerAngles ypr_diff = ypr2 - ypr1;
    EXPECT_DOUBLE_EQ(ypr_diff.roll, 3);
    EXPECT_DOUBLE_EQ(ypr_diff.pitch, 3);
    EXPECT_DOUBLE_EQ(ypr_diff.yaw, 3);
    
    EulerAngles ypr_scalar_mul = ypr1 * 2;
    EXPECT_DOUBLE_EQ(ypr_scalar_mul.roll, 0);
    EXPECT_DOUBLE_EQ(ypr_scalar_mul.pitch, 2);
    EXPECT_DOUBLE_EQ(ypr_scalar_mul.yaw, 4);
    
    EulerAngles ypr_scalar_div = ypr2 / 2;
    EXPECT_DOUBLE_EQ(ypr_scalar_div.roll, 1.5);
    EXPECT_DOUBLE_EQ(ypr_scalar_div.pitch, 2);
    EXPECT_DOUBLE_EQ(ypr_scalar_div.yaw, 2.5);
    
    ypr1 += ypr2;
    EXPECT_DOUBLE_EQ(ypr1.roll, 3);
    EXPECT_DOUBLE_EQ(ypr1.pitch, 5);
    EXPECT_DOUBLE_EQ(ypr1.yaw, 7);
    
    ypr2 -= ypr1;
    EXPECT_DOUBLE_EQ(ypr2.roll, 0);
    EXPECT_DOUBLE_EQ(ypr2.pitch, -1);
    EXPECT_DOUBLE_EQ(ypr2.yaw, -2);
    
    ypr1 *= 2;
    EXPECT_DOUBLE_EQ(ypr1.roll, 6);
    EXPECT_DOUBLE_EQ(ypr1.pitch, 10);
    EXPECT_DOUBLE_EQ(ypr1.yaw, 14);
    
    ypr2 /= 2;
    EXPECT_DOUBLE_EQ(ypr2.roll, 0);
    EXPECT_DOUBLE_EQ(ypr2.pitch, -0.5);
    EXPECT_DOUBLE_EQ(ypr2.yaw, -1);
    
    EulerAngles ypr3(0, 1, 2);
    EXPECT_TRUE(ypr1 == ypr1);
    EXPECT_FALSE(ypr1 == ypr2);
    EXPECT_TRUE(ypr1 != ypr2);
    EXPECT_FALSE(ypr1 != ypr1);
    EXPECT_DOUBLE_EQ(ypr1[0], 6);
    EXPECT_DOUBLE_EQ(ypr1[1], 10);
    EXPECT_DOUBLE_EQ(ypr1[2], 14);
}