#include <gtest/gtest.h>
#include <ganisakta/angle/angle.utils.hpp>

using namespace ganisakta::angle;

TEST(AngleTest, NormalizeRadTest) {
    // Test with radian value greater than pi
    double rad = 4.5;
    double normalizedRad = radian::normalize(rad);
    EXPECT_NEAR(normalizedRad, -1.78318530718, 0.001);

    // Test with radian value less than -pi
    rad = -4.5;
    normalizedRad = radian::normalize(rad);
    EXPECT_NEAR(normalizedRad, 1.78318530718, 0.001);

    // Test with radian value between -pi and pi
    rad = 2.5;
    normalizedRad = radian::normalize(rad);
    EXPECT_NEAR(normalizedRad, 2.5, 0.001);

    // Test normalization of angles within the range [-pi, pi)
    EXPECT_NEAR(radian::normalize(0), 0, 0.001);
    EXPECT_NEAR(radian::normalize(M_PI), -M_PI, 0.001);
    EXPECT_NEAR(radian::normalize(-M_PI), -M_PI, 0.001);
    EXPECT_NEAR(radian::normalize(2 * M_PI), 0, 0.001);
    EXPECT_NEAR(radian::normalize(-2 * M_PI), 0, 0.001);
    EXPECT_NEAR(radian::normalize(3 * M_PI), -M_PI, 0.001);
    EXPECT_NEAR(radian::normalize(-3 * M_PI), -M_PI, 0.001);

    // Test normalization of angles outside the range [-pi, pi)
    EXPECT_NEAR(radian::normalize(4 * -M_PI), 0, 0.001);
    EXPECT_NEAR(radian::normalize(-4 * -M_PI), 0, 0.001);
}

TEST(AngleTest, NormalizeDegTest) {
    // Test with degree value greater than 180
    double deg = 540;
    double normalizedDeg = degree::normalize(deg);
    EXPECT_DOUBLE_EQ(normalizedDeg, -180);

    // Test with degree value less than -180
    deg = -540;
    normalizedDeg = degree::normalize(deg);
    EXPECT_DOUBLE_EQ(normalizedDeg, -180);

    // Test with degree value between -180 and 180
    deg = 120;
    normalizedDeg = degree::normalize(deg);
    EXPECT_DOUBLE_EQ(normalizedDeg, 120);

      // Test normalization of angles within the range [-180, 180)
    EXPECT_DOUBLE_EQ(degree::normalize(0), 0);
    EXPECT_DOUBLE_EQ(degree::normalize(180), -180);
    EXPECT_DOUBLE_EQ(degree::normalize(-180), -180);
    EXPECT_DOUBLE_EQ(degree::normalize(360), 0);
    EXPECT_DOUBLE_EQ(degree::normalize(-360), 0);
    EXPECT_DOUBLE_EQ(degree::normalize(540), -180);
    EXPECT_DOUBLE_EQ(degree::normalize(-540), -180);

    // Test normalization of angles outside the range [-180, 180)
    EXPECT_DOUBLE_EQ(degree::normalize(720), 0);
    EXPECT_DOUBLE_EQ(degree::normalize(-720), 0);
}

TEST(AngleTest, RadToDegTest) {
    double rad = 1.5;
    double deg = radian::toDegree(rad);
    EXPECT_NEAR(deg, 85.9437, 0.01);

    rad = -2.5;
    deg = radian::toDegree(rad);
    EXPECT_NEAR(deg, -143.239449, 0.01);
}

TEST(AngleTest, DegToRadTest) {
    double deg = 150;
    double rad = degree::toRadian(deg);
    EXPECT_NEAR(rad, 2.61799, 0.001);

    deg = -30;
    rad = degree::toRadian(deg);
    EXPECT_NEAR(rad, -0.523599, 0.001);
}
