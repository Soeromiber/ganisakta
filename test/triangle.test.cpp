#include "gtest/gtest.h"
#include "ganisakta/triangle/triangle.hpp"

TEST(TriangleTest, DefaultConstructor) {
  ganisakta::type::Triangle t;
  EXPECT_EQ(t[0], 0);
  EXPECT_EQ(t[1], 0);
  EXPECT_EQ(t[2], 0);
  EXPECT_EQ(t[3], 0);
  EXPECT_EQ(t[4], 0);
  EXPECT_EQ(t[5], 0);
}

TEST(TriangleTest, Constructor) {
  ganisakta::type::Triangle t(3, 4, 5, 0, 0, 0);
  EXPECT_EQ(t[0], 3);
  EXPECT_EQ(t[1], 4);
  EXPECT_EQ(t[2], 5);
  EXPECT_NEAR(t[3], 0.6435, 0.0001);
  EXPECT_NEAR(t[4], 0.9273, 0.0001);
  EXPECT_NEAR(t[5], 1.5708, 0.0001);
}

TEST(TriangleTest, CalculateAll) {
  ganisakta::type::Triangle t(3, 4, 5, 0, 0, 0);
  t.calculateAll();
  EXPECT_NEAR(t[3], 0.6435, 0.0001);
  EXPECT_NEAR(t[4], 0.9273, 0.0001);
  EXPECT_NEAR(t[5], 1.5708, 0.0001);
}

TEST(TriangleTest, EqualsOperator) {
  ganisakta::type::Triangle t1(3, 4, 5, 0, 0, 0);
  ganisakta::type::Triangle t2(3, 4, 5, 0, 0, 0);
  EXPECT_TRUE(t1 == t2);
}

TEST(TriangleTest, NotEqualsOperator) {
  ganisakta::type::Triangle t1(3, 4, 5, 0, 0, 0);
  ganisakta::type::Triangle t2(3, 5, 4, 0, 0, 0);
  EXPECT_TRUE(t1 != t2);
}

TEST(TriangleTest, BracketOperator) {
  ganisakta::type::Triangle t(3, 4, 5, 0, 0, 0);
  EXPECT_EQ(t[0], 3);
  EXPECT_EQ(t[1], 4);
  EXPECT_EQ(t[2], 5);
  EXPECT_NEAR(t[3], 0.6435, 0.0001);
  EXPECT_NEAR(t[4], 0.9273, 0.0001);
  EXPECT_NEAR(t[5], 1.5708, 0.0001);
}