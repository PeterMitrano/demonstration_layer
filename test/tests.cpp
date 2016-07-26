#include <gtest/gtest.h>
#include "demonstration_layer/feature.h"

#include <ros/ros.h>

namespace demonstration_layer
{
TEST(FeatureTest, BucketIndexTest)
{
  Feature f(0, 10, 10);
  for (int i=0;i<10;i++)
  {
    EXPECT_EQ(i, f.bucketIndexForValue(i));
  }
  EXPECT_EQ(1, f.bucketIndexForValue(1.0001));
  EXPECT_EQ(1, f.bucketIndexForValue(1.9999));
  EXPECT_EQ(9, f.bucketIndexForValue(10));

  ROS_WARN("The following two should print errors.");
  EXPECT_EQ(9, f.bucketIndexForValue(11));
  EXPECT_EQ(0, f.bucketIndexForValue(-1));

  f = Feature(5, 10, 100);
  EXPECT_EQ(0, f.bucketIndexForValue(5));
  EXPECT_EQ(0, f.bucketIndexForValue(5.01));
  EXPECT_EQ(1, f.bucketIndexForValue(5.06));
  EXPECT_EQ(1, f.bucketIndexForValue(5.06));
  EXPECT_EQ(98, f.bucketIndexForValue(9.94));
  EXPECT_EQ(99, f.bucketIndexForValue(9.950001));
  EXPECT_EQ(99, f.bucketIndexForValue(10));
}

TEST(FeatureTest, InitializeWeightTest)
{
  Feature f(0, 10, 10, 2);
}

TEST(FeatureTest, UpdateWeighTest)
{
}
}  //  namespace demonstration_layer

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
