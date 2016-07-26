#include <gtest/gtest.h>
#include "demonstration_layer/feature.h"

#include <ros/ros.h>
#include <cmath>

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
  Feature f(0, 1000, 10, 0);
  EXPECT_EQ(0, f.costForValue(0));
  EXPECT_EQ(0, f.costForValue(1));
  EXPECT_EQ(0, f.costForValue(100));

  f = Feature(0, 1000, 10, 1);
  EXPECT_EQ(0, f.costForValue(0));
  EXPECT_EQ(1, f.costForValue(1));
  EXPECT_EQ(100, f.costForValue(100));

  f = Feature(0, 1000, 10, 0.5);
  EXPECT_EQ(0, f.costForValue(0));
  EXPECT_EQ(0.5, f.costForValue(1));
  EXPECT_EQ(50, f.costForValue(100));

  f = Feature(0, 1000, 10, 5);
  EXPECT_EQ(0, f.costForValue(0));
  EXPECT_EQ(5, f.costForValue(1));
  EXPECT_EQ(500, f.costForValue(100));
}

TEST(FeatureTest, UpdateWeightTest)
{
  Feature f(-M_PI, M_PI, 8);
  EXPECT_FLOAT_EQ(0, f.costForValue(1));

  // the second arg (delta) is a signed learning_rate
  f.updateWeightForValue(1, 0.01);

  // bias and weight should up by .01 each
  EXPECT_FLOAT_EQ(0.02, f.costForValue(1));
  // the same should apply for other values in that bucket
  EXPECT_FLOAT_EQ(0.021, f.costForValue(1.1));
  // but not ones in different buckets
  EXPECT_FLOAT_EQ(0, f.costForValue(2));
  EXPECT_FLOAT_EQ(0, f.costForValue(-1));

  // and if we change the weight for another bucket
  // the others shouldn't bet messed up
  f.updateWeightForValue(2, -0.01);
  EXPECT_FLOAT_EQ(-0.05, f.costForValue(2));
  EXPECT_FLOAT_EQ(-0.052, f.costForValue(2.1));
  EXPECT_FLOAT_EQ(0.02, f.costForValue(1));
  EXPECT_FLOAT_EQ(0.021, f.costForValue(1.1));
  EXPECT_FLOAT_EQ(0, f.costForValue(-1));

  f.updateWeightForValue(1, -0.01);
  EXPECT_EQ(0, f.costForValue(1));
}
}  //  namespace demonstration_layer

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
