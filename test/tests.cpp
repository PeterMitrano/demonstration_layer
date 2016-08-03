#include <gtest/gtest.h>
#include "demonstration_layer/feature.h"
#include "demonstration_layer/macrocell.h"

#include <ros/ros.h>
#include <cmath>
#include <cstdio>

namespace demonstration_layer
{
const float TEST_LEARNING_RATE = 0.1;

TEST(FeatureTest, BucketIndexTest)
{
  Feature f(1);
  for (int i = 0; i < 10; i++)
  {
    EXPECT_EQ(std::vector<int>{i}, f.bucketIndecesForValue(i));
  }
  EXPECT_EQ(std::vector<int>{1}, f.bucketIndecesForValue(1.0001));
  EXPECT_EQ(std::vector<int>{1}, f.bucketIndecesForValue(1.9999));
  EXPECT_EQ(std::vector<int>{10}, f.bucketIndecesForValue(10));
  EXPECT_EQ(std::vector<int>{11}, f.bucketIndecesForValue(11));
  EXPECT_EQ(std::vector<int>{-1}, f.bucketIndecesForValue(-1));

  f = Feature(0.05);
  EXPECT_EQ(std::vector<int>{0}, f.bucketIndecesForValue(0));
  EXPECT_EQ(std::vector<int>{0}, f.bucketIndecesForValue(0.01));
  EXPECT_EQ(std::vector<int>{0}, f.bucketIndecesForValue(0.04));
  EXPECT_EQ(std::vector<int>{1}, f.bucketIndecesForValue(0.05));
  EXPECT_EQ(std::vector<int>{1}, f.bucketIndecesForValue(0.06));
  EXPECT_EQ(std::vector<int>{99}, f.bucketIndecesForValue(4.950));
  EXPECT_EQ(std::vector<int>{100}, f.bucketIndecesForValue(5));
}

TEST(FeatureTest, UpdateWeightTest)
{
  Feature f(M_PI / 8);
  EXPECT_FLOAT_EQ(0, f.costForValue(1));

  float local_learning_rate = 0.01;

  // the second arg (delta) is a signed learning_rate
  f.updateWeightForValue(1, local_learning_rate);

  // bias and weight should up by .01 each
  EXPECT_FLOAT_EQ(0.01, f.costForValue(1));
  // the same should apply for other values in that bucket
  EXPECT_FLOAT_EQ(0.01, f.costForValue(1.1));
  // but not ones in different buckets
  EXPECT_FLOAT_EQ(0, f.costForValue(2));
  EXPECT_FLOAT_EQ(0, f.costForValue(-1));

  // and if we change the weight for another bucket
  // the others shouldn't bet messed up
  f.updateWeightForValue(2, -local_learning_rate);
  EXPECT_FLOAT_EQ(-0.01, f.costForValue(2));
  EXPECT_FLOAT_EQ(-0.01, f.costForValue(2.1));
  EXPECT_FLOAT_EQ(0.01, f.costForValue(1));
  EXPECT_FLOAT_EQ(0.01, f.costForValue(1.1));
  EXPECT_FLOAT_EQ(0, f.costForValue(-1));

  f.updateWeightForValue(1, -local_learning_rate);
  EXPECT_EQ(0, f.costForValue(1));
}

TEST(MacroCellTest, InitializeTest)
{
  MacroCell cell = MacroCell(0, 0, 4);

  recovery_supervisor_msgs::XYThetaFeature zero_feature;
  zero_feature.x = 0;
  zero_feature.y = 0;
  zero_feature.theta = 0;

  recovery_supervisor_msgs::XYThetaFeature big_x_feature;
  zero_feature.x = 10;
  zero_feature.y = 0;
  zero_feature.theta = 0;

  // until we update weights, it should simply be based on map cost
  EXPECT_EQ(cell.rawCostGivenFeatures(0, zero_feature), 0);
  EXPECT_EQ(cell.rawCostGivenFeatures(10, zero_feature), 10);
  EXPECT_EQ(cell.rawCostGivenFeatures(0, big_x_feature), 0);
  EXPECT_EQ(cell.rawCostGivenFeatures(10, big_x_feature), 10);
}
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
