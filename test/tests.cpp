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
    EXPECT_EQ(i, f.bucketIndexForValue(i));
  }
  EXPECT_EQ(1, f.bucketIndexForValue(1.0001));
  EXPECT_EQ(1, f.bucketIndexForValue(1.9999));
  EXPECT_EQ(10, f.bucketIndexForValue(10));
  EXPECT_EQ(11, f.bucketIndexForValue(11));
  EXPECT_EQ(-1, f.bucketIndexForValue(-1));

  f = Feature(0.05);
  EXPECT_EQ(0, f.bucketIndexForValue(0));
  EXPECT_EQ(0, f.bucketIndexForValue(0.01));
  EXPECT_EQ(0, f.bucketIndexForValue(0.04));
  EXPECT_EQ(1, f.bucketIndexForValue(0.05));
  EXPECT_EQ(1, f.bucketIndexForValue(0.06));
  EXPECT_EQ(99, f.bucketIndexForValue(4.950));
  EXPECT_EQ(100, f.bucketIndexForValue(5));
}

TEST(FeatureTest, InitializeWeightTest)
{
  Feature f(100, 0);
  EXPECT_EQ(0, f.costForValue(0));
  EXPECT_EQ(0, f.costForValue(1));
  EXPECT_EQ(0, f.costForValue(100));

  f = Feature(100, 1);
  EXPECT_EQ(0, f.costForValue(0));
  EXPECT_EQ(1, f.costForValue(1));
  EXPECT_EQ(100, f.costForValue(100));

  f = Feature(100, 0.5);
  EXPECT_EQ(0, f.costForValue(0));
  EXPECT_EQ(0.5, f.costForValue(1));
  EXPECT_EQ(50, f.costForValue(100));

  f = Feature(100, 5);
  EXPECT_EQ(0, f.costForValue(0));
  EXPECT_EQ(5, f.costForValue(1));
  EXPECT_EQ(500, f.costForValue(100));
}

TEST(FeatureTest, InitializeAndUpdateWeightTest)
{
  Feature map_feature(1, 1);
  EXPECT_EQ(0, map_feature.costForValue(0));
  EXPECT_EQ(1, map_feature.costForValue(1));
  EXPECT_EQ(100, map_feature.costForValue(100));

  map_feature.updateWeightForValue(50, TEST_LEARNING_RATE);
  map_feature.updateWeightForValue(10, 0);
  EXPECT_GT(map_feature.costForValue(50), 50);

  map_feature.updateWeightForValue(50, -TEST_LEARNING_RATE);
  map_feature.updateWeightForValue(10, 0);
  EXPECT_FLOAT_EQ(map_feature.costForValue(50), 50);

  map_feature.updateWeightForValue(50, -TEST_LEARNING_RATE);
  map_feature.updateWeightForValue(10, 0);
  EXPECT_LT(map_feature.costForValue(50), 50);
}

TEST(FeatureTest, UpdateWeightTest)
{
  Feature f(M_PI/8);
  EXPECT_FLOAT_EQ(0, f.costForValue(1));

  float local_learning_rate = 0.01;

  // the second arg (delta) is a signed learning_rate
  f.updateWeightForValue(1, local_learning_rate);

  // bias and weight should up by .01 each
  EXPECT_FLOAT_EQ(0.02, f.costForValue(1));
  // the same should apply for other values in that bucket
  EXPECT_FLOAT_EQ(0.021, f.costForValue(1.1));
  // but not ones in different buckets
  EXPECT_FLOAT_EQ(0, f.costForValue(2));
  EXPECT_FLOAT_EQ(0, f.costForValue(-1));

  // and if we change the weight for another bucket
  // the others shouldn't bet messed up
  f.updateWeightForValue(2, -local_learning_rate);
  EXPECT_FLOAT_EQ(-0.05, f.costForValue(2));
  EXPECT_FLOAT_EQ(-0.052, f.costForValue(2.1));
  EXPECT_FLOAT_EQ(0.02, f.costForValue(1));
  EXPECT_FLOAT_EQ(0.021, f.costForValue(1.1));
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

TEST(MacroCellTest, LearnBigXIsGood)
{
  MacroCell cell = MacroCell(0, 0, 4);

  recovery_supervisor_msgs::XYThetaFeature zero_feature;
  zero_feature.x = 0;
  zero_feature.y = 0;
  zero_feature.theta = 0;

  recovery_supervisor_msgs::XYThetaFeature big_x_feature;
  big_x_feature.x = 10;
  big_x_feature.y = 3;
  big_x_feature.theta = M_PI;

  recovery_supervisor_msgs::XYThetaFeature not_others_feature;
  not_others_feature.x = 0;
  not_others_feature.y = 3;
  not_others_feature.theta = M_PI;

  int map_cost = 5;

  cell.updateWeights(true, map_cost, big_x_feature);
  cell.updateWeights(false, map_cost, not_others_feature);

  // 1 is a good value because in practice cost is an int from 0 to 128
  EXPECT_NEAR(cell.rawCostGivenFeatures(map_cost, zero_feature), map_cost, 1);
  EXPECT_NEAR(cell.rawCostGivenFeatures(map_cost, not_others_feature), map_cost, 1);
  EXPECT_GT((int)cell.rawCostGivenFeatures(map_cost, big_x_feature), map_cost);
}

TEST(MacroCellTest, RandZerosWeight)
{
  time_t seed = time(NULL);
  FILE *f = fopen("RandZerosWeightTest.log", "w");
  printf("Using seed time: %lu\n", seed);
  srand(seed);

  MacroCell cell = MacroCell(0, 0, 1);
  cell.learning_rate_ = 0.01;

  // getting a fairly random distrobution of a decent number of samples
  // should mean we learn near-zero weights
  for (int i = 0; i < 10000; i++)
  {
    bool random_direction = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) > 0.5;
    recovery_supervisor_msgs::XYThetaFeature feature;
    feature.x = 0;
    feature.y = 0;
    feature.theta = 0;

    // update weights
    cell.updateWeights(random_direction, 0, feature);
    fprintf(f, "%f, %f\n", cell.map_feature_.bucket_to_weight_map_[0].first,
            cell.map_feature_.bucket_to_weight_map_[0].second);
  }

  // 1 is reasonable tolerance because we want to use cost as an int
  for (int map_cost = 0; map_cost < 1; map_cost++)
  {
    auto weight = cell.map_feature_.weightForValue(map_cost);
    // kind arbitrarily small value here
    //EXPECT_NEAR(weight.first, 1, cell.learning_rate_ * 10);
    //EXPECT_NEAR(weight.second, 0, cell.learning_rate_ * 10);
  }

  fclose(f);
}
}  //  namespace demonstration_layer

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
