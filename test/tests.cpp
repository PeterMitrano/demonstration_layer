#include <gtest/gtest.h>
#include "demonstration_layer/feature.h"
#include "demonstration_layer/macrocell.h"
#include <recovery_supervisor_msgs/GoalFeature.h>
#include <recovery_supervisor_msgs/PosTimeGoalFeature.h>

#include <ros/ros.h>
#include <cmath>
#include <cstdio>

namespace demonstration_layer
{
const float TEST_LEARNING_RATE = 0.1;

int randInt(int lo, int hi)
{
  return (int)(lo + ((hi - lo) * rand()/RAND_MAX));
}

float randFlt(float lo, float hi)
{
  return (lo + ((hi - lo) * rand()/RAND_MAX));
}

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

  int seed = time(NULL);
  printf("using seed: %i\n", seed);
  srand(seed);

  // weights start at zero, so outpout should be map cost
  for (int map_cost=0; map_cost < 256; map_cost++)
  {
    recovery_supervisor_msgs::PosTimeGoalFeature f;
    f.x = randFlt(-10,10);
    f.y = randFlt(-10,10);
    f.theta = randFlt(-M_PI, M_PI);
    f.goal = randInt(0,20);
    f.hour = randInt(0,24);

    EXPECT_FLOAT_EQ(map_cost, cell.rawCostGivenFeatures(map_cost, f));
  }
}

TEST(MacroCellTest, LearnDoorScenario)
{
  //the test here is to learn that the cell in front of a door
  //is worse when your goal is through that door
  MacroCell bad_cell(0,0, 1);

  // bad_cell is one of the cells that needs higher cost
  // when we are going through the door. "going through the door"
  // is desribed by the feature below. The values are fairly arbitrary
  recovery_supervisor_msgs::PosTimeGoalFeature going_through_door_f;
  going_through_door_f.x = -10;
  going_through_door_f.y = 2;
  going_through_door_f.theta = 0;
  going_through_door_f.goal = 1;
  going_through_door_f.hour = 13;

  // this feature is just some other feature, and cost for it shouldn't change
  recovery_supervisor_msgs::PosTimeGoalFeature some_other_f;
  some_other_f.x = -2;
  some_other_f.y = -2;
  some_other_f.theta = 1;
  some_other_f.goal = 2;
  some_other_f.hour = 12;

  recovery_supervisor_msgs::PosTimeGoalFeature some_other_f2(going_through_door_f);
  some_other_f2.x = 9;
  some_other_f2.goal = 3;
  some_other_f2.hour = 2;


  // these features are similiar to the learned feature but differ slightly
  recovery_supervisor_msgs::PosTimeGoalFeature x_f(going_through_door_f);
  x_f.x = 2;
  recovery_supervisor_msgs::PosTimeGoalFeature y_f(going_through_door_f);
  y_f.y = 5;
  recovery_supervisor_msgs::PosTimeGoalFeature goal_f(going_through_door_f);
  goal_f.goal = 12;

  // fairly arbitrary map cost
  int initial_map_cost = 23;
  float initial_cost = bad_cell.rawCostGivenFeatures(initial_map_cost, going_through_door_f);

  // we receieve the demonstration for this bad cell
  // after this, plugging in the same feature should
  // yield lower cost then initially
  bad_cell.updateWeights(true, initial_map_cost, going_through_door_f);
  float bad_cost = bad_cell.rawCostGivenFeatures(initial_map_cost, going_through_door_f);

  EXPECT_GT(bad_cost, initial_cost);
  EXPECT_GT(bad_cell.rawCostGivenFeatures(initial_map_cost, x_f), initial_cost);
  EXPECT_GT(bad_cell.rawCostGivenFeatures(initial_map_cost, y_f), initial_cost);
  EXPECT_GT(bad_cell.rawCostGivenFeatures(initial_map_cost, goal_f), initial_cost);
  EXPECT_FLOAT_EQ(bad_cell.rawCostGivenFeatures(initial_map_cost, some_other_f), initial_cost);
  EXPECT_FLOAT_EQ(bad_cell.rawCostGivenFeatures(initial_map_cost, some_other_f2), initial_cost);
}
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
