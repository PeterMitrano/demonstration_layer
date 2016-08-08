#pragma once

#include <demonstration_layer_msgs/BucketCost.h>
#include <gtest/gtest_prod.h>
#include <map>
#include <vector>

namespace demonstration_layer
{
/* @class Feature
 * @brief features have buckets and buckets have costs. Because
 * demonstrations are sparce, most of the buckets will have no cost. To store
 * buckets efficiently, we only need to know the min, max, and number
 * divisions. So, each feature needs those three numbers. The feature also
 * needs a set of costs, corresponding to some buckets. For storing
 * efficiency, the cost and bucket index are store together as a member of
 * the feature.
 */
class Feature
{
public:
  /** @brief initializes all costs to zero */
  Feature(std::vector<float> bucket_sizes);
  Feature(float bucket_sizes);

  std::vector<demonstration_layer_msgs::BucketCost> bucketsMsg();

  void updateCostForValue(std::vector<double> feature_value, double delta);
  double costForValue(std::vector<double> feature_value);

  void updateCostForValue(double feature_value, double delta);
  double costForValue(double feature_value);

  void zeroAllCosts();

private:
  std::vector<float> bucket_sizes_;

  typedef std::vector<int> key_t;
  std::map<key_t, double> bucket_to_cost_map_;

  std::vector<int> bucketIndecesForValue(std::vector<double> feature_value);
  std::vector<int> bucketIndecesForValue(double feature_value);

  FRIEND_TEST(FeatureTest, BucketIndexTest);
  FRIEND_TEST(FeatureTest, InitializeCostTest);
  FRIEND_TEST(FeatureTest, InitializeAndUpdateCostTest);
  FRIEND_TEST(FeatureTest, UpdateCostTest);
  FRIEND_TEST(MacroCellTest, RandZerosCost);
};
}
