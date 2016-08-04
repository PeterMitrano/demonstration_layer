#pragma once

#include <demonstration_layer_msgs/BucketWeight.h>
#include <gtest/gtest_prod.h>
#include <map>
#include <vector>

namespace demonstration_layer
{
/* @class Feature
 * @brief features have buckets and buckets have weights. Because
 * demonstrations are sparce, most of the buckets will have no weight. To store
 * buckets efficiently, we only need to know the min, max, and number
 * divisions. So, each feature needs those three numbers. The feature also
 * needs a set of weights, corresponding to some buckets. For storing
 * efficiency, the weight and bucket index are store together as a member of
 * the feature.
 */
class Feature
{
public:
  /** @brief initializes all weights to zero */
  Feature(std::vector<double> bucket_size);
  Feature(double bucket_size);

  std::vector<demonstration_layer_msgs::BucketWeight> bucketsMsg();

  void updateWeightForValue(std::vector<double> feature_value, double delta);
  double costForValue(std::vector<double> feature_value);

  void updateWeightForValue(double feature_value, double delta);
  double costForValue(double feature_value);

  void zeroAllWeights();

private:
  std::vector<double> bucket_size_;

  // the first is the weight
  typedef std::vector<int> key_t;
  std::map<key_t, double> bucket_to_weight_map_;

  std::vector<int> bucketIndecesForValue(std::vector<double> feature_value);
  std::vector<int> bucketIndecesForValue(double feature_value);

  FRIEND_TEST(FeatureTest, BucketIndexTest);
  FRIEND_TEST(FeatureTest, InitializeWeightTest);
  FRIEND_TEST(FeatureTest, InitializeAndUpdateWeightTest);
  FRIEND_TEST(FeatureTest, UpdateWeightTest);
  FRIEND_TEST(MacroCellTest, RandZerosWeight);
};
}
