#pragma once

#include <gtest/gtest_prod.h>
#include <map>

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
  Feature(double bucket_size);

  /** @brief initializes all weights to to some value*/
  Feature(double bucket_size, double initial_weight_for_new_buckets);

  void updateWeightForValue(double feature_value, double delta);
  double costForValue(double feature_value);
  void zeroAllWeights();

private:
  double initial_weight_for_new_buckets_;
  double bucket_size_;

  // the first is the weight, the second is the bias
  typedef std::pair<double, double> val_t;
  std::map<int, std::pair<double, double>> bucket_to_weight_map_;

  int bucketIndexForValue(double feature_value);
  val_t weightForValue(double feature_value);

  FRIEND_TEST(FeatureTest, BucketIndexTest);
  FRIEND_TEST(FeatureTest, InitializeWeightTest);
  FRIEND_TEST(FeatureTest, InitializeAndUpdateWeightTest);
  FRIEND_TEST(FeatureTest, UpdateWeightTest);
  FRIEND_TEST(MacroCellTest, RandZerosWeight);
};
}
