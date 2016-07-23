#pragma once

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
  Feature(double min, double max, int bucket_count);

  /** @brief initializes all weights to to some value*/
  Feature(double min, double max, int bucket_count, double initial_weight_for_new_buckets);

  double weightForValue(double feature_value);
  void updateWeightForValue(double feature_value, double delta);

private:
  double initial_weight_for_new_buckets_;
  double min_;
  double max_;
  int bucket_count_;
  std::map<int, double> bucket_to_weight_map_;
};
}
