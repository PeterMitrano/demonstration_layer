#include "demonstration_layer/feature.h"

namespace demonstration_layer
{
Feature::Feature(double min, double max, int bucket_count, double initial_weight_for_new_buckets)
  : min_(min), max_(max), bucket_count_(bucket_count)
{
  initial_weight_for_new_buckets_  = initial_weight_for_new_buckets;
}

Feature::Feature(double min, double max, int bucket_count) : Feature(min, max, bucket_count, 0)
{
}

double Feature::weightForValue(double feature_value)
{
  int bucket_index = (feature_value - min_) / bucket_count_;
  auto weight = bucket_to_weight_map_.find(bucket_index);

  if (weight == bucket_to_weight_map_.end())
  {
    return initial_weight_for_new_buckets_;
  }
  else
  {
    return weight->second;
  }
}

void Feature::updateWeightForValue(double feature_value, double delta)
{
  int bucket_index = (feature_value - min_) / bucket_count_;
  auto weight = bucket_to_weight_map_.find(bucket_index);

  if (weight == bucket_to_weight_map_.end())
  {
    std::pair<int, double> new_value;
    new_value.first = bucket_index;
    new_value.second = delta;
    bucket_to_weight_map_.insert(new_value);
  }
  else
  {
    weight->second += delta;
  }
}
}
