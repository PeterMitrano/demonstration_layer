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


double Feature::costForValue(double feature_value)
{
  int bucket_index = (feature_value - min_) / bucket_count_;
  auto weight = bucket_to_weight_map_.find(bucket_index);

  if (weight == bucket_to_weight_map_.end())
  {
    return initial_weight_for_new_buckets_;
  }
  else
  {
    // cost = feature_value * weight + bias
    double cost = feature_value * weight->second.first + weight->second.second;
    return cost;
  }
}

void Feature::updateWeightForValue(double feature_value, double delta)
{
  int bucket_index = (feature_value - min_) / bucket_count_;
  auto weight = bucket_to_weight_map_.find(bucket_index);

  if (weight == bucket_to_weight_map_.end())
  {
    std::pair<int, val_t> new_mapping;
    val_t new_value;
    new_mapping.first = bucket_index;
    // new weights start at 0, and 0 + delta = delta
    // so the weights just start at delta. the bias does the same
    new_value.first = delta * feature_value;
    new_value.second = delta;
    new_mapping.second = new_value;

    bucket_to_weight_map_.insert(new_mapping);
  }
  else
  {
    weight->second.first += delta * feature_value;
    weight->second.second += delta;
  }
}
}
