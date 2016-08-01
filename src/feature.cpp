#include "demonstration_layer/feature.h"
#include <ros/ros.h>

namespace demonstration_layer
{
Feature::Feature(double bucket_size, double initial_weight_for_new_buckets)
  : initial_weight_for_new_buckets_(initial_weight_for_new_buckets), bucket_size_(bucket_size) {}

Feature::Feature(double bucket_size) : Feature(bucket_size, 0)
{
}

int Feature::bucketIndexForValue(double feature_value)
{
  return feature_value / bucket_size_;
}

double Feature::costForValue(double feature_value)
{
  int bucket_index = bucketIndexForValue(feature_value);
  auto weight = bucket_to_weight_map_.find(bucket_index);

  if (weight == bucket_to_weight_map_.end())
  {
    return feature_value * initial_weight_for_new_buckets_;
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
  int bucket_index = bucketIndexForValue(feature_value);
  auto weight = bucket_to_weight_map_.find(bucket_index);
  if (weight == bucket_to_weight_map_.end())
  {
    std::pair<int, val_t> new_mapping;
    val_t new_value;
    new_mapping.first = bucket_index;

    new_value.first = initial_weight_for_new_buckets_ + (delta * feature_value);

    // new biases start at 0, and 0 + delta = delta
    // so the bias just start at delta
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

Feature::val_t Feature::weightForValue(double feature_value)
{
  int bucket_index = bucketIndexForValue(feature_value);
  auto weight = bucket_to_weight_map_.find(bucket_index);
  if (weight == bucket_to_weight_map_.end())
  {
    return val_t(0, 0);
  }
  else
  {
    return weight->second;
  }
}
}
