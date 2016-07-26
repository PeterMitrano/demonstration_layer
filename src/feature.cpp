#include "demonstration_layer/feature.h"
#include <ros/ros.h>

namespace demonstration_layer
{
Feature::Feature(double min, double max, int bucket_count, double initial_weight_for_new_buckets)
  : min_(min), max_(max), bucket_count_(bucket_count)
{
  initial_weight_for_new_buckets_ = initial_weight_for_new_buckets;
}

Feature::Feature(double min, double max, int bucket_count) : Feature(min, max, bucket_count, 0)
{
}

int Feature::bucketIndexForValue(double feature_value)
{
  if (feature_value > max_)
  {
    ROS_ERROR("feature_value of %f is greater than max of %f. Using max instead", feature_value, max_);
    feature_value = max_;
  }
  else if (feature_value < min_)
  {
    ROS_ERROR("feature_value of %f is less tan min of %f. Using max instead", feature_value, min_);
    feature_value = min_;
  }

  // our buckets are normally [a,b) but the last one must be [a,b]
  if (fabs(max_ - feature_value) < 0.00001)
  {
    return bucket_count_ - 1;
  }
  return (feature_value - min_) / ((max_ - min_) / bucket_count_);
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
}
