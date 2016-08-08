#include "demonstration_layer/feature.h"
#include <ros/ros.h>

namespace demonstration_layer
{
Feature::Feature(std::vector<float> bucket_sizes) : bucket_sizes_(bucket_sizes)
{
}

Feature::Feature(float bucket_sizes) : Feature(std::vector<float>{bucket_sizes})
{
}

std::vector<demonstration_layer_msgs::BucketCost> Feature::bucketsMsg()
{
  std::vector<demonstration_layer_msgs::BucketCost> buckets;

  for (auto element : bucket_to_cost_map_)
  {
    std::vector<int> bucket_indeces = element.first;
    std::vector<float> feature_values;
    for (size_t i = 0; i < bucket_indeces.size(); i++)
    {
      feature_values.push_back(bucket_indeces[i] * bucket_sizes_[i]);
    }

    auto cost = element.second;
    demonstration_layer_msgs::BucketCost bucket_msg;
    bucket_msg.bucket_indeces = bucket_indeces;
    bucket_msg.feature_value = feature_values;
    bucket_msg.cost = cost;
    buckets.push_back(bucket_msg);
  }

  return buckets;
}

std::vector<int> Feature::bucketIndecesForValue(double feature_value)
{
  return bucketIndecesForValue(std::vector<double>{feature_value});
}

std::vector<int> Feature::bucketIndecesForValue(std::vector<double> feature_value)
{
  std::vector<int> indeces;
  indeces.reserve(feature_value.size());

  for (size_t i = 0; i < feature_value.size(); i++)
  {
    indeces.push_back(feature_value[i] / bucket_sizes_[i]);
  }

  return indeces;
}

double Feature::costForValue(double feature_value)
{
  return costForValue(std::vector<double>{feature_value});
}

double Feature::costForValue(std::vector<double> feature_value)
{
  std::vector<int> bucket_indeces = bucketIndecesForValue(feature_value);
  auto cost = bucket_to_cost_map_.find(bucket_indeces);

  if (cost == bucket_to_cost_map_.end())
  {
    return 0;
  }
  else
  {
    double c = cost->second;
    return c;
  }
}

void Feature::updateCostForValue(double feature_value, double delta)
{
  updateCostForValue(std::vector<double>{feature_value}, delta);
}

void Feature::updateCostForValue(std::vector<double> feature_value, double delta)
{
  std::vector<int> bucket_indeces = bucketIndecesForValue(feature_value);
  auto cost = bucket_to_cost_map_.find(bucket_indeces);
  if (cost == bucket_to_cost_map_.end())
  {
    std::pair<key_t, double> new_cost;
    new_cost.first = bucket_indeces;

    new_cost.second = delta;
    bucket_to_cost_map_.insert(new_cost);
  }
  else
  {
    cost->second += delta;
  }
}

void Feature::zeroAllCosts()
{
  // lol yea so this is probably the fatest way
  bucket_to_cost_map_.clear();
}
}
