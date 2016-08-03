#include "demonstration_layer/feature.h"
#include <ros/ros.h>

namespace demonstration_layer
{

Feature::Feature(std::vector<double> bucket_size) : bucket_size_(bucket_size)
{
}

Feature::Feature(double bucket_size) : Feature(std::vector<double>{bucket_size})
{
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
    indeces.push_back(feature_value[i] / bucket_size_[i]);
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
  auto weight = bucket_to_weight_map_.find(bucket_indeces);

  if (weight == bucket_to_weight_map_.end())
  {
    return 0;
  }
  else
  {
    double cost = weight->second;
    return cost;
  }
}

void Feature::updateWeightForValue(double feature_value, double delta)
{
  updateWeightForValue(std::vector<double>{feature_value}, delta);
}

void Feature::updateWeightForValue(std::vector<double> feature_value, double delta)
{
  std::vector<int> bucket_indeces = bucketIndecesForValue(feature_value);
  auto weight = bucket_to_weight_map_.find(bucket_indeces);
  if (weight == bucket_to_weight_map_.end())
  {
    std::pair<key_t, double> new_weight;
    new_weight.first = bucket_indeces;

    new_weight.second = delta;
    bucket_to_weight_map_.insert(new_weight);
  }
  else
  {
    weight->second += delta;
  }
}

void Feature::zeroAllWeights()
{
  // lol yea so this is probably the fatest way
  bucket_to_weight_map_.clear();
}
}
