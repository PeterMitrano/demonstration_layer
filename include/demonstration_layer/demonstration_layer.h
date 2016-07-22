#pragma once

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <demonstration_layer/DemonstrationLayerConfig.h>
#include <demonstration_layer/demo_pose_stamped.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <recovery_supervisor_msgs/Demo.h>
#include <recovery_supervisor_msgs/SimpleFloatArray.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread/shared_mutex.hpp>
#include <map>
#include <mutex>

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
  Feature(double min, double max, int bucket_count) : min_(min), max_(max), bucket_count_(bucket_count)
  {
    // initialize the first weight to 1 because that's the map cost
    // the rest start as 0
    auto it = bucket_to_weight_map_.begin();
    it->second = 1;
    it++;
    for (; it != bucket_to_weight_map_.end(); it++)
    {
      it->second = 0;
    }
  }

  double weightForValue(double feature_value)
  {
    int bucket_index = (feature_value - min_) / bucket_count_;
    auto weight = bucket_to_weight_map_.find(bucket_index);

    if (weight == bucket_to_weight_map_.end())
    {
      return 0;
    }
    else
    {
      return weight->second;
    }
  }

  void updateWeightForValue(double feature_value, double delta)
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

private:
  double min_;
  double max_;
  int bucket_count_;
  std::map<int, double> bucket_to_weight_map_;
};

/** @class MacroCell
 * @brief A macro cell is just a group of cells. Each macrocell has a set of
 * corresponding weights there is one weight for each bucket, and several
 * buckets for every feature. We only create a macro cell for areas that recieve
 * demonstration, so it's not a continuous grid.
 */
class MacroCell
{
public:
  MacroCell(unsigned int x, unsigned int y, unsigned int size) : x_(x), y_(y), size_(size)
  {
  }
  /** @brief computes the linear combination of weights
   * and values for features of a given state
   */
  int costGivenFeatures(int underlying_map_cost, recovery_supervisor_msgs::SimpleFloatArray feature_values)
  {
    return underlying_map_cost;
  }

  void updateWeights(bool increase, int underlying_map_cost, recovery_supervisor_msgs::SimpleFloatArray feature_values)
  {
    // the +1 accounts for the underlying map feature which isn't part of the message.
    if (feature_values.data.size() + 1 != features_.size())
    {
      ROS_ERROR("MacroCell has %lu features, but feature value message only has %lu",
          features_.size(), (feature_values.data.size() + 1));
      return;
    }

    // first handle updating weight of map cost
    //features_[0].updateWeightForValue(underlying_map_cost, 0);
  }

private:
  unsigned int x_;     // starting x in map frame
  unsigned int y_;     // starting y in map frame
  unsigned int size_;  // width and height in number of cells
  std::vector<Feature> features_;
};

/**
 * @class DemonstrationLayer
 * @brief A costmap layer for reducing the cost of specific paths.
 * It subscribes to a path topic, and is a costmap_2d layer plugin.
 */
class DemonstrationLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  static double learning_rate_;
  DemonstrationLayer();

  virtual void onInitialize();

  /**
   * @brief sets the bounds and intended costs. All cells are increased in cost to the value of no_demo_cost_.
   * Then, all squares where a demonstrated path has crossed are set to demo_cost_.
   */
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  /**
   * @brief sets intended costs. All cells with cost heigher than 253 are left alone,
   * since those are assumed to be from a lower layer (static layer usually).
   */
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();

private:
  bool new_demonstration_;
  unsigned int macro_cell_size_;

  mutable std::mutex update_mutex_;

  unsigned int map_width_;
  unsigned int map_height_;

  dynamic_reconfigure::Server<DemonstrationLayerConfig>* dsrv_;

  ros::Subscriber demo_sub_;

  /** @brief recieves feature vectors representing the current state */
  ros::Subscriber state_feature_sub_;

  recovery_supervisor_msgs::SimpleFloatArray latest_feature_values_;
  recovery_supervisor_msgs::Demo latest_demo_;

  // container for macrocells. When we get a demo, we need to find or create
  // the macrocells for various poses. So since we only ever lookup macrocells,
  // and only ever by their x/y, we can use a map
  typedef std::pair<int, int> key_t;
  typedef std::pair<key_t, MacroCell> pair_t;
  std::map<key_t, MacroCell> macrocell_map_;

  void macroCellExists(int x, int y, MacroCell* output);

  void demoCallback(const recovery_supervisor_msgs::Demo& msg);
  void reconfigureCB(demonstration_layer::DemonstrationLayerConfig& config, uint32_t level);
  void stateFeatureCallback(const recovery_supervisor_msgs::SimpleFloatArray& msg);

  /** @brief updates the weights for all the macrocells along a path, given a set of feature values */
  void updateCellWeights(nav_msgs::Path path, costmap_2d::Costmap2D& master_grid,
      recovery_supervisor_msgs::SimpleFloatArray feature_vector, bool increase);
};
}
