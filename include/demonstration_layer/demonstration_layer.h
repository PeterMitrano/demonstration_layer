#pragma once

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <demonstration_layer/DemonstrationLayerConfig.h>
#include <demonstration_layer/demo_pose_stamped.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <recovery_supervisor/Demo.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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
  Feature(double min, double max, int bucket_count) : min_(min), max_(max), bucket_count_(bucket_count)
  {
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
 * buckets for every feature.
 */
class MacroCell
{
public:
  MacroCell();
  int x;     // starting x in map frame
  int y;     // starting y in map frame
  int size;  // width and height in number of cells

private:
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
  double learning_rate_;
  int macro_cell_size_;

  unsigned int map_width_;
  unsigned int map_height_;

  dynamic_reconfigure::Server<demonstration_layer::DemonstrationLayerConfig>* dsrv_;

  ros::Subscriber demo_sub_;

  void demoCallback(const recovery_supervisor::Demo& msg);

  void reconfigureCB(demonstration_layer::DemonstrationLayerConfig& config, uint32_t level);
};
}
