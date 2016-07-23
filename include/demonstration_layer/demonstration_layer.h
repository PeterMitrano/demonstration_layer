#pragma once

#include "demonstration_layer/feature.h"
#include "demonstration_layer/macrocell.h"

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
  bool new_demonstration_;
  unsigned int macro_cell_size_;
  unsigned int number_of_features_;

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
  typedef std::pair<key_t, MacroCell*> pair_t;
  std::map<key_t, MacroCell*> macrocell_map_;

  void macroCellExists(int x, int y, MacroCell** output);

  void demoCallback(const recovery_supervisor_msgs::Demo& msg);
  void reconfigureCB(demonstration_layer::DemonstrationLayerConfig& config, uint32_t level);
  void stateFeatureCallback(const recovery_supervisor_msgs::SimpleFloatArray& msg);

  /** @brief updates the weights for all the macrocells along a path, given a set of feature values */
  void updateCellWeights(nav_msgs::Path path, costmap_2d::Costmap2D& master_grid,
                         recovery_supervisor_msgs::SimpleFloatArray feature_vector, bool increase);
};
}
