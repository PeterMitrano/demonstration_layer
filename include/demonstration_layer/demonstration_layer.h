#pragma once

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <demonstration_layer/DemonstrationLayerConfig.h>
#include <demonstration_layer/demo_pose_stamped.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <unordered_set>

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
  double initial_growth_;

  /** @brief the cost of a free cell where no demonstrated path has been */
  int no_demo_cost_;

  unsigned int map_width_;
  unsigned int map_height_;
  dynamic_reconfigure::Server<demonstration_layer::DemonstrationLayerConfig>* dsrv_;
  ros::Subscriber demo_path_sub_;
  std::unordered_set<DemoPoseStamped> path_set_;

  void demoPathCallback(const nav_msgs::Path& msg);
  void reconfigureCB(demonstration_layer::DemonstrationLayerConfig& config, uint32_t level);
};
}
