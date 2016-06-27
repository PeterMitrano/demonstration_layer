#pragma once

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <demonstration_layer/demo_pose_stamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <unordered_set>

namespace demonstration_layer
{

class DemonstrationLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  DemonstrationLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  ros::Subscriber demo_path_sub_;
  std::unordered_set<DemoPoseStamped> path_set_;

  void demoPathCallback(const nav_msgs::Path& msg);
};
}
