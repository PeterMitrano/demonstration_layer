#include <demonstration_layer/demonstration_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(demonstration_layer::DemonstrationLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace demonstration_layer
{
DemonstrationLayer::DemonstrationLayer()
{
}

void DemonstrationLayer::onInitialize()
{
  ros::NodeHandle private_nh("~/" + name_);
  ros::NodeHandle nh;
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(private_nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb =
      boost::bind(&DemonstrationLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  demo_path_sub_ = nh.subscribe("demo_path", 10, &DemonstrationLayer::demoPathCallback, this);
}

void DemonstrationLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(),
            master->getOriginY());
}

void DemonstrationLayer::reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level)
{
  enabled_ = config.enabled;
}

void DemonstrationLayer::demoPathCallback(const nav_msgs::Path& msg)
{
  ROS_INFO_ONCE("demo path recieved!");
  for (auto pose_stamped : msg.poses)
  {
    DemoPoseStamped demo_pose(layered_costmap_->getCostmap(), pose_stamped);
    path_set_.insert(demo_pose);
  }

  ROS_INFO("demo point set size: %lu", path_set_.size());
}

void DemonstrationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                      double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  // double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
  // unsigned int mx;
  // unsigned int my;
  // if(worldToMap(mark_x, mark_y, mx, my)){
  // setCost(mx, my, LETHAL_OBSTACLE);
  //}

  //*min_x = std::min(*min_x, mark_x);
  //*min_y = std::min(*min_y, mark_y);
  //*max_x = std::max(*max_x, mark_x);
  //*max_y = std::max(*max_y, mark_y);
}

void DemonstrationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  // go through the entire map and increase cost to non-zero value (such as 1)
  // go through all the points in all the paths and set their cost to 0.

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      // int index = getIndex(i, j);
      // if (costmap_[index] == NO_INFORMATION)
      // continue;
      // master_grid.setCost(i, j, costmap_[index]);
    }
  }
}

}  // end namespace
