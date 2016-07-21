#include <demonstration_layer/demonstration_layer.h>
#include <demonstration_layer/line_iterator.h>
#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <sstream>

PLUGINLIB_EXPORT_CLASS(demonstration_layer::DemonstrationLayer, costmap_2d::Layer)

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
  default_value_ = costmap_2d::NO_INFORMATION;
  matchSize();

  private_nh.param<int>("macro_cell_size", macro_cell_size_, 4);
  private_nh.param<double>("learning_rate", learning_rate_, 0.1);

  dsrv_ = new dynamic_reconfigure::Server<demonstration_layer::DemonstrationLayerConfig>(private_nh);
  dynamic_reconfigure::Server<demonstration_layer::DemonstrationLayerConfig>::CallbackType cb;
  cb = boost::bind(&DemonstrationLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  demo_sub_ = nh.subscribe("complete_demo_path", 10, &DemonstrationLayer::demoCallback, this);
}

void DemonstrationLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  map_width_ = master->getSizeInCellsX();
  map_height_ = master->getSizeInCellsY();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(),
            master->getOriginY());
}

void DemonstrationLayer::reconfigureCB(demonstration_layer::DemonstrationLayerConfig& config, uint32_t level)
{
  enabled_ = config.enabled;
  learning_rate_ = config.learning_rate;
  macro_cell_size_ = config.macro_cell_size;
}

void DemonstrationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                      double* max_x, double* max_y)
{
  if (!enabled_)
    return;
}

void DemonstrationLayer::demoCallback(const recovery_supervisor::Demo& msg)
{
  ROS_INFO_ONCE("Demos are being recieved");
}

void DemonstrationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  // iterate over cells in the actual plan that aren't in demo plan.
  // we want to decrease the weights by some fraction LEARNING_RATE_ of their inputs

  // iterate over cells in the demo plan that aren't in actual plan.
  // we want to increase the weights by some fraction LEARNING_RATE_ of their inputs

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int x0 = master_grid.getCost(i, j);
      master_grid.setCost(i, j, x0);
    }
  }
}
}
