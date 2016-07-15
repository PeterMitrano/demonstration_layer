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

  private_nh.param<int>("no_demo_cost", no_demo_cost_, 10);
  private_nh.param<double>("initial_growth_", initial_growth_, 1.0);

  dsrv_ = new dynamic_reconfigure::Server<demonstration_layer::DemonstrationLayerConfig>(private_nh);
  dynamic_reconfigure::Server<demonstration_layer::DemonstrationLayerConfig>::CallbackType cb;
  cb = boost::bind(&DemonstrationLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  demo_path_sub_ = nh.subscribe("complete_demo_path", 10, &DemonstrationLayer::demoPathCallback, this);
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
  no_demo_cost_ = config.no_demo_cost;
  initial_growth_ = config.initial_growth_;
}

void DemonstrationLayer::demoPathCallback(const nav_msgs::Path& msg)
{
  ROS_INFO_ONCE("demo path recieved!");

  // go through each points and trace along it to the next
  // adding all points in between (uses ray tracing)
  std::vector<geometry_msgs::PoseStamped>::size_type i = 0;
  while (i < msg.poses.size() - 1)
  {
    DemoPoseStamped p1(this, msg.poses[i], initial_growth_, no_demo_cost_);
    DemoPoseStamped p2(this, msg.poses[i + 1], initial_growth_, no_demo_cost_);

    LineIterator line(p1.getMapX(), p1.getMapY(), p2.getMapX(), p2.getMapY());
    while (line.isValid())
    {
      geometry_msgs::PoseStamped intermediate_pose;
      intermediate_pose.header.frame_id = p1.getFrameID();
      intermediate_pose.header.stamp = ros::Time::now();
      double x, y;
      mapToWorld(line.getX(), line.getY(), x, y);
      intermediate_pose.pose.position.x = x;
      intermediate_pose.pose.position.y = y;

      DemoPoseStamped demo_pose(this, intermediate_pose, initial_growth_, no_demo_cost_);

      // if the pose already exists in the set, reset it to 0
      auto current_pose = path_set_.find(demo_pose);
      if (current_pose != path_set_.end())
      {
        current_pose->seenAgain();
      }
      else
      {
        path_set_.insert(demo_pose);
      }

      line.advance();
    }

    i++;
  }
}

void DemonstrationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                      double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  // increase all free space squares to a constant
  for (unsigned int x = 0; x < map_width_; x++)
  {
    for (unsigned int y = 0; y < map_height_; y++)
    {
      // it seems that getCost is always initialized to zero
      // instead of taking the cost of the grid according to lower layers
      setCost(x, y, no_demo_cost_);
    }
  }

  std::unordered_set<DemoPoseStamped>::const_iterator it;
  std::stringstream ss;
  for (it = path_set_.begin(); it != path_set_.end(); it++)
  {
    ss << it->getMapX() << "," << it->getMapY() << "," << it->cost << "  ";
    setCost(it->getMapX(), it->getMapY(), it->cost);

    // update every pose's cost, increasing by it's growth
    it->update();
  }

  ROS_DEBUG_STREAM(ss.str());

  this->mapToWorld(map_width_, map_height_, *max_x, *max_y);
  this->mapToWorld(0, 0, *min_x, *min_y);
}

void DemonstrationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);

      // 253, 254, and 255 are reserved
      // everything else is a custom value
      if (master_grid.getCost(i, j) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        continue;
      }

      master_grid.setCost(i, j, costmap_[index]);
    }
  }
}
}
