#include <demonstration_layer/demonstration_layer.h>
#include <demonstration_layer/line_iterator.h>
#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <sstream>

PLUGINLIB_EXPORT_CLASS(demonstration_layer::DemonstrationLayer, costmap_2d::Layer)

namespace demonstration_layer
{
DemonstrationLayer::DemonstrationLayer() : has_warned_(false), new_demonstration_(false)
{
}

void DemonstrationLayer::onInitialize()
{
  ros::NodeHandle private_nh("~/" + name_);
  ros::NodeHandle nh;
  current_ = true;
  default_value_ = costmap_2d::NO_INFORMATION;
  matchSize();

  // this has to come after matchSize
  min_cost_learned_ = std::numeric_limits<double>::max();
  max_cost_learned_ = std::numeric_limits<double>::min();
  learned_costs_ = new unsigned int*[map_width_];
  for (unsigned int i = 0; i < map_width_; i++)
  {
    learned_costs_[i] = new unsigned int[map_height_];
  }

  {
    int macro_cell_size_tmp;
    double feature_timeout_tmp;
    private_nh.param<double>("learning_rate", MacroCell::learning_rate_, 0.1);
    private_nh.param<int>("macro_cell_size", macro_cell_size_tmp, 4);
    private_nh.param<double>("feature_timeout", feature_timeout_tmp, 1);  // seconds

    if (feature_timeout_tmp <= 0)
    {
      ROS_WARN("feature_timeout can't be less than or equal to 0. Setting to 1");
      feature_timeout_tmp = 1;
    }
    feature_timeout_ = ros::Duration(feature_timeout_tmp);

    if (macro_cell_size_tmp < 1)
    {
      ROS_WARN("macro cell size can't be less than 1. Setting to 1");
      macro_cell_size_ = 1;
    }
    else
    {
      macro_cell_size_ = (unsigned int)macro_cell_size_tmp;
    }
  }

  dsrv_ = new dynamic_reconfigure::Server<DemonstrationLayerConfig>(private_nh);
  dynamic_reconfigure::Server<DemonstrationLayerConfig>::CallbackType cb;
  cb = boost::bind(&DemonstrationLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  demo_sub_ = nh.subscribe("demo", 10, &DemonstrationLayer::demoCallback, this);
  state_feature_sub_ = private_nh.subscribe("state_feature", 10, &DemonstrationLayer::stateFeatureCallback, this);

  ROS_INFO("MacroCell size: %i", macro_cell_size_);
  ROS_INFO("Learning rate: %f", MacroCell::learning_rate_);
}

void DemonstrationLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  map_width_ = master->getSizeInCellsX();
  map_height_ = master->getSizeInCellsY();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(),
            master->getOriginY());
}

void DemonstrationLayer::reconfigureCB(DemonstrationLayerConfig& config, uint32_t level)
{
  enabled_ = config.enabled;
  MacroCell::learning_rate_ = config.learning_rate;

  if (config.macro_cell_size < 1)
  {
    ROS_WARN("macro cell size can't be less than 1. Setting to 1");
    macro_cell_size_ = 1;
  }
  else
  {
    macro_cell_size_ = (unsigned int)config.macro_cell_size;
  }
}

void DemonstrationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                      double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  // for nwe we update everything. Eventually this should be based on which
  // macrocells receieve demonstrations and other fancy stuff.
  *min_x = 0;
  *min_y = 0;
  mapToWorld(map_width_, map_height_, *max_x, *max_y);
}

void DemonstrationLayer::demoCallback(const recovery_supervisor_msgs::GoalDemo& msg)
{
  ROS_INFO_ONCE("Demos are being recieved");

  update_mutex_.lock();

  new_demonstration_ = true;
  latest_demo_ = msg;

  update_mutex_.unlock();
}

void DemonstrationLayer::stateFeatureCallback(const recovery_supervisor_msgs::GoalFeature& msg)
{
  ROS_INFO_ONCE("State features are being recieved");
  update_mutex_.lock();

  latest_feature_time_ = ros::Time::now();
  latest_feature_values_ = msg;

  update_mutex_.unlock();
}

/** @brief checks if a macrocell exists
 * @param x the macrocell x (or map x / macro_cell_size_)
 * @param y the macrocell y (or map y / macro_cell_size_)
 */
void DemonstrationLayer::macroCellExists(int x, int y, MacroCell** output)
{
  auto macrocell_it = macrocell_map_.find(key_t(x, y));
  if (macrocell_it != macrocell_map_.end())
  {
    (*output) = macrocell_it->second;
  }
  else
  {
    (*output) = nullptr;
  }
}

void DemonstrationLayer::updateCellWeights(nav_msgs::Path path, costmap_2d::Costmap2D& master_grid,
                                           recovery_supervisor_msgs::GoalFeature feature_vector, bool increase)
{
  // iterate over cells in the odom path that aren't in demo path.
  // we want to increase the weights by some fraction LEARNING_RATE_ of their inputs
  geometry_msgs::PoseStamped prev_pose;
  geometry_msgs::PoseStamped pose;
  for (size_t i = 1; i < path.poses.size(); i++)
  {
    pose = path.poses[i];
    prev_pose = path.poses[i - 1];

    unsigned int prev_mapx, prev_mapy, mapx, mapy;
    worldToMap(prev_pose.pose.position.x, prev_pose.pose.position.y, prev_mapx, prev_mapy);
    worldToMap(pose.pose.position.x, pose.pose.position.y, mapx, mapy);

    ROS_INFO("%i,%i to %i,%i", prev_mapx, prev_mapy, mapx, mapy);
    LineIterator line(prev_mapx, prev_mapy, mapx, mapy);
    while (line.isValid())
    {
      int underlying_map_cost = master_grid.getCost(line.getY(), line.getY());
      unsigned int macrocell_x = line.getX() / macro_cell_size_;
      unsigned int macrocell_y = line.getY() / macro_cell_size_;

      MacroCell* macrocell = nullptr;
      macroCellExists(macrocell_x, macrocell_y, &macrocell);
      if (macrocell == nullptr)
      {
        // no demos here yet, so we need to initialize the new macrocell first
        macrocell = new MacroCell(macrocell_x, macrocell_y, macro_cell_size_);
        pair_t pair = pair_t(key_t(macrocell_x, macrocell_y), macrocell);
        macrocell_map_.insert(pair);
      }

      ROS_INFO("updating cell (%i,%i) of macrocell (%i,%i) of size %i", line.getX(), line.getY(), macrocell_x, macrocell_y,
          macro_cell_size_);
      macrocell->updateWeights(increase, underlying_map_cost, feature_vector);
      line.advance();
    }
  }
}

void DemonstrationLayer::renormalizeLearnedCosts(int min_i, int max_i, int min_j, int max_j,
                                                 costmap_2d::Costmap2D& master_grid)
{
  // First normalize the costs so they're between 0 and 128
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      float cost = master_grid.getCost(i, j);

      // if the cell is part of a macrocell, calculate its cost based on the
      // features & weights. Also make sure we're basing this off recent data.
      int mx = i / macro_cell_size_;
      int my = j / macro_cell_size_;
      MacroCell* macrocell = nullptr;
      macroCellExists(mx, my, &macrocell);
      if (macrocell != nullptr)
      {
        float learned_cost = macrocell->rawCostGivenFeatures(cost, latest_feature_values_);
        ROS_INFO("%i,%i, %.3f + %.3f", i, j, cost, learned_cost);
        learned_costs_[i][j] = cost;
      }

      if (cost < min_cost_learned_)
      {
        min_cost_learned_ = cost;
      }
      if (cost > max_cost_learned_)
      {
        max_cost_learned_ = cost;
      }
    }
  }
}

void DemonstrationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  update_mutex_.lock();
  // first handle updating all the weights
  if (new_demonstration_)
  {
    new_demonstration_ = false;

    for (recovery_supervisor_msgs::GoalFeature feature_vector : latest_demo_.feature_values)
    {
      updateCellWeights(latest_demo_.odom_path, master_grid, feature_vector, true);
      updateCellWeights(latest_demo_.demo_path, master_grid, feature_vector, false);
    }

    renormalizeLearnedCosts(min_i, max_i, min_j, max_j, master_grid);
  }

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int cost = master_grid.getCost(i, j);

      // if the cell is part of a macrocell, calculate its cost based on the
      // features & weights. Also make sure we're basing this off recent data.
      int mx = i / macro_cell_size_;
      int my = j / macro_cell_size_;
      MacroCell* macrocell = nullptr;
      macroCellExists(mx, my, &macrocell);
      ros::Duration dt = ros::Time::now() - latest_feature_time_;
      bool state_features_up_to_date = dt < feature_timeout_;
      if (macrocell != nullptr)
      {
        if (state_features_up_to_date)
        {
          int learned_cost = macrocell->rawCostGivenFeatures(cost, latest_feature_values_);
          cost = std::min(std::max(learned_cost, 0), 128);
          has_warned_ = false;
        }
        else
        {
          if (!has_warned_)
          {
            has_warned_ = true;
            ROS_WARN("State features are out of date by %.2fs. Are you publishing to /state_feature?", dt.toSec());
          }
        }
      }
      master_grid.setCost(i, j, cost);
    }
  }
  update_mutex_.unlock();
}
}
