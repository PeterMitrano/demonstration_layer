#include <demonstration_layer/demonstration_layer.h>
#include <demonstration_layer/line_iterator.h>
#include <demonstration_layer_msgs/MacroCellCost.h>
#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <cmath>
#include <sstream>

PLUGINLIB_EXPORT_CLASS(demonstration_layer::DemonstrationLayer, costmap_2d::Layer)

namespace demonstration_layer
{
DemonstrationLayer::DemonstrationLayer() : new_demonstration_(false), min_cost_learned_(0)
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
  cached_costs_ = new unsigned int*[map_width_];
  for (unsigned int i = 0; i < map_width_; i++)
  {
    cached_costs_[i] = new unsigned int[map_height_];
  }

  {
    int macro_cell_size_tmp;
    double feature_timeout_tmp;
    private_nh.param<double>("learning_rate", MacroCell::learning_rate_, 0.1);
    private_nh.param<int>("macro_cell_size", macro_cell_size_tmp, 4);
    private_nh.param<double>("feature_timeout", feature_timeout_tmp, 2);  // seconds

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

  costs_pub_ = private_nh.advertise<demonstration_layer_msgs::Costs>("costs", 10, true);

  demo_sub_ = nh.subscribe("demo", 10, &DemonstrationLayer::demoCallback, this);
  state_feature_sub_ = private_nh.subscribe("state_feature", 10, &DemonstrationLayer::stateFeatureCallback, this);
  clear_service_ = private_nh.advertiseService("clear", &DemonstrationLayer::clearCallback, this);
  cost_service_ = private_nh.advertiseService("cost", &DemonstrationLayer::costCallback, this);

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
  Costmap2D* master = layered_costmap_->getCostmap();
  *min_x = master->getOriginX();
  *min_y = master->getOriginY();
  mapToWorld(map_width_, map_height_, *max_x, *max_y);
}

bool DemonstrationLayer::costCallback(demonstration_layer_msgs::CostRequest& request, demonstration_layer_msgs::CostResponse & response)
{
  MacroCell* macrocell = nullptr;
  int macrocell_x = request.cell_x / macro_cell_size_;
  int macrocell_y = request.cell_y / macro_cell_size_;

  macroCellExists(macrocell_x, macrocell_y, &macrocell);
  if (macrocell != nullptr)
  {
    response = macrocell->costResponse(request);
  }
  else
  {
    response.exists = false;
    response.cost = 0;
  }

  return true;
}

bool DemonstrationLayer::clearCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response)
{
  update_mutex_.lock();
  macrocell_map_.clear();
  update_mutex_.unlock();
  return true;
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

demonstration_layer_msgs::Costs DemonstrationLayer::buildCostsMsg(std::map<key_t, MacroCell*> macrocell_map_)
{
  demonstration_layer_msgs::Costs msg;
  for (const auto pair : macrocell_map_)
  {
    const auto macrocell = pair.second;
    demonstration_layer_msgs::MacroCellCost cell;
    cell.x = macrocell->getX();
    cell.y = macrocell->getY();
    cell.size = macrocell->getSize();
    cell.costs = macrocell->costsMsg();
    msg.cells.push_back(cell);
  }
  return msg;
}

void DemonstrationLayer::updateCellCosts(nav_msgs::Path path, costmap_2d::Costmap2D& master_grid,
                                           recovery_supervisor_msgs::GoalFeature feature_vector, bool increase)
{
  // iterate over cells in the odom path that aren't in demo path.
  // we want to increase the costs by some fraction LEARNING_RATE_ of their inputs
  geometry_msgs::PoseStamped prev_pose;
  geometry_msgs::PoseStamped pose;
  for (size_t i = 1; i < path.poses.size(); i++)
  {
    pose = path.poses[i];
    prev_pose = path.poses[i - 1];

    unsigned int prev_mapx, prev_mapy, mapx, mapy;
    worldToMap(prev_pose.pose.position.x, prev_pose.pose.position.y, prev_mapx, prev_mapy);
    worldToMap(pose.pose.position.x, pose.pose.position.y, mapx, mapy);

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

      macrocell->updateCosts(increase, underlying_map_cost, feature_vector);
      line.advance();
    }
  }
}

void DemonstrationLayer::renormalizeLearnedCosts(int min_i, int max_i, int min_j, int max_j,
                                                 costmap_2d::Costmap2D& master_grid)
{
  // First normalize the costs so they're between 0 and 128
  min_cost_learned_ = 0;
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int cost = master_grid.getCost(i, j);

      // if the cell is part of a macrocell, calculate its cost based on the
      // features & costs. Also make sure we're basing this off recent data.
      int mx = i / macro_cell_size_;
      int my = j / macro_cell_size_;
      MacroCell* macrocell = nullptr;
      macroCellExists(mx, my, &macrocell);
      if (macrocell != nullptr)
      {
        // blindly cast to int! hooray!
        cost = (int)macrocell->rawCostGivenFeatures(cost, latest_feature_values_);
      }

      cached_costs_[i][j] = cost;
      if (cost < min_cost_learned_)
      {
        // we use this value to add to our cost in updateCosts
        min_cost_learned_ = cost;
      }
    }
  }
}

void DemonstrationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  update_mutex_.lock();
  // first handle updating all the costs
  if (new_demonstration_)
  {
    new_demonstration_ = false;

    for (auto feature_vector : latest_demo_.feature_values)
    {
      updateCellCosts(latest_demo_.odom_path, master_grid, feature_vector, true);
      updateCellCosts(latest_demo_.demo_path, master_grid, feature_vector, false);
      costs_pub_.publish(DemonstrationLayer::buildCostsMsg(macrocell_map_));
    }
  }

  // TODO: for efficiency, find a way to not do this every time
  renormalizeLearnedCosts(min_i, max_i, min_j, max_j, master_grid);
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int underlying_map_cost = master_grid.getCost(i, j);

      if (underlying_map_cost == costmap_2d::LETHAL_OBSTACLE || underlying_map_cost == costmap_2d::NO_INFORMATION ||
          underlying_map_cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        master_grid.setCost(i, j, underlying_map_cost);
      }
      else
      {
        int cost = cached_costs_[i][j];

        if (min_cost_learned_ < 0)
        {
          cost += std::abs(min_cost_learned_);
        }

        cost = std::min(std::max(cost, 0), 254);
        master_grid.setCost(i, j, cost);
      }
    }
  }
  update_mutex_.unlock();
}
}
