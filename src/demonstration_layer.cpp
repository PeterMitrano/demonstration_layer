#include <demonstration_layer/demonstration_layer.h>
#include <demonstration_layer/line_iterator.h>
#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <sstream>

PLUGINLIB_EXPORT_CLASS(demonstration_layer::DemonstrationLayer, costmap_2d::Layer)

namespace demonstration_layer
{

double DemonstrationLayer::learning_rate_ = 0.1;

DemonstrationLayer::DemonstrationLayer() : new_demonstration_(false)
{
}

void DemonstrationLayer::onInitialize()
{
  ros::NodeHandle private_nh("~/" + name_);
  ros::NodeHandle nh;
  current_ = true;
  default_value_ = costmap_2d::NO_INFORMATION;
  matchSize();

  int macro_cell_size_tmp;
  private_nh.param<double>("learning_rate", learning_rate_, 0.1);
  private_nh.param<int>("macro_cell_size", macro_cell_size_tmp, 4);
  if (macro_cell_size_tmp < 1)
  {
    ROS_WARN("macro cell size can't be less than 1. Setting to 1");
    macro_cell_size_ = 1;
  }
  else
  {
    macro_cell_size_ = (unsigned int)macro_cell_size_tmp;
  }

  dsrv_ = new dynamic_reconfigure::Server<DemonstrationLayerConfig>(private_nh);
  dynamic_reconfigure::Server<DemonstrationLayerConfig>::CallbackType cb;
  cb = boost::bind(&DemonstrationLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  demo_sub_ = nh.subscribe("complete_demo_path", 10, &DemonstrationLayer::demoCallback, this);
  state_feature_sub_ = private_nh.subscribe("state_feature", 10, &DemonstrationLayer::stateFeatureCallback, this);
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
  learning_rate_ = config.learning_rate;
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

void DemonstrationLayer::demoCallback(const recovery_supervisor_msgs::Demo& msg)
{
  ROS_INFO_ONCE("Demos are being recieved");

  update_mutex_.lock();

  new_demonstration_ = true;
  latest_demo_ = msg;

  update_mutex_.unlock();
}

void DemonstrationLayer::stateFeatureCallback(const recovery_supervisor_msgs::SimpleFloatArray& msg)
{
  update_mutex_.lock();

  latest_feature_values_ = msg;

  update_mutex_.unlock();
}

void DemonstrationLayer::macroCellExists(int x, int y, MacroCell* output)
{
  auto macrocell_it = macrocell_map_.find(std::pair<int, int>(x, y));
  if (macrocell_it != macrocell_map_.end())
  {
    output = &macrocell_it->second;
  }
  else
  {
    output = nullptr;
  }
}

void DemonstrationLayer::updateCellWeights(nav_msgs::Path path, costmap_2d::Costmap2D& master_grid,
                                           recovery_supervisor_msgs::SimpleFloatArray feature_vector, bool increase)
{
  // iterate over cells in the odom path that aren't in demo path.
  // we want to increase the weights by some fraction LEARNING_RATE_ of their inputs
  for (auto pose : path.poses)
  {
    MacroCell* macrocell = nullptr;
    unsigned int mapx, mapy;
    worldToMap(pose.pose.position.x, pose.pose.position.y, mapx, mapy);
    int underlying_map_cost = master_grid.getCost(mapx, mapy);

    macroCellExists(mapx, mapy, macrocell);
    if (macrocell == nullptr)
    {
      // no demos here yet, so we need to initialize the new macrocell first
      unsigned int macrocell_x = mapx + macro_cell_size_ / 2;
      unsigned int macrocell_y = mapy + macro_cell_size_ / 2;
      MacroCell new_macrocell = MacroCell(macrocell_x, macrocell_y, macro_cell_size_);
      pair_t pair = pair_t(key_t(macrocell_x, macrocell_y), new_macrocell);
      macrocell_map_.insert(pair);

      ROS_INFO("Inserting new macro cell of size %i,(%i,%i) -> (%i,%i).", macro_cell_size_, mapx, mapy, macrocell_x,
               macrocell_y);
    }

    macrocell->updateWeights(increase, underlying_map_cost, feature_vector);
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
  }

  for (recovery_supervisor_msgs::SimpleFloatArray feature_vector : latest_demo_.feature_values)
  {
    updateCellWeights(latest_demo_.odom_path, master_grid, feature_vector, true);
    updateCellWeights(latest_demo_.demo_path, master_grid, feature_vector, false);
  }

  // then set the actual cost
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int cost = master_grid.getCost(i, j);

      // if the cell is part of a macrocell, calculate its cost based on the
      // features & weights
      MacroCell* macrocell = nullptr;
      macroCellExists(i, j, macrocell);
      if (macrocell != nullptr)
      {
        cost = macrocell->costGivenFeatures(cost, latest_feature_values_);
        ROS_INFO("cell at (%i,%i) is part of existing macrocell. Cost is %i", i, j, cost);
      }

      master_grid.setCost(i, j, cost);
    }
  }
  update_mutex_.unlock();
}
}
