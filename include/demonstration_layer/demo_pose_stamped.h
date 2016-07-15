#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>

/**
 * @class DemoPoseStamped
 * @brief A class capable of hashing that wraps geometry_msg::PoseStamped.
 * It can retrieve cell indeces used in costmap_2d.
 */
class DemoPoseStamped
{
public:
  mutable double cost;

  DemoPoseStamped(costmap_2d::Costmap2D* costmap, double growth, double max_cost)
    : DemoPoseStamped(costmap, geometry_msgs::PoseStamped(), growth, max_cost)
  {
  }
  DemoPoseStamped(costmap_2d::Costmap2D* costmap, geometry_msgs::PoseStamped pose_stamped, double growth,
                  double max_cost)
    : cost(0), growth_(growth), max_cost_(max_cost), costmap_(costmap), pose_stamped_(pose_stamped)
  {
  }

  std::string getFrameID()
  {
    return pose_stamped_.header.frame_id;
  }

  /** @brief get the column cell index (map x) of the pose */
  unsigned int getMapX() const
  {
    unsigned int map_x, map_y;
    costmap_->worldToMap(pose_stamped_.pose.position.x, pose_stamped_.pose.position.y, map_x, map_y);
    return map_x;
  }

  /** @brief get the row cell index (map y) of the pose */
  unsigned int getMapY() const
  {
    unsigned int map_x, map_y;
    costmap_->worldToMap(pose_stamped_.pose.position.x, pose_stamped_.pose.position.y, map_x, map_y);
    return map_y;
  }

  void update() const
  {
    cost = std::min(max_cost_, cost + growth_);
  }

  void seenAgain() const
  {
    cost = 0;

    if (growth_ < 1.0)
    {
      ROS_WARN("growth  for %i, %i was %f, which is less than 1!", getMapX(), getMapY(), growth_);
    }

    growth_ /= 2;
  }

private:
  mutable double growth_;
  mutable double max_cost_;
  costmap_2d::Costmap2D* costmap_;
  geometry_msgs::PoseStamped pose_stamped_;
};

namespace std
{
template <>
struct hash<DemoPoseStamped>
{
  std::size_t operator()(const DemoPoseStamped& pose) const
  {
    std::size_t seed = 0;

    boost::hash_combine(seed, pose.getMapX());
    boost::hash_combine(seed, pose.getMapY());

    return seed;
  }
};
}

bool operator==(const DemoPoseStamped& lhs, const DemoPoseStamped& rhs)
{
  return (lhs.getMapX() == rhs.getMapX()) && (lhs.getMapY() == rhs.getMapY());
}
