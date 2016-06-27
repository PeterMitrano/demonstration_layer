#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>

class DemoPoseStamped
{
  public:
    DemoPoseStamped(costmap_2d::Costmap2D* costmap) : DemoPoseStamped(costmap, geometry_msgs::PoseStamped())
    {
    }

    DemoPoseStamped(costmap_2d::Costmap2D* costmap, geometry_msgs::PoseStamped pose_stamped) : costmap(costmap), pose_stamped(pose_stamped)
    {
    }

    unsigned int getMapX() const
    {
      unsigned int map_x, map_y;
      costmap->worldToMap(pose_stamped.pose.position.x, pose_stamped.pose.position.y, map_x, map_y);
      return map_x;
    }

    unsigned int getMapY() const
    {
      unsigned int map_x, map_y;
      costmap->worldToMap(pose_stamped.pose.position.x, pose_stamped.pose.position.y, map_x, map_y);
      return map_y;
    }

    costmap_2d::Costmap2D* costmap;
    geometry_msgs::PoseStamped pose_stamped;
};

namespace std {

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
