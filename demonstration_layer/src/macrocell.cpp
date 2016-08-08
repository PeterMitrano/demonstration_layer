#include "demonstration_layer/macrocell.h"

#include <ros/ros.h>

namespace demonstration_layer
{
double MacroCell::learning_rate_ = 1;

// TODO: Consider passing in all the features
// this whole class/message thing in general is still super shitty.
MacroCell::MacroCell(unsigned int x, unsigned int y, unsigned int size)
  : x_(x)
  , y_(y)
  , size_(size)
  , xytheta_feature_(Feature(std::vector<float>{0.1, 0.1, M_PI / 8}))  // meters,meters,radians in global frame
  , stamp_feature_(Feature(1))                                          // hours
  , goal_feature_(Feature(1))                                           // goal number
{
}

demonstration_layer_msgs::CostResponse MacroCell::costResponse(demonstration_layer_msgs::CostRequest request)
{
  demonstration_layer_msgs::CostResponse response;
  response.exists = true;
  // we only care about *change* in cost, so use 0 for underlying map cost
  response.cost = rawCostGivenFeatures(0, request.input);
  response.xytheta_cost = xytheta_feature_.costForValue(std::vector<double>{request.input.x, request.input.y, request.input.theta});
  response.stamp_cost = stamp_feature_.costForValue(request.input.hour);
  response.goal_cost = goal_feature_.costForValue(request.input.goal);

  return response;
}

std::vector<demonstration_layer_msgs::FeatureCost> MacroCell::costsMsg()
{
  std::vector<demonstration_layer_msgs::FeatureCost> costs;

  demonstration_layer_msgs::FeatureCost xytheta_msg;
  xytheta_msg.name = "xytheta";
  xytheta_msg.buckets = xytheta_feature_.bucketsMsg();

  demonstration_layer_msgs::FeatureCost stamp_msg;
  stamp_msg.name = "stamp";
  stamp_msg.buckets = stamp_feature_.bucketsMsg();

  demonstration_layer_msgs::FeatureCost goal_msg;
  goal_msg.name = "goal";
  goal_msg.buckets = goal_feature_.bucketsMsg();

  costs.push_back(xytheta_msg);
  costs.push_back(stamp_msg);
  costs.push_back(goal_msg);

  return costs;
}

unsigned int MacroCell::getSize()
{
  return size_;
}

unsigned int MacroCell::getX()
{
  return x_;
}

unsigned int MacroCell::getY()
{
  return y_;
}

/** @brief computes the linear combination of costs
 * and values for features of a given state
 */
double MacroCell::rawCostGivenFeatures(int underlying_map_cost,
                                       recovery_supervisor_msgs::PosTimeGoalFeature feature_values)
{
  double cost = underlying_map_cost;
  cost += xytheta_feature_.costForValue(std::vector<double>{feature_values.x, feature_values.y, feature_values.theta});
  cost += goal_feature_.costForValue(feature_values.goal);
  cost += stamp_feature_.costForValue(feature_values.hour);
  return cost;
}

double MacroCell::rawCostGivenFeatures(int underlying_map_cost, recovery_supervisor_msgs::GoalFeature feature_values)
{
  double cost = underlying_map_cost;
  cost += goal_feature_.costForValue(feature_values.goal);
  return cost;
}

void MacroCell::updateCosts(bool increase, int underlying_map_cost,
                              recovery_supervisor_msgs::PosTimeGoalFeature feature_values)
{
  double delta = increase ? MacroCell::learning_rate_ : -MacroCell::learning_rate_;
  xytheta_feature_.updateCostForValue(std::vector<double>{feature_values.x, feature_values.y, feature_values.theta},
                                        delta);
  goal_feature_.updateCostForValue(feature_values.goal, delta);
  stamp_feature_.updateCostForValue(feature_values.hour, delta);
}

void MacroCell::updateCosts(bool increase, int underlying_map_cost,
                              recovery_supervisor_msgs::GoalFeature feature_values)
{
  double delta = increase ? MacroCell::learning_rate_ : -MacroCell::learning_rate_;
  goal_feature_.updateCostForValue(feature_values.goal, delta);
}

void MacroCell::zeroAllCosts()
{
  xytheta_feature_.zeroAllCosts();
  stamp_feature_.zeroAllCosts();
  goal_feature_.zeroAllCosts();
}
}
