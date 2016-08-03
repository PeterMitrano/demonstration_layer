#include "demonstration_layer/macrocell.h"

#include <ros/ros.h>

namespace demonstration_layer
{
double MacroCell::learning_rate_ = 2;

// TODO: Consider passing in all the features
// this whole class/message thing in general is still super shitty.
MacroCell::MacroCell(unsigned int x, unsigned int y, unsigned int size)
  : x_(x)
  , y_(y)
  , size_(size)
  , xytheta_feature_(Feature(std::vector<double>{0.1, 0.1, M_PI / 2}))  // meters,meters,radians in global frame
  , stamp_feature_(Feature(1))                                          // hours
  , goal_feature_(Feature(1))                                           // goal number
{
}

/** @brief computes the linear combination of weights
 * and values for features of a given state
 */
double MacroCell::rawCostGivenFeatures(int underlying_map_cost, recovery_supervisor_msgs::XYThetaFeature feature_values)
{
  double cost = underlying_map_cost;
  cost += xytheta_feature_.costForValue(std::vector<double>{feature_values.x, feature_values.y, feature_values.theta});

  // remember, this isn't normalized at all... so have fun!
  return cost;
}

double MacroCell::rawCostGivenFeatures(int underlying_map_cost, recovery_supervisor_msgs::GoalFeature feature_values)
{
  double cost = underlying_map_cost;
  cost += goal_feature_.costForValue(feature_values.goal);

  // remember, this isn't normalized at all... so have fun!
  return cost;
}

void MacroCell::updateWeights(bool increase, int underlying_map_cost,
                              recovery_supervisor_msgs::XYThetaFeature feature_values)
{
  double delta = increase ? MacroCell::learning_rate_ : -MacroCell::learning_rate_;
  xytheta_feature_.updateWeightForValue(std::vector<double>{feature_values.x, feature_values.y, feature_values.theta},
                                        delta);
}

void MacroCell::updateWeights(bool increase, int underlying_map_cost,
                              recovery_supervisor_msgs::GoalFeature feature_values)
{
  double delta = increase ? MacroCell::learning_rate_ : -MacroCell::learning_rate_;
  goal_feature_.updateWeightForValue(feature_values.goal, delta);
}

void MacroCell::zeroAllWeights()
{
  xytheta_feature_.zeroAllWeights();
  stamp_feature_.zeroAllWeights();
  goal_feature_.zeroAllWeights();
}
}
