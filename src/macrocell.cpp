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
  , map_feature_(Feature(0, 128, 128, 128))     // map cost, initialize weights to 1
  , x_feature_(Feature(-20, 20, 1000))        // meters, global frame
  , y_feature_(Feature(-20, 20, 1000))        // meters, global frame
  , theta_feature_(Feature(0, 2 * M_PI, 16))  // rad, global frame
  , vx_feature_(Feature(0, 0.5, 10))          // meters/sec, global frame
  , vy_feature_(Feature(0, 0.1, 10))          // meters/sec, global frame
  , vtheta_feature_(Feature(0, 0.5, 10))      // rad/sec, global frame
  , stamp_feature_(Feature(0, 24, 24))         // rad, global frame
  , goal_feature_(Feature(0, 100, 100))         // 100 should be enough
{
}

/** @brief computes the linear combination of weights
 * and values for features of a given state
 */
double MacroCell::rawCostGivenFeatures(int underlying_map_cost, recovery_supervisor_msgs::XYThetaFeature feature_values)
{
  double cost = 0;
  cost += map_feature_.costForValue(underlying_map_cost);
  cost += x_feature_.costForValue(feature_values.x);
  cost += y_feature_.costForValue(feature_values.y);
  cost += theta_feature_.costForValue(feature_values.theta);

  // remember, this isn't normalized at all... so have fun!
  return cost;
}

double MacroCell::rawCostGivenFeatures(int underlying_map_cost, recovery_supervisor_msgs::GoalFeature feature_values)
{
  double cost = 0;
  cost += goal_feature_.costForValue(feature_values.goal);

  // remember, this isn't normalized at all... so have fun!
  return cost;
}

void MacroCell::updateWeights(bool increase, int underlying_map_cost,
                              recovery_supervisor_msgs::XYThetaFeature feature_values)
{
  double delta = increase ? MacroCell::learning_rate_ : -MacroCell::learning_rate_;
  map_feature_.updateWeightForValue(underlying_map_cost, delta);
  x_feature_.updateWeightForValue(feature_values.x, delta);
  y_feature_.updateWeightForValue(feature_values.y, delta);
  theta_feature_.updateWeightForValue(feature_values.theta, delta);
}

void MacroCell::updateWeights(bool increase, int underlying_map_cost,
                              recovery_supervisor_msgs::GoalFeature feature_values)
{
  double delta = increase ? MacroCell::learning_rate_ : -MacroCell::learning_rate_;
  goal_feature_.updateWeightForValue(feature_values.goal, delta);
}
}
