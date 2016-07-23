#include "demonstration_layer/macrocell.h"

#include <ros/ros.h>

namespace demonstration_layer
{
double MacroCell::learning_rate_ = 0.1;

MacroCell::MacroCell(unsigned int x, unsigned int y, unsigned int size, unsigned int number_of_features)
  : x_(x), y_(y), size_(size), number_of_features_(number_of_features)
{
  /** This is the sketchies part of the code. Keep in mind there should be
   * exactly number_of_features + 1 features. The +1 being for
   * underlayin_map_cost, which isn't sent as a feature_value since only the
   * costmap knows it.  The order matters.  I may eventually need to refactor a
   * ton of stuff because this is just disgusting, prone to error, and
   * impossible to change for other features. Not safe at all...
   */

  // underlaying map cost gets weight of 1
  // so that before any demos the cost is
  // exactly equal to the underlying cost
  Feature map_feature = Feature(0, 128, 128, 1);
  Feature robot_x_feature = Feature(0, 10, 100);                 // meters, global frame
  Feature robot_y_feature = Feature(0, 10, 100);                 // meters, global frame
  Feature robot_theta_feature = Feature(0, 2 * M_PI, M_PI / 4);  // rad, global frame

  features_.push_back(map_feature);
  features_.push_back(robot_x_feature);
  features_.push_back(robot_y_feature);
  features_.push_back(robot_theta_feature);
}

/** @brief computes the linear combination of weights
 * and values for features of a given state
 */
double MacroCell::rawCostGivenFeatures(int underlying_map_cost,
                                       recovery_supervisor_msgs::SimpleFloatArray feature_values)
{
  if (feature_values.data.size() != number_of_features_)
  {
    ROS_ERROR("MacroCell is looking for %i features, but feature value message only has %lu", number_of_features_,
              (feature_values.data.size()));
    return -1;
  }

  double cost = underlying_map_cost * features_[0].weightForValue(underlying_map_cost);

  for (unsigned int i = 0; i < number_of_features_; i++)
  {
    cost += feature_values.data[i] * features_[i].weightForValue(feature_values.data[i]);
  }

  // remember, this isn't normalized at all... so have fun!
  return cost;
}

void MacroCell::updateWeights(bool increase, int underlying_map_cost,
                              recovery_supervisor_msgs::SimpleFloatArray feature_values)
{
  if (feature_values.data.size() != number_of_features_)
  {
    ROS_ERROR("MacroCell is looking for %i features, but feature value message only has %lu", number_of_features_,
              (feature_values.data.size()));
    return;
  }

  double delta = increase ? MacroCell::learning_rate_ : -MacroCell::learning_rate_;
  // handle underlaying map cost
  features_[0].updateWeightForValue(underlying_map_cost, delta * underlying_map_cost);

  // then all the other ones
  for (unsigned int i = 0; i < number_of_features_; i++)
  {
    features_[i].updateWeightForValue(feature_values.data[i], delta * feature_values.data[i]);
  }
}
}
