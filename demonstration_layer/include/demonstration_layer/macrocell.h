#pragma once

#include "demonstration_layer/feature.h"

#include <demonstration_layer_msgs/FeatureWeight.h>
#include <gtest/gtest_prod.h>
#include <recovery_supervisor_msgs/GoalFeature.h>
#include <recovery_supervisor_msgs/PosTimeGoalFeature.h>
#include <vector>

namespace demonstration_layer
{
/** @class MacroCell
 * @brief A macro cell is just a group of cells. Each macrocell has a set of
 * corresponding weights there is one weight for each bucket, and several
 * buckets for every feature. We only create a macro cell for areas that recieve
 * demonstration, so it's not a continuous grid.
 */
class MacroCell
{
public:
  static double learning_rate_;

  MacroCell(unsigned int x, unsigned int y, unsigned int size);

  /** @brief computes the linear combination of weights
   * and values for features of a given state
   */
  double rawCostGivenFeatures(int underlying_map_cost, recovery_supervisor_msgs::PosTimeGoalFeature feature_values);
  double rawCostGivenFeatures(int underlying_map_cost, recovery_supervisor_msgs::GoalFeature feature_values);

  std::vector<demonstration_layer_msgs::FeatureWeight> weightsMsg();

  void updateWeights(bool increase, int underlying_map_cost,
                     recovery_supervisor_msgs::PosTimeGoalFeature feature_values);
  void updateWeights(bool increase, int underlying_map_cost, recovery_supervisor_msgs::GoalFeature feature_values);

  unsigned int getSize();
  unsigned int getX();
  unsigned int getY();

  void zeroAllWeights();

private:
  unsigned int x_;     // starting x in map frame
  unsigned int y_;     // starting y in map frame
  unsigned int size_;  // width and height in number of cells
  Feature xytheta_feature_;
  Feature stamp_feature_;
  Feature goal_feature_;

  FRIEND_TEST(MacroCellTest, InitializeTest);
  FRIEND_TEST(MacroCellTest, LearnDoorScenario);
};
}
