#pragma once

#include "demonstration_layer/feature.h"

#include <recovery_supervisor_msgs/SimpleFloatArray.h>
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

  MacroCell(unsigned int x, unsigned int y, unsigned int size, unsigned int number_of_features);
  /** @brief computes the linear combination of weights
   * and values for features of a given state
   */
  double rawCostGivenFeatures(int underlying_map_cost, recovery_supervisor_msgs::SimpleFloatArray feature_values);

  void updateWeights(bool increase, int underlying_map_cost, recovery_supervisor_msgs::SimpleFloatArray feature_values);

private:
  unsigned int x_;     // starting x in map frame
  unsigned int y_;     // starting y in map frame
  unsigned int size_;  // width and height in number of cells
  unsigned int number_of_features_;
  std::vector<Feature> features_;
};
}
