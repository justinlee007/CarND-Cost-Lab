#include <functional>
#include "cost.h"
#include "cmath"

using namespace std;

float goal_distance_cost(int goal_lane, int intended_lane, int final_lane, float distance_to_goal) {
  int delta_d = (int) (2.0 * goal_lane - intended_lane - final_lane);
  float cost = 1 - exp(-(abs(delta_d) / distance_to_goal));
  return cost;
}