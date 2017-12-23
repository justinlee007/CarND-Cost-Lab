#include <functional>
#include "cost.h"
#include "cmath"

using namespace std;

float inefficiency_cost(int target_speed, int intended_lane, int final_lane, vector<int> lane_speeds) {
  float speed_intended = lane_speeds[intended_lane];
  float speed_final = lane_speeds[final_lane];
  float cost = (float) ((2.0 * target_speed - speed_intended - speed_final) / target_speed);
  return cost;
}

float goal_distance_cost(int goal_lane, int intended_lane, int final_lane, float distance_to_goal) {
  int delta_d = (int) (2.0 * goal_lane - intended_lane - final_lane);
  float cost = 1 - exp(-(abs(delta_d) / distance_to_goal));
  return cost;
}