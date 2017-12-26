#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

float calculate_cost(const Vehicle &vehicle, const map<int, vector<Vehicle>> &predictions, const vector<Vehicle> &trajectory);

/**
 * Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
 * Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
 * This function is very similar to what you have already implemented in the "Implement a Cost Function in C++" quiz.
 */
float goal_distance_cost(const Vehicle &vehicle, const vector<Vehicle> &trajectory, const map<int, vector<Vehicle>> &predictions, map<string, float> &data);

/**
 * Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed.
 * You can use the lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) function to determine the speed for a lane.
 * This function is very similar to what you have already implemented in the "Implement a Second Cost Function in C++" quiz.
 */
float inefficiency_cost(const Vehicle &vehicle, const vector<Vehicle> &trajectory, const map<int, vector<Vehicle>> &predictions, map<string, float> &data);

/**
 * All non ego vehicles in a lane have the same speed, so to get the speed limit for a lane, we can just find one vehicle in that lane.
 */
float lane_speed(const map<int, vector<Vehicle>> &predictions, int lane);

/**
 * Generate helper data to use in cost functions:
 * indended_lane: +/- 1 from the current lane if the ehicle is planning or executing a lane change.
 * final_lane: The lane of the vehicle at the end of the trajectory. The lane is unchanged for KL and PLCL/PLCR trajectories.
 * distance_to_goal: The s distance of the vehicle to the goal.
 *
 * Note that indended_lane and final_lane are both included to help differentiate between planning and executing a lane change in the cost functions.
 */
map<string, float> get_helper_data(const Vehicle &vehicle, const vector<Vehicle> &trajectory, const map<int, vector<Vehicle>> &predictions);

#endif