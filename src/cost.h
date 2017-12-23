#ifndef COST_H
#define COST_H

/**
 * The cost increases with both the distance of intended lane from the goal and the distance of the final lane from the goal.
 * The cost of being out of the goal lane also becomes larger as vehicle approaches the goal.
 *
 * @param goal_lane
 * @param intended_lane
 * @param final_lane
 * @param distance_to_goal
 * @return
 */
float goal_distance_cost(int goal_lane, int intended_lane, int final_lane, float distance_to_goal);

#endif