#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
 public:

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};

  struct collider {

    bool collision; // is there a collision?
    int time; // time collision happens

  };

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane;

  int s;

  float v;

  float a;

  float target_speed;

  int lanes_available;

  float max_acceleration;

  int goal_lane;

  int goal_s;

  string state;

  /**
   * Constructor
   */
  Vehicle();
  Vehicle(int lane, float s, float v, float a, string state = "CS");

  /**
   * Destructor
   */
  virtual ~Vehicle();

  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);

  /**
   * Provides the possible next states given the current state for the FSM discussed in the course,
   * with the exception that lane changes happen instantaneously, so LCL and LCR can only transition back to KL.
   */
  vector<string> successor_states();

  /**
   * Given a possible next state, generate the appropriate trajectory to realize the next state.
   */
  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);

  /**
   * Gets next timestep kinematics (position, velocity, acceleration) for a given lane.
   * Tries to choose the maximum velocity and acceleration, given other vehicle positions and accel/velocity constraints.
   */
  vector<float> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);

  /**
   * Generates a constant speed trajectory.
   */
  vector<Vehicle> constant_speed_trajectory();

  /**
   * Generates a keep lane trajectory.
   */
  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

  /**
   * Generate a trajectory preparing for a lane change.
   */
  vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  void increment(int dt);

  float position_at(int t);

  /**
   * @param rVehicle - updated if a vehicle is found
   * @return - true if a vehicle is found behind the current vehicle, false otherwise.
   */
  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle &rVehicle);

  /**
   * @param rVehicle - updated if a vehicle is found
   * @return - true if a vehicle is found ahead of the current vehicle, false otherwise.
   */
  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle &rVehicle);

  /**
   * Generates predictions for non-ego vehicles to be used in trajectory generation for the ego vehicle.
   */
  vector<Vehicle> generate_predictions(int horizon = 2);

  /**
   * Sets state and kinematics for ego vehicle using the last state of the trajectory.
   */
  void realize_next_state(vector<Vehicle> trajectory);

  /**
   * Called by simulator before simulation begins. Sets various parameters which will impact the ego vehicle.
   */
  void configure(vector<int> road_data);

};

#endif