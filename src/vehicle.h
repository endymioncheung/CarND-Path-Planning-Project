#ifndef VEHICLE
#define VEHICLE

#include <vector>
#include <map>
#include <string>

#include "constants.h"

using namespace std;

class Vehicle {
public:

  // Vehicle in Freenet Coordinates (longitudinal along the lane line)
  double s;       // s position
  double s_dot;   // s velocity
  double s_ddot;  // s acceleration

  // Vehicle in Freenet Coordinates (clock-wise perpendicular to the lane line)
  double d;       // d position
  double d_dot;   // d velocity
  double d_ddot;  // d acceleration

  // *********************************//
  //         Vehicle state            //
  // *********************************//
  // KL   = Keep Lane                 //
  // PLCL = Prepare Lane Change Left  //
  // LCL  = Lane Change Left          //
  // PLCR = Prepare Lane Change Right //
  // LCR  = Lane Change Right         //
  // CS   = Constant speed            //
  //        (not used for ego car)    //
  // *********************************//
  
  // Mapping for lane change
  map<string,int> lane_direction = {{"KL", 0}, {"CS", 0},
                                    {"PLCL", -1}, {"LCL", -1},
                                    {"LCR", 1}, {"PLCR", 1}};

  int lane;
  string state = "KL";
  vector<double> s_traj_coeffs, d_traj_coeffs;
  bool car_to_left, car_to_right, car_in_front;
  
  // Buffer distance between ego vehicle and vehicle ahead
  double preferred_buffer = VEHICLE_RADIUS + FOLLOW_DISTANCE;
  
  /**
  * Constructors
  */
  Vehicle();
  Vehicle(int lane, string state, double s, double s_dot, double s_ddot,
                                  double d, double d_dot, double d_ddot);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  // Vehicle functions
  
  // *********************************************** //
  //             Useful helper functions             //
  // *********************************************** //
  // Check other cars nearby ego vehicle
  void check_nearby_cars(vector<Vehicle> other_cars);
  
  // Update the vehicle state
  void realize_next_state(Vehicle &next_state);
  
  // DEBUGGING TOOLS (not used for final release)
  // string display_vehicle_state();
  // vector<Vehicle> constant_speed_trajectory(double duration);
  
  // *********************************************** //
  //               FSM implementation                //
  // *********************************************** //
  Vehicle get_current_car();
  
  // Generate predictions for (non-ego) vehicle
  vector<Vehicle> generate_predictions(double traj_start_time, double duration);
  
  // (Ego vehicle) trajectory generation
  vector<string> successor_states();
  Vehicle choose_next_state(map<int,vector<Vehicle>> &predictions, double traj_start_time, double duration);
  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> &predictions, double duration);
  
  // Calculate kinematics for selected trajectory
  vector<float>   get_kinematics(string state, map<int, vector<Vehicle>> &predictions, int lane, double duration);
  
  // Trajectory generation
  vector<Vehicle> lane_keep_trajectory(map<int, vector<Vehicle>> &predictions,double duration);
  vector<Vehicle> lane_change_trajectory(string state,map<int, vector<Vehicle>> &predictions,double duration);
  vector<Vehicle> prep_lane_change_trajectory(string state,map<int, vector<Vehicle>> &predictions,double duration);

  // Find the vehicles behind and ahead
  bool get_car_behind(map<int, vector<Vehicle>> &predictions, int lane, Vehicle &ref_vehicle);
  bool get_car_ahead( map<int, vector<Vehicle>> &predictions, int lane, Vehicle &ref_vehicle);

  // *********************************************** //
  // Non-FSM implementation (currently not in used)  //
  // *********************************************** //
  Vehicle get_nearest_leading_car_for_lane(int target_lane, map<int,vector<Vehicle>> predictions, double duration);
  Vehicle get_target_for_state(string state, map<int,vector<Vehicle>> &predictions,double duration);
  vector<vector<double>> generate_trajectory_path_for_target(Vehicle perturbed_target, double duration);
  
};

#endif
