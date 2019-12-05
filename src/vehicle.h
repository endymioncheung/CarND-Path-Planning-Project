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
  // CS   = Constant speed            //
  // KL   = Keep Lane                 //
  // PLCL = Prepare Lane Change Left  //
  // LCL  = Lane Change Left          //
  // PLCR = Prepare Lane Change Right //
  // LCR  = Lane Change Right         //
  // *********************************//
  
  // Mapping for lane change
  map<string,int> lane_direction = {{"PLCL", 1}, {"LCL", 1},
                                    {"LCR", -1}, {"PLCR", -1}};
  
  string state = "CS";
  // vector<string> available_states; // TBD to keep this in the vehicle class
  vector<double> s_traj_coeffs, d_traj_coeffs;
  bool car_to_left, car_to_right, car_in_front;
  
  int lane, target_lane, target_s, lanes_available;
//  float v, target_speed, a, max_acceleration;
  
  // Buffer distance between ego vehicle and vehicle ahead
  // Impact "keep lane" behavior
  int preferred_buffer = FOLLOW_DISTANCE;
  
  /**
  * Constructors
  */
  Vehicle();
  Vehicle(int lane, string state, double s, double s_dot, double s_ddot, double d, double d_dot, double d_ddot);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  // Vehicle functions
  vector<string> successor_states() ;
  void check_nearby_cars(vector<Vehicle> other_cars);
  Vehicle get_nearest_leading_car(int target_lane, map<int,vector<Vehicle>> predictions,
                                                   double duration);
  Vehicle get_target_for_state(string state, map<int,vector<Vehicle>> &predictions,double duration);
  vector<Vehicle> generate_predictions(double traj_start_time, double duration);
  vector<vector<double>> generate_trajectory_for_target(Vehicle perturbed_target, double duration);
  void realize_next_state(Vehicle &next_state);
  
  // DEBUGGING TOOLS (not used for final release)
  // string display_vehicle_state();
  
  // new from behaivor planner
  float s_position_at(Vehicle &vehicle, int t);
  float d_position_at(Vehicle &vehicle, int t);
  
//  vector<string>  successor_states();
  Vehicle choose_next_state(map<int,vector<Vehicle>> &predictions, double traj_start_time, double duration);
  
  vector<Vehicle> generate_trajectory(string state,
                                      map<int, vector<Vehicle>> &predictions);

  vector<float>   get_kinematics(map<int, vector<Vehicle>> &predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();
  vector<Vehicle> lane_keep_trajectory(map<int, vector<Vehicle>> &predictions);
  vector<Vehicle> lane_change_trajectory(string state,
                                         map<int, vector<Vehicle>> &predictions);
  vector<Vehicle> prep_lane_change_trajectory(string state,
                                              map<int, vector<Vehicle>> &predictions);

  bool get_vehicle_behind(map<int, vector<Vehicle>> &predictions, int lane,
                          Vehicle &ref_vehicle);
  bool get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, int lane,
                         Vehicle &ref_vehicle);

  vector<Vehicle> generate_predictions(int horizon=2);
  
};

#endif
