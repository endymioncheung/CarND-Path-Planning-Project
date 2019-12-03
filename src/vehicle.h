#ifndef VEHICLE
#define VEHICLE

#include <vector>
#include <map>
#include <string>

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

  // Vehicle state (i.e. constant speed, keep lane, change lane to left/right)
  string state;
  vector<string> available_states;
  vector<double> s_traj_coeffs, d_traj_coeffs;

  /**
  * Constructors
  */
  Vehicle();
  Vehicle(double s, double s_dot, double s_ddot, double d, double d_dot, double d_ddot);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  // Vehicle functions
  void update_available_states(bool car_to_left, bool car_to_right);
  Vehicle get_nearest_leading_car(int target_lane, map<int,vector<vector<double>>> predictions,
                                                   double duration);
  Vehicle get_target_for_state(string state, map<int, vector<vector<double>>> predictions,
                                             double duration, bool car_in_front);
  vector<vector<double>> generate_predictions(double traj_start_time, double duration);
  vector<vector<double>> generate_trajectory_for_target(Vehicle perturbed_target, double duration);
  
  // DEBUGGING TOOLS (not used for final release)
  // string display_vehicle_state();
};

#endif
