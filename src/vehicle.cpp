#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <iterator>
#include <algorithm>

#include "vehicle.h"
#include "jmt.h"
#include "costs.h"

// Initialize Vehicle
Vehicle::Vehicle() {}

Vehicle::Vehicle(int lane, string state, double s, double s_dot, double s_ddot, double d, double d_dot, double d_ddot) {
  
  // Initalize the vehicle class with Freenet coordinates
  
  this->lane   = lane;
  this->state  = state;
  
  this->s      = s;
  this->s_dot  = s_dot;
  this->s_ddot = s_ddot;
  
  this->d      = d;
  this->d_dot  = d_dot;
  this->d_ddot = d_ddot;
}

Vehicle::~Vehicle() {}

Vehicle Vehicle::get_current_car() {

  // Return the current vehicle class as
  // a vehicle object
  return {Vehicle(this->lane,this->state,
                  this->s,this->s_dot,this->s_ddot,
                  this->d,this->d_dot,this->d_ddot)};
}

Vehicle Vehicle::choose_next_state(map<int, vector<Vehicle>> &predictions, double traj_start_time, double duration) {

  // Return the trajectory with the lowest cost

  /**
   *
   * @param A predictions map. This is a map of vehicle id keys with predicted
   *   vehicle trajectories as values. Trajectories are a vector of Vehicle
   *   objects representing the vehicle at the current timestep and one timestep
   *   in the future.
   * @output The best (lowest cost) trajectory corresponding to the next ego
   *   vehicle state.
   *
   * Functions that will be useful:
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * 2. generate_trajectory - Returns a vector of Vehicle objects representing
   *    a vehicle trajectory, given a state and predictions. Note that
   *    trajectory vectors might have size 0 if no possible trajectory exists
   *    for the state.
   * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.
   */

  // Evaluate the cost of the ego vehicle's each possible state
  // by determining the target state and generating trajectory
  // based on this target state

  vector<float> costs;
  vector<Vehicle> final_targets;

  // Find all possible states (for ego vehicle)
  vector<string> states = successor_states();
  
  for (string state: states) {
    // No FSM logic
//    Vehicle target = get_target_for_state(state, predictions, duration);
    
    // with FSM logic
    vector<Vehicle> trajectory = generate_trajectory(state,predictions,duration);
    Vehicle target = trajectory[1];
    vector<vector<double>> trajectory_path = generate_trajectory_path_for_target(target, duration);

    if (trajectory_path.size() != 0) {
      double cost = calculate_cost(trajectory_path, predictions);
      // cout << "total cost: " << cost << endl;
      costs.push_back(cost);
      final_targets.push_back(target);
    }
  }

  // Find the trajectory that has the lowest cost
  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);
  Vehicle best_target = final_targets[best_idx];
  cout << "Best state = " << best_target.state << "\t\tCosts = " << costs[best_idx] << endl;
  return best_target;
}

vector<string> Vehicle::successor_states() {
  
  // Provides the possible next states given the current state for the FSM
  //   discussed in the course, with the exception that lane changes happen
  //   instantaneously, so LCL and LCR can only transition back to KL.

  // "FSM" (Finite State Machine)
  // (https://en.wikipedia.org/wiki/Finite-state_machine)
  // ---------------------------------------------------------------------
  // States
  // ---------------------------------------------------------------------
  // "KL"          = Keep Lane
  // "LCL"/"LCR"   = Lane Change Left / Lane Change Right
  // "PLCL"/"PLCR" = Prepare Lane Change Left / Prepare Lane Change Right

  // Current vehicle state
  int  current_lane  = this->lane;
  string state       = this->state;
  bool car_to_left   = this->car_to_left;
  bool car_to_right  = this->car_to_right;
  
  // Always start FSM with the lane keep state
  vector<string> states;
  states.push_back("KL");
  
//  cout << "Lane #" << lane << endl << "State: " << state << endl;
  
  // Sucessor states
  if(state.compare("KL") == 0) {
//    cout << "State = KL" << endl;
      // Get ready to prepare to change lane
      // Allow to turn left if there is no car on the left lane
      if ((current_lane > 0) && !car_to_left) {
        //cout << "Ready to change to LEFT" << endl;
        states.push_back("LCL");
      }

      // Allow to turn right if there is no car on the right lane
      if ((current_lane < NUM_LANES-1) && !car_to_right) {
        //cout << "Ready to change to RIGHT" << endl;
        states.push_back("LCR");
      }
    
    
//    states.push_back("PLCL");
//    states.push_back("PLCR");
//  } else if (state.compare("PLCL") == 0) {
//    //cout << "State = PLCL" << endl;
//    // Allow change lane to left
//    // so long it is not in the outermost left lane
//    if ((current_lane > 0) && !car_to_left) {
//      states.push_back("PLCL");
//      states.push_back("LCL");
//    }
//  } else if (state.compare("PLCR") == 0) {
//    //cout << "State = PLCR" << endl;
//    // Allow change lane to right
//    // so long it is not in the outermost right lane
//    if ((current_lane < NUM_LANES-1) && !car_to_right){
//      states.push_back("PLCR");
//      states.push_back("LCR");
//    }
  }
  
  return states;
}

Vehicle Vehicle::get_target_for_state(string state, map<int,vector<Vehicle>> &predictions, double duration) {
  
  // Return target vehicle state
  // (position, velocity and acceleration) in Freenet coordinates

  // Current lane of ego vehicle
  int    current_lane  = this->d / LANE_WIDTH;
  bool   car_in_front  = this->car_in_front;
  
  double target_s_dot  = BELOW_SPEED_LIMIT*MPH2MPS; // [m/s]
  double target_s_ddot = 0; // Constant acceleration to minimize jerk
  double target_s      = this->s + (this->s_dot + target_s_dot) / 2 * duration;
  
  // Target lane of ego vehicle
  string target_state  = this->state;
  double target_d      = current_lane*LANE_WIDTH + LANE_WIDTH/2;
  double target_d_dot  = 0;
  double target_d_ddot = 0;
  
  // Return emergency stop for target vehicle state
  // when a car suddenly cuts in front of ego vehicle
  if (car_in_front) {
    target_s      = this->s;
    target_s_dot  = 0.0;
    target_s_ddot = (target_s_dot - this->s_dot) / (N_SAMPLES * DT);
    
    return Vehicle(current_lane, "KL",
                   target_s, target_s_dot, target_s_ddot,
                   target_d, target_d_dot, target_d_ddot);
  }
  
  // Determine the target lane
  if(state.compare("KL") == 0)
  {
    target_d = current_lane*LANE_WIDTH + LANE_WIDTH/2;
  }
  else if(state.compare("LCL") == 0)
  {
    // cout << "Calculate target d for LCL" << endl;
    target_d = (current_lane-1)*LANE_WIDTH + LANE_WIDTH/2;
  }
  else if(state.compare("LCR") == 0)
  {
    // cout << "Calculate target d for LCR" << endl;
    target_d = (current_lane+1)*LANE_WIDTH + LANE_WIDTH/2;
  }
  int     target_lane       = target_d / LANE_WIDTH;
    
  // Find the leading vehicle in the target lane
  Vehicle leading_car       = get_nearest_leading_car(target_lane, predictions, duration);
  double  leading_car_s     = leading_car.s;
  double  leading_car_s_dot = leading_car.s_dot;
  
  // If the target vehicle poition is too close to the leading vehicle
  // then match the position and speed of target vehicle to leading vehicle.
  // Otherwise reduce the target speed by SPEED_DECREMENT [mph]
  
  double distance_leading_to_target = fabs(leading_car_s - target_s);
  if (leading_car_s > this->s && distance_leading_to_target < FOLLOW_DISTANCE) {
    if (distance_leading_to_target >= FOLLOW_DISTANCE/2) {
      target_s_dot = leading_car_s_dot;
    } else {
      target_s_dot -= SPEED_DECREMENT*MPH2MPS;
    }
    
    // cout << "NEARBY LEADING VEHICLE DETECTED!  ";
    // cout << ", lane: "  << target_lane
    //      << "s: "       << leading_car_s
    //      << ", speed: " << leading_car_s_dot << endl;
  }
  
  Vehicle target = Vehicle(target_lane, target_state,
                           target_s, target_s_dot, target_s_ddot,
                           target_d, target_d_dot, target_d_ddot);
  return target;
}

void Vehicle::check_nearby_cars(vector<Vehicle> other_cars) {
  
  // Prepare to change lane logic to check if there is a car
  // close to left or right of the ego vehicle and
  // restricting the next possible vehicle states
  
  // Check if there are other cars nearby the ego vehicle
  this->car_to_left  = false;
  this->car_to_right = false;
  this->car_in_front = false;
  
  for (Vehicle other_car: other_cars) {
    // Longitudinal and lateral distance
    // between the ego vehicle to any other vehicles
    double s_diff = fabs(other_car.s - this->s);
    double d_diff = other_car.d - this->d;

    if (other_car.s > this->s && s_diff < FOLLOW_DISTANCE) {
      if (-2 < d_diff && d_diff < 2) {
        this->car_in_front = true;
          //cout << "CAR AHEAD!!!        Distance to the car ahead: " << s_diff << " meters" << endl;
      } else if (s_diff < FOLLOW_DISTANCE) {
        if (-6 < d_diff && d_diff < -2) {
          this->car_to_left = true;
          //cout << "CAR ON THE LEFT!!   Distance to left car: " << s_diff << " meters" << endl;
        } else if (2 < d_diff && d_diff < 6) {
          this->car_to_right = true;
          //cout << "CAR ON THE RIGHT!!  Distance to right car: " << s_diff << " meters" << endl;
        }
      }
    }
  }
}

Vehicle Vehicle::get_nearest_leading_car(int target_lane,
                                        map<int,vector<Vehicle>> predictions,
                                        double duration) {
    
  // Return the nearest vehicle state in target lane
  // for a given set of predictions and duration of the trajectory

  double dt = duration / N_SAMPLES;
  
  double nearest_leading_car_s      = 99999;
  double nearest_leading_car_s_dot  = 0;
  double nearest_leading_car_s_ddot = 0;
  
  double nearest_leading_car_d      = 0;
  double nearest_leading_car_d_dot  = 0;
  double nearest_leading_car_d_ddot = 0;
  
  for (auto prediction : predictions) {
    vector<Vehicle> pred_traj = prediction.second;
    double pred_start_d = pred_traj[0].d;
    double pred_end_d   = pred_traj[pred_traj.size()-1].d;
    
    // Calculate the non-ego vehicles trajectory
    // if it's trajectory ends in the target lane of the ego vehicle
    int pred_lane = pred_start_d / LANE_WIDTH;
    if (pred_lane == target_lane) {
      double pred_start_s              = pred_traj[0].s;
      double pred_end_s                = pred_traj[pred_traj.size()-1].s;
      
      double next_to_pred_end_s        = pred_traj[pred_traj.size()-2].s;
      double second_last_to_pred_end_s = pred_traj[pred_traj.size()-3].s;
      double pred_s_dot                = (pred_end_s - next_to_pred_end_s) / dt;
      double last_pred_s_dot           = (next_to_pred_end_s - second_last_to_pred_end_s) / dt;
      double pred_s_ddot               = (pred_s_dot - last_pred_s_dot) / dt;
      
      double next_to_pred_end_d        = pred_traj[pred_traj.size()-2].d;
      double second_last_to_pred_end_d = pred_traj[pred_traj.size()-3].d;
      double pred_d_dot                = (pred_end_d - next_to_pred_end_d) / dt;
      double last_pred_d_dot           = (next_to_pred_end_d - second_last_to_pred_end_d) / dt;
      double pred_d_ddot               = (pred_d_dot - last_pred_d_dot) / dt;
      
      // Update the nearest vehicle speed and distance
      // if the non-ego vehicle is ahead of ego-vehicle
      if (pred_start_s >= this->s && pred_end_s < nearest_leading_car_s) {
        nearest_leading_car_s      = pred_end_s;
        nearest_leading_car_s_dot  = pred_s_dot;
        nearest_leading_car_s_ddot = pred_s_ddot;
        
        nearest_leading_car_d      = pred_end_d;
        nearest_leading_car_d_dot  = pred_d_dot;
        nearest_leading_car_d_ddot = pred_d_ddot;
      }
    }
  }
  
  // TODO: Replace the default constant speed with GNB classifier to predict whether it is "left/keep/right"
  string nearet_leading_car_state = "KL";
  
  Vehicle nearest_leading_car = {Vehicle(target_lane, nearet_leading_car_state,
                                         nearest_leading_car_s, nearest_leading_car_s_dot, nearest_leading_car_s_ddot,
                                         nearest_leading_car_d, nearest_leading_car_d_dot, nearest_leading_car_d_ddot)};
  return nearest_leading_car;
}

vector<Vehicle> Vehicle::generate_predictions(double traj_start_time, double duration) {
  // Generate N_SAMPLE numbers of predicted vehicle positions
  // for a given trajectory start time and duration (sec)

  int horizon = N_SAMPLES;
  vector<Vehicle> predictions;
  for(int i = 0; i < horizon; ++i)
  {
    // Simple constant speed prediction model
    // Assuming the vehicle is travelling at constant speed at the end of previous path
    // and the heading same direction as the lane line
    double dt     = traj_start_time + (i * duration/N_SAMPLES);
    double next_s = this->s + this->s_dot * dt;
//    double next_d = this->d;
    double next_d = this->d + this->d_dot * dt;
    int next_lane = next_d / LANE_WIDTH;
    
    // TODO: Replace the default constant speed with GNB classifier to predict whether it is "left/keep/right"
    string next_state = "KL";
    
    Vehicle prediction = Vehicle(next_lane,next_state,next_s,this->s_dot,0,next_d,this->d_dot,0);
    predictions.push_back(prediction);
  }
  return predictions;
}

vector<vector<double>> Vehicle::generate_trajectory_path_for_target(Vehicle target, double duration) {
  
  // Generate Jerk-Minimized Trajectory (JMT) in Frenet coordinates
  // that connects the current (ego) vehicle state to target vehicle state for a given duration
  // with N_SAMPLES number of trajectory points
  
  // Current and target Freenet coordinate for JMT generation
  vector<double> current_s = {this->s,   this->s_dot,  this->s_ddot};
  vector<double> current_d = {this->d,   this->d_dot,  this->d_ddot};
  vector<double> target_s  = {target.s, target.s_dot, target.s_ddot};
  vector<double> target_d  = {target.d, target.d_dot, target.d_ddot};
  
  // Calculate JMT coefficients using the quintic polynomial solver
  this->s_traj_coeffs = calculate_JMT_coeffs(current_s, target_s, duration);
  this->d_traj_coeffs = calculate_JMT_coeffs(current_d, target_d, duration);

  // Generate JMT in Freenet coordinate
  vector<double> s_traj, d_traj;
  for (int i = 0; i < N_SAMPLES; i++) {
    double s_val = 0;
    double d_val = 0;
    double     t = i * duration / N_SAMPLES;
    for (int j = 0; j < s_traj_coeffs.size(); j++) {
      s_val += this->s_traj_coeffs[j] * pow(t, j);
      d_val += this->d_traj_coeffs[j] * pow(t, j);
    }
    s_traj.push_back(s_val);
    d_traj.push_back(d_val);
  }

  return {s_traj, d_traj};
}

//string Vehicle::display_vehicle_state() {
//
//  // DEBUGGING TOOLS (not used for final release)
//  // Stream the ego vehicle localization data
//  // back to terminal
//
//  ostringstream oss;
//  oss << "s     :  " << this->s      << "\n";
//  oss << "s_dot :  " << this->s_dot  << "\n";
//  oss << "s_ddot:  " << this->s_ddot << "\n";
//  oss << "d     :  " << this->d      << "\n";
//  oss << "d_dot :  " << this->d_dot  << "\n";
//  oss << "d_ddot:  " << this->d_ddot << "\n";
//
//  return oss.str();
//}

// Work in progress FSM logic..

vector<Vehicle> Vehicle::generate_trajectory(string state,
                                             map<int, vector<Vehicle>> &predictions, double duration) {
  // Given a possible next state, generate the appropriate trajectory
  //   to realize the next state
  vector<Vehicle> trajectory;
  
  // EXPERIMENT
  Vehicle start, target;
  
  if (state.compare("KL") == 0) {
    //cout << "lane keep trajectory : " << state << endl;
    trajectory = lane_keep_trajectory(predictions,duration);
  }
  else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    //cout << "lane change trajectory : " << state << endl;
    trajectory = lane_change_trajectory(state, predictions, duration);
    
    // EXPERIMENT
//    start  = get_current_car();
//    target = get_target_for_state(state, predictions, duration);
//    trajectory.push_back(start);
//    trajectory.push_back(target);
  }
  
//  else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
//    trajectory = prep_lane_change_trajectory(state, predictions, duration);
//  }
//  else {
//    cout << "OTHER STATE : " << state << endl;
//    trajectory = lane_keep_trajectory(predictions,duration);
//  }
  
  return trajectory;
}

bool Vehicle::get_vehicle_behind(map<int,vector<Vehicle>> &predictions,
                                 int lane, Vehicle &ref_vehicle) {
  // Returns true if a vehicle is found behind the current vehicle, false
  //   otherwise. The passed reference ref_vehicle is updated if a vehicle is found.

  double current_car_s = this->s;
  double preferred_buffer = this->preferred_buffer; // [m]
  bool   found_vehicle = false;
  
  Vehicle temp_car;
  for (auto prediction: predictions) {
    temp_car = prediction.second[0];
    
    if (temp_car.lane == this->lane && temp_car.s < current_car_s && current_car_s - temp_car.s < preferred_buffer) {
      ref_vehicle = temp_car;
      found_vehicle = true;
      return found_vehicle;
    }
  }

  return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int,vector<Vehicle>> &predictions,
                                int lane, Vehicle &ref_vehicle) {
  
  // Return true if a vehicle is found ahead of the current vehicle, false
  //   otherwise. The passed reference reference vehicle is updated if a vehicle is found

  double current_car_s = this->s;
  double preferred_buffer = this->preferred_buffer; // [m]
  bool found_vehicle = false;
  
  
  Vehicle temp_car;
  for(auto prediction: predictions) {
    temp_car = prediction.second[0];
    
    if (temp_car.lane == this->lane && temp_car.s > current_car_s && (temp_car.s - current_car_s) < preferred_buffer) {
      ref_vehicle = temp_car;
      found_vehicle = true;
      return found_vehicle;
    }
  }

  return found_vehicle;
}

vector<float> Vehicle::get_kinematics(string state, map<int,vector<Vehicle>> &predictions,
                                      int lane, double duration) {
  // Get next time step kinematics (position, velocity, acceleration)
  //   for a given lane. Try to choose the maximum velocity and acceleration,
  //   given other vehicle positions and accel/velocity constraints

  double dt = duration / N_SAMPLES;
  
  double current_car_s      = this->s;
  double current_car_s_dot  = this->s_dot;
  double current_car_s_ddot = this->s_ddot;
  
  double max_velocity = std::min(this->s_dot,BELOW_SPEED_LIMIT*MPH2MPS);
  
  float target_s, target_s_dot, target_s_ddot;
  float target_d, target_d_dot, target_d_ddot;
  
  // EXPERIMENT (need to replace max speed with limiting logic)
  //target_s_dot = BELOW_SPEED_LIMIT*MPH2MPS;
  
  Vehicle car_ahead, car_behind;
  bool is_car_ahead  = get_vehicle_ahead(predictions, lane, car_ahead);
  bool is_car_behind = get_vehicle_behind(predictions, lane, car_behind);

  if (is_car_ahead) {
    if (is_car_behind) {
      // When there are vehicles both in front and behind ego vehicle,
      // it must travel at the speed of traffic, regardless of preferred buffer
      cout << "Found vehicle AND behind" << endl;
      cout << "Vehicle ahead is " << car_ahead.s - current_car_s << endl;
      cout << "Vehicle behind is " << current_car_s - car_behind.s << endl;
      target_s_dot = car_ahead.s_dot;
    } else {
      // When there is only a car ahead
      cout << "Found vehicle ahead only" << endl;
      cout << "Vehicle is " << car_ahead.s - current_car_s << " meters ahead" << endl;
//      float max_velocity_in_front = (car_ahead.s - current_car_s - this->preferred_buffer) + car_ahead.s_dot
                                  - 0.5 * (current_car_s_ddot) * dt;

      // Limit the target speed up to the vehicle ahead
      target_s_dot = std::min(max_velocity,car_ahead.s_dot-5*MPH2MPS);
    }
  } else {
    // If there is no car in front or behind
    // then set the velocity up to the max velocity limit
    target_s_dot = BELOW_SPEED_LIMIT*MPH2MPS;
    //cout << "Normal driving: " << target_s_dot*MPS2MPH << " MPH" << endl;
  }
  
  // Update the new position and acceleration
  target_s_ddot = 0;  // Constant acceleration to minimize jerk
  target_s = this->s + (this->s_dot + target_s_dot) / 2 * duration;
  
  target_d_dot  = 0;
  target_d_ddot = 0;

  int current_lane = this->d / LANE_WIDTH;
  if(state.compare("KL") == 0)
  {
    target_d = current_lane*LANE_WIDTH + LANE_WIDTH/2;
  }
  else if(state.compare("LCL") == 0)
  {
    // cout << "Calculate target d for LCL" << endl;
    target_d = (current_lane-1)*LANE_WIDTH + LANE_WIDTH/2;
  }
  else if(state.compare("LCR") == 0)
  {
    // cout << "Calculate target d for LCR" << endl;
    target_d = (current_lane+1)*LANE_WIDTH + LANE_WIDTH/2;
  } else {
    // Default: target lane is the same as the current lane
    target_d = current_lane*LANE_WIDTH + LANE_WIDTH/2;
  }
  
//  // Find the leading vehicle in the target lane
//  int     target_lane       = target_d / LANE_WIDTH;
//  Vehicle leading_car       = get_nearest_leading_car(target_lane, predictions, duration);
//  double  leading_car_s     = leading_car.s;
//  double  leading_car_s_dot = leading_car.s_dot;
//
//  // If the target vehicle poition is too close to the leading vehicle
//  // then match the position and speed of target vehicle to leading vehicle.
//  // Otherwise reduce the target speed by SPEED_DECREMENT [mph]
//
//  double distance_leading_to_target = fabs(leading_car_s - target_s);
//  if (leading_car_s > this->s && distance_leading_to_target < FOLLOW_DISTANCE) {
//    if (distance_leading_to_target >= FOLLOW_DISTANCE/2) {
//      target_s_dot = leading_car_s_dot;
//    } else {
//      target_s_dot -= SPEED_DECREMENT*MPH2MPS;
//    }
//
//    // cout << "NEARBY LEADING VEHICLE DETECTED!  ";
//    // cout << ", lane: "  << target_lane
//    //      << "s: "       << leading_car_s
//    //      << ", speed: " << leading_car_s_dot << endl;
//  }
  
  return {target_s, target_s_dot, target_s_ddot, target_d, target_d_dot, target_d_ddot};
}

// Calculate new vehicle position
float Vehicle::s_position_at(Vehicle &vehicle, int t) {
  // Kinematics Equation: final distance = s + v*delta_t + 1/2 * a * (delta_t)^2
  return vehicle.s + vehicle.s_dot * t + vehicle.s_ddot *t *t /2.0;
}
float Vehicle::d_position_at(Vehicle &vehicle, int t) {
  // Kinematics Equation: final distance = s + v*delta_t + 1/2 * a * (delta_t)^2
  return vehicle.d + vehicle.d_dot * t + vehicle.d_ddot *t *t /2.0;
}

vector<Vehicle> Vehicle::constant_speed_trajectory(double duration) {

  double dt = duration / N_SAMPLES;
  
  // Target lane of ego vehicle
  int    target_lane   = this->lane;
  string target_state  = "CS";
  double target_s_dot  = BELOW_SPEED_LIMIT*MPH2MPS; // [m/s]
  double target_s_ddot = 0; // Constant acceleration to minimize jerk
  double target_s      = this->s + (this->s_dot + target_s_dot) / 2 * dt;
  
  double target_d      = this->d;
  double target_d_dot  = 0;
  double target_d_ddot = 0;
  
  // Constant speed trajectory
  vector<Vehicle> trajectory= {Vehicle(this->lane,this->state,this->s,this->s_dot,this->s_ddot,this->d,this->d_dot,this->d_ddot)};
  trajectory.push_back(Vehicle(target_lane,target_state,target_s,target_s_dot,target_s_ddot,target_d,target_d_dot,target_d_ddot));
  return trajectory;
}

vector<Vehicle> Vehicle::lane_keep_trajectory(map<int,vector<Vehicle>> &predictions, double duration) {
  
  // Generate a keep lane trajectory
  int    target_lane  = this->lane;
  string target_state = "KL";
  
  vector<float> lane_keep_kinematics = get_kinematics(target_state, predictions,this->lane,duration);

  float target_s      = lane_keep_kinematics[0];
  float target_s_dot  = lane_keep_kinematics[1];
  float target_s_ddot = lane_keep_kinematics[2];

  float target_d      = lane_keep_kinematics[3];
  float target_d_dot  = lane_keep_kinematics[4];
  float target_d_ddot = lane_keep_kinematics[5];

  // Keep lane trajectory
  vector<Vehicle> trajectory;
  trajectory.push_back(Vehicle(this->lane,this->state,this->s,this->s_dot,this->s_ddot,this->d,this->d_dot,this->d_ddot));
  trajectory.push_back(Vehicle(target_lane,target_state,target_s,target_s_dot,target_s_ddot,target_d,target_d_dot,target_d_ddot));
  
  return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state,
                                                     map<int,vector<Vehicle>> &predictions, double duration) {
  // Generate a trajectory preparing for a lane change
  float target_s, target_s_dot, target_s_ddot;
  float target_d, target_d_dot, target_d_ddot;
  int current_lane = this->lane;
  vector<float> current_lane_kinematics = get_kinematics(state,predictions,current_lane,duration);
  
  Vehicle car_behind;
  bool is_car_behind = get_vehicle_behind(predictions, current_lane, car_behind);
  if (is_car_behind) {
    // If there is a vehicle behind in the lane
    // then keep the speed of current lane so not to collide with car behind.
    target_s      = current_lane_kinematics[0];
    target_s_dot  = current_lane_kinematics[1];
    target_s_ddot = current_lane_kinematics[2];
    
    target_d      = current_lane_kinematics[3];
    target_d_dot  = current_lane_kinematics[4];
    target_d_ddot = current_lane_kinematics[5];
  } else {
    // If there is no vehicle beind in the lane
    // then calculate the kinematics of ego vehicle driving in the new lane
    int target_lane = this->lane + lane_direction[state];
    vector<float> new_lane_kinematics = get_kinematics(state,predictions, target_lane,duration);

    // Choose kinematics with the lowest velocity
    vector<float> best_kinematics = (new_lane_kinematics[1] < current_lane_kinematics[1]) ?
                                     new_lane_kinematics : current_lane_kinematics;

    target_s      = best_kinematics[0];
    target_s_dot  = best_kinematics[1];
    target_s_ddot = best_kinematics[2];
    
    target_d      = best_kinematics[3];
    target_d_dot  = best_kinematics[4];
    target_d_ddot = best_kinematics[5];
  }

  // Prepare lane change trajectory
  
  int    target_lane  = this->lane;
  string target_state = (target_lane > current_lane) ? "PLCR" : "PLCL";
  vector<Vehicle> trajectory;
  trajectory.push_back(Vehicle(this->lane,this->state,this->s,this->s_dot,this->s_ddot,this->d,this->d_dot,this->d_ddot));
  trajectory.push_back(Vehicle(target_lane,target_state,target_s,target_s_dot,target_s_ddot,target_d,target_d_dot,target_d_ddot));
  
  return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state,
                                                map<int,vector<Vehicle>> &predictions, double duration) {
  // Generate a lane change trajectory
  
  vector<Vehicle> trajectory;
  int     target_lane = this->lane + lane_direction[state];
  string target_state = state;

//  // Check if a lane change is possible
//  //   (check if another vehicle occupies that spot).
//  for (auto prediction : predictions) {
//    Vehicle next_lane_car = prediction.second[0];
//
//    // Final check before changing to the new lane
//
//    if (next_lane_car.lane == target_lane) {
//      // If there is another vehicle turning in the same position in the new lane,
//      // then lane change is not possible, return empty trajectory.
//      double current_car_s = this->s;
//      double diff_current_car_next_lane_car_s = fabs(next_lane_car.s - current_car_s);
//      if (diff_current_car_next_lane_car_s < VEHICLE_RADIUS/2) {
//        cout << "Lane change failed: Too close to change lane because a nearby vehicle" << endl;
//        return lane_keep_trajectory(predictions,duration);
//      }
//    }
//  }

  // Final check if there are any cars nearby
  for (auto prediction : predictions) {
    check_nearby_cars(prediction.second);
  }
  bool car_to_left  = this->car_to_left;
  bool car_to_right = this->car_to_right;
  if (car_to_left || car_to_right) return lane_keep_trajectory(predictions,duration);
  
  // Lane change trajectory
  vector<float> new_lane_kinematics = get_kinematics(state,predictions, target_lane, duration);
  double target_s      = new_lane_kinematics[0];
  double target_s_dot  = new_lane_kinematics[1];
  double target_s_ddot = new_lane_kinematics[2];

  double target_d      = new_lane_kinematics[3];
  double target_d_dot  = new_lane_kinematics[4];
  double target_d_ddot = new_lane_kinematics[5];
    
  trajectory.push_back(Vehicle(this->lane,this->state,this->s,this->s_dot,this->s_ddot,this->d,this->d_dot,this->d_ddot));
  trajectory.push_back(Vehicle(target_lane,target_state,target_s,target_s_dot,target_s_ddot,target_d,target_d_dot,target_d_ddot));

  return trajectory;
}

void Vehicle::realize_next_state(Vehicle &next_state) {

  // Set vehicle state and kinematics for ego vehicle
  //   using the last state of the trajectory.
  
  this->lane   = next_state.lane;
  this->state  = next_state.state;
  
  this->s      = next_state.s;
  this->s_dot  = next_state.s_dot;
  this->s_ddot = next_state.s_ddot;
  
  this->d      = next_state.d;
  this->d_dot  = next_state.d_dot;
  this->d_ddot = next_state.d_ddot;
}
