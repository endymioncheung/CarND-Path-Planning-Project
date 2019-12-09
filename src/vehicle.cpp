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

  vector<float>   costs;
  vector<Vehicle> final_targets;

  // Find all possible states (for ego vehicle)
  vector<string> states = successor_states();
  
  for (string state: states) {
    
    // No FSM logic
    Vehicle target = get_target_for_state(state, predictions, duration);
    
    // with FSM logic
//    vector<Vehicle> trajectory = generate_trajectory(state,predictions,duration);
//    Vehicle target = trajectory[1];
    
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
  //cout << "Best state = " << best_target.state << " \t\tCosts = " << costs[best_idx] << endl;
  
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
  int  current_lane    = this->lane;
  bool car_to_left     = this->car_to_left;
  bool car_to_right    = this->car_to_right;
  
  // Always start FSM with the lane keep state
  vector<string> states;
  states.push_back("KL");
  
//  cout << "Lane #" << lane << endl << "State: " << state << endl;
  
  // Create the list of plausible sucessor states
  if(state.compare("KL") == 0) {
    //cout << "State = KL" << endl;
    
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

  // Current ego vehicle state
  //string current_state  = this->state;
  double current_s      = this->s;
  double current_s_dot  = this->s_dot;
  
  double current_d      = this->d;
  double current_d_dot  = this->d_dot;
  double current_d_ddot = this->d_ddot;
  
  string target_state   = state;
  int    current_lane   = std::round((current_d - LANE_WIDTH/2) / LANE_WIDTH);
  
  int target_lane;
  bool car_to_left  = this->car_to_left;
  bool car_to_right = this->car_to_right;
  
  if(state.compare("LCL") == 0 && !car_to_left && (current_lane > 0)) {
    // Allow to change lane to left if there is no car on the left lane
    target_lane = current_lane - 1;
  } else if (state.compare("LCR") == 0 && !car_to_right && (current_lane < NUM_LANES-1)) {
    // Allow to change lane to right if there is no car on the right lane
    target_lane = current_lane + 1;
  } else {
    target_lane = current_lane;
  }
  
  //**************************************//
  //    Ego Vehice Longtidunal Dynamics   //
  //**************************************//
  
  // Set the ego target vehicle speed to max speed limit.
  // Limit the vehicle vehicle speed if it exceeds the
  // instanenous acceleration limit
  double target_s_dot  = BELOW_SPEED_LIMIT*MPH2MPS; // [m/s]
  double target_s_ddot = (target_s_dot - current_s_dot) / duration;
  if (target_s_ddot >= INSTANT_ACCEL_LIMIT)
  {
    cout << "Limiting the instaneous acceleration limit to " << INSTANT_ACCEL_LIMIT << " m/2" << endl;
    target_s_ddot = INSTANT_ACCEL_LIMIT;
    target_s_dot  = current_s_dot + target_s_ddot * duration;
  }
  double target_s = current_s + (current_s_dot + target_s_dot) / 2 * duration;
  
  //**************************************//
  //      Ego Vehice Lateral Dynamics     //
  //**************************************//

  double target_d      = target_lane*LANE_WIDTH + LANE_WIDTH/2;
  double target_d_dot  = 0;
  double target_d_ddot = 0;
  
//  double target_d_dot  = (target_d - current_d) / duration;
//  double target_d_ddot = (target_d_dot - current_d_dot) / duration;
  double combined_accel = sqrt(target_s_ddot*target_s_ddot + target_d_ddot*target_d_ddot);
  
  // No change in lateral dynamics if the
  // combined acceleration is close to the limit
  if (combined_accel >= INSTANT_ACCEL_LIMIT)
  {
    cout << "Limiting lateral acceleration to INSTANT_ACCEL_LIMIT" << endl;
    target_state  = "KL";
    
    target_d      = current_d;
    target_d_dot  = current_d_dot;
    target_d_ddot = current_d_ddot;
  }

  // Find the leading vehicle in the target lane
  Vehicle leading_car       = get_nearest_leading_car_for_lane(target_lane, predictions, duration);
  double  leading_car_s     = leading_car.s;
  //double  leading_car_s_dot = leading_car.s_dot;

  // If the target vehicle poition is too close to the leading vehicle
  // then match the position and speed of target vehicle to leading vehicle.
  // Otherwise reduce the target speed by SPEED_DECREMENT [mph]

  double distance_leading_to_target = fabs(leading_car_s - target_s);
  if (leading_car_s > target_s && distance_leading_to_target < FOLLOW_DISTANCE) {
    if (distance_leading_to_target >= FOLLOW_DISTANCE/2) {
      // Match the leading vehicle speed if the ego vehicle
      // has reasonable gap to the leading vehicle
      //cout << "Match leading vehicle within " << FOLLOW_DISTANCE << " [m]" << endl;
      target_s       = leading_car_s - FOLLOW_DISTANCE;
      target_s_dot   = (target_s - current_s) / duration;
      target_s_ddot  = (target_s_dot - current_s_dot) / duration;
    } else {
      // Reduce the ego vehicle speed
      //cout << "Reducing the ego vehicle speed by " << SPEED_DECREMENT << " [MPH]" << endl;
      target_s_dot   -= SPEED_DECREMENT*MPH2MPS;
      target_s_ddot  = (target_s_dot - current_s_dot) / duration;
      target_s       = current_s + (current_s_dot + target_s_dot) / 2 * duration;
    }

    // cout << "NEARBY LEADING VEHICLE DETECTED!  ";
    // cout << ", lane: "  << target_lane
    //      << "s: "       << leading_car_s
    //      << ", speed: " << leading_car_s_dot << endl;
  }

  Vehicle target = Vehicle(target_lane, target_state,
                           target_s, target_s_dot, target_s_ddot,
                           target_d, target_d_dot, target_d_ddot);
  
  //**************************************//
  //         Emergency vehicle stop       //
  //**************************************//

  // Return emergency stop for target vehicle state
  // when a car suddenly cuts in front of ego vehicle
  bool car_in_front  = this->car_in_front;
  if (target_state.compare("KL") == 0 && car_in_front) {
//    cout << "Emergency vehicle stop" << endl;
    target_lane   = current_lane;
    target_state  = "KL";
    target_s_dot  = 0.0; // Request emergency stop immediately
    target_s_ddot = (target_s_dot - current_s_dot) / duration;
    target_s      = current_s + (current_s_dot + target_s_dot) / 2 * duration;

    return Vehicle(target_lane, target_state,
                   target_s, target_s_dot, target_s_ddot,
                   target_d, target_d_dot, target_d_ddot);
  }

  return target;
}

void Vehicle::check_nearby_cars(vector<Vehicle> other_cars) {
  
  // Prepare to change lane logic to check if there is a car
  // close to left or right of the ego vehicle and
  // restricting the next possible vehicle states
  
  // Check if there are other cars nearby the ego vehicle
  
  // Assume there are no cars nearby until they are found
  this->car_to_left  = false;
  this->car_to_right = false;
  this->car_in_front = false;
  
  double current_s = this->s;
  
  for (Vehicle other_car: other_cars) {
    
    // Longitudinal and lateral distance
    // between the ego vehicle to any other vehicles
    double s_diff = fabs(other_car.s - current_s);
    double d_diff = other_car.d - this->d;
    
    if (other_car.s > this->s && s_diff < FOLLOW_DISTANCE) {
      if (-2 < d_diff && d_diff < 2) {
        this->car_in_front = true;
//          cout << "CAUTION: CAR AHEAD within FOLLOW_DISTANCE - Distance to the car ahead: " << s_diff << " meters" << endl;
      } else if (s_diff < FOLLOW_DISTANCE) {
        if (-6 < d_diff && d_diff < -2) {
          this->car_to_left = true;
//          cout << "CAUTION: CAR TO THE LEFT!!   Distance to left car: " << other_car.s - this->s << " meters" << endl;
        } else if (2 < d_diff && d_diff < 6) {
          this->car_to_right = true;
//          cout << "CAUTION: CAR TO THE RIGHT!!  Distance to right car: " << other_car.s - this->s << " meters" << endl;
        }
      }
    }
  }
}

Vehicle Vehicle::get_nearest_leading_car_for_lane(int target_lane,
                                        map<int,vector<Vehicle>> predictions,
                                        double duration) {
    
  // Return the nearest vehicle state in target lane
  // for a given set of predictions and duration of the trajectory

  double current_s = this->s;
  
  double dt = duration / N_SAMPLES;
  
  // Initalize leading vehicle state
  double nearest_leading_car_s      = 99999;
  double nearest_leading_car_s_dot  = 0;
  double nearest_leading_car_s_ddot = 0;
  
  double nearest_leading_car_d      = 0;
  double nearest_leading_car_d_dot  = 0;
  double nearest_leading_car_d_ddot = 0;
  
  for (auto prediction : predictions) {
    
    // Calculate the non-ego vehicles trajectory
    // if it's trajectory ends in the target lane of the ego vehicle
    vector<Vehicle> pred_traj = prediction.second;

    double pred_end_d = pred_traj[pred_traj.size()-1].d;
    int    pred_lane  = std::round( (pred_end_d - LANE_WIDTH/2) / LANE_WIDTH);
    
    if (pred_lane == target_lane) {
      //************************************************//
      // Predicted leading vehicle longitudnal dynamics //
      //************************************************//
      double pred_start_s              = pred_traj[0].s;
      // Predicted leading vehicle longitudnal position
      double pred_end_s                = pred_traj[pred_traj.size()-1].s;
      double next_to_pred_end_s        = pred_traj[pred_traj.size()-2].s;
      double second_last_to_pred_end_s = pred_traj[pred_traj.size()-3].s;
      // Predicted leading vehicle longitudnal velocity
      double pred_s_dot                = (pred_end_s - next_to_pred_end_s) / dt;
      double last_pred_s_dot           = (next_to_pred_end_s - second_last_to_pred_end_s) / dt;
      // Predicted leading vehicle longitudnal accleration
      double pred_s_ddot               = (pred_s_dot - last_pred_s_dot) / dt;
      
      //************************************************//
      //   Predicted leading vehicle lateral dynamics   //
      //************************************************//
      // Predicted leading vehicle lateral position
      double next_to_pred_end_d        = pred_traj[pred_traj.size()-2].d;
      double second_last_to_pred_end_d = pred_traj[pred_traj.size()-3].d;
      // Predicted leading vehicle lateral velocity
      double pred_d_dot                = (pred_end_d - next_to_pred_end_d) / dt;
      double last_pred_d_dot           = (next_to_pred_end_d - second_last_to_pred_end_d) / dt;
      // Predicted leading vehicle lateral acceleration
      double pred_d_ddot               = (pred_d_dot - last_pred_d_dot) / dt;
      
      //************************************************//
      // Get the nearest vehicle speed and distance     //
      // if the non-ego vehicle is ahead of ego-vehicle //
      //************************************************//
      if (pred_start_s >= current_s && pred_end_s < nearest_leading_car_s) {
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
  string nearet_leading_car_state = "CS";
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
  
  if (state.compare("KL") == 0) {
    //cout << "lane keep trajectory : " << state << endl;
    trajectory = lane_keep_trajectory(predictions,duration);
  }
  else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    //cout << "lane change trajectory : " << state << endl;
    trajectory = lane_change_trajectory(state, predictions, duration);
    
    // EXPERIMENT
//    Vehicle start, target;
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

bool Vehicle::get_car_behind(map<int,vector<Vehicle>> &predictions,
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

bool Vehicle::get_car_ahead(map<int,vector<Vehicle>> &predictions,
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

  float target_s, target_s_dot, target_s_ddot;
  float target_d, target_d_dot, target_d_ddot;
  
  // Set the ego target position and acceleration
  target_s_dot  = BELOW_SPEED_LIMIT*MPH2MPS; // [m/s]
  target_s_ddot = 0;  // Constant acceleration to minimize jerk
  target_s      = this->s + (this->s_dot + target_s_dot) / 2 * duration;
  
//  // Check if there are cars ahead or behind
//  Vehicle car_ahead, car_behind;
//  bool is_car_ahead  = get_car_ahead( predictions, lane, car_ahead);
//  bool is_car_behind = get_car_behind(predictions, lane, car_behind);
//  if (is_car_ahead) {
//    if (is_car_behind) {
//      // When there are vehicles both in front and behind ego vehicle,
//      // it must travel at the speed of traffic, regardless of preferred buffer
//      cout << "WARNING: Distance to the car ahead  is too close - " << car_ahead.s  - current_car_s << " meters" << endl;
//      cout << "WARNING: Distance to the car behind is too close - " << car_behind.s - current_car_s << " meters" << endl;
//      target_s_dot = (car_ahead.s_dot+car_behind.s_dot)/2;
//    } else {
//      // When there is only a car ahead
//      cout << "WARNING: Distance to the car ahead is too close - " << car_ahead.s - current_car_s << " meters" << endl;
//
//      // Set the ego vehicle target speed close to the vehicle ahead
//      target_s_dot = std::min(target_s_dot,car_ahead.s_dot-SPEED_DECREMENT*MPH2MPS);
//    }
//  } else {
//    // If there is no car in front or behind
//    // then set the velocity up to the max velocity limit
//    target_s_dot = BELOW_SPEED_LIMIT*MPH2MPS;
//    //cout << "Normal driving: " << target_s_dot*MPS2MPH << " MPH" << endl;
//  }

  int current_lane  = this->d / LANE_WIDTH;
  
  int target_lane;
  bool car_to_left  = this->car_to_left;
  bool car_to_right = this->car_to_right;
  
  if(state.compare("LCL") == 0 && !car_to_left && (current_lane > 0)) {
    // Allow to change lane to left if there is no car on the left lane
    target_lane = current_lane - 1;
  } else if (state.compare("LCR") == 0 && !car_to_right && (current_lane < NUM_LANES-1)) {
    // Allow to change lane to right if there is no car on the right lane
    target_lane = current_lane + 1;
  } else {
    target_lane = current_lane;
  }
  
  target_d = target_lane*LANE_WIDTH + LANE_WIDTH/2;
  target_d_dot  = 0;
  target_d_ddot = 0;
  
  // Find the leading vehicle in the target lane
  Vehicle leading_car       = get_nearest_leading_car_for_lane(target_lane, predictions, duration);
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
  
  return {target_s, target_s_dot, target_s_ddot,
          target_d, target_d_dot, target_d_ddot};
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
  bool is_car_behind = get_car_behind(predictions, current_lane, car_behind);
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
    int target_lane;
    bool car_to_left  = this->car_to_left;
    bool car_to_right = this->car_to_right;
    
    if(state.compare("LCL") == 0 && !car_to_left && (current_lane > 0)) {
      // Allow to change lane to left if there is no car on the left lane
      target_lane = current_lane - 1;
    } else if (state.compare("LCR") == 0 && !car_to_right && (current_lane < NUM_LANES-1)) {
      // Allow to change lane to right if there is no car on the right lane
      target_lane = current_lane + 1;
    } else {
      target_lane = current_lane;
    }
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

  // Final check if there are any cars nearby
  // if so fallback to the lane keep trajectory
  
  // if ego vehicle is changing lane to left
  // and there is a car to left, then fall back to lane keep
  if ((state.compare("LCL") == 0) && this->car_to_left)
  {
    //cout << "Final check - unable to " << state << " because its too close to the car on left: " << endl;
    return lane_keep_trajectory(predictions,duration);
  }
  
  // if ego vehicle is changing lane to right
  // and there is a car to right, then fall back to lane keep
  if ((state.compare("LCR") == 0) && this->car_to_right) {
    //cout << "Final check - unable to " << state << " because its too close to the car on the right" << endl;
    return lane_keep_trajectory(predictions,duration);
  }
  
  // Lane change trajectory
  double current_d      = this->d;
  int current_lane  = std::round((current_d - LANE_WIDTH/2) / LANE_WIDTH);
  int target_lane;
  bool car_to_left  = this->car_to_left;
  bool car_to_right = this->car_to_right;
  
  if(state.compare("LCL") == 0 && !car_to_left && (current_lane > 0)) {
    // Allow to change lane to left if there is no car on the left lane
    target_lane = current_lane - 1;
  } else if (state.compare("LCR") == 0 && !car_to_right && (current_lane < NUM_LANES-1)) {
    // Allow to change lane to right if there is no car on the right lane
    target_lane = current_lane + 1;
  } else {
    target_lane = current_lane;
  }
  string target_state = state;
  
  vector<float> new_lane_kinematics = get_kinematics(state, predictions, target_lane, duration);
  double target_s      = new_lane_kinematics[0];
  double target_s_dot  = new_lane_kinematics[1];
  double target_s_ddot = new_lane_kinematics[2];

  double target_d      = new_lane_kinematics[3];
  double target_d_dot  = new_lane_kinematics[4];
  double target_d_ddot = new_lane_kinematics[5];
    
  vector<Vehicle> trajectory;
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
  
  // DEBUG
  cout << "State selected : " << next_state.state << endl;
  //cout << "Target lane  : " << next_state.lane << endl << endl;
}

//vector<Vehicle> Vehicle::constant_speed_trajectory(double duration) {
//
//  double dt = duration / N_SAMPLES;
//
//  // Ego vehicle target state
//  int    target_lane   = this->lane;
//  string target_state  = "CS";
//  double target_s_dot  = BELOW_SPEED_LIMIT*MPH2MPS; // [m/s]
//  double target_s_ddot = 0; // Constant acceleration to minimize jerk
//  double target_s      = this->s + (this->s_dot + target_s_dot) / 2 * dt;
//
//  double target_d      = this->d;
//  double target_d_dot  = 0;
//  double target_d_ddot = 0;
//
//  // Constant speed trajectory
//  vector<Vehicle> trajectory= {Vehicle(this->lane,this->state,this->s,this->s_dot,this->s_ddot,this->d,this->d_dot,this->d_ddot)};
//  trajectory.push_back(Vehicle(target_lane,target_state,target_s,target_s_dot,target_s_ddot,target_d,target_d_dot,target_d_ddot));
//  return trajectory;
//}
