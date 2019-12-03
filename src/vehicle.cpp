#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <iterator>
#include <algorithm>

#include "constants.h"
#include "jmt.h"
#include "vehicle.h"

// Initialize Vehicle
Vehicle::Vehicle() {}

Vehicle::Vehicle(double s, double s_dot, double s_ddot, double d, double d_dot, double d_ddot) {
  
  // Initalize the vehicle class with Freenet coordinates
  this->s      = s;
  this->s_dot  = s_dot;
  this->s_ddot = s_ddot;
  
  this->d      = d;
  this->d_dot  = d_dot;
  this->d_ddot = d_ddot;
}

Vehicle::~Vehicle() {}

void Vehicle::update_available_states(bool car_to_left, bool car_to_right) {
  
  // Set the list of available states
  // for the ego vehicle to prepare for lane change
  
  // Initalize the state to keep lane
  this->available_states = {"KL"};
  
  // Allow to turn left if there is no car on the left lane
  if (this->d > LANE_WIDTH && !car_to_left) {
    this->available_states.push_back("LCL");
  }

  // Allow to turn right if there is no car on the right lane
  if (this->d < 2*LANE_WIDTH && !car_to_right) {
    this->available_states.push_back("LCR");
  }
}

Vehicle Vehicle::get_nearest_leading_car(int target_lane,
                                             map<int,vector<vector<double>>> predictions,
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
    vector<vector<double>> pred_traj = prediction.second;
    double pred_start_d = pred_traj[0][1];
    double pred_end_d   = pred_traj[pred_traj.size()-1][1];
    
    // Calculate the non-ego vehicles trajectory
    // if it's trajectory ends in the target lane of the ego vehicle
    int pred_lane = pred_start_d / LANE_WIDTH;
    if (pred_lane == target_lane) {
      double pred_start_s              = pred_traj[0][0];
      double pred_end_s                = pred_traj[pred_traj.size()-1][0];
      
      double next_to_pred_end_s        = pred_traj[pred_traj.size()-2][0];
      double second_last_to_pred_end_s = pred_traj[pred_traj.size()-3][0];
      double pred_s_dot                = (pred_end_s - next_to_pred_end_s) / dt;
      double last_pred_s_dot           = (next_to_pred_end_s - second_last_to_pred_end_s) / dt;
      double pred_s_ddot               = (pred_s_dot - last_pred_s_dot) / dt;
      
      double next_to_pred_end_d        = pred_traj[pred_traj.size()-2][1];
      double second_last_to_pred_end_d = pred_traj[pred_traj.size()-3][1];
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
  
  Vehicle nearest_leading_car = {Vehicle(nearest_leading_car_s, nearest_leading_car_s_dot, nearest_leading_car_s_ddot,
                                         nearest_leading_car_d, nearest_leading_car_d_dot, nearest_leading_car_d_ddot)};
  return nearest_leading_car;
}

Vehicle Vehicle::get_target_for_state(string state, map<int, vector<vector<double>>> predictions,
                                      double duration, bool car_ahead) {
  
  // Return target vehicle state
  // (position, velocity and acceleration) in Freenet coordinates

  // Current lane of ego vehicle
  int    current_lane  = this->d / LANE_WIDTH;
  
  double target_s_dot  = BELOW_SPEED_LIMIT*MPH2MPS; // [m/s]
  double target_s_ddot = 0; // Constant acceleration to minimize jerk
  double target_s = this->s + (this->s_dot + target_s_dot) / 2 * duration;
  
  // Target lane of ego vehicle
  double target_d      = current_lane*LANE_WIDTH + LANE_WIDTH/2;
  double target_d_dot  = 0;
  double target_d_ddot = 0;
  
  // Return emergency stop for target vehicle state
  // when a car suddenly cuts in front of ego vehicle
  if (car_ahead) {
    target_s      = this->s;
    target_s_dot  = 0.0;
    target_s_ddot = (target_s_dot - this->s_dot) / (N_SAMPLES * DT);
    
    return Vehicle(target_s, target_s_dot, target_s_ddot,
                   target_d, target_d_dot, target_d_ddot);
  }
  
  if(state.compare("KL") == 0)
  {
    target_d = current_lane*LANE_WIDTH + LANE_WIDTH/2;
  }
  else if(state.compare("LCL") == 0)
  {
    target_d = (current_lane-1)*LANE_WIDTH + LANE_WIDTH/2;
  }
  else if(state.compare("LCR") == 0)
  {
    target_d = (current_lane+1)*LANE_WIDTH + LANE_WIDTH/2;
  }
  
  // Leading vehicle position and speed
  int     target_lane       = target_d / LANE_WIDTH;
  Vehicle leading_car       = get_nearest_leading_car(target_lane, predictions, duration);
  double  leading_car_s     = leading_car.s;
  double  leading_car_s_dot = leading_car.s_dot;
  
  // If the target vehicle poition is too close to the leading vehicle
  // then match the position and speed of target vehicle to leading vehicle.
  // Otherwise reduce the target speed by SPEED_DECREMENT [mph]
  
  double dist_leading_to_target = fabs(leading_car_s - target_s);
  if (leading_car_s > this->s && dist_leading_to_target < FOLLOW_DISTANCE) {
    if (dist_leading_to_target >= FOLLOW_DISTANCE/2) {
      target_s_dot = leading_car_s_dot;
    } else {
      target_s_dot -= SPEED_DECREMENT*MPH2MPS;
    }
    
    // // DEBUG
    // cout << "NEARBY LEADING VEHICLE DETECTED!  ";
    // cout << ", lane: "  << target_lane
    //      << "s: "       << leading_car_s
    //      << ", speed: " << leading_car_s_dot << endl;
  }
  
  return Vehicle(target_s, target_s_dot, target_s_ddot,
                 target_d, target_d_dot, target_d_ddot);
}

vector<vector<double>> Vehicle::generate_predictions(double traj_start_time, double duration) {

  // Generate N_SAMPLE numbers of predicted vehicle positions
  // for a given trajectory start time and duration (sec)

  vector<vector<double>> predictions;
  for(int i = 0; i < N_SAMPLES; i++)
  {
    // Simple constant speed prediction model
    // Assuming the vehicle is travelling at constant speed at the end of previous path
    // and the heading same direction as the lane line
    double dt     = traj_start_time + (i * duration/N_SAMPLES);
    double next_s = this->s + this->s_dot * dt;
    double next_d = this->d;
    
    vector<double> prediction = {next_s,next_d};
    predictions.push_back(prediction);
  }
  return predictions;
}

vector<vector<double>> Vehicle::generate_trajectory_for_target(Vehicle target, double duration) {
  
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
