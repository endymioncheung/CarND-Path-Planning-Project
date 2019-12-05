#ifndef COSTS
#define COSTS

#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>
#include <cmath>

#include "constants.h"

using namespace std;

vector<double> velocities_in_traj(vector<double> &traj) {
  
  // Return a list of average velocities for each
  // pair of adjacent trajectory points
  
  vector<double> velocities;
  for (int i = 1; i < traj.size(); i++) {
    velocities.push_back((traj[i] - traj[i-1]) / DT);
  }
  return velocities;
}

double logistic(double x){
  
  // A function that always returns a value
  // between 0 and 1 for x in the range[0, infinity] and
  // - 1 to 1 for x in the range[-infinity, infinity]

  return 2.0 / (1 + exp(-x)) - 1.0;
}

double nearest_approach(vector<double> &s_traj, vector<double> &d_traj,
                        vector<vector<double>> &prediction) {
  
  // Find the closest distance between the trajectory and the prediction
  
  double closest_distance = 999999;
  for (int i = 0; i < N_SAMPLES; i++) {
    double current_distance = sqrt(pow(s_traj[i] - prediction[i][0], 2) + pow(d_traj[i] - prediction[i][1], 2));
    if (current_distance < closest_distance) {
      closest_distance = current_distance;
    }
  }
  return closest_distance;
}

double nearest_approach_to_any_cars(vector<double> &s_traj, vector<double> &d_traj,
                                    map<int,vector<vector<double>>> &predictions) {
  
  // Find the closest distance between the trajectory and the list of predictions

  double closest_distance = 999999;
  for (auto prediction : predictions) {
    double current_distance = nearest_approach(s_traj, d_traj, prediction.second);
    if (current_distance < closest_distance) {
      closest_distance = current_distance;
    }
  }
  return closest_distance;
}

double nearest_approach_to_any_cars_in_lane(vector<double> &s_traj, vector<double> &d_traj,
                                            map<int,vector<vector<double>>> &predictions) {
  
  // Find the closest distance between the trajactory and
  // predictions of other vehicles within the same lane
  
  // Ego vehicle target lane
  double ego_end_d = d_traj[d_traj.size() - 1];
  int  target_lane = ego_end_d / LANE_WIDTH;
  
  double closest_distance = 999999;
  for (auto prediction : predictions) {
    
    // Non-ego vehicle predicted lane
    vector<vector<double>> pred_traj = prediction.second;
    double pred_d = pred_traj[pred_traj.size() - 1][1];
    int    pred_lane = pred_d / LANE_WIDTH;
    
    // Find the closest distance between the trajactory and
    // predictions of other vehicles within the same lane
    if (target_lane == pred_lane) {
      double current_distance = nearest_approach(s_traj, d_traj, prediction.second);
      if (current_distance < closest_distance) {
        closest_distance = current_distance;
      }
    }
  }
  return closest_distance;
}

// COST FUNCTIONS

double collision_cost(vector<double> &s_traj, vector<double> &d_traj,
                      map<int,vector<vector<double>>> &predictions) {
  
  // Binary cost function that penalizes collisions
  
  double nearest = nearest_approach_to_any_cars(s_traj, d_traj, predictions);
  if (nearest < 2 * VEHICLE_RADIUS) {
    return 1;
  } else {
    return 0;
  }
}

double exceed_speed_limit_cost(vector<double> &s_traj) {
  
  // Binary cost fuction that penalizes
  // when ego vehicle exceeds the SPEED_LIMIT

  vector<double> s_dot_traj = velocities_in_traj(s_traj);
  for (double s_dot : s_dot_traj) {
    if (s_dot > SPEED_LIMIT) {
      return 1;
    }
  }
  return 0;
}

double time_diff_cost(double &target_time, double &actual_time) {
  
  // Penalize trajectories when the target and actual
  // time duration is different
  
  return logistic(fabs(actual_time - target_time) / target_time);
}

double traj_diff_cost(vector<double> &s_traj, vector<double> &target_s) {
  
  // Penalize trajectories whose s coordinate (and derivatives) differ from the goal.
  // Target is s, s_dot, and s_ddot.
  // can be used for d trajectories as well (or any other 1-d traj)
  
  int s_end = s_traj.size();
  double s1, s2, s3, s_dot1, s_dot2, s_ddot, cost = 0;
  
  s1     = s_traj[s_end - 1];
  s2     = s_traj[s_end - 2];
  s3     = s_traj[s_end - 3];
  s_dot1 = (s1 - s2) / DT;
  s_dot2 = (s2 - s3) / DT;
  s_ddot = (s_dot1 - s_dot2) / DT;
  
  cost += fabs(s1 - target_s[0])     / SIGMA_S;
  cost += fabs(s_dot1 - target_s[1]) / SIGMA_S_DOT;
  cost += fabs(s_ddot - target_s[2]) / SIGMA_S_DDOT;
  return logistic(cost);
}

double buffer_cost(vector<double> &s_traj, vector<double> &d_traj, map<int,vector<vector<double>>> &predictions) {
  
  // Penalize getting close to other vehicles
  
  double nearest = nearest_approach_to_any_cars(s_traj, d_traj, predictions);
  return logistic(2 * VEHICLE_RADIUS / nearest);
}

double in_lane_buffer_cost(vector<double> &s_traj, vector<double> &d_traj, map<int,vector<vector<double>>> &predictions) {
  
  // Penalize getting close to other vehicles
  
  double nearest = nearest_approach_to_any_cars_in_lane(s_traj, d_traj, predictions);
  return logistic(2 * VEHICLE_RADIUS / nearest);
}

double efficiency_cost(vector<double> &s_traj) {
  
  // Reward high average speeds
  
  vector<double> s_dot_traj = velocities_in_traj(s_traj);
  double final_s_dot;

  // double total = 0;
  // cout << "DEBUG - s_dot: ";
  // for (double s_dot: s_dot_traj) {
  //   cout << s_dot << ", ";
  //   total += s_dot;
  // }
  // cout << "/DEBUG" << endl;
  // double avg_vel = total / s_dot_traj.size();

  final_s_dot = s_dot_traj[s_dot_traj.size() - 1];
  // cout << "DEBUG - final s_dot: " << final_s_dot << endl;
  return logistic((SPEED_LIMIT - final_s_dot) / SPEED_LIMIT);
} 

double max_accel_cost(vector<double> &s_traj) {
  
  // Penalize acceleration exceeding MAX_INSTANTANEOUS_ACCEL
  
  vector<double> s_dot_traj  = velocities_in_traj(s_traj);
  vector<double> s_ddot_traj = velocities_in_traj(s_dot_traj);
  
  for (double s_ddot : s_ddot_traj) {
    if (s_ddot > MAX_INSTANTANEOUS_ACCEL) {
      return 1;
    }
  }
  return 0;
}

double max_jerk_cost(vector<double> &s_traj) {
  
  // Binary cost function which penalize
  // maximum instantaneous jerk
  
  vector<double> s_dot_traj   = velocities_in_traj(s_traj);
  vector<double> s_ddot_traj  = velocities_in_traj(s_dot_traj);
  vector<double> s_dddot_traj = velocities_in_traj(s_ddot_traj);
  for (double s_dddot : s_dddot_traj) {
    if (s_dddot > MAX_INSTANTANEOUS_JERK) {
      return 1;
    }
  }
  return 0;
}

double avg_accel_cost(vector<double> &s_traj) {
  
  // Binary cost function which penalize
  // higher average acceleration
  
  vector<double> s_dot_traj = velocities_in_traj(s_traj);
  vector<double> s_ddot_traj = velocities_in_traj(s_dot_traj);
  double total = 0;
  for (double s_ddot: s_ddot_traj) {
    total += s_ddot;
  }
  
  // Calculate average acceleration
  double avg_accel = total / s_ddot_traj.size();
  return logistic(avg_accel / EXPECTED_ACC_IN_ONE_SEC);
}

double avg_jerk_cost(vector<double> &s_traj) {
  
  // Penalize higher average jerk
  
  vector<double> s_dot_traj   = velocities_in_traj(s_traj);
  vector<double> s_ddot_traj  = velocities_in_traj(s_dot_traj);
  vector<double> s_dddot_traj = velocities_in_traj(s_ddot_traj);
  
  double total = 0;
  for (double s_dddot: s_dddot_traj) {
    total += s_dddot;
  }
  
  // Calculate average jerk
  double avg_jerk = total / s_dddot_traj.size();
  return logistic(avg_jerk / EXPECTED_JERK_IN_ONE_SEC );
}

double out_of_middle_lane_cost(vector<double> d_traj) {
  
  // Penalize not staying in the middle lane
  
  double end_d      = d_traj[d_traj.size()-1];
  int d_middle_lane = LANE_WIDTH + 2;
  return logistic(pow(end_d-d_middle_lane, 2));
}

double calculate_total_cost(vector<vector<double>> &traj, map<int,vector<vector<double>>> &predictions) {
  
  vector<double> s_traj = traj[0];
  vector<double> d_traj = traj[1];
  
  vector<double> weighted_costs;
  double weighted_col = collision_cost(s_traj, d_traj, predictions) * COLLISION_COST_WEIGHT;
  double weighted_buf = buffer_cost(s_traj, d_traj, predictions) * BUFFER_COST_WEIGHT;
  double weighted_ilb = in_lane_buffer_cost(s_traj, d_traj, predictions) * IN_LANE_BUFFER_COST_WEIGHT;
  double weighted_eff = efficiency_cost(s_traj) * EFFICIENCY_COST_WEIGHT;
  double weighted_oml = out_of_middle_lane_cost(d_traj) * NOT_MIDDLE_LANE_COST_WEIGHT;
  
  weighted_costs.push_back(weighted_col);
  weighted_costs.push_back(weighted_buf);
  weighted_costs.push_back(weighted_ilb);
  weighted_costs.push_back(weighted_eff);
  weighted_costs.push_back(weighted_oml);
  
//  double weighted_esl = exceed_speed_limit_cost(s_traj) * SPEED_LIMIT_COST_WEIGHT;
//  double weighted_mas = max_accel_cost(s_traj) * MAX_ACCEL_COST_WEIGHT;
//  double weighted_aas = avg_accel_cost(s_traj) * AVG_ACCEL_COST_WEIGHT;
//  double weighted_mad = max_accel_cost(d_traj) * MAX_ACCEL_COST_WEIGHT;
//  double weighted_aad = avg_accel_cost(d_traj) * AVG_ACCEL_COST_WEIGHT;
//  double weighted_mjs = max_jerk_cost(s_traj) * MAX_JERK_COST_WEIGHT;
//  double weighted_ajs = avg_jerk_cost(s_traj) * AVG_JERK_COST_WEIGHT;
//  double weighted_mjd = max_jerk_cost(d_traj) * MAX_JERK_COST_WEIGHT;
//  double weighted_ajd = avg_jerk_cost(d_traj) * AVG_JERK_COST_WEIGHT;
//  double weighted_tdiff = time_diff_cost(target_time, actual_time) * TIME_DIFF_COST_WEIGHT;
//  double weighted_strajd = traj_diff_cost(s_traj, target_s) * TRAJ_DIFF_COST_WEIGHT;
//  double weighted_dtrajd = traj_diff_cost(d_traj, target_d) * TRAJ_DIFF_COST_WEIGHT;

  // Sum up all the costs
  double total_cost = 0;
  for (auto weighted_cost : weighted_costs) {
    total_cost += weighted_cost;
  }

  // Print costs
  bool display_costs = false;
  if (display_costs) {
    cout << "Costs:" << endl;;
    cout << "weighted_col: " << weighted_col << endl;
    cout << "weighted_buf: " << weighted_buf << endl;
    cout << "weighted_ilb: " << weighted_ilb << endl;
    cout << "weighted_eff: " << weighted_eff << endl;
    cout << "weighted_oml: " << weighted_oml << endl;
    cout << "total cost: " << total_cost << endl;
  }
  
  return total_cost;
}

#endif