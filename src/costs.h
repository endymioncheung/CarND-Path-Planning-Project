#ifndef COSTS
#define COSTS

#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>
#include <cmath>

#include "constants.h"

using namespace std;

/* ******************************************** */
//      Helper functions for cost functions     //
/* ******************************************** */

double logistic(double x){
  
  // A function that always returns a value
  // between 0 and 1 for x in the range[0, infinity] and
  // - 1 to 1 for x in the range[-infinity, infinity]

  return 2.0 / (1 + exp(-x)) - 1.0;
}

vector<double> velocities_in_traj(vector<double> &traj) {
  
  // Return a list of average velocities for each
  // pair of adjacent trajectory points
  
  vector<double> velocities;
  for (int i = 1; i < traj.size(); i++) {
    velocities.push_back((traj[i] - traj[i-1]) / DT);
  }
  return velocities;
}

double nearest_approach(vector<double> &s_traj, vector<double> &d_traj,
                        vector<Vehicle> &prediction) {
  
  // Find the closest distance between the trajectory and the prediction
  
  double closest_distance = 999999;
  for (int i = 0; i < N_SAMPLES; i++) {
    double current_distance = sqrt(pow(s_traj[i] - prediction[i].s, 2) + pow(d_traj[i] - prediction[i].d, 2));
    if (current_distance < closest_distance) {
      closest_distance = current_distance;
    }
  }
  return closest_distance;
}

double nearest_approach_to_any_cars(vector<double> &s_traj, vector<double> &d_traj,
                                    map<int,vector<Vehicle>> &predictions) {
  
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
                                            map<int,vector<Vehicle>> &predictions) {
  
  // Find the closest distance between the trajactory and
  // predictions of other vehicles within the same lane
  
  // Ego vehicle target lane
  double ego_end_d = d_traj[d_traj.size() - 1];
  int  target_lane = ego_end_d / LANE_WIDTH;
  
  double closest_distance = 999999;
  for (auto prediction : predictions) {
    
    // Non-ego vehicle predicted lane
    vector<Vehicle> pred_traj = prediction.second;
    double pred_d = pred_traj[pred_traj.size() - 1].d;
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

/* ******************************************** */
// Cost Functions for evaluating effectiveness  //
//       of the path planning trajectory        //
/* ******************************************** */

double collision_cost(vector<double> &s_traj, vector<double> &d_traj,
                      map<int,vector<Vehicle>> &predictions) {
  
  // Binary cost function that penalizes collisions
  
  double nearest = nearest_approach_to_any_cars(s_traj, d_traj, predictions);
  if (nearest < 2 * VEHICLE_RADIUS) {
    return 1;
  } else {
    return 0;
  }
}

double buffer_cost(vector<double> &s_traj, vector<double> &d_traj, map<int,vector<Vehicle>> &predictions) {
  
  // Penalize getting close to other vehicles
  
  double nearest = nearest_approach_to_any_cars(s_traj, d_traj, predictions);
  return logistic(2 * VEHICLE_RADIUS / nearest);
}

double in_lane_buffer_cost(vector<double> &s_traj, vector<double> &d_traj, map<int,vector<Vehicle>> &predictions) {
  
  // Penalize getting close to other vehicles in the lane
  
  double nearest = nearest_approach_to_any_cars_in_lane(s_traj, d_traj, predictions);
  return logistic(2 * VEHICLE_RADIUS / nearest);
}

double efficiency_cost(vector<double> &s_traj) {
  
  // Reward high average speeds
  
  vector<double> s_dot_traj = velocities_in_traj(s_traj);
  double final_s_dot = s_dot_traj[s_dot_traj.size() - 1];

  return logistic((SPEED_LIMIT - final_s_dot) / SPEED_LIMIT);
}

double out_of_middle_lane_cost(vector<double> d_traj) {
  
  // Penalize not staying in the middle lane
  
  double end_d      = d_traj[d_traj.size()-1];
  int d_middle_lane = LANE_WIDTH + 2;
  return logistic(pow(end_d-d_middle_lane, 2));
}

double calculate_cost(vector<vector<double>> &traj, map<int,vector<Vehicle>> &predictions) {
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
