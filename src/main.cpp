#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "helpers.h"
#include "constants.h"
#include "spline_wrapper.h"
#include "vehicle.h"
//#include "costs.h"
#include "classifier.h"

// for convenience
using namespace std;
using json = nlohmann::json;

using std::cout;
using std::endl;
using std::ifstream;

int main() {
  // Read in the training and testing datasets
  // Each observation is a tuple with 4 values: s, d, s_dot and d_dot
  vector<vector<double>> X_train = Load_State("./all_states.txt");
  vector<string>         Y_train = Load_Label("./all_labels.txt");

  // Experiment: making both training and test datasets the same
  vector<vector<double>> X_test = X_train;
  vector<string>         Y_test = Y_train;
  
  // Gaussian Naive Bayes (GNB) classifier
  GNB gnb = GNB();
  
  // Train GNB classifier with datasets
  gnb.train(X_train, Y_train);
  
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Initalize ego vehicle to track the state
  Vehicle ego_car = Vehicle();

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  // The max s value before wrapping around the track back to 0
  // double max_s = TRACK_LENGTH; // not used

  // Open and highway waypoints CSV
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  // Load the highway waypoints CSV to data
  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x, y, s;
  	float d_x, d_y;

  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;

  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &ego_car, &gnb](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          // Ego car's localization Data
          double car_x     = j[1]["x"];
          double car_y     = j[1]["y"];
          double car_s     = j[1]["s"];
          double car_d     = j[1]["d"];
          double car_yaw   = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Convert vehicle speed from MPH to m/s
          car_speed *= MPH2MPS;

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          // Previous path's end s and d values
          // double end_path_s    = j[1]["end_path_s"]; // not used
          // double end_path_d    = j[1]["end_path_d"]; // not used

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //****************************************************//
          //   Create higher resolution interpolated waypoints  //
          //****************************************************//

          int num_waypoints       = map_waypoints_x.size(); // highway_map.csv has 181 waypoints
          int next_waypoint_index = NextWaypoint(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
          vector<double> coarse_waypoints_s, coarse_waypoints_x, coarse_waypoints_y,
                                            coarse_waypoints_dx, coarse_waypoints_dy;

          // Sample waypoints behind and ahead of the ego vehicle
          for (int i = -NUM_WAYPOINTS_BEHIND; i < NUM_WAYPOINTS_AHEAD; i++) {

            // Referencing the waypoints behind index
            // from the end of the highway map waypoints
            int idx = (next_waypoint_index+i) % num_waypoints;
            if (idx < 0) {
                idx += num_waypoints;
            }

            // Correction for spline interpolation
            // at the end of the track
            double current_s = map_waypoints_s[idx];
            double next_s    = map_waypoints_s[next_waypoint_index];
            if (i < 0 && current_s > next_s) {
                current_s -= TRACK_LENGTH;
            }
            if (i > 0 && current_s < next_s) {
                current_s += TRACK_LENGTH;
            }
            coarse_waypoints_s.push_back(current_s);
            coarse_waypoints_x.push_back(map_waypoints_x[idx]);
            coarse_waypoints_y.push_back(map_waypoints_y[idx]);
            coarse_waypoints_dx.push_back(map_waypoints_dx[idx]);
            coarse_waypoints_dy.push_back(map_waypoints_dy[idx]);
          }

          // Waypoints interpolation
          int num_interpolation_points = (coarse_waypoints_s[coarse_waypoints_s.size()-1] - coarse_waypoints_s[0]) / WAYPOINT_DIST_INCREMENT;
          vector<double> interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y, interpolated_waypoints_dx, interpolated_waypoints_dy;
          interpolated_waypoints_x  = interpolate_points(coarse_waypoints_s, coarse_waypoints_x,  WAYPOINT_DIST_INCREMENT, num_interpolation_points);
          interpolated_waypoints_y  = interpolate_points(coarse_waypoints_s, coarse_waypoints_y,  WAYPOINT_DIST_INCREMENT, num_interpolation_points);
          interpolated_waypoints_dx = interpolate_points(coarse_waypoints_s, coarse_waypoints_dx, WAYPOINT_DIST_INCREMENT, num_interpolation_points);
          interpolated_waypoints_dy = interpolate_points(coarse_waypoints_s, coarse_waypoints_dy, WAYPOINT_DIST_INCREMENT, num_interpolation_points);

          // Interpolated Freenet s position
          interpolated_waypoints_s.push_back(coarse_waypoints_s[0]);
          for (int i = 1; i < num_interpolation_points; i++) {
              interpolated_waypoints_s.push_back(coarse_waypoints_s[0] + i * WAYPOINT_DIST_INCREMENT);
          }

          //****************************************//
          //     Sub path and ego vehicle states    //
          //****************************************//

          double s_pos, s_dot, s_ddot;
          double d_pos, d_dot, d_ddot;

          double ref_x1, ref_y1, ref_x2, ref_y2, ref_x3, ref_y3, ref_angle,
                 vel_x1, vel_y1, vel_x2, vel_y2, acc_x, acc_y;

          int subpath_size = min(PREVIOUS_PATH_POINTS_TO_KEEP, (int)previous_path_x.size());
          double traj_start_time = subpath_size * PATH_DT;

          // If the sub path size is almost empty, use the car as starting reference
          if (subpath_size < MIN_PATH_POINTS) {
            ref_x1 = car_x;
            ref_y1 = car_y;
            ref_angle = deg2rad(car_yaw);

            s_pos  = car_s;
            d_pos  = car_d;
            s_dot  = car_speed;
            d_dot  = 0;
            s_ddot = 0;
            d_ddot = 0;

          } else {
            // Update current vehicle position to be last point of previous path
            ref_x1     = previous_path_x[subpath_size-1];
            ref_y1     = previous_path_y[subpath_size-1];
            ref_x2     = previous_path_x[subpath_size-2];
            ref_y2     = previous_path_y[subpath_size-2];
            ref_x3     = previous_path_x[subpath_size-3];
            ref_y3     = previous_path_y[subpath_size-3];
            ref_angle  = atan2(ref_y1-ref_y2,ref_x1-ref_x2);
            vector<double> pt_frenet = getFrenet(ref_x1, ref_y1, ref_angle, interpolated_waypoints_x, interpolated_waypoints_y, interpolated_waypoints_s);
            s_pos = pt_frenet[0];
            d_pos = pt_frenet[1];

            // Calculate the dx, dy normal vector from the set of interpoated waypoints
            int next_interp_waypoint_index = NextWaypoint(ref_x1, ref_y1, ref_angle, interpolated_waypoints_x, interpolated_waypoints_y);
            double dx = interpolated_waypoints_dx[next_interp_waypoint_index - 1];
            double dy = interpolated_waypoints_dy[next_interp_waypoint_index - 1];

            // sx,sy vector is perpendicular to dx,dy
            double sx = -dy;
            double sy = dx;

            // Calculate velocity for Freenet coordinate
            vel_x1 = (ref_x1 - ref_x2) / PATH_DT;
            vel_y1 = (ref_y1 - ref_y2) / PATH_DT;

            // Project velocity vector onto S (sx,sy) and D (dx,dy) vectors
            s_dot = vel_x1 * sx + vel_y1 * sy;
            d_dot = vel_x1 * dx + vel_y1 * dy;

            // Calculate acceleration for Freenet coordinate
            vel_x2 = (ref_x2 - ref_x3) / PATH_DT;
            vel_y2 = (ref_y2 - ref_y3) / PATH_DT;
            acc_x  = (vel_x1 - vel_x2) / PATH_DT;
            acc_y  = (vel_y1 - vel_y2) / PATH_DT;

            // Project acceleration vector onto S (sx,sy) and D (dx,dy) vectors
            s_ddot = acc_x * sx + acc_y * sy;
            d_ddot = acc_x * dx + acc_y * dy;
          }

          // Set the ego vehicle Freenet coordinates (position, velocity and acceleration)
          ego_car.s      = s_pos;
          ego_car.s_dot  = s_dot;
          ego_car.s_ddot = s_ddot;
          
          ego_car.d      = d_pos;
          ego_car.d_dot  = d_dot;
          ego_car.d_ddot = d_ddot;
          
          ego_car.lane   = d_pos / LANE_WIDTH;

          //**********************************************************//
          // Generate trajectory predictions using sensor fusion data //
          //**********************************************************//

          // Sensor fusion data format for each car in the CSV file is:
          // [id, x, y, vx, vy, s, d]

          double duration = N_SAMPLES * DT - subpath_size * PATH_DT; // 20* 0.2 -subpath_size * 0.02
          // double dt    = traj_start_time + duration;
          map<int,vector<Vehicle>> predictions;
          
          vector<Vehicle> other_cars;
          for (auto car_sf: sensor_fusion) {
            int    other_car_id    = car_sf[0];   // Vehicle ID
            //double other_car_x   = car_sf[1];   // not used; global map coordinate x
            //double other_car_y   = car_sf[2];   // not used; global map coordinate y
            double other_car_vx    = car_sf[3];   // Velocity x-component
            double other_car_vy    = car_sf[4];   // Velocity y-component
            double other_car_s     = car_sf[5];   // Frenet coordinate, s
            double other_car_d     = car_sf[6];   // Frenet coordinate, d
            int    other_car_lane  = other_car_d / LANE_WIDTH;
            double other_car_vel   = sqrt(pow(other_car_vx, 2) + pow(other_car_vy, 2));
            string other_car_state = "CS";        // Assume the non-ego vehicles are in constant speed initally
            
//            double next_s = other_car_s + this->s_dot * dt;
//            double next_d = other_car_d + this->d_dot * dt;
            
            // Store the other car sensor fusion data
            // to list of vehicle objects
            Vehicle other_car = Vehicle(other_car_lane, other_car_state, other_car_s, other_car_vel, 0, other_car_d, 0, 0);
            other_car.lane    = other_car_lane;
            other_cars.push_back(other_car);

            // Generate and store prediction for each non-ego vehicle
            vector<Vehicle> preds = other_car.generate_predictions(traj_start_time, duration);
            predictions[other_car_id] = preds;
          }

          //*****************************//
          //   Finding the best target   //
          //*****************************//
          
          ego_car.check_nearby_cars(other_cars);
          Vehicle best_target = ego_car.choose_next_state(predictions,traj_start_time, duration);
          ego_car.realize_next_state(best_target);
          
//          Vehicle vehicle_ahead;
//          ego_car.get_vehicle_ahead(predictions,ego_car.lane,vehicle_ahead);
//          
//          Vehicle vehicle_behind;
//          ego_car.get_vehicle_behind(predictions,ego_car.lane,vehicle_behind);
          
          //*****************************//
          //      Creating new path      //
          //*****************************//

          // Converting the Freenet coordinate back to the global map coordinate
          
          vector<double> coarse_s_traj, coarse_x_traj, coarse_y_traj,
              interpolated_s_traj,interpolated_x_traj, interpolated_y_traj;

          // Ensure spline begins smoothly by setting the first two points of
          // coarse trajectory based on the last two points of previous path
          double prev_s = s_pos - s_dot * PATH_DT;
          if (subpath_size >= 2) {
              coarse_s_traj.push_back(prev_s);
              coarse_x_traj.push_back(previous_path_x[subpath_size-2]);
              coarse_y_traj.push_back(previous_path_y[subpath_size-2]);

              coarse_s_traj.push_back(s_pos);
              coarse_x_traj.push_back(previous_path_x[subpath_size-1]);
              coarse_y_traj.push_back(previous_path_y[subpath_size-1]);
          } else {
              double prev_s = s_pos - 1;
              double prev_x = ref_x1 - cos(ref_angle);
              double prev_y = ref_y1 - sin(ref_angle);

              coarse_s_traj.push_back(prev_s);
              coarse_x_traj.push_back(prev_x);
              coarse_y_traj.push_back(prev_y);

              coarse_s_traj.push_back(s_pos);
              coarse_x_traj.push_back(ref_x1);
              coarse_y_traj.push_back(ref_y1);
          }

          // Last two points of coarse trajectory:
          // current s + 30 meters, 60 meters
          double target_s1 = s_pos + 30;
          double target_d1 = best_target.d;
          vector<double> target_x1y1 = getXY(target_s1, target_d1, interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y);
          double target_x1 = target_x1y1[0];
          double target_y1 = target_x1y1[1];
          coarse_s_traj.push_back(target_s1);
          coarse_x_traj.push_back(target_x1);
          coarse_y_traj.push_back(target_y1);

          double target_s2 = s_pos + 60;
          double target_d2 = best_target.d;
          vector<double> target_x2y2 = getXY(target_s2, target_d2, interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y);
          double target_x2 = target_x2y2[0];
          double target_y2 = target_x2y2[1];
          coarse_s_traj.push_back(target_s2);
          coarse_x_traj.push_back(target_x2);
          coarse_y_traj.push_back(target_y2);

          double current_s      = s_pos;
          double current_s_dot  = s_dot;
          // double current_s_ddot = s_ddot; // not used

          double target_s_dot   = best_target.s_dot;
          for (int i = 0; i < (NUM_PATH_POINTS - subpath_size); i++) {
            double vel_increment;
            if (fabs(target_s_dot - current_s_dot) < 2 * VELOCITY_INCREMENT) {
              vel_increment = 0;
            } else {
              // Increase the ego vehicle speed if target speed is not reached yet
              // Decrease the ego vehicle speed if it is faster than target speed
              vel_increment = target_s_dot > current_s_dot? VELOCITY_INCREMENT: -VELOCITY_INCREMENT;
            }
            current_s_dot += vel_increment;
            current_s     += current_s_dot * PATH_DT;
            interpolated_s_traj.push_back(current_s);

          }

          interpolated_x_traj = interpolate_points(coarse_s_traj, coarse_x_traj, interpolated_s_traj);
          interpolated_y_traj = interpolate_points(coarse_s_traj, coarse_y_traj, interpolated_s_traj);

          // Resume the next path with the previous path points from last run
          for(int i = 0; i < subpath_size; i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }
          // Add the generated xy points to the planning path
          for (int i = 0; i < interpolated_x_traj.size(); i++) {
              next_x_vals.push_back(interpolated_x_traj[i]);
              next_y_vals.push_back(interpolated_y_traj[i]);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
