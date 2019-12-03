#ifndef SPLINE_WRAPPER
#define SPLINE_WRAPPER

#include <vector>
#include <iostream>
#include "spline.h"

using namespace std;

vector<double> interpolate_points(vector<double> pts_x, vector<double> pts_y,
                                  vector<double> eval_at_pts_x) {
  
  // Wrapper function to use the spline library to return interpolated y-value
  // for a given list of x-value data points based on a spline
  // of connected series of (x,y) points

  // Check if the data points size matches
  if (pts_x.size() != pts_y.size()) {
    cout << "ERROR: Mismatch data point size for pts_x and pts_y" << endl;
    return {-1.0};
  }

  // Create a spline
  tk::spline s;
  
  // Set (x,y) points to the spline
  s.set_points(pts_x,pts_y);
  
  // Interpolate y-value for each x-value
  vector<double> interpolated_pts_y;
  for (double x: eval_at_pts_x) {
    interpolated_pts_y.push_back(s(x));
  }
  return interpolated_pts_y;
}

vector<double> interpolate_points(vector<double> pts_x, vector<double> pts_y, 
                                       double interval, int output_size) {
  
  // Wrapper function to use the spline library to return
  // number of interpolated y-values with a given interval
  // based on a spline of connected series of (x,y) points
  
  // Check if the data points size matches
  if (pts_x.size() != pts_y.size()) {
    cout << "ERROR: Mismatch data point size for pts_x and pts_y" << endl;
    return {-1.0};
  }

  // Create a spline
  tk::spline s;
  
  // Set (x,y) points to the spline
  s.set_points(pts_x,pts_y);
  
  // Interpolate y-value for a specified interval
  vector<double> interpolated_pts_y;
  for (int i = 0; i < output_size; i++) {
    interpolated_pts_y.push_back(s(pts_x[0] + i * interval));
  }
  return interpolated_pts_y;
}

#endif
