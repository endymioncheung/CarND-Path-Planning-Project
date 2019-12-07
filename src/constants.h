#ifndef CONSTANTS
#define CONSTANTS

//**********************************//
// Cost Function Weights [unitless] //
//**********************************//

#define COLLISION_COST_WEIGHT       99999
#define IN_LANE_BUFFER_COST_WEIGHT  1000
#define EFFICIENCY_COST_WEIGHT      1000
#define NOT_MIDDLE_LANE_COST_WEIGHT 100
#define BUFFER_COST_WEIGHT          10

// #define SPEED_LIMIT_COST_WEIGHT    9999
// #define MAX_ACCEL_COST_WEIGHT      9999
// #define MAX_JERK_COST_WEIGHT       9999
// #define AVG_ACCEL_COST_WEIGHT      1000
// #define AVG_JERK_COST_WEIGHT       1000
// #define TIME_DIFF_COST_WEIGHT      10
// #define TRAJ_DIFF_COST_WEIGHT      10

//***********************//
//    Unit Conversion    //
//***********************//

#define MPH2MPS 0.44704                   // [m/s]
#define MPS2MPH 2.23694                   // [mph]

//***********************//
//     Rules & Limits    //
//***********************//

#define SPEED_LIMIT              50       // Speed limit [mph]
#define BELOW_SPEED_LIMIT (SPEED_LIMIT-2) // Below speed limit [mph]
#define EXPECTED_ACC_IN_ONE_SEC  1.0      // m/s
#define EXPECTED_JERK_IN_ONE_SEC 2.0      // m/s/s
#define MAX_INSTANTANEOUS_ACCEL  10.0     // m/s/s
#define MAX_INSTANTANEOUS_JERK   10.0     // m/s/s/s

#define SPEED_DECREMENT    5              // Speed decrement [mph]
#define VELOCITY_INCREMENT 0.125          // Velocity increment limit [m/s]

//******************************//
// Vehicle and Track Dimensions //
//******************************//

#define TRACK_LENGTH    6945.554         // Overall track length [m]; approx. 4.32miles
#define NUM_LANES       3                // Number of lanes [counts]
#define FOLLOW_DISTANCE 10.0              // Gap follow distance to leading car [m]
#define VEHICLE_RADIUS  1.25             // Model vehicle as circle to simplify collision detection [m]

//***********************//
//         Lanes         //
//***********************//

#define LANE_WIDTH      4                // Lane width [m]

//****************************//
// Non-ego vehicle prediction //
// and trajectory generation  //
//****************************//

#define N_SAMPLES 20                      // Number of path samples for prediction
#define DT 0.2                            // [sec]

//************************//
// Ego Vehicle Path Point //
//************************//

#define NUM_PATH_POINTS              50 // Number of path points [counts]
#define PATH_DT                    0.02 // Time between waypoints along the path [sec]
#define MIN_PATH_POINTS               4 // Minimum number of path points [counts]
#define PREVIOUS_PATH_POINTS_TO_KEEP 25 // Number of points to keep in previous path [counts]

// Number of waypoints to use for interpolation around the ego vehicle
#define NUM_WAYPOINTS_BEHIND      5     // [counts]
#define NUM_WAYPOINTS_AHEAD       5     // [counts]
#define WAYPOINT_DIST_INCREMENT 0.5     // Distance increment [m]

// Sigma values for perturbing goal to target
#define SIGMA_S      10.0               // Sigma for longitudinal distance, s [m]
#define SIGMA_S_DOT  3.0                // Sigma for longitudinal velocity, s_dot [m/s]
#define SIGMA_S_DDOT 0.1                // Sigma for longitudinal acceleration, s_ddot [m/s/s]

#endif
