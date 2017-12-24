//
//  settings.h
//  Path_Planning
//
//  Created by Eren on 23.09.2017.
//
//

#ifndef settings_h
#define settings_h

using namespace std;

/** @brief Number of lanes in the highway map. */
#define N_LANES 3

/** @brief Width of each lane in meters. */
#define W_LANE 4.0

/** @brief Number of planned waypoints. */
#define N_PLAN 15

/** @brief Number of planned waypoints to keep between updates. */
#define N_KEEP 5

/** @brief Time gap between waypoints. */
#define T_PLAN 0.02

/** @brief Optimal speed in m/s. */
#define V_PLAN 22

/** @brief Order of the polynomial used to fit waypoints. */
#define N_FIT 3

/** @brief Number of sample waypoints extracted from the lane map. */
#define N_SAMPLES 2

/** @brief Maximum longitudinal position along the highway map. */
#define S_MAX 6945.554

/** @brief Maximum longitudinal position along the highway map. */
const double BIG_NUMBER = std::numeric_limits<double>::max();


enum class LaneType {
  LEFT = 2, MID = 1, RIGHT = 0, NONE
};

/*
enum class BehaviorType {
  START, CRUISE, TAIL, CHANGE_LANE
};
*/

/* State - stores seven doubles x, y, yaw, s, d, speed, lane
 * intended to store position, velocity, and acceleration components in the s, or d axis
 */

struct State {
  double x;
  double y;
  double yaw;
  double s;
  double d;
  double speed;
  double target_speed;
  int current_lane;
  int target_lane;
  int prev_path_size;
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  
};


/* XYPoints stores two vectors x, y which are the map coordinates to be passed to
 * the simulator. Also holds an int n intended to store the number of (x, y) pairs
 */
struct XYPoints {
  std::vector<double> xs;
  std::vector<double> ys;
  int n;
};

/* MapWaypoints stores three vectors x, y, s which are the....
 */
struct MapWaypoints {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
};

#endif /* settings_h */
