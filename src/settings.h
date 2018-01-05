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

/** @brief Maximum longitudinal position along the highway map. */
#define S_MAX 6945.554

/** @brief Maximum longitudinal position along the highway map. */
const double BIG_NUMBER = std::numeric_limits<double>::max();

/** @brief Equation to calculate which lane a vehicle belong to. */
#define LANE(l) (2. + l * 4.)

/** @brief Maximum number of lanes that given highway has. */
#define MAX_LANES 3.0

/** @brief Maximum speed that given highway permits. */
#define TARGET_VELOCITY 22

/** @brief Penalties. */
#define PENALTY_NOT_ON_RIGHT_MOST_LANE 1e4
#define PENALTY_ON_DISTANCE 1e4
#define PENALTY_ON_VELOCITY 1e3

enum class LaneType {
  LEFT = 2, MID = 1, RIGHT = 0, NONE
};

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
