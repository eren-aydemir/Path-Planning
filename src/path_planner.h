//
//  path_planner.h
//  Path_Planning
//
//  Created by Eren on 30.09.2017.
//
//

#ifndef path_planner_h
#define path_planner_h

#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "settings.h"
#include "vehicle.h"
#include "behavior_planner.h"
#include "helper.h"
#include "spline.h"

using namespace std;

struct PathPlanner{
  
  XYPoints traj;
  
  PathPlanner();
  
  State state;
  
  //XYPoints update(Vehicle &vehicle, const MapWaypoints &map_waypoints, const XYPoints &prevPath);

  
};


#endif /* path_planner_h */
