//
//  trajectory_planner.h
//  Path_Planning
//
//  Created by Eren on 2.10.2017.
//
//

#ifndef trajectory_planner_h
#define trajectory_planner_h

#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "settings.h"
#include "helper.hpp"
#include "vehicle.h"
#include "spline.h"

//#include "spline.h"
//#include "json.hpp"

using namespace std;

struct TrajectoryPlanner{
  
  XYPoints traj;
  
  TrajectoryPlanner();
  
  //State state;
  
  void update(const Vehicle &vehicle, const MapWaypoints &map_waypoints, const XYPoints &prevPath);
  XYPoints test_run(const Vehicle &vehicle);
  
};

#endif /* trajectory_planner_h */
