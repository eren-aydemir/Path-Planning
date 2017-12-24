//
//  highway.hpp
//  Path_Planning
//
//  Created by Eren on 15.10.2017.
//
//

#ifndef highway_hpp
#define highway_hpp

#include <stdio.h>
#include <iostream>
#include <fstream>
#include "json.hpp"
#include "settings.h"
#include "vehicle.h"

using namespace std;

struct Highway{
  
  int lane_size;
  vector<double> flow_speed;
  vector<double> forward_gap;
  vector<double> backward_gap;
  
  Highway();
  Highway(const int lane_size);
  
  void update(Vehicle &egoVehicle, vector<Vehicle> &otherVehicles);
  void obstacles(Vehicle &egoVehicle, vector<Vehicle> &otherVehicles, Highway &highway);
  vector<double> obstaclesSearch(Vehicle &egoVehicle, vector<Vehicle> &otherVehicles, int searchLane);
};

#endif /* highway_hpp */
