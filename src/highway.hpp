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

class Highway{
public:
  
  int lane_size;
  
  vector<double> costs;
  vector<double> ttcollision;
  vector<double> flow_speed;
  vector<double> forward_gap;
  vector<double> backward_gap;
  vector<double> fwd_gap;
  
  Highway();
  Highway(const int lane_size);
  
  void update(Vehicle &egoVehicle, vector<Vehicle> &otherVehicles);
  
};

#endif /* highway_hpp */
