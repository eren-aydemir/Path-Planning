//
//  vehicle.h
//  Path_Planning
//
//  Created by Eren on 29.09.2017.
//
//

#ifndef vehicle_h
#define vehicle_h

#include <stdio.h>
#include <iostream>
#include <fstream>
#include "json.hpp"
#include "settings.h"

using namespace std;

struct Vehicle{
  
  int id;
  State state;
  
  double forward_gap_mylane;
  double backward_gap_mylane;
  double forward_gap_leftlane;
  double backward_gap_leftlane;
  double forward_gap_rightlane;
  double backward_gap_rightlane;
  double follow_speed;
  //State ref_state;
  
  Vehicle(const int i);
  Vehicle();

  void update_state(const nlohmann::json &json);
  void update_state(const State &state);
  void obstacles(vector<Vehicle> &otherVehicles, int searchLane);
  //void save_state(const State &state);
  
};


#endif /* vehicle_h */
