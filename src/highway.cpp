//
//  highway.cpp
//  Path_Planning
//
//  Created by Eren on 15.10.2017.
//
//

#include "highway.hpp"

Highway::Highway(){
  
};

Highway::Highway(const int lane_size){

  this->lane_size = lane_size;
  this->forward_gap = {0.0, 0.0, 0.0};
  this->backward_gap = {0.0, 0.0, 0.0};
  this->flow_speed = {0.0, 0.0, 0.0};
  
};

void Highway::update(Vehicle &egoVehicle, vector<Vehicle> &otherVehicles){
  obstacles(egoVehicle, otherVehicles, *this);
};

vector<double> Highway::obstaclesSearch(Vehicle &egoVehicle, vector<Vehicle> &otherVehicles, int searchLane){
  
  double forward_gap = 5555555;
  double backward_gap = 5555555;
  double follow_speed = V_PLAN;
  double fwd_temp;
  double bwd_temp;

  for (int i=0; i<sizeof(otherVehicles); i++) {
    
    if (otherVehicles[i].state.current_lane == searchLane){
      if (otherVehicles[i].state.s > egoVehicle.state.s){ //forward
        
        fwd_temp = otherVehicles[i].state.s - egoVehicle.state.s;
        
        if (fwd_temp < forward_gap){
          forward_gap = fwd_temp;
          follow_speed = otherVehicles[i].state.speed;
        }
        
      }
      
      if (egoVehicle.state.s > otherVehicles[i].state.s){ //backward
        
        bwd_temp = egoVehicle.state.s - otherVehicles[i].state.s;
        
        if (bwd_temp < backward_gap){
          backward_gap = bwd_temp;
        }
        
      }
    }
  }

  return {forward_gap, backward_gap, follow_speed};
};

void Highway::obstacles(Vehicle &egoVehicle, vector<Vehicle> &otherVehicles, Highway &highway){
  
  vector<double> result;
  
  for (int i = 0; i<highway.lane_size; i++){
    
    result = obstaclesSearch(egoVehicle, otherVehicles, i);
    
    if (result.size() != 0){
      highway.forward_gap[i] = result[0];
      highway.backward_gap[i] = result[1];
      highway.flow_speed[i] = result[2];
    }else{
      highway.forward_gap[i] = BIG_NUMBER;
      highway.backward_gap[i] = BIG_NUMBER;
      highway.flow_speed[i] = V_PLAN;
    }
  }
};
