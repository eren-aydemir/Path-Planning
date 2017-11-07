//
//  vehicle.cpp
//  Path_Planning
//
//  Created by Eren on 29.09.2017.
//
//

#include <stdio.h>
#include "vehicle.h"
#include <typeinfo>

Vehicle::Vehicle(const int i){
  this->id = i;
  this->state.current_lane = 1;
  this->state.d = 6;
  
  
}

Vehicle::Vehicle(){

}

 void Vehicle::update_state(const State &state)
 {
 
 this->state.x = state.x;
 this->state.y = state.y;
 this->state.s = state.s;
 this->state.d = state.d;
 this->state.speed = state.speed;
 
 if((this->state.d > 0.0) && (this->state.d < 4.0)){
   this->state.current_lane = 0; //LEFT

 }else if((this->state.d > 4.0) && (this->state.d < 8.0)){
   this->state.current_lane = 1; //MID

 }else if((this->state.d > 8.0) && (this->state.d < 12.0)){
   this->state.current_lane = 2; //RIGHT
 }
 
 };
 
void Vehicle::update_state(const nlohmann::json &json)
 {
 
 this->state.x = json[1]["x"];
 this->state.y = json[1]["y"];
 this->state.yaw = json[1]["yaw"];
 this->state.s = json[1]["s"];
 this->state.d = json[1]["d"];
 this->state.speed = json[1]["speed"];
 
 if((this->state.d > 0.0) && (this->state.d < 4.0)){
   this->state.current_lane = 0; //LEFT
     
 }else if((this->state.d > 4.0) && (this->state.d < 8.0)){
   this->state.current_lane = 1; //MID
     
 }else if((this->state.d > 8.0) && (this->state.d < 12.0)){
   this->state.current_lane = 2; //RIGHT
 }
   
 if (json[1]["previous_path_x"].size() > 0)
 {
   this->state.s = json[1]["end_path_s"];
 }
   
 };


vector<double> obstaclesSearch(Vehicle &egoVehicle, vector<Vehicle> &otherVehicles, int searchLane){
  
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

void Vehicle::obstacles(vector<Vehicle> &otherVehicles, int searchLane){
  
  vector<double> result;
  
  result = obstaclesSearch(*this, otherVehicles, this->state.current_lane);
  
  this->forward_gap_mylane = result[0];
  this->backward_gap_mylane = result[1];
  this->follow_speed = result[2];
  
  if(this->state.current_lane != 0){ //@leftmost there is no left lane.
    result = obstaclesSearch(*this, otherVehicles, this->state.current_lane-1);
  
    this->forward_gap_leftlane = result[0];
    this->backward_gap_leftlane = result[1];
    this->follow_speed = result[2];
    
  }else{
    
    this->forward_gap_leftlane = 0;
    this->backward_gap_leftlane = 0;
    //this->follow_speed = V_PLAN;
  }
  
  if(this->state.current_lane != 2){ //@rightmost there is no right lane.
    result = obstaclesSearch(*this, otherVehicles, this->state.current_lane+1);
    
    this->forward_gap_rightlane = result[0];
    this->backward_gap_rightlane = result[1];
    this->follow_speed = result[2];
    
  }else{
    
    this->forward_gap_rightlane = 0;
    this->backward_gap_rightlane = 0;
    //this->follow_speed = V_PLAN;
  }
  
};

 /*
 void Vehicle::save_state(const State &state){
 this->ref_state.x = state.x;
 this->ref_state.y= state.y;
 this->ref_state.yaw = state.yaw;
 this->ref_state.s = state.s;
 this->ref_state.d = state.d;
 this->ref_state.speed = state.speed;
 };
 */
