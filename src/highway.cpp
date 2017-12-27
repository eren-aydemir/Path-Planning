//
//  highway.cpp
//  Path_Planning
//
//  Created by Eren on 15.10.2017.
//
//

#include "highway.hpp"

double notKeepingRightPenalty(double myLane) {
  return (2 - myLane) * 200;
};

double lowDistancePenalty(double lane, Vehicle egoVehicle, vector<Vehicle> otherVehicles) {
  
  const double CLEAR_ROAD = 150;
  double closestCarAhead = CLEAR_ROAD;
  double car_s = egoVehicle.state.s;
  
  for(int i = 0; i < otherVehicles.size(); i++) {
    
  		double d = otherVehicles[i].state.d;
    
  		// is a vehicle in my lane? I occupy the range [(car_d) - 2, (car_d) + 2]
  		if(d > (LANE(lane)) - 2 && d < (LANE(lane)+2)){
        //YES, how far away is it?
        double other_veh_speed = otherVehicles[i].state.speed;
        double other_veh_s = otherVehicles[i].state.s;
        
        if(other_veh_s > car_s){
          double dist = other_veh_s - car_s;
          closestCarAhead = min(closestCarAhead, dist);
        }
      }
  }
  
  return PENALTY_ON_DISTANCE * ((CLEAR_ROAD-closestCarAhead)/CLEAR_ROAD);
}

double slowLanePenalty(double lane, Vehicle egoVehicle, vector<Vehicle> otherVehicles) {
  double speedCarAhead = TARGET_VELOCITY;
  double ref_target = speedCarAhead;
  double car_s = egoVehicle.state.s;
  
  for(int i = 0; i < otherVehicles.size(); i++) {
  		double d = otherVehicles[i].state.d;
    
  		// is a vehicle in my lane? I occupy the range [(car_d) - 2, (car_d) + 2]
  		if(d > (LANE(lane)) - 2 && d < (LANE(lane)+2)){
        //YES, how far away is it?
        double other_veh_speed = otherVehicles[i].state.speed;
        double other_veh_s = otherVehicles[i].state.s;
        
        if(other_veh_s > car_s){
          speedCarAhead = min(speedCarAhead, other_veh_speed);
        }
      }
  }
  return (TARGET_VELOCITY - speedCarAhead) * PENALTY_ON_VELOCITY;
}

// this function estimates how much other traffic participants have when the ego performs an action. How much time is left assuming the follower vehicle does an emergency break down to the velocity of the ege vehicle minus an optimistic 0.5 second reaction time
// the higher the values the better
double reactionTime(double lane, Vehicle &egoVehicle, vector<Vehicle> &otherVehicles) {
  
  double car_s = egoVehicle.state.s;
  double car_speed = egoVehicle.state.speed;
  double ret = 5.0;
  
  for(int i = 0; i < otherVehicles.size(); i++) {
  		
  		double d = otherVehicles[i].state.d;
    
  		// is a vehicle in my lane? I occupy the range [(car_d) - 2, (car_d) + 2]
  		//if(egoVehicle.state.current_lane == otherVehicles[i].state.current_lane){
      if(d > (LANE(lane)) - 2 && d < (LANE(lane)+2)){
        //YES, how far away is it?
        double other_veh_speed = otherVehicles[i].state.speed;
        double other_veh_s = otherVehicles[i].state.s;
        
        if((other_veh_s > car_s) && (other_veh_s - car_s) < 25.) { //following ahead vehicle closely
          ret = 0.5;
        }
        
        if((other_veh_s < car_s)) {
          double distanceBackwards = car_s - other_veh_s;
          
          const double a = 7.0; //full brake m/s2
          
          double bdist = (pow(other_veh_speed,2) - pow(car_speed,2)) / (2*a); //required distance to stop at full brake
          double ttcoll = ((distanceBackwards - bdist) / car_speed) - 0.5; // 0.5 seconds to react
          
          if(ttcoll < 0)
            ttcoll = 0;
          
          ret = min(ret, ttcoll);
        }
      }
  }
  return ret;
}

double flowSpeed(double lane, Vehicle &egoVehicle, vector<Vehicle> &otherVehicles) {
  
  double car_s = egoVehicle.state.s;
  double gap = 0;
  double closest = BIG_NUMBER;
  double flow_speed = INFINITY;
  
  for(int i = 0; i < otherVehicles.size(); i++) {
    
  		double d = otherVehicles[i].state.d;
      double other_veh_s = otherVehicles[i].state.s;
    
  		// is a vehicle in my lane? I occupy the range [(car_d) - 2, (car_d) + 2]
  		//if(egoVehicle.state.current_lane == otherVehicles[i].state.current_lane){
    if(d > (LANE(lane)) - 2 && d < (LANE(lane)+2)){
      if(other_veh_s > car_s) { //following ahead vehicle closely
        gap = other_veh_s - car_s;
        if (gap < closest){
          closest = gap;
          flow_speed = otherVehicles[i].state.speed * 2.27;
        }
      }
    }
  }
  return flow_speed;
};

double fwdGap(double lane, Vehicle &egoVehicle, vector<Vehicle> &otherVehicles) {
  
  double car_s = egoVehicle.state.s;
  double car_d = egoVehicle.state.d;
  double gap = 0;
  double fwd_gap = INFINITY;
  int counter = 0;
  
  for(int i = 0; i < otherVehicles.size(); i++) {
    
    double d = otherVehicles[i].state.d;
    double other_veh_s = otherVehicles[i].state.s;
    
  		// is a vehicle in my lane? I occupy the range [(car_d) - 2, (car_d) + 2]
  		//if(egoVehicle.state.current_lane == otherVehicles[i].state.current_lane){
    if(d > (LANE(lane)) - 2 && d < (LANE(lane)+2)){
      if(other_veh_s > car_s) { //following ahead vehicle closely
        gap = other_veh_s - car_s;
        
        if (gap > S_MAX / 2){
          gap = gap - S_MAX;
        }
        if (gap < -1 * S_MAX / 2){
          gap = gap + S_MAX;
        }
        
        if (gap < fwd_gap){
          fwd_gap = gap;
        }
        counter++;
        //cout << "Vehicle id: " << otherVehicles[i].id << " meters away: " << otherVehicles[i].state.s - egoVehicle.state.s << endl;
      }
    }
  }
  cout << "Number of vehicle: " << counter << endl;
  if (fwd_gap == INFINITY){fwd_gap = 0;};
  return fwd_gap;
};

Highway::Highway(){
  
};

Highway::Highway(const int lane_size){

  this->lane_size = lane_size;
  this->forward_gap = {0.0, 0.0, 0.0};
  this->backward_gap = {0.0, 0.0, 0.0};
  this->flow_speed = {0.0, 0.0, 0.0};
  
};

void Highway::update(Vehicle &egoVehicle, vector<Vehicle> &otherVehicles){
  
  this->costs = {0.0, 0.0, 0.0};
  this->ttcollision = {0.0, 0.0, 0.0};
  this->flow_speed = {0.0, 0.0, 0.0};
  this->fwd_gap = {0.0, 0.0, 0.0};
  
  //determine costs
  for(int i = 0; i < MAX_LANES; i++) {
    double toLane = double(i);
    // a penalty on not keeping right when possible leads to local minima the car can only get out with a double lane change.
    // this requires different checks if both lane is clear
    //this->costs[i] += notKeepingRightPenalty(toLane);
    this->costs[i] += lowDistancePenalty(toLane, egoVehicle, otherVehicles);
    this->costs[i] += slowLanePenalty(toLane, egoVehicle, otherVehicles);
    this->ttcollision[i] = reactionTime(toLane, egoVehicle, otherVehicles);
    this->costs[i] = this->costs[i] / this->ttcollision[i];
    this->flow_speed[i] = flowSpeed(toLane, egoVehicle, otherVehicles);
    this->fwd_gap[i] = fwdGap(toLane, egoVehicle, otherVehicles);

  }
};
