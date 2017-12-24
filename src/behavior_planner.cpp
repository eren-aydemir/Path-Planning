//
//  behavior_planner.cpp
//  Path_Planning
//
//  Created by Eren on 27.09.2017.
//
//

#include <stdio.h>
#include "behavior_planner.h"

vector<double> start(void){
  
  double target_speed = V_PLAN;
  double target_lane = 1;
  return {target_speed, target_lane, 0};
};

vector<double> cruise(void){
  
  double target_speed = V_PLAN;
  double target_lane = 1;
  return {target_speed, target_lane, 1};

};

vector<double> tail(void){
  
  double target_speed = 5;
  double target_lane = 1;
  
  return {target_speed, target_lane, 2};

};

vector<double> lanechange(void){
  return {V_PLAN, 0, 3};
};

void StateMachine_Init(stateMachine_t *stateMachine){
  
  static int count = 0;
  count += 1;
  
  if (count == 1){
    printf("Initialising state machine.\r\n");
    stateMachine->currBehavior = START;
  }
  
};

EventType StateMachine_GetEvent(Vehicle vehicle, stateMachine_t *stateMachine){
  
  //buraya eventler tanimlanacak
  //return EV0;
  static int count = 0;
  count += 1;
  
  EventType event = NONEV;
  
  int myLane = stateMachine->vehicle.state.current_lane;
  //stateMachine->lanechange_dir;
  BehaviorType currBehavior = stateMachine->currBehavior;
  
  bool left_fwd_free = stateMachine->highway.forward_gap[0] >= stateMachine->vehicle.state.speed;
  bool mid_fwd_free = stateMachine->highway.forward_gap[1] >= stateMachine->vehicle.state.speed;
  bool right_fwd_free = stateMachine->highway.forward_gap[2] >= stateMachine->vehicle.state.speed;
  
  bool left_bwd_free = stateMachine->highway.backward_gap[0] >= stateMachine->vehicle.state.speed/2;
  bool mid_bwd_free = stateMachine->highway.backward_gap[1] >= stateMachine->vehicle.state.speed/2;
  bool right_bwd_free = stateMachine->highway.backward_gap[2] >= stateMachine->vehicle.state.speed/2;
  /*
  bool forward_free = vehicle.forward_gap_mylane >= stateMachine->vehicle.state.speed;
  bool backward_free = vehicle.backward_gap_mylane >= stateMachine->vehicle.state.speed/2;
  bool left_forward_free = vehicle.forward_gap_leftlane >= stateMachine->vehicle.state.speed/2;
  bool right_forward_free = vehicle.forward_gap_rightlane >= stateMachine->vehicle.state.speed/2;
  bool left_backward_free = vehicle.backward_gap_leftlane >= stateMachine->vehicle.state.speed/2;
  bool right_backward_free = vehicle.backward_gap_rightlane >= stateMachine->vehicle.state.speed/2;
  */
  
  if(count == 1){ //START
    cout << "EV0" << endl;
    event = EV0; //CRUISE
  }else if(\
           (currBehavior == CRUISE && (myLane == 0 && (!mid_fwd_free || !mid_bwd_free))) ||\
           (currBehavior == CRUISE && (myLane == 1 && mid_fwd_free)) ||\
           (currBehavior == CRUISE && (myLane == 2 && (!mid_fwd_free || !mid_bwd_free)))\
           ){
    cout << "EV3" << endl;
    event = EV3; //CRUISE
    //stateMachine->lanechange_active = false;
    
  }else if(\
           (currBehavior == CRUISE && (myLane == 0 && !left_fwd_free)) ||\
           (currBehavior == CRUISE && (myLane == 1 && !mid_fwd_free)) ||\
           (currBehavior == CRUISE && (myLane == 2 && !right_fwd_free))\
           ){
    cout << "EV6" << endl;
    event = EV6; //TAIL
    //stateMachine->lanechange_active = false;
    
    
  }else if(\
           (currBehavior == TAIL && (myLane == 0 && left_fwd_free)) ||\
           (currBehavior == TAIL && (myLane == 1 && mid_fwd_free)) ||\
           (currBehavior == TAIL && (myLane == 2 && right_fwd_free))\
           ){
    cout << "EV5" << endl;
    event = EV5; //CRUISE
    
    
  }else if(
           (currBehavior == TAIL && (myLane == 0 && !left_fwd_free && !mid_fwd_free)) ||\
           (currBehavior == TAIL && (myLane == 1 && !mid_fwd_free && !left_fwd_free)) ||\
           (currBehavior == TAIL && (myLane == 2 && !right_fwd_free && !mid_fwd_free))\
           ){
    cout << "EV7" << endl;
    event = EV7; //TAIL
    
    
  }else if(\
           (currBehavior == TAIL && (myLane == 0 && !left_fwd_free && (mid_fwd_free && mid_bwd_free))) ||\
           (currBehavior == TAIL && (myLane == 1 && !mid_fwd_free && (left_fwd_free && left_bwd_free))) ||\
           (currBehavior == TAIL && (myLane == 1 && !mid_fwd_free && (right_fwd_free && right_bwd_free))) ||\
           (currBehavior == TAIL && (myLane == 2 && !right_fwd_free && (mid_fwd_free && mid_bwd_free)))\
           ){
    cout << "EV4" << endl;
    event = EV4; //LANECHANGE
    if (myLane == 0){stateMachine->lanechange_dir = 1;};
    if (myLane == 1 && (left_fwd_free && left_bwd_free)){stateMachine->lanechange_dir = -1;};
    if (myLane == 1 && (right_fwd_free && right_bwd_free)){stateMachine->lanechange_dir = 1;};
    stateMachine->lanechange_active = true;
    
    
  }else if(\
           (currBehavior == LANECHANGE && (myLane == 0 && !stateMachine->lanechange_active)) ||\
           (currBehavior == LANECHANGE && (myLane == 1 && !stateMachine->lanechange_active)) ||\
           (currBehavior == LANECHANGE && (myLane == 2 && !stateMachine->lanechange_active))\
           ){
    cout << "EV2" << endl;
    event = EV2; //CRUISE
    
  }else if(\
           (currBehavior == CRUISE && (myLane == 0 && (mid_fwd_free && mid_bwd_free && stateMachine->highway.forward_gap[0]<stateMachine->highway.forward_gap[1]))) ||\
           (currBehavior == CRUISE && (myLane == 2 && (mid_fwd_free && mid_bwd_free && stateMachine->highway.forward_gap[2]<stateMachine->highway.forward_gap[1])))\
           ){
    if (myLane == 0){stateMachine->lanechange_dir = 1;};
    if (myLane == 2){stateMachine->lanechange_dir = -1;};
    stateMachine->lanechange_active = true;
    cout << "EV1" << endl;
    event = EV1; //LANECHANGE
    
    
  }else if(\
           currBehavior == LANECHANGE && stateMachine->lanechange_active\
           ){
    stateMachine->lanechange_active = true;
    cout << "EV8" << endl;
    event = EV8; //LANECHANGE
    
  }//else{
    //stateMachine->lanechange_active = true;
  //}

  cout << "current state" << endl;
  cout << stateMachine->currBehavior << endl;
  
  cout << "mid_fwd_free" << endl;
  cout << mid_fwd_free << endl;

  cout << "mid_bwd_free" << endl;
  cout << mid_bwd_free << endl;

  cout << "left_fwd_free" << endl;
  cout << left_fwd_free << endl;

  cout << "left_bwd_free" << endl;
  cout << left_bwd_free << endl;

  cout << "right_forward_free" << endl;
  cout << right_fwd_free << endl;

  cout << "right_backward_free" << endl;
  cout << right_bwd_free << endl;
  
  cout << "lanechange_active" << endl;
  cout << stateMachine->lanechange_active << endl;

  return event;
};

vector<double> StateMachine_RunIteration(stateMachine_t *stateMachine){
  
  EventType event;
  vector<double> result;
  event = StateMachine_GetEvent(stateMachine->vehicle, stateMachine);
  
  // Iterate through the state transition matrix, checking if there is both a match with the current state
  // and the event
  
  for(int i = 0; i < sizeof(stateTransitionMatrix)/sizeof(stateTransitionMatrix[0]); i++) {
    
    if(stateTransitionMatrix[i].currBehavior == stateMachine->currBehavior) {

      if((stateTransitionMatrix[i].event == event) || (stateTransitionMatrix[i].event == EV0)) {

        // Transition to the next state
        stateMachine->currBehavior =  stateTransitionMatrix[i].nextBehavior;

        // Call the function associated with transition
        result = (stateFunctionBehavior[stateMachine->currBehavior].func)();
        result[1] = stateMachine->vehicle.state.current_lane + stateMachine->lanechange_dir;
        break;
      }
      
      if(stateMachine->lanechange_active == true){
        if (stateMachine->lanechange_dir == -1){
          result = {stateMachine->highway.flow_speed[stateMachine->vehicle.state.current_lane], 0, 3};
          if(stateMachine->vehicle.state.current_lane == 0){stateMachine->lanechange_active = false;};
        }
        if (stateMachine->lanechange_dir == 1){
          result = {stateMachine->highway.flow_speed[stateMachine->vehicle.state.current_lane], 1, 3};
          if(stateMachine->vehicle.state.current_lane == 1){stateMachine->lanechange_active = false;};
        }
      }
    }
  }
  
  return result;
};

const char *StateMachine_GetStateName(BehaviorType behavior){
  return stateFunctionBehavior[behavior].name;
}

stateMachine_t::stateMachine_t(Vehicle &vehicle, Highway &highway){
  this->vehicle = vehicle;
  this->highway = highway;
  //this->currBehavior = START;
};
