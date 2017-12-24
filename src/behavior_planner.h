//
//  behavior_planner.h
//  Path_Planning
//
//  Created by Eren on 27.09.2017.
//
//

#ifndef behavior_planner_h
#define behavior_planner_h

#include <stdio.h>
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
#include "highway.hpp"

using namespace std;

typedef enum{
  START,
  CRUISE,
  TAIL,
  LANECHANGE,
}BehaviorType;

typedef enum{
  EV0,
  EV1,
  EV2,
  EV3,
  EV4,
  EV5,
  EV6,
  EV7,
  EV8,
  NONEV
}EventType;

typedef struct{
  
  BehaviorType currBehavior;
  EventType event;
  BehaviorType nextBehavior;
  
}STMrow_t;


static STMrow_t stateTransitionMatrix[] = {
  
  //CURR STATE  //EVENT     //NEXT STATE
  {START,       EV0,        CRUISE},
  {CRUISE,      EV1,        LANECHANGE},
  {CRUISE,      EV3,        CRUISE},
  {CRUISE,      EV6,        TAIL},
  {LANECHANGE,  EV2,        CRUISE},
  {TAIL,        EV7,        TAIL},
  {TAIL,        EV5,        CRUISE},
  {TAIL,        EV4,        LANECHANGE},
  {LANECHANGE,  EV8,        LANECHANGE},

};


typedef struct {
  const char *name;
  vector<double> (*func)(void);
} SFrow_t;


vector<double> start(void);

vector<double> cruise(void);

vector<double> tail(void);

vector<double> lanechange(void);

static SFrow_t stateFunctionBehavior[] = {
  
  {"START",       &start},
  {"CRUISE",      &cruise},
  {"TAIL",        &tail},
  {"LANECHANGE",  &lanechange},
  
};

struct stateMachine_t{
  BehaviorType currBehavior;
  Vehicle vehicle;
  Highway highway;
  int lanechange_dir;
  bool lanechange_active;
  stateMachine_t(Vehicle &vehicle, Highway &highway);
};

void StateMachine_Init(stateMachine_t *stateMachine);
vector<double> StateMachine_RunIteration(stateMachine_t *stateMachine);
const char *StateMachine_GetStateName(BehaviorType behavior);

#endif /* behavior_planner_h */
