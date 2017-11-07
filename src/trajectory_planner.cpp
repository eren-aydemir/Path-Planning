//
//  trajectory_planner.cpp
//  Path_Planning
//
//  Created by Eren on 2.10.2017.
//
//

#include <stdio.h>
#include "trajectory_planner.h"

TrajectoryPlanner::TrajectoryPlanner(){


};


XYPoints TrajectoryPlanner::test_run(const Vehicle &vehicle){

  double dist_inc = 0.4;
  vector<double> x;
  vector<double> y;
  
  for(int i = 0; i < 40; i++)
  {
    x.push_back(vehicle.state.x + (dist_inc*i)*cos(0));
    y.push_back(vehicle.state.y + (dist_inc*i)*sin(0));
  }
  this->traj.xs = x;
  this->traj.ys = y;
  
  return traj;
};
/*
XYPoints TrajectoryPlanner::update(const Vehicle &vehicle, const MapWaypoints &map_waypoints, const XYPoints &prevPath)
{
  
  // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
  
  //list of widely spaced (x,y) waypoints, evenly spaced at 30m
  //later we will interpolate these waypoints with a spline and fill it with more points that control speed.
  vector<double> ptsx;
  vector<double> ptsy;
  
  //reference x,y and yaw states
  //either we will reference the starting point as where the car is or at previous paths end point
  double ref_x = vehicle.state.x;
  double ref_y = vehicle.state.y;
  double ref_yaw = deg2rad(vehicle.state.yaw); //yaw
  int prev_size = prevPath.n;
  
  if (prev_size < 2){
    
    double prev_car_x = vehicle.state.x - cos(vehicle.state.yaw);
    double prev_car_y = vehicle.state.y - sin(vehicle.state.yaw);
    
    ptsx.push_back(prev_car_x);
    ptsx.push_back(vehicle.state.x);
    ptsy.push_back(prev_car_y);
    ptsy.push_back(vehicle.state.y);
    
    ref_x = prev_car_x;
    ref_y = prev_car_y;
    
  }
  
  else{
    
    //use previous path's end point as a reference and re-define the state.
    ref_x = prevPath.xs[prev_size-1];
    ref_y = prevPath.ys[prev_size-1];
    
    double ref_x_prev = prevPath.xs[prev_size-2];
    double ref_y_prev = prevPath.ys[prev_size-2];
    
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
    
    ref_x = ref_x_prev;
    ref_y = ref_y_prev;

  }
  
  //In Frenet add evenly 30m spaced points ahead of the starting reference.
  vector<double> next_wp0 = getXY(vehicle.state.s+30, (2+4*vehicle.state.lane), map_waypoints.s, map_waypoints.x, map_waypoints.y);
  vector<double> next_wp1 = getXY(vehicle.state.s+60, (2+4*vehicle.state.lane), map_waypoints.s, map_waypoints.x, map_waypoints.y);
  vector<double> next_wp2 = getXY(vehicle.state.s+90, (2+4*vehicle.state.lane), map_waypoints.s, map_waypoints.x, map_waypoints.y);
  
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  
  //Global coordinate to car's local coordinate. Heading and position of the car at end point of previous_path_x
  //and previous_path_y will be 0.
  for (int i=0; i<ptsx.size(); i++){
    
    //shift amount to make each ptsx and ptsy values starts from 0.
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    
    //if (shift_x < 0){shift_x = 0;};
    
    ptsx[i] = (shift_x * cos(0-ref_yaw)) - (shift_y * sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw)) + (shift_y * cos(0-ref_yaw));
    

  }
  
  //create a spline
  tk::spline s;
  s.set_points(ptsx, ptsy);
  
  //define actual path (x,y) vector that we will use for planner
  XYPoints next_path;
  
  //start with all previous_path_x values from last time
  for (int i=0; i<prevPath.n; i++){
    //shift amount to make each ptsx and ptsy values starts from 0.
    next_path.xs.push_back(prevPath.xs[i]);
    next_path.ys.push_back(prevPath.ys[i]);
  }
  
  //break up spline points so that we travel at our desired reference velocity each time
  double target_x = 30;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
  
  double x_add_on = 0;
  
  for (int i = 1; i <= 50 - prevPath.n; i++)
  {
    double N = (target_dist / (0.02*vehicle.state.target_speed / 2.24)); //2.24 to convert to m/s
    //std::cerr << (target_x) / N  << std::endl;
    
    double x_point = x_add_on + (target_x) / N;
    // s provides the y value to x
    double y_point = s(x_point);
    
    x_add_on = x_point;
    
    double x_ref = x_point;
    double y_ref = y_point;
    
    // rotate back to normal after rotating
    x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
    
    x_point += ref_x;
    y_point += ref_y;
    
    std::cout << x_point << endl;
    
    next_path.xs.push_back(x_point);
    next_path.ys.push_back(y_point);
  }
  this->traj = next_path;
  return traj;
};
*/
void TrajectoryPlanner::update(const Vehicle &vehicle, const MapWaypoints &map_waypoints, const XYPoints &prevPath)
{
  
/*
cout << "---------------------------------" << endl;
cout << "STATE: s, d --- x, y --- v:" << endl;
cout << vehicle.state.s << " , "
<< vehicle.state.d << " --- "
<< vehicle.state.x << " , "
<< vehicle.state.y  << " --- "
<< vehicle.state.speed << ":" << endl;
*/
  
// ptsx, ptsy will be anchor points
vector<double> ptsx;
vector<double> ptsy;
double running_speed;
  
if (vehicle.state.target_speed > vehicle.state.speed){
  running_speed = vehicle.state.speed + 2;
}
  
if (vehicle.state.target_speed < vehicle.state.speed){
  running_speed = vehicle.state.speed - 2;
}
  
//read in the car's global x and y position
// and assign it to ref x and ref y
double ref_x = vehicle.state.x;
double ref_y = vehicle.state.y;
double ref_yaw = deg2rad(vehicle.state.yaw);

if (prevPath.n < 2)
{
  // this happens once at the beginning

  double prev_car_x = vehicle.state.x - cos(vehicle.state.yaw);
  double prev_car_y = vehicle.state.y - sin(vehicle.state.yaw);
  ptsx.push_back(prev_car_x);
  ptsx.push_back(vehicle.state.x);
  ptsy.push_back(prev_car_y);
  ptsy.push_back(vehicle.state.y);
  
}
else
{

  ref_x = prevPath.xs[prevPath.n - 1];
  ref_y = prevPath.ys[prevPath.n - 1];
  
  double ref_x_prev = prevPath.xs[prevPath.n - 2];
  double ref_y_prev = prevPath.ys[prevPath.n - 2];
  ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
  
  ptsx.push_back(ref_x_prev);
  ptsx.push_back(ref_x);
  ptsy.push_back(ref_y_prev);
  ptsy.push_back(ref_y);
  
}
  //cout << "ref_yaw" << endl;
  //cout << ref_yaw << endl;
  
// in frenet add evenly 30m spaced points ahead of the start
// In freenet add evenly the spaced points

vector<double> next_wp0 = getXY(vehicle.state.s + 30, (2.0 + 4 * vehicle.state.target_lane), map_waypoints.s, map_waypoints.x, map_waypoints.y);
vector<double> next_wp1 = getXY(vehicle.state.s + 60, (2.0 + 4 * vehicle.state.target_lane), map_waypoints.s, map_waypoints.x, map_waypoints.y);
vector<double> next_wp2 = getXY(vehicle.state.s + 90, (2.0 + 4 * vehicle.state.target_lane), map_waypoints.s, map_waypoints.x, map_waypoints.y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);


// now we have 5 points on ptsy ptsx
// 2 previous points and 30, 60, 90
  for (size_t i = 0; i < ptsx.size(); i++)
{
  // shift car reference angle to 0 degrees
  // at car coordinates 0,0
  double shift_x = ptsx[i] - ref_x;
  double shift_y = ptsy[i] - ref_y;
  // shift to car coordinate, 0,0 0 degrees
  ptsx[i] = (shift_x*cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw));
  ptsy[i] = (shift_x*sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw));
}

tk::spline s;

// set x y points to the spline

s.set_points(ptsx, ptsy);

vector<double> next_x_vals;
vector<double> next_y_vals;
  
// Start with all of the previous points
for (size_t i = 0; i < prevPath.n; i++)
{
  next_x_vals.push_back(prevPath.xs[i]);
  next_y_vals.push_back(prevPath.ys[i]);
}
// Calculate how to break up splines
// pts so we travel at desired velocity
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

double x_add_on = 0;  // start at orgin

// Fill up the rest of the path planner
// after filling it with previous points
// what is previous_path
for (int i = 1; i <= 50 - prevPath.n; i++)
{
  double N = (target_dist / (0.02*running_speed / 2.24)); //2.24 to convert to m/s

  double x_point = x_add_on + (target_x) / N;
  // s provides the y value to x
  double y_point = s(x_point);
  
  x_add_on = x_point;
  
  double x_ref = x_point;
  double y_ref = y_point;
  
  // rotate back to normal after rotating
  x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
  y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
  
  x_point += ref_x;
  y_point += ref_y;
  
  next_x_vals.push_back(x_point);
  next_y_vals.push_back(y_point);
}
  this->traj.xs = next_x_vals;
  this->traj.ys = next_y_vals;
}
