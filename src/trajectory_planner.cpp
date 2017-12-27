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
  //do nothing
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

void TrajectoryPlanner::update(const Vehicle &vehicle, const MapWaypoints &map_waypoints, const XYPoints &prevPath)
{
  vector<double> splinepoints_x, splinepoints_y;
  
  double ref_x = vehicle.state.x;
  double ref_y = vehicle.state.y;
  double ref_yaw = deg2rad(vehicle.state.yaw);
  
  double car_x = ref_x;
  double car_y = ref_y;
  double car_s = vehicle.state.s;
  double car_yaw = ref_yaw;
  
  int prev_size = prevPath.n;
  
  vector<double> previous_path_x = prevPath.xs;
  vector<double> previous_path_y = prevPath.ys;
  vector<double> map_waypoints_s = map_waypoints.s;
  vector<double> map_waypoints_x = map_waypoints.x;
  vector<double> map_waypoints_y = map_waypoints.y;
  
  static double running_speed = 0;
  if (vehicle.state.target_speed > (vehicle.state.speed)){ //*20/48
    running_speed = (vehicle.state.speed*20/48) + 1.5;
    printf("vehicle.state.target_speed is %f.\r\n", vehicle.state.target_speed);
    printf("vehicle.state.speed is %f.\r\n", vehicle.state.speed);
    printf("running speed is %f.\r\n", running_speed);
  }
  
  if (vehicle.state.target_speed < (vehicle.state.speed)){
    running_speed = (vehicle.state.speed*20/48) - 1.5;
  }
  
  if(prev_size < 2){
    // get two points so the spline attaches to the current heading of the vehicle
    splinepoints_x.push_back(car_x - cos(car_yaw));
    splinepoints_y.push_back(car_y - sin(car_yaw));
    
    splinepoints_x.push_back(car_x);
    splinepoints_y.push_back(car_y);
  }
  
  // instead of attaching to the end of previous_path, lets cut it after a set of points to be more responsive
  else{
    // same here, but we can access the previous_path vector
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];
    
    const double ref_x_prev = previous_path_x[prev_size-2];
    const double ref_y_prev = previous_path_y[prev_size-2];
    
    ref_yaw = atan2(ref_y-ref_y_prev,ref_x - ref_x_prev);
    
    // the "older" first, the newer second
    splinepoints_x.push_back(ref_x_prev);
    splinepoints_y.push_back(ref_y_prev);
    
    //
    splinepoints_x.push_back(ref_x);
    splinepoints_y.push_back(ref_y);
  }
  
  // spline generation depends on reference velocity. If intended to go fast, stretch the anchor waypoints
  for(double s_step = 30.; s_step < 120.0; s_step += 10 + 1.2 * running_speed){
    vector<double> xy = getXY(car_s + s_step, (2.0 + 4 * vehicle.state.target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    splinepoints_x.push_back(xy[0]);
    splinepoints_y.push_back(xy[1]);
  }
  
  // now bring points in line with car heading. Necessary to create uniform stepping
  
  //shift & rotate
  for(int i = 0; i < splinepoints_x.size(); i++) {
				double shift_x = splinepoints_x[i] - ref_x;
				double shift_y = splinepoints_y[i] - ref_y;
    
				splinepoints_x[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
				splinepoints_y[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
  }
  
  tk::spline s;
  /*
			printf("splinepoints:\n");
			for(int i = 0; i < splinepoints_x.size(); i++) {
   printf("x: %f y: %f\n", splinepoints_x[i], splinepoints_y[i]);
			}
   */
  s.set_points(splinepoints_x, splinepoints_y);
  
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
  // linear approximation of point distance
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(pow(target_x,2) + pow(target_y,2));
  
  double x_add_on = 0.0;
  
  for(int i = 0; i < prev_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
  
  for(int i = 1; i <= 75 - prev_size; i++) {
    double N = target_dist/(0.02 * running_speed);
    
    double x_point = x_add_on + (target_x / N);
    double y_point = s(x_point);
    
    x_add_on = x_point;
    
    // reverse rotate & shift
    double x_p = (x_point * cos(ref_yaw) - y_point * sin(ref_yaw)) + ref_x;
    double y_p = (x_point * sin(ref_yaw) + y_point * cos(ref_yaw)) + ref_y;
    
    next_x_vals.push_back(x_p);
    next_y_vals.push_back(y_p);
  }
  this->traj.xs = next_x_vals;
  this->traj.ys = next_y_vals;
}

