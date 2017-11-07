# Path-Planning

[//]: # (Image References)

[image1]: ./flow-diagram.png "Flow Chart"
[image2]: ./system-architecture.png "System Architecture"

This project designs a Path Planner composed of a Vehicle, a Highway, a Behavior Planner and a Trajectory Planner to make smooth, comfortable, safe paths for a autonomous vehicle drives along a highway. It communicates with Udacity simulator to receive localization, sensor fusion data and transmit trajectory of the vehicle.The diagram below can be seen for overall system architecture:


![System Architecture][image2]


The finite state machine starts at the START state which determines speed limit and switches directly to CRUISE state. In this state the vehicle wants to keep its lane (preferably mid lane) and speed limit. If it get closer to slower vehicle than switches to TAIL state and follows the vehicle ahead. During this state the  vehicle keeps track of adjacent lanes to find opportunity to make a lane change. If there is then the vehicle swithes to LANECHANGE state and returns CRUISE state once lane change completed.


![Flow Chart][image1]


Event-1: IF the vehicle at CRUISE 
         and 
         ((vehicle at LeftLane and mid lane free) 
         or
         (vehicle at RightLane and mid lane free)) THEN
         goes to LANECHANGE
         
Event-2: IF the vehicle at LANECHANGE 
         and
         ((vehicle at LeftLane and lane change not active) 
         or
         (vehicle at MidLane and lane change not active)
         or
         (vehicle at RightLane and lane change not active)) THEN
         goes to CRUISE

Event-3: IF the vehicle at CRUISE state 
         and 
         ((vehicle at LeftLane and mid lane blocked)
         or
         (vehicle at MidLane and mid lane free)
         or
         (vehicle at RightLane and mid lane blocked)) THEN
         goes to CRUISE state
         
Event-4: IF the vehicle at TAIL
         and
         ((vehicle at LeftLane and mid lane free and left lane blocked) 
         or 
         (vehicle at MidLane and mid lane blocked and left lane free)
         or 
         (vehicle at RightLane and mid lane free and right lane blocked)) THEN
         goes to LANECHANGE
         
Event-5: IF the vehicle at TAIL
         and
         ((vehicle at LeftLane and left lane free) 
         or 
         (vehicle at MidLane and mid lane free)  
         or 
         (vehicle at RightLane and right lane free)) THEN
         goes to CRUISE state
         
Event-6: IF the vehicle at CRUISE
         and
         ((vehicle at LeftLane and left lane blocked) 
         or 
         (vehicle at MidLane and mid lane blocked)  
         or 
         (vehicle at RightLane and right lane blocked)) THEN
         goes to TAIL state

Event-7: IF the vehicle at TAIL
         and 
         ((vehicle at LeftLane and mid lane blocked and left lane blocked) 
         or 
         (vehicle at MidLane and mid lane blocked and left lane blocked)  
         or 
         (vehicle at RightLane and mid lane blocked and right lane blocked)) THEN
         goes to TAIL

Event-8: IF the vehicle at LANECHANGE
         and
         ((vehicle at LeftLane and lane change active) 
         or
         (vehicle at MidLane and lane change active)
         or
         (vehicle at RightLane and lane change active)) THEN
         goes to LANECHANGE

After each cycle of Behavior Planner run, target lane and target velocity passes to Trajectory Planner which calculates smooth and comfortable trajectories. The Trajectory Planner achieves this fitting a spline [http://kluge.in-chemnitz.de/opensource/spline/] to the waypoints by using previous path of the vehicle and upcoming path of the vehicle until 30m ahead of it. Here lane following can be done easily by using map data and Frenet coordinate system of it.

--

Project instructions given by Udacity are as follows:
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
