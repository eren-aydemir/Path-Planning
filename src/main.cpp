#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "settings.h"
#include "vehicle.h"
#include "trajectory_planner.h"
#include "highway.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //std::cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);
      //std::cout << s.size() << endl;
      
      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	//double car_x = j[1]["x"];
          	//double car_y = j[1]["y"];
          	//double car_s = j[1]["s"];
          	//double car_d = j[1]["d"];
          	//double car_yaw = j[1]["yaw"];
          	//double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	vector<double> previous_path_x = j[1]["previous_path_x"];
          	vector<double> previous_path_y = j[1]["previous_path_y"];
          
            XYPoints previousPath;
            previousPath.xs = previous_path_x;
            previousPath.ys = previous_path_y;
            previousPath.n = previous_path_x.size();

          
            //std::cerr << "previousPath.n:" << std::endl;
            //std::cerr << previousPath.n  << std::endl;
          
          	// Previous path's end s and d values
          	auto end_path_s = j[1]["end_path_s"];
          	auto end_path_d = j[1]["end_path_d"];
          
            MapWaypoints map_waypoints;
            map_waypoints.x = map_waypoints_x;
            map_waypoints.y = map_waypoints_y;
            map_waypoints.s = map_waypoints_s;
          
          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
            vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
        
          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            cout << "-----------CYCLE----------------" << endl;
          
            Vehicle egoVehicle(999);
            egoVehicle.update_state(j);
          
            if (previousPath.n > 0)
            {
            egoVehicle.state.s = end_path_s;
            }
          
            vector<Vehicle> otherVehicles;
          
            for (int i = 0; i < sensor_fusion.size(); i++) {
            
              State state;
              int id = sensor_fusion[i][0];
              state.x = sensor_fusion[i][1];
              state.y = sensor_fusion[i][2];
              state.s = sensor_fusion[i][5];
              state.d = sensor_fusion[i][6];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              state.speed = sqrt(vx*vx + vy*vy);
            
              Vehicle vehicle(id);
              vehicle.update_state(state);
              otherVehicles.emplace_back(vehicle);
            }
            egoVehicle.obstacles(otherVehicles, egoVehicle.state.current_lane);
          
            Highway highway(3);
            highway.update(egoVehicle, otherVehicles);
          
            vector<double>::iterator highway_iterator = min_element(begin(highway.costs), end(highway.costs));
          
            // Run first iteration
            vector<double> result = {V_PLAN, 1.0, 0.0};
          
            result[2] = 1; //CRUISE
            if (highway.costs[distance(begin(highway.costs), highway_iterator)] > 5000 && highway.ttcollision[egoVehicle.state.current_lane] < 3.0){
              result[2] = 2; //TAIL
            }else if(highway.costs[distance(begin(highway.costs), highway_iterator)] < 5000 && highway.ttcollision[distance(begin(highway.costs), highway_iterator)] > 3.0){
              result[2] = 3; //LANECHANGE
            };
            double speed_target = highway.flow_speed[egoVehicle.state.current_lane];
          
            if (speed_target == INFINITY){
              speed_target = 48;
            }
          
            if (result.size() != 0){ //START
              
              egoVehicle.state.target_speed = result[0];
              egoVehicle.state.target_lane = 1;
              
              if(result[2] == 1){ //CRUISE
                egoVehicle.state.target_speed = speed_target; //V_PLAN;
                egoVehicle.state.target_lane = egoVehicle.state.current_lane;
              }
              if(result[2] == 2){ //TAIL
                egoVehicle.state.target_speed = speed_target;
                egoVehicle.state.target_lane = egoVehicle.state.current_lane;
              }
              if(result[2] == 3){ //LANECHANGE
                egoVehicle.state.target_speed = speed_target;
                egoVehicle.state.target_lane = distance(begin(highway.costs), highway_iterator);
              }
            }
          
            cout.precision(1);
            cout << "Highway costs: " << fixed << highway.costs[0] << " " << highway.costs[1] << " " << highway.costs[2] << endl;
            cout << "Time2Collision: " << fixed << highway.ttcollision[0] << " " << highway.ttcollision[1] << " " << highway.ttcollision[2] << endl;
            cout << "Fwd gap: " << fixed << highway.fwd_gap[0] << " " << highway.fwd_gap[1] << " " << highway.fwd_gap[2] << endl;
            cout << "Flow speed: " << fixed << highway.flow_speed[0] << " " << highway.flow_speed[1] << " " << highway.flow_speed[2] << endl;
            cout << "Vehicle speed: " << fixed << egoVehicle.state.speed << endl;
            cout << "Efficient lane: " << distance(std::begin(highway.costs), highway_iterator) << endl;
            cout << "Vehicle at lane: " << fixed << egoVehicle.state.current_lane << endl;
            cout << "Highway updated!" << endl;
          
            //cout << stateMachine.vehicle.state.target_speed << endl;
            //cout << stateMachine.vehicle.state.target_lane << endl;

            //cout << egoVehicle.state.target_speed << endl;
            //cout << egoVehicle.state.target_lane << endl;
          
            printf("Vehicle curr lane %d.\r\n", egoVehicle.state.current_lane);
          
            TrajectoryPlanner path;
            path.update(egoVehicle, map_waypoints, previousPath);
            //path.test_run(egoVehicle);
          
          	msgJson["next_x"] = path.traj.xs;
          	msgJson["next_y"] = path.traj.ys;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
