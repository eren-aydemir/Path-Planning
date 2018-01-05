//
// Created by Eren on 5.01.2018.
//

#ifndef PATH_PLANNİNG_HELPER_H
#define PATH_PLANNİNG_HELPER_H

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

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double);
double rad2deg(double);

double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

double lowDistancePenalty(double lane, vector<double> egoVehicle, vector<vector<double>> otherVehicles);
double slowLanePenalty(double lane, vector<double> egoVehicle, vector<vector<double>> otherVehicles);
double reactionTime(double lane, vector<double> egoVehicle, vector<vector<double>> otherVehicles);
double adjustSpeed(double lane, vector<double> egoVehicle, vector<vector<double>> otherVehicles);
#endif //PATH_PLANNİNG_HELPER_H
