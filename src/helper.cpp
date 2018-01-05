//
// Created by Eren on 5.01.2018.
//

#include "helper.h"
#include "settings.h"

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2( (map_y-y),(map_x-x) );

    double angle = abs(theta-heading);

    if(angle > pi()/4)
    {
        closestWaypoint++;
    }

    return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%maps_x.size();

    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};

}

double notKeepingRightPenalty(double myLane) {
    return (2 - myLane) * 200;
};

double lowDistancePenalty(double lane, vector<double> egoVehicle, vector<vector<double>> otherVehicles) {

    const double CLEAR_ROAD = 150;
    double closestCarAhead = CLEAR_ROAD;
    double car_s = egoVehicle[1];

    for(int i = 0; i < otherVehicles.size(); i++) {
        /*  ["sensor_fusion"] A 2d vector of cars and then that car's
            [0][car's unique ID,
            [1]car's x position in map coordinates,
            [2]car's y position in map coordinates,
            [3]car's x velocity in m/s,
            [4]car's y velocity in m/s,
            [5]car's s position in frenet coordinates,
            [6]car's d position in frenet coordinates.
            vx = sf[i][3]
            vy = sf[i][4]
            s = sf[i][5]
              d = sf[i][6]
        */
        double d = otherVehicles[i][6];

        // is a vehicle in my lane? I occupy the range [(car_d) - 2, (car_d) + 2]
        if(d > (LANE(lane)) - 2 && d < (LANE(lane)+2)){
            //YES, how far away is it?
            double vx = otherVehicles[i][3], vy = otherVehicles[i][4];
            double other_veh_speed = sqrt(pow(vx,2)+pow(vy,2));
            double other_veh_s = otherVehicles[i][5];

            if(other_veh_s > car_s){
                double dist = other_veh_s - car_s;
                closestCarAhead = min(closestCarAhead, dist);
            }
        }
    }

    return PENALTY_ON_DISTANCE * ((CLEAR_ROAD-closestCarAhead)/CLEAR_ROAD);
}

double slowLanePenalty(double lane, vector<double> egoVehicle, vector<vector<double>> otherVehicles) {
    double speedCarAhead = TARGET_VELOCITY;
    double ref_target = speedCarAhead;
    double car_s = egoVehicle[1];

    for(int i = 0; i < otherVehicles.size(); i++) {
        double d = otherVehicles[i][6];

        // is a vehicle in my lane? I occupy the range [(car_d) - 2, (car_d) + 2]
        if(d > (LANE(lane)) - 2 && d < (LANE(lane)+2)){
            //YES, how far away is it?
            double vx = otherVehicles[i][3], vy = otherVehicles[i][4];
            double other_veh_speed = sqrt(pow(vx,2)+pow(vy,2));
            double other_veh_s = otherVehicles[i][5];

            if(other_veh_s > car_s){
                speedCarAhead = min(speedCarAhead, other_veh_speed);
            }
        }
    }
    return (TARGET_VELOCITY - speedCarAhead) * PENALTY_ON_VELOCITY;
}

// this function estimates how much other traffic participants have when the ego performs an action. How much time is left assuming the follower vehicle does an emergency break down to the velocity of the ege vehicle minus an optimistic 0.5 second reaction time
// the higher the values the better
double reactionTime(double lane, vector<double> egoVehicle, vector<vector<double>> otherVehicles) {

    double car_s = egoVehicle[1];
    double car_speed = egoVehicle[3];
    double ret = 5.0;

    for(int i = 0; i < otherVehicles.size(); i++) {

        double d = otherVehicles[i][6];

        // is a vehicle in my lane? I occupy the range [(car_d) - 2, (car_d) + 2]
        if(d > (LANE(lane) -2) && d < (LANE(lane) +2)){
            //YES, how far away is it?
            double vx = otherVehicles[i][3], vy = otherVehicles[i][4];
            double other_veh_speed = sqrt(pow(vx,2)+pow(vy,2));
            double other_veh_s = otherVehicles[i][5];

            if((other_veh_s > car_s) && (other_veh_s - car_s) < 25.) {
                ret = 0.5;
            }

            if((other_veh_s < car_s)) {
                double distanceBackwards = car_s - other_veh_s;
                //printf("Distance Backwards %f\n", distanceBackwards);

                const double a = 7.0;

                double bdist = (pow(other_veh_speed,2) - pow(car_speed,2)) / (2*a);
                double ttcoll = ((distanceBackwards - bdist) / car_speed) - 0.5; // 0.5 seconds to react

                if(ttcoll < 0)
                    ttcoll = 0;

                ret = min(ret, ttcoll);
            }
        }
    }
    return ret;
}
double adjustSpeed(double lane, vector<double> egoVehicle, vector<vector<double>> otherVehicles) {
    const double CLEAR_ROAD = 300;
    double closestCarAhead = CLEAR_ROAD;
    double speedCarAhead = TARGET_VELOCITY;
    double ref_target = speedCarAhead;
    double car_s = egoVehicle[1];

    for(int i = 0; i < otherVehicles.size(); i++) {
        double d = otherVehicles[i][6];

        // is a vehicle in my lane? I occupy the range [(car_d) - 2, (car_d) + 2]
        if(d > (LANE(lane)) - 2 && d < (LANE(lane)+2)){
            //YES, how far away is it?
            double vx = otherVehicles[i][3], vy = otherVehicles[i][4];
            double other_veh_speed = sqrt(pow(vx,2)+pow(vy,2));
            double other_veh_s = otherVehicles[i][5];

            if(other_veh_s > car_s){
                double dist = other_veh_s - car_s;
                closestCarAhead = min(closestCarAhead, dist);
                speedCarAhead = min(speedCarAhead, other_veh_speed);
            }
        }
    }

    if(closestCarAhead > 50.)
        ref_target = TARGET_VELOCITY;
    if(closestCarAhead < 30.)
        ref_target = speedCarAhead;
    if(closestCarAhead < 20.)
        ref_target = speedCarAhead - 3;

    return ref_target;
}