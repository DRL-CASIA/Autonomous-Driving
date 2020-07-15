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
#include "spline.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cstdlib>
#include <ctime>
#define BUF_SIZE 2000
#define ADDR "127.0.0.1" //在本机测试用这个地址，如果连接其他电脑需要更换IP
#define SERVERPORT 1234

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

string doubleToString(double data){
    stringstream ss;
    ss << data;
    return ss.str();
}

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

        double heading = atan2((map_y-y),(map_x-x));

        double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
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

double cars = 0.0;
double card = 0.0;
unsigned int coll = 0;
unsigned int change_lane = 0;

int main() {

  int sock;
  char opmsg[BUF_SIZE];
  int result, opnd_cnt, i;
  bool if_first = true;
  struct sockaddr_in serv_addr;

  sock = socket(AF_INET, SOCK_STREAM, 0);
  if(sock == -1){
      return -1;
  }
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = inet_addr(ADDR);
  serv_addr.sin_port = htons(SERVERPORT);
  if(connect(sock, (struct sockaddr*) &serv_addr, sizeof(serv_addr))==-1){
      cout << "connect error\n";
      return -1;
  }
  else{
      cout << "connected ...\n" << endl;
  }

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

  // Car's lane. Starting at middle lane.
  int lane = 1;

  // Reference velocity.
  double ref_vel = 0.0; // mph
  double reward = 0.0;
  double avg_vel = 0.0;
  double avg_spd = 0.0;
  unsigned long cnt = 0;
  int count = 0;

  h.onMessage([&ref_vel,&lane,&reward,&avg_vel,&count,&cnt,&avg_spd,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,sock]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    int action = 0;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
         // cout << s << endl;
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

                // Main car's localization Data
                double last_s = cars;
                double last_d = card;
                double car_x = j[1]["x"];
                double car_y = j[1]["y"];
                double car_s = j[1]["s"];
                cars = j[1]["s"];
                card = j[1]["d"];
                double car_d = j[1]["d"];
                double car_yaw = j[1]["yaw"];
                double car_speed = j[1]["speed"];
                double original_car_s = j[1]["s"];
                // cout << cos(deg2rad(car_yaw)) << endl;
                avg_vel = avg_vel + car_speed;
                avg_spd = avg_spd + car_speed;
                count += 1;
                cnt += 1;
                if (last_s > 6930 && car_s < 20) {
                    avg_spd = avg_spd / cnt;
                    cout << avg_spd << endl;
//                     ofstream ofile;
// //                    ofile.open("your_path_to/CarND-test/src/train/train/train.txt", ios::app);
//                     ofile.open("your_path_to/CarND-test/src/test/test/test.txt", ios::app);
//                     ofile<<"avg_speed "<<avg_spd<<" collisions "<<coll<<" change_lane "<<change_lane;
//                     ofile.close();
                    write(sock, "over", 4);
                    cnt = 0;
                    avg_spd = 0;
                    coll = 0;
                    change_lane = 0;
                }

                // Previous path data given to the Planner
                auto previous_path_x = j[1]["previous_path_x"];
                auto previous_path_y = j[1]["previous_path_y"];
                // Previous path's end s and d values
                double end_path_s = j[1]["end_path_s"];
                double end_path_d = j[1]["end_path_d"];

                // Sensor Fusion Data, a list of all other cars on the same side of the road.
                auto sensor_fusion = j[1]["sensor_fusion"];

                // Provided previous path point size.
                int prev_size = previous_path_x.size();

                // Preventing collitions.
                if (prev_size > 0) {
                  car_s = end_path_s;
                }

//                double grid[51][3];
//                int i,j = 0;
//                for(j=0;j<3;j++){
//                    for(i=0;i<51;i++){
//                        grid[i][j] = 1.0;
//                    }
//                }
                int ego_car_lane = -1;
                if ( car_d > 0 && car_d < 4 ) {
                  ego_car_lane = 0;
                } else if ( car_d > 4 && car_d < 8 ) {
                  ego_car_lane = 1;
                } else if ( car_d > 8 && car_d < 12 ) {
                  ego_car_lane = 2;
                }
//                grid[31][ego_car_lane] = car_speed/100.0;
//                grid[32][ego_car_lane] = car_speed/100.0;
//                grid[33][ego_car_lane] = car_speed/100.0;
//                grid[34][ego_car_lane] = car_speed/100.0;

                bool car_ahead = false;
                bool car_left = false;
                bool car_right = false;
                bool left_ahead = false;
                bool left_behind = false;
                bool right_ahead = false;
                bool right_behind = false;
                double ahead_speed = 50.0;
                double min_s_dis = 30.0;
                vector<int> left_cars;
                vector<int> right_cars;

                for ( int i = 0; i < sensor_fusion.size(); i++ ) {
                    float s = sensor_fusion[i][5];
                    float d = sensor_fusion[i][6];
                    int car_lane = -1;
                    // is it on the same lane we are
                    if ( d > 0 && d < 4 ) {
                      car_lane = 0;
                    } else if ( d > 4 && d < 8 ) {
                      car_lane = 1;
                    } else if ( d > 8 && d < 12 ) {
                      car_lane = 2;
                    }
                    if (car_lane < 0) {
                      continue;
                    }
                    if (abs(d-car_d) < 2.5 && abs(s-original_car_s) < 5.5){
                        reward = -200.0;
                    }
                    // Find car speed.
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx*vx + vy*vy);
                    double check_car_s = sensor_fusion[i][5];
                    double current_car_s = sensor_fusion[i][5];

//                    double s_dis = check_car_s - original_car_s;
//                    if (s_dis > -36 && s_dis < 66){
//                        int pers = - int(floor(s_dis/2.0)) + 30;
//                        grid[pers][car_lane] = check_speed/100.0;
//                        grid[pers+1][car_lane] = check_speed/100.0;
//                        grid[pers+2][car_lane] = check_speed/100.0;
//                        grid[pers+3][car_lane] = check_speed/100.0;
//                    }
                    // Estimate car s position after executing previous trajectory.
                    check_car_s += ((double)prev_size*0.02*check_speed);

                    if ( car_lane == lane ) {
                      // Car in our lane.
                      car_ahead |= check_car_s > car_s && check_car_s - car_s < 20;
                      car_ahead |= current_car_s > cars && current_car_s - cars < 20;
                      if (check_car_s > car_s && check_car_s - car_s < min_s_dis){
                          min_s_dis = check_car_s - car_s;
                          ahead_speed = check_speed;
                      }
                    }
                    if ( car_lane - ego_car_lane == -1 ) {
                      // Car left
//                      car_left |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
                      left_behind |= current_car_s > cars - 30 && current_car_s < cars;
                      left_ahead |= current_car_s > cars && current_car_s < cars + 25;
                      left_cars.push_back(i);
                    } else if ( car_lane - ego_car_lane == 1 ) {
                      // Car right
//                      car_right |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
                      right_behind |= current_car_s > cars - 30 && current_car_s < cars;
                      right_ahead |= current_car_s > cars && current_car_s < cars + 25;
                      right_cars.push_back(i);
                    }
                }

//                double state[45][3];
//                for(j=0;j<3;j++){
//                    for(i=0;i<45;i++){
//                        state[i][j] = 0.0;
//                    }
//                }
//                if (ego_car_lane == 0){
//                    for(j=1;j<3;j++){
//                        for(i=0;i<45;i++){
//                            state[i][j] = grid[i+3][j-1];
//                        }
//                    }
//                }else if (ego_car_lane == 1) {
//                    for(j=0;j<3;j++){
//                        for(i=0;i<45;i++){
//                            state[i][j] = grid[i+3][j];
//                        }
//                    }
//                }else if (ego_car_lane == 2) {
//                    for(j=0;j<2;j++){
//                        for(i=0;i<45;i++){
//                            state[i][j] = grid[i+3][j+1];
//                        }
//                    }
//                }

//                string give_state = "[{\"state\":[";
//                for(j=0;j<3;j++){
//                    for(i=0;i<45;i++){
//                        give_state = give_state + doubleToString(state[i][j]) + ',';
//                    }
//                }
//                give_state.pop_back();
//                give_state = give_state + "]},{\"reward\":";
//                give_state = give_state + doubleToString(last_reward) + "}]";

                double speed_diff = 0;
                const double MAX_SPEED = 49.5;
                const double MAX_ACC = .224;


                if ( car_ahead && ahead_speed * 2.23 < car_speed) { // Car ahead
                    speed_diff -= MAX_ACC;
                    // cout << ahead_speed << ',' << min_s_dis << endl;
                } else if (!car_ahead){
                    speed_diff += MAX_ACC;
                }

                if (abs(car_d - (lane*4+2)) > 0.3 || count < 5) { //
                    // cout << "car_d:" << car_d << ",lane:" << lane << endl;
                }else {
//                    for(j=0;j<3;j++){
//                        for(i=0;i<45;i++){
//                            cout << grid[i][j];
//                        }
//                    }
                    if (abs(reward + 200.0) < 0.1) {
                        coll += 1;
                        cout << "collision:" << coll << endl;
                    }
                    avg_vel = avg_vel / count;
                    // cout << count << endl;
                    int len;
                    char get_msg[3] = {0};
                    string data = s.c_str();
                    data.pop_back();
                    data = data + "," + doubleToString(reward) + "," + doubleToString(avg_vel) + ']';
                    // cout << data << endl;
                    len = write(sock, data.c_str(), data.size());
                    // len = write(sock, give_state.c_str(), give_state.size());
                    // cout << len << endl;
                    int read_msg = read(sock, get_msg, len);
                    // cout << get_msg << endl;
                    reward = 0.0;
                    avg_vel = 0.0;
                    count = 0;
                    int next_lane = lane;
                    string state = "keep_lane";

                    if (!strcmp(get_msg,"1")) { // && !car_left
                        next_lane--;
                        state = "change_lane_left";
//                        change_lane++;
                        if (next_lane < 0){
                            reward = -50.0;
                            next_lane = 0;
                        }
                    }else if (!strcmp(get_msg,"2")) { //
                        next_lane++;
                        state = "change_lane_right";
//                        change_lane++;
                        if (next_lane > 2){
                            reward = -50.0;
                            next_lane = 2;
                        }
                    }
                    if (state != "keep_lane") {
                        string action = "execute";
//                        vector<double> start_and_end_s;
//                        vector<double> start_and_end_d;
//    //                    double ref_s = car_s;
//    //                    double ref_d = car_d;
//                        if (last_s < cars) {
//                            start_and_end_s.push_back(last_s);
//                        }
//                        else {
//                            start_and_end_s.push_back(cars - 2);
//                        }
//                        start_and_end_s.push_back(cars);
//                        start_and_end_s.push_back(cars + 30);
//                        start_and_end_s.push_back(cars + 60);
//                        start_and_end_d.push_back(last_d);
//                        start_and_end_d.push_back(card);

////                        start_and_end_s.push_back(end_path_s - 2);
////                        start_and_end_s.push_back(end_path_s);
////                        start_and_end_s.push_back(end_path_s + 30);
////                        start_and_end_s.push_back(end_path_s + 60);
////                        start_and_end_d.push_back(end_path_d);
////                        start_and_end_d.push_back(end_path_d);
////                        cout << last_s << cars << last_d << card << endl;
//                        start_and_end_d.push_back(next_lane * 4.0 + 2.0);
//                        start_and_end_d.push_back(next_lane * 4.0 + 2.0);
//                        tk::spline tj;
//                        tj.set_points(start_and_end_s, start_and_end_d);
//                        vector<double> tjs, tjd;
//                        double s_tj, d_tj;
//                        s_tj = cars; // end_path_s
//                        d_tj = tj(s_tj);
//                        double vel = car_speed;
//                        for ( int i = 0; i < 100; i++ ) {
//                            vel += speed_diff;
//                            if ( vel > MAX_SPEED ) {
//                              vel = MAX_SPEED;
//                            } else if ( vel < MAX_ACC ) {
//                              vel = MAX_ACC;
//                            }
//                            tjs.push_back(s_tj);
//                            tjd.push_back(d_tj);
//                            s_tj += 0.02*vel/2.24;
//                            d_tj = tj(s_tj);
//                        }
////                        cout << "tjs" << tjs << endl;
////                        cout << "tjd" << tjd << endl;
//                        if (state == "change_lane_left") {
//                            for ( int j = 0; j < left_cars.size(); j++ ) {
//                                double left_s = sensor_fusion[left_cars[j]][5];
//                                double left_d = sensor_fusion[left_cars[j]][6];
//                                double left_vx = sensor_fusion[left_cars[j]][3];
//                                double left_vy = sensor_fusion[left_cars[j]][4];
//                                double left_speed = sqrt(left_vx*left_vx + left_vy*left_vy);
////                                left_s += ((double)prev_size*0.02*left_speed);
//                                for ( int i = 0; i < tjs.size(); i++ ) {
//                                    s_tj = tjs[i];
//                                    d_tj = tjd[i];
//                                    double dis = sqrt((s_tj-left_s)*(s_tj-left_s) + (d_tj-left_d)*(d_tj-left_d));
////                                    if (dis < 40) cout << dis << endl;
//                                    if (dis < 8) action = "cancel";
//                                    left_s += 0.02*left_speed;
//                                }
//                            }
//                        }
//                        if (state == "change_lane_right") {
//                            for ( int j = 0; j < right_cars.size(); j++ ) {
//                                double right_s = sensor_fusion[right_cars[j]][5];
//                                double right_d = sensor_fusion[right_cars[j]][6];
//                                double right_vx = sensor_fusion[right_cars[j]][3];
//                                double right_vy = sensor_fusion[right_cars[j]][4];
//                                double right_speed = sqrt(right_vx*right_vx + right_vy*right_vy);
////                                right_s += ((double)prev_size*0.02*right_speed);
//                                for ( int i = 0; i < tjs.size(); i++ ) {
//                                    s_tj = tjs[i];
//                                    d_tj = tjd[i];
//                                    double dis = sqrt((s_tj-right_s)*(s_tj-right_s) + (d_tj-right_d)*(d_tj-right_d));
////                                    if (dis < 40) cout << dis << endl;
//                                    if (dis < 8) action = "cancel";
//                                    right_s += 0.02*right_speed;
//                                }
//                            }
//                        }
//                        if (lane==0 && right_ahead && right_behind) action = "cancel";
//                        if (lane==1 && left_ahead && right_ahead) action = "cancel";
//                        if (lane==2 && left_ahead && left_behind) action = "cancel";
                        if (action != "cancel") {
                            lane = next_lane;
                            change_lane++;
                        }
                        cout << state << " " << action << endl;
                    }
                }

                vector<double> ptsx;
                vector<double> ptsy;

                double ref_x = car_x;
                double ref_y = car_y;
                double ref_yaw = deg2rad(car_yaw);

                // Do I have have previous points
                if ( prev_size < 2 ) {
                    // There are not too many...
                    double prev_car_x = car_x - cos(car_yaw);
                    double prev_car_y = car_y - sin(car_yaw);

                    ptsx.push_back(prev_car_x);
                    ptsx.push_back(car_x);

                    ptsy.push_back(prev_car_y);
                    ptsy.push_back(car_y);
                } else {
                    // Use the last two points.
                    ref_x = previous_path_x[prev_size - 1];
                    ref_y = previous_path_y[prev_size - 1];

                    double ref_x_prev = previous_path_x[prev_size - 2];
                    double ref_y_prev = previous_path_y[prev_size - 2];
                    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

                    ptsx.push_back(ref_x_prev);
                    ptsx.push_back(ref_x);

                    ptsy.push_back(ref_y_prev);
                    ptsy.push_back(ref_y);
                }

                // Setting up target points in the future.
                vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                ptsx.push_back(next_wp0[0]);
                ptsx.push_back(next_wp1[0]);
                ptsx.push_back(next_wp2[0]);

                ptsy.push_back(next_wp0[1]);
                ptsy.push_back(next_wp1[1]);
                ptsy.push_back(next_wp2[1]);

                // Making coordinates to local car coordinates.
                for ( int i = 0; i < ptsx.size(); i++ ) {
                  double shift_x = ptsx[i] - ref_x;
                  double shift_y = ptsy[i] - ref_y;

                  ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                  ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
                }

                // Create the spline.
                tk::spline s;
                s.set_points(ptsx, ptsy);

                // Output path points from previous path for continuity.
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
                for ( int i = 0; i < prev_size; i++ ) {
                  next_x_vals.push_back(previous_path_x[i]);
                  next_y_vals.push_back(previous_path_y[i]);
                }

                // Calculate distance y position on 30 m ahead.
                double target_x = 30.0;
                double target_y = s(target_x);
                double target_dist = sqrt(target_x*target_x + target_y*target_y);

                double x_add_on = 0;

                for( int i = 1; i < 50 - prev_size; i++ ) {
                  ref_vel += speed_diff;
                  if ( ref_vel > MAX_SPEED ) {
                    ref_vel = MAX_SPEED;
                  } else if ( ref_vel < MAX_ACC ) {
                    ref_vel = MAX_ACC;
                  }
                  double N = target_dist/(0.02*ref_vel/2.24);
                  double x_point = x_add_on + target_x/N;
                  double y_point = s(x_point);

                  x_add_on = x_point;

                  double x_ref = x_point;
                  double y_ref = y_point;

                  x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                  y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

                  x_point += ref_x;
                  y_point += ref_y;

                  next_x_vals.push_back(x_point);
                  next_y_vals.push_back(y_point);
                }

                json msgJson;

                // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                msgJson["next_x"] = next_x_vals;
                msgJson["next_y"] = next_y_vals;

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
  close(sock);
  return 0;
}
