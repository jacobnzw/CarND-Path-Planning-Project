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

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// stores map waypoints for convenience
struct Map {
  vector<double> waypts_x;
  vector<double> waypts_y;
  vector<double> waypts_s;
  vector<double> waypts_dx;
  vector<double> waypts_dy;
};

struct Trajectory {
  vector<double> waypts_x;
  vector<double> waypts_y;
};

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

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
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

  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
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
  prev_wp = next_wp - 1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s*cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s*sin(heading);

  double perp_heading = heading - pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x, y};

}

// define simple finite state machine
// given a state, return successor states
vector<string> successor_states(string state)
{
  vector<string> next_states;
  if (state == "KL")
  {
    next_states.push_back("KL");
    next_states.push_back("CLL");
    next_states.push_back("CLR");
  }
  else if (state == "CLL")
  {
    next_states.push_back("KL");
  }
  else if (state == "CLR")
  {
    next_states.push_back("KL");
  }
  return next_states;
}

Trajectory generate_trajectory(int current_lane, double ref_vel, json &sim_data, Map &map)
{
  // unpack localization data
  double car_x = sim_data[1]["x"];
  double car_y = sim_data[1]["y"];
  double car_s = sim_data[1]["s"];
  double car_yaw = sim_data[1]["yaw"];
  auto previous_path_x = sim_data[1]["previous_path_x"];
  auto previous_path_y = sim_data[1]["previous_path_y"];

  vector<double> ptsx;
  vector<double> ptsy;
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  int lane = current_lane;
  // if (next_state == "CLR")
  // {
  //   lane++;
  // }
  // else if (next_state == "CLL")
  // {
  //   lane--;
  // }

  int prev_size = previous_path_x.size();
  if (prev_size > 0) car_s = sim_data[1]["end_path_s"];
  if (prev_size < 2)
  {
    // use two points that make the path tangent to the car
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  else // use the previous path's endpoint as starting reference
  {
    // redefine reference state as previous path and point
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // use two points that make the path tangent to the previous path's endpoint
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // in Frenet, add evenly 30m spaced points ahead of the starting reference
  vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map.waypts_s, map.waypts_x, map.waypts_y);
  vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map.waypts_s, map.waypts_x, map.waypts_y);
  vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map.waypts_s, map.waypts_x, map.waypts_y);
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // tranform points to car frame
  for (int i = 0; i < ptsx.size(); i++)
  {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  // fit a spline
  tk::spline s;
  s.set_points(ptsx, ptsy);

  vector<double> next_x_vals;
  vector<double> next_y_vals;
  for (int i = 0; i < prev_size; i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
  double x_add_on = 0;

  // fill up the rest of our path planner after filling it with previous points
  // making sure we always output 50 points in next_x_vals, next_y_vals
  for (int i = 1; i <= 50 - prev_size; i++)
  {
    double N = (target_dist / (.02 * ref_vel / 2.24));
    double x_point = x_add_on + target_x / N;
    double y_point = s(x_point);
    x_add_on = x_point;
    double x_ref = x_point;
    double y_ref = y_point;
    // transform back to world frame
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  Trajectory result = {next_x_vals, next_y_vals};
  return result;
}

bool detect_collision(Trajectory trajectory, vector<vector<double>> predictions_xy, int steps)
{
  // T = prev_size = predictions are assumed to be made steps steps ahead, steps < trajectory.size()
  // TODO: is there find collision
  double x = trajectory.waypts_x[steps-1];
  double y = trajectory.waypts_y[steps-1];
  const float GAP = 10;  //  [m]
  bool collision = false;
  for (auto pred : predictions_xy)
  {
    if (sqrt(pow(pred[0] - x,2) + pow(pred[1] - y,2)) < GAP)
    {
      collision = true;
    }
  }
  return collision;
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  Map map;
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
    float dx;
    float dy;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dx;
    iss >> dy;
    map.waypts_x.push_back(x);
    map.waypts_y.push_back(y);
    map.waypts_s.push_back(s);
    map.waypts_dx.push_back(dx);
    map.waypts_dy.push_back(dy);
  }

  int lane = 1;
  double ref_vel = 0; //49.5;
  string state = "KL";  // default initial state of the ego vehicle

  h.onMessage([&map, &lane, &ref_vel, &state]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Ego-vehicles's localization data
          double car_x      = j[1]["x"];  // Cartesian coordinates in world frame
          double car_y      = j[1]["y"];
          double car_s      = j[1]["s"];  // Frenet coordinates in world frame
          double car_d      = j[1]["d"];
          double car_yaw    = j[1]["yaw"];  // in degrees
          double car_speed  = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }
          
          // scan for cars in front of me within SAFETY_GAP
          const float SAFETY_GAP = 20;  // distance in front of ego car to check for other cars
          const float BACK_TOL = 5;
          const float FRONT_TOL = 20;
          bool car_ahead = false;
          bool car_left  = false;
          bool car_right = false;
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            float d = sensor_fusion[i][6];

            // which lane is the sensed car in?
            int car_lane = -1;
            if (d > 0 && d < 4)        car_lane = 0;
            else if (d > 4 && d < 8)   car_lane = 1;
            else if (d > 8 && d < 12)  car_lane = 2;

            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(pow(vx,2) + pow(vy,2));
            double other_car_s = sensor_fusion[i][5];
            // predict where the sensed car will be in the future 
            other_car_s += prev_size*0.02 * check_speed;  // s_k+1 = s + dt * v

            // if the sensed car is in front of ego and is less than SAFETY_GAP [m] away
            bool safe_overtake = other_car_s < car_s-BACK_TOL || other_car_s > car_s+FRONT_TOL;
            // bool safe_overtake = fabs(other_car_s - car_s) > 5;
            // cout << "safe_overtake: " << safe_overtake << endl;
            if (car_lane == lane && other_car_s > car_s && (other_car_s - car_s) < SAFETY_GAP)
            {
              car_ahead = true;
            }
            else if (car_lane == lane + 1 && !safe_overtake)
            {
              car_right = true;
            }
            else if (car_lane == lane - 1 && !safe_overtake)
            {
              car_left = true;
            }
          }

          if (car_ahead)
          {
            if (!car_left && lane > 0)
            {
              lane--;
            }
            else if (!car_right && lane < 2)
            {
              lane++;
            }
            else 
            {
              ref_vel -= .224;
            }
          }
          else if (ref_vel < 49.5)
          {
            ref_vel += .336;
          }

          Trajectory t = generate_trajectory(lane, ref_vel, j, map);
          vector<double> next_x_vals = t.waypts_x;
          vector<double> next_y_vals = t.waypts_y;
          
          // if (car_ahead)
          // {
          //   // generate predictions
          //   vector<vector<double>> predictions;
          //   for (int i = 0; i < sensor_fusion.size(); ++i)
          //   {
          //     double vx = sensor_fusion[i][3];
          //     double vy = sensor_fusion[i][4];
          //     double s = sensor_fusion[i][5];
          //     double d = sensor_fusion[i][6];
          //     s += prev_size*.02 * sqrt(pow(vx,2) + pow(vy,2));
          //     auto xy_pt = getXY(s, d, map.waypts_s, map.waypts_x, map.waypts_y);
          //     predictions.push_back(xy_pt);
          //   }

          //   // for each successor state, generate trajectory and assign cost to it
          //   int min_cost = 9999;
          //   string min_state;
          //   Trajectory min_trajectory;
          //   for (auto st : successor_states(state))
          //   {
          //     Trajectory tr = generate_trajectory(st, lane, ref_vel, j, map);
          //     bool collision = detect_collision(tr, predictions, prev_size);
          //     if (!collision)
          //     {
          //       min_trajectory = tr;
          //       min_state = st;
          //     }
          //     else 
          //     {
          //       min_state = "KL";
          //       min_trajectory = generate_trajectory(min_state, lane, ref_vel, j, map);
          //     }
          //     // // select the minimal cost trajectory for execution
          //     // if (tr_cost < min_cost)
          //     // {
          //     // 	min_cost = tr_cost;
          //     // 	min_state = st;
          //     // 	min_trajectory = tr;
          //     // }
          //   }
          //   state = min_state;
          //   next_x_vals = min_trajectory.waypts_x;
          //   next_y_vals = min_trajectory.waypts_y;
          // }
          // else
          // {
          //   state = "KL";
          //   Trajectory tr = generate_trajectory(state, lane, ref_vel, j, map);
          //   next_x_vals = tr.waypts_x;
          //   next_y_vals = tr.waypts_y;
          // }
          // cout << "state: " << state << endl;
          // NOTES
          // based on sensor information, look for cars in front of me in the same lane that I am too close to
          // if car found
          // - for each succesor state, determine the cost of trajectory associated to that state
          // else if no such car found, keep lane

          // Trajectory tr = generate_trajectory("KL", lane, ref_vel, j, map);
          // next_x_vals = tr.waypts_x;
          // next_y_vals = tr.waypts_y;

          json msgJson;
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

  // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
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
