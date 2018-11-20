# Path Planning
Self-Driving Car Engineer Nanodegree Program


### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.



### How it works?
The path planner usually consists of 3 main modules: behavior planning, prediction and trajectory generation.


#### Prediction
In the prediction phase, I loop over the sensor fusion data for each car, supplied by the simulator.
I take the `d`-coordinate in Frenet frame to determine, which lane does the sensed car occupy (`430-434:main.cpp`).

```cpp
// which lane is the sensed car in?
int car_lane = -1;
if (d > 0 && d < 4)        car_lane = 0;
else if (d > 4 && d < 8)   car_lane = 1;
else if (d > 8 && d < 12)  car_lane = 2;
```

For each sensed car, I'm using a simple linear motion model in Frenet coordinates to predict its future `s`-coordinate (`438:main.cpp`).

```cpp
// predict where the sensed car will be in the future using linear motion model in Frenet frame
other_car_s += prev_size*0.02 * check_speed;  // s_k+1 = s + dt * v
```

Based on the future position of a sensed car, I assess the situation using simple boolean flags.
That is to say, I determine whether there is a car ahead, on the left or on the right of the ego-vehicle and when it is safe to change lanes. (`440-457:main.cpp`)

```cpp
// it's safe to overtake if the predicted position of other car is at least BACK_TOL [m] behind me
// or at least FRONT_TOL [m] ahead of me
bool safe_overtake = other_car_s < car_s-BACK_TOL || other_car_s > car_s+FRONT_TOL;

// if the sensed car is in the same lane, in front of ego-car and is less than SAFETY_GAP [m] away
if (car_lane == lane && other_car_s > car_s && (other_car_s - car_s) < SAFETY_GAP)
{
  car_ahead = true;
}
else if (car_lane == lane + 1 && !safe_overtake)
{
  // car is on the right, if it's in the right lane and it's not safe to overtake
  car_right = true;
}
else if (car_lane == lane - 1 && !safe_overtake)
{
  car_left = true;
}
```


#### Behavior Planning

The simple behavior planner takes the situation assessment from the predictor and using the following logic (`461-482:main.cpp`) decides what lane to change to.

```cpp
if (car_ahead)
{
  if (!car_left && lane > 0)
  {
    // change to the left, if there is a car ahead, no car on the left and ego-car is in center or right lane
    lane--;
  }
  else if (!car_right && lane < 2)
  {
    lane++;
  }
  else
  {
    // reduce speed, if car ahead
    ref_vel -= .224;
  }
}
else if (ref_vel < SPEED_LIMIT-0.5)
{
  // step on it, if no car ahead and speed is less than limit
  ref_vel += .336;
}
```


#### Trajectory Generation

For generating a smooth trajectory based on the supplied lane and velocity, I used the suggested single-header spline library available at: http://kluge.in-chemnitz.de/opensource/spline/.
The code itself pretty much follows the procedure presented in the project Q&A.

I define 3 waypoints in the Frenet frame that are 30, 60 and 90 meters ahead of the ego-vehicle (`s`) and in the target lane (`d`).
The spline model is fitted with points transformed to Cartesian car frame.

```cpp
// add evenly 30m spaced waypoints ahead of the s-coordinate of the ego-vehicle ...
vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map.waypts_s, map.waypts_x, map.waypts_y);
vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map.waypts_s, map.waypts_x, map.waypts_y);
vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map.waypts_s, map.waypts_x, map.waypts_y);
ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);
ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);
// ... tranform these waypoints to car frame ...
for (int i = 0; i < ptsx.size(); i++)
{
  double shift_x = ptsx[i] - ref_x;
  double shift_y = ptsy[i] - ref_y;
  ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
  ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
}
// ... and fit a spline to them
tk::spline spline_model;
spline_model.set_points(ptsx, ptsy);
```

To generate a trajectory, I reuse the leftover points from the previous iteration
```cpp
// for continuity reasons, we take the leftover points from the trajectory computed in the previous iteration
// (that were returned by the simulator in previous_path_x, previous_path_y) ...
Trajectory tr;
for (int i = 0; i < prev_size; i++)
{
  tr.waypts_x.push_back(previous_path_x[i]);
  tr.waypts_y.push_back(previous_path_y[i]);
}
```
and compute the remaining points to make the trajectory 50 points long.

```cpp
// ... and compute the remaining (new) points of the trajectory, such that velocity ref_vel is achieved
double target_x = 30.0;
double target_y = spline_model(target_x);
double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
double N = target_dist / (.02 * ref_vel/2.24);  // ref_vel [mph] --> ref_vel/2.24 [mps]
double x_add_on = 0;
for (int i = 1; i <= 50 - prev_size; i++)
{
  double x_point = x_add_on + target_x / N;
  double y_point = spline_model(x_point);
  x_add_on = x_point;

  // transform the point of the trajectory back to the world frame
  double x_ref = x_point;
  double y_ref = y_point;
  x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
  y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
  x_point += ref_x;
  y_point += ref_y;

  tr.waypts_x.push_back(x_point);
  tr.waypts_y.push_back(y_point);
}
```


---
### Build & Run
This project is designed to work with Udacity's [Term3 Simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run the simulator: `term3_sim.exe`
5. Run: `./path_planning`

### Dependencies

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
---
