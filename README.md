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
