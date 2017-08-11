# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.


### Reflection

In order to safely navigate through the course, the following steps were taken in the project.  

#### Initialization ####

The highway spline is used to map from the Frenet coordinates into highway coordinates in a continuous fashion. The spline uses the compact library from http://kluge.in-chemnitz.de/opensource/spline/ for ease of use. The spline is only generated at the start instance for speed and efficiency.

However, since the track is continuous and wraps at the ~6900m mark, the starting points are tacked onto the end of the spline. This ensures a continuous function as the transition happens over the wrapping mark.

##### Configurations #####

The application settings are found in the ```Constants.h``` file. Key to the discussions here are;

```
// Maximum target acceleration for any movements
const double TARGET_ACCEL = 4; // m/s^2

// Maximum target velocity
const double TARGET_MAX_VEL = mph2ms(46); // mph

// The number of seconds to change a lane
const double LANE_CHANGE_TIME = 3; // time in s to change a lane

```

##### The map of the highway is in data/highway_map.txt #####
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


#### Simulator Flow ####

The main entry class is ```PathPlanner```. It maintains the state of the sensor_fusion vehicles and interpolates the track determined by the ```TrajectorPlanner``` class. It is also responsible for translating coordinates from Frenet to XY.  

One each message from the simulator the ```PathPlanner```;
- Updates the positions of vehicles
- Calls the ```TrajectoryPlanner::generatePlan``` method
- Generate the next points based on the track decided in the ```TrajectoryPlanner```

##### Updates the positions of vehicles#####


Each vehicle and the current driver is tracked in the ```PathPlanner```. The speed, yaw and distance to the driver are all computed when the vehicles are updated. For the purposes of efficiency only the vehicles within 100m of the driver are sent on to the ```TrajectoryPlanner``` for planning.

##### Calls the ```TrajectoryPlanner::generatePlan``` method #####

The ```TrajectoryPlanner::generatePlan``` method is the core decision making tool for the process. The decisions are based on a minimum cost function and can have 3 outcomes. Keep Current Lane, Change Lanes Left, Change Lanes Right.

For each of those outcomes, we generate a cost function.

a. *Keep Current Lane* This cost function is based upon a sigmoid function of the TARGET_MAX_VEL/{current speed}. This produces a lower value the faster the lane is travelling

b. *Change Lanes (left and right)* This cost function is also based on the TARGET_MAX_VEL as above but with some constraints. First, if there are vehicles in the lane +/- 20m the cost is set to 1. Second, left lane changes are slighty higher function. This serves to limit the occurances of passing on the left unless it is significantly faster lane. Third, right lane changes have a reduced value to encourage the car to move right. Highway driving rules encourage travelling in the rightmost lane.

Once the lowest cost path is chosen a trajectory is built using a combination of adjusting speed and lane location. (Jerk-Minimizing Thresholds are used to ensure a smooth transition).

a. *Keep Current Lane* Speed is adjusted to either TARGET_MAX_VEL or the speed of a vehicle in front of the driver (~40m)

b. *Change Lane* Speed is adjusted to either TARGET_MAX_VEL or the speed of the desired lane. The lane location is also set to the new lane.



##### Generate the next points based on the track decided in the ```TrajectoryPlanner``` #####

The ```PathPlanner``` maintains a queue of the next 40points (~0.8s) to be executed at 0.02s intervals. When a new point is calculated it is incremented from the previous state.

We only maintain a speed and lane direction in the trajectory so the, S value will be computed from the speed (This also simplifies the wrapping problem).

```
t = 0.2;

prev_v = trajectory_planner.getDeltaV(t);

// Enforce the maximum target vel
prev_v = (prev_v > TARGET_MAX_VEL)?TARGET_MAX_VEL:prev_v;

// Adjust the current location from the speed
prev_s = prev_s + prev_v * t;

// Check for wrapping of the s value
prev_s = checkMaxS(prev_s);

// Adjust the lane location
prev_d = trajectory_planner.getDeltaD(trajectory_planner.jmt_d_time);

// Ensure it doesn't exceed the lane bounds
prev_d = checkMaxD(prev_d);

// Map to spline
new_xy = map_.getXY(prev_s,prev_d);
```
The new speed and lane direction are computed from the JMT function. The JMT provides a simple way to record a track without having to maintain state and ensure a smooth transition.
```
trajectory_planner.getDeltaV(t)
trajectory_planner.getDeltaD(t)
```



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
