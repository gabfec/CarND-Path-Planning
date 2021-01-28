# CarND-Path-Planning
Self-Driving Car Engineer Nanodegree Program



[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

![Path Planning](./gif/demo.gif)


### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### Highway map
The map is in data/highway_map.txt. Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Simulator.
The Simulator for this Path Planning project can be downloaded from:   https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2.

The data provided from the Simulator to the C++ application contain:

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

## Implementation details

The planner code is developed in a separate PathPlanner class. This class is responsible for processing the telemetry data and returning the planned trajectory for the ego vehicle. It contains a nested Vehicle class which is instantiated for every vehicle reported by sensor fusion data and for the ego vehicle itself. This Vehicle class is the relic of an attempt to make an more ambitious implementation, until I realized that the code walktrough from David and Aaron gives a very simple and robust solution.
The Vehicle class provides handy methods which can describe the relative positions of the vehicles in a very readable way.
For instance, checking the state of the left/right lane is done like this:

<pre> <font color="#C4A000">if</font> ((vehicle.isInFrontOf(ego) <font color="#C4A000">and</font> vehicle.distanceTo(ego) &lt; <font color="#AF5F00">40</font>) <font color="#C4A000">or</font>
   (<font color="#C4A000">not</font> vehicle.isInFrontOf(ego) <font color="#C4A000">and</font> vehicle.distanceTo(ego) &lt; <font color="#AF5F00">10</font>))
 {
    lane_busy[vehicle.lane()] = <font color="#AF5F00">true</font>;
 }
</pre>

Every 20 ms a decision is taken following these simple rules:
   - if no car ahead, speed up, but don't exceed the speed limit
   - whenever possible, come back to the middle lane
   - keep the lane if no vehicle ahead is closer than 30m
   - when changing the lane ensure there is no car 10m behind or 40m ahead on that lane
   - for changing the lane try first left lane, and then the right one
   - if no overtake is possible, just keep the front car speed

This decisions are taken based on the predicted positions of the nearby vehicles. The predictions takes as input the received telemetry data (Frenet s distance), the previous trajectory and it assumes the vehicles maintain constant speed.

For creating a smooth trajectory I used the spline header-only library http://kluge.in-chemnitz.de/opensource/spline/.
The spline is initialized using last 2 points from the previous trajectory and 3 points ahead evenly distrubuted at 30m.
The trajectory consist of 50 points. In order to have a smooth trajectory, the newly computed points (using the above spline) will be added on top of the previous computed trajectory.

The GIF from the top is a real case recording of a lane change with this implementation.


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

## Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.






