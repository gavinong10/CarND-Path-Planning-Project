# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Project Write-up

### Criteria: The car is able to drive at least 4.32 miles without incident.
A YouTube video of the car driving over 5 miles without incident is provided [here](https://youtu.be/UvFJizw7EBk).

<a href="http://www.youtube.com/watch?feature=player_embedded&v=UvFJizw7EBk
" target="_blank"><img src="http://img.youtube.com/vi/UvFJizw7EBk/0.jpg" 
alt="Path Planning Video for Gavin Ong" width="240" height="180" border="10" /></a>

### Criteria: The car drives according to the speed limit
In the video, it can be seen that the car attempts to maintain 49 mph where possible. It will slow down if a car is in front and in its lane, and whenever possible change lanes (if the lane next to it does not have a car within 30 m in front and behind it).

### Criteria: Max Acceleration and Jerk are not Exceeded
MAX_ACCEL was set to 0.224 mph. The acceleration figure is used to set a new 'target speed' of a planning session. Assuming we achieve this 'target speed', we then ensure that we achieve our target distance in approximately 0.02 seconds. Acceleration is handled by the framework to evenly apply it; hence, we get 0.224 / 0.02 / 2.24 which is approximately 10 m/s^3 of jerk.

### Criteria: Car does not have collisions
Passed as per video.

### Criteria: The car stays in its lane, except for the time between changing lanes
Passed as per video.

### Criteria: The car is able to change lanes
Passed as per video. It achieves this if it finds a car in front of it < 30m away and there is a gap either to the left or right of it.

### Criteria: There is a reflection on how to generate paths
Firstly, for the car to move, it needed waypoints generated that were spaced at intervals specified such that if it traversed the distances between them, the velocity calculated would be distance / time=0.02 seconds. Originally starting with the map waypoints, these were sampled at 30m, 60m and 90m to create points that could be fit with a spline for smoothness. Then, assuming a particular velocity and a trigonometric approximation of distance for the spline, we determine the position and how many points we need to project out to achieve our 50 point horizon. We do this by roughly estimating a point on the spline 30 m in front of the car, determine its trigonometric distance, then approximate how far each waypoint needs to be spaced if we were travelling at some target velocity. Once we determine these waypoints, we append these to an array of x and y values that get used by the simulator.

One particular important detail was that in the spline fitting process, we centered the car at (0, 0) using transformations and had the car's initial orientation be parallel to the x axis. This is to simplify fitting splines and was retransformed back to map coordinates at the end of the process.

To deal with jerk, acceleration and lane changing - the initial car velocity was set to zero, and on each update cycle, the target velocity was changed by +- 0.224. It accelerated if it was not yet reaching the target velocity, and decelerated if a car was in front and less than 30 m.

By checking for the presence of a car in front, we created a logical state in which it is viable to look for changing lanes. Hence, when the car was looking to potentially slow down, it observed if adjacent lanes had cars within 30 m of it. If not, it would change into the adjacent lane. 

Lane changes would also occur if the car was not in the middle lane if it was not penalized by speed to do so.

Conceptually, we have a poor man's discrete cost function which adheres to the following relationship of increasing cost:

* If in lane 1, stay in lane if no other car is in front (30m)
* If car is in front and car is not in adjacent lane, change lane
* If not in lane 1 and it is possible to change lanes, change to lane 1

We do not do any prediction of obstacles on the road for this project. We assume that the other cars on the road will be relatively well-behaved in speed and acceleration, such that they will NOT

* Collide with us if they are adjacent to us
* If they are within 30 m of us and in our lane, they will not drastically change behavior that they themselves will yield high jerk

## END

## Project Readme
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

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

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

