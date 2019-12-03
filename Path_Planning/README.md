# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Overview
In this project, the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Necessary information is provided the car's localization and sensor fusion data of the car itself and other cars, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the speed limit of highway, 50 MPH, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all costs as well as driving inside of the marked road lanes at all times unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also, the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.
Besides, the path planning algorithm is implemented in order to drive a car on a highway by using the simulator provided by Udacity [releases tab]: (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2). Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop. The communication between the path planner algorithm and the simulator is done by uWebSocket [releases tab]: (https://github.com/uNetworking/uWebSockets). 

### Details
1. The car uses a perfect controller and visits every (x,y) point it receives in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner receives should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay, the simulator continues using points that it was last given, because of this its a good idea to store the last points that were used to have a smooth transition. previous_path_x and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single header file is easy to use.


## Project Rubric

### Project rules

The car is able to drive without any accident for at least 4.32 miles. Below is the image of the simulator running for several minutes.

[image1]: (./images/simulator_screenshot.jpg)

According to the rules of this project, no speed limit red message was seen and jerk at the beginning of the running was controlled for not exceeding the maximum. In addition, the car stays on the lane and change the lane smoothly just in case of front car is slow. 

### Code explanation
The code consists of three parts:
#### Prediction [line 250]: (./src/main.cpp#250)
This section of the code is dealing with sensor fusion data. Three questions can be answered in this part:
* Is the front car blocking the traffic?
* Is there any car on the left hand of the car, which making lane change not safe?
* Is there any car on the right hand of the car, which making lane change not safe?
//These questions can be answered by computing the lane change of each car is and the position it will be at the end of the last plan trajectory. When the distance between the front/rear car is less than 30 meters, the status of the car is considered as dangerous.
#### Behaviour [line 287]: (./src/main.cpp#287)
In this section, the car decides to change lane, speed up, or slow down because of the front car.
Based on the prediction information, the car takes action, which is suitable and safe for the current situation. In the code, speed_diff is used for speed changes when generating a trajectory in the last section of the code. This method makes the car faster in deciding whether to change a lane or apply the brakes. 
#### Trajectory [line 312]: (./src/main.cpp#312)
This part calculates the trajectory based on the output of speed and lane from the behavior section, car coordinates, and past path points. The last two points of the previous trajectory or previous position in case of no previous trajectory are used to initialize the spline calculation. The coordinates are transformed into local car coordinates to make the spline calculation less complicated.
In order to ensure more continuity on the trajectory, the passed trajectory points are added to the new trajectory (./src/main.cpp#368). The rest of the points are computed by assessing the spline and transforming the output to local coordinates(./src/main.cpp#380). Lines (./src/main.cpp#387) to (./src/main.cpp#393) are responsible to increase/decrease speed in every trajectory points instead of applying it to the whole trajectory.
