# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Overview
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Necassry information is provided the car's localization and sensor fusion data of car itself and other cars, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the speed limit  of highway, 50 MPH, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.
In addition, path planning algorithms is implemented in order to drive a car on a highway by using the simulator provided by Udacity [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)]. Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop. The communication between the path planner algorithm and the simulator is done by uWebSocket [releases tab (https://github.com/uNetworking/uWebSockets)]. 

### Details
1. The car uses a perfect controller and visits every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator continue using points that it was last given, because of this its a good idea to store the last points that were used to have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.


## Project Rubric

### Code

[//]: # (Image References)

[image1]: ./images/simulator_screenshot.jpg "Visualisation"






