# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
This project's goal is to use a Model Predictive Control (MPC) to drive an autonomous car around a test track. The car will be provided with GPS checkpoints and it's current location and needs to successfully dive around the track.

The MPC also needs to deal with a 100ms latency between it's commands and the actuators.

## MPC

Differently than the [PID](https://github.com/arash-ha/ND---Self-Driving-Car-Engineer/tree/master/PID_Control), that uses only observations of the current measured error, the MPC relies on appropriately modeling the behavior of the process that it will control.

This gives it a great advantage, as it can estimate future errors and how these would be affected by changes in the controls - in our case speed and steering angle.

The ability to anticipate future events and take actions to prevent bad outcomes is what makes MPC a much more robust option for controlling autonomous vehicles.

## The Kinematic Model

This implementation uses a kinematic model to control it's actuators. Kinematic models are a simplified version of how cars actually behave and they ignore tire forces, lateral forces, gravity and mass.

Although such simplifications really limit the use of these models in a real world application, they are extremely useful to develop control methods such as an MPC. It is also relevant to mention that at low and moderate speeds, the kinematic models offer a good approximation of the real vehicle dynamics.

### 1. Vehicle State

The first step in our model is to define what kind of information we want to model, this is reflected in our "State" - a state is a set of variables that describe the situation the vehicle is in.

Four variables were used to represent the state of our vehicle at every time step:
- Cartesian position on the X axis of our map (similar to longitude) [x]
- Cartesian position on the Y axis of our map (similar to latitude) [y]
- Orientation [psi]
- Velocity [v]

### 2. Actuators State

In order to accurately predict the vehicle's position in the future we also need to consider the actuators state, which describes how the orientation and velocity of our vehicle will change in time.

The two actuators we have in our model are the following:
- Steering angle [delta]
- Throttle [a]

### 3. Global Kinematic Model

The six variables above define what we call "State" of the vehicle, that means they describe the exact current situation of the vehicle and give us enough information to model where the car will be in a future time.

That is achieved by modeling the four vehicle state variables as functions of themselves, the actuators state and the time between observations. We will call the time gap between observations delta t (dt).

Using trigonometry and basic physics we can derive the four equations that will describe how the vehicle state changes over time:

1) x(t + dt) = x(t) + v(t) \* cos(psi(t)) \* dt

2) x(t + dt) = y(t) + v(t) \* sin(psi(t)) \* dt

3) psi(t + dt) = psi(t) + v(t)/Lf \* delta \* dt

4) v(t + dt) = v(t) + a(t) \* dt

Lf in Eq. 3 is the distance between the front wheels of the vehicle and it's center of gravity.

### 4. Hyperparameters

Given the state and update functions we have just described, the MPC calculates a number of timesteps ahead of it's current state and measures if the predicted outcome of the current trajectory is adequate or not.

The number of predicted steps, the time gap between steps and the predicted error are all parameters that have to be optimized for each application.

#### Number of Timesteps (N)

N determines how many timesteps will be calculated at each point in time. The caveat is that it is also the major driver of computational cost, not for the obvious reason that more timesteps need to be calculated, but also that for each timestep you add you increase the number of variable to be optimized when minimizing your cost function.

#### Timestep duration

The MPC model tries to approximate the continue path that cars drive by a discrete trajectory of N timesteps, that means that the larger the gap between these points, the higher our uncertainty will be.

In this model, the prediction horizon was set to be two seconds, split into ten timesteps, so the timestep duration (dt) is 0.2s. This was optimized through trial and error and It was found out that the latency introduces a significant amount of uncertainty into the model and that the later predictions were actually harming the optimization of the actuators inputs.

## Dealing with different coordinate systems

The path our car is supposed to follow is given to us in a global reference, just like a real car would get GPS locations, but the actions it takes are calculated with reference to the car's local coordinate system, so similarly to the Localization problem, we need to convert the GPS coordinates to the car reference.

Conversely what could be done is perform all calculations using the map reference, but converting the waypoints to car reference simplifies some of the state update equations in a way that our approach is more efficient, since the coordinate conversion is a very simple and cheap computation.

## Fitting third degree polynomials to the waypoints

At this point the last decision to be made is the complexity of the polynomial we are going to use to model our predicted path. Simpler polynomials are cheap to calculate, but will deliver higher error, more complex polynomials are expensive to calculate, but will more accurately predict the future states of the vehicle.

Since our prediction horizon is fairly small, two seconds in this case, most roads can be well approximated by a third degree polynomial, for most cases probably even a second degree polynomial would be fine, but the third degree allows for better path estimation when two tight turns are closely connected.

## Error modeling

The most challenging part of designing an MPC is modeling how to optimize the use of the actuators and which of the many different errors you can measure are going to be prioritized.

The several different cost parameters we have are:
1) Squared cross track error - measures the lateral distance between the center of the car and the path it should follow.

2) Squared Orientation error - measures the difference between the desired vehicle orientation and the current vehicle orientation.

3) Squared Reference Velocity error - measures the difference in desired speed and the current vehicle speed. This is a lot less important than the other two errors above, but it serves the purpose of not allowing the MPC to find an optimal solution where the vehicle stops moving.

But just modeling state errors is not enough, in driving constant changing directions or jumping from acceleration to braking and back to accelerating are dangerous behaviors. These type of behavior received the highest weight on our error modeling, meaning that the MPC would allow for smaller deviations from the "best path" in order to have smoother controls.

I have also implemented a cost that would prevent abrupt changes in directions, so the MPC would prefer a constant smooth turn rather than an abrupt change of direction.

## Dealing with latency

All that we have implemented so far are enough to drive in a perfect world, where the effects of your inputs are immediate. In reality, however, there is a delay between your decision to act, the change in the controls and ultimately the change in the behavior of the vehicle.

To account for this latency, the MPC uses the same model that it uses to predict future states, meaning that it will make all it's decision knowing that by the time it needs to change the controls, the state of the vehicle would not be the same as the current state.

In other words, the vehicle will first predict it's state using the current status of all controls (throttle and steering angle in this case) and using the latency as timestep, and only then try to optimize it's path.

## Final Considerations

MPCs are a much more robust model than PIDs and this fine tuned MPC is capable of autonomously driving around our test track carrying significant speeds.

As it was discussed before, the simplifications we assumed under our Kinematic model are not valid at such high speeds as we have on the test video, but since it is a lot more challenging to make the model go fast, even under simplified assumptions, I decided to go for a fast lap.

The final result can be seen below: 

![gif](./fast_lap.gif). 

The yellow line is the ground truth path coming from the GPS waypoints and the green line is the MPC designed path.


---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We have kept editor configuration files out of this repo to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. We omitted IDE profiles to ensure
students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. Most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio and develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
