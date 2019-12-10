# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

In this project, the main goal is to implement PID control for the car in order to control steering and throttle. Two parameters are given to us through the simulator provided by Udacity:
* Cross-track error: how far the car is from the circuit
* Orientation error: the angle between the car’s orientation and the tangent to the path

## PID Control
The proportional gain causes the car to steer with respect to the car’s distance from the lane center. For instance, if the car is so close to the right it steers to the left side, and in case of close to the left, it steers to the right side. By applying only P control, it was seen that the vehicle behavior was not stable and it was oscillating during the maneuver.

After tuning the P coefficient, the D coefficient was tuned base on the performance of the car. It can be seen that the D coefficient reduces the oscillation of the car around the centerline. The integral component (I) is responsible to contract a bias in the cross-track error where the P-D controller is not able to reach the centerline.

In order to initialize the Kp, Ki, and Kd, the trial and error method was selected and it was started with P-value by setting the other coefficients to zero. After finding the proper value for the P coefficient, the D coefficient was tuned to decrease the oscillation and then the I coefficient was tuned to reduce the steady-state error. The initial coefficients are in the following:
[0.05, 0.0001, 1.5]

In order to optimize the coefficients, the Twiddle method was used in the main.cpp file. When the Twiddle variable set to be true, the simulator runs the car until the maximum steps set initially and go through the twiddle algorithm. After each iteration, the simulator reset to the initial stage and the car starts from the beginning to the maximum steps. This process continues until the tolerance value below the allowed value. The final optimized coefficients are obtained by Twiddle method as follows:
[0.06, 0.00031, 1.29]

