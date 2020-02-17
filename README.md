# PID controller Design
Self-Driving Car Engineer Nanodegree Program

## Overview

In this project, a PID controller was implemented in C++ to maneuver the vehicle around the track in the Udacity simulator. The simulator provides us the cross track error (CTE), the speed, and steering angle data via local websocket in order to compute the appropriate steering angle in each time step. The PID controller must respond with steering and throttle commands to drive SAFELY as fast as possible! The maximum speed is 100 mph and there is no need to meet a minimum speed.

---

## Files
Following are the project files in the src folder:
- [../src/main.cpp] main.cpp: 
-- Creates PID controller objects for steering angle and throttle
-- Initializes them with the parameter values once
-- Passes value of CTE from simulator to the PID objects
-- Collects total error, one from each PID object
-- Calculates steering angle value and throttle value based on respective total errors
-- Calls the simulator and passes in the PID parameters for steering angle and throttle to the simulator

- [../src/PID.cpp] PID.cpp:
-- Init - initializes PID controller member variables
-- UpdateError - updates different errors of PID controller based on input CTE
-- TotalError- calculates total error and updates cumulative total error

---
## Reflection
The proportional portion of the controller tries to steer the car toward the center line against the cross-track error. (CTE) If used along, the car overshoots the central line very easily and go out of the road very quickly. 
In order to avoid this overshoot, differential controller is added which considers the temporal derivative, dCTE/dt and reduces the P component's tendency to ring and overshoot the center line. A properly tuned D parameter will cause the car to approach the center line smoothly without ringing.
Finally, the integral component of the controller tries to eliminate a bias in the CTE which prevents the P-D controller from reaching the center line. 

Two controller have been employed here for finding the good steering angle and throttle value. 
The gains of both controllers were tunned manually. Initially, the steering controller was explored while throttle was set to 0.3. To do so, first only a proportional component with different gains was employed (Kp= 0.1-0.9). As expected, overshooting, getting off the track and oscillation were observed. Kp's in the range of 0.1-0.3 performed better. 
Then, a derivative term was added to the controller providing the ability of the controller to anticipate error. So, with derivative control, the control signal can become large if the error begins sloping upward, even while the magnitude of the error is still relatively small. This anticipation adds damping to the system, thereby decreasing overshoot. However, derivative is looking at fast, short-term changes in the error. So, in the noisy envoriment where error changes rapidly, a high derivative gain can cause the control output to go very high or low. Accordingly, the values in the range of 0.0001-0.0005 were examined. 
Due to presence of a seady-state error found. Hence, the integral part was added (Kd = 0.5-5). 
The initial goal was driving the car in the track safely. It was found that the car has some problem in turning the sharp bends in high speeds. So, another controller was designed to control the speed of the car. The desired speed was set to lower value in sharp bends and the same procedure was followed to tune the controller gains. 
The final parametrs are as follows: 
- Steering control gains: (0.20, 0.0005, 3.1)
- Speed control gains: (0.07, 0.0003, 0.1)
With these parameters, the car can drive with the average speed more than 40 mph and smooth enough, without leaving the road surface.


--------------
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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 




