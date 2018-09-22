# CarND-Controls-PID Project Report
Self-Driving Car Engineer Nanodegree Program

## Overview

Purpose of the project is understand controller mechanism and its tuning methods. 
Objective of the project is to design controller that controls car using accel, brakes and steering, given input of cross track error(difference between current position of a car and its ideal trajectory).
I am using a PID controller to control steering angle and another PID controller to control speed of the car. I am using manual tuning for hyperparameter tuning for both of PID controllers.


### Description of the effect each of the P, I, D components had in your implementation
Input given in simulation environment is cross-track error. Using this error, I am computing what shall be steering angle input. 
Proportional parameter causes to compute steering angle directly proportional to cross track error. This parameter helps to calculate ideal steering angle but it is never going to be perfect, since error is never going to be precisely 0. This parameter is the cause of overshoot and causes system to overshoot-undershoot around target value.
As the cross track error changes with respect to environment on differnt turns, overshoot and undershoot starts resonating causing raise in cross track error.

Differential parameter compensates resonance caused by proportional factor and increases stability of car to reach to target value.

Integral parameter compensates bias in cross track error and helps to reach target precisely. PD controller might end up with small deviation in actual value and target value, and this difference is accumulated by integral term. As accumulation increases, this factor becomes significant and corrects control input so as to reach to the target value precisely.


### Description of final hyperparameters
Hyperparameters are tuned manually. I started with the simplest system Kp=-1, Ki=0, Kd=0, basically without any appropriate control. This set controls car well for very small cross track error. When cross track error starts increasing on any simple turns, proportional factor being set to 1, car starts driving towards edges rapidly stating clearly proportional control param is too high. Tuning this param to Kp=-0.2 the car starts resonating with rapid changes in steering angle and starts oscillating around the center of the track.

To compensate these oscillations, I used differential term, Kp=-0.2, Kd=-1. This set offers descent stability to follow the straight track, however, at turns, the car gets unstable, with considerable oscillations, stating that differental term is not tuned very well and can further be increased. By updating parameters to Kp=-0.2, Kd=-2.5, the car drives well, still the car shows sudden changes in steering angle at sharp turns indicating that proportional factor is contributing a little more than required. Tuning this parameter to Kp=0.75, Kd=2.5, car drives almost perfectly on the track. With little more efforts and trying tuning parameters, here are my tuned parameters: Kp=-0.15, Kd=-2.5.

Integral term was not required until this moment, however, when added, the car starts turning precisly more responsive at turns. After considering effect of integral term, parameters are tuned to drive a little bit agressively. Parameters are tuned at Kp=-0.13, Kd=-3.5, Ki=-0.002.

I also added a controller to control the speed of the car so that it can accelerate to high speeds up to 60 on straight track and can decelerate on sharp turns, still maintaining speed upto 30-33. The PID controller for speed uses steering angle as input parameter to calculate the speed, as the steering angle increases, which is case of turn, the speed shall decrease.
Speed controller parameters are also tuned to drive a little aggressively.

[Watch YouTube video](https://youtu.be/ow9HzETLxn8)


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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

