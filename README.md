# CarND-Controls-PID
**Author: Liang Zhang**
Self-Driving Car Engineer Nanodegree Program

The goals / steps of this project are the following:
* implement PID control algorithm.
* parameter tunning/optimization using twiddle or other methods.
* describe how the parameters are tuned.
---
## Description for the effect of the P, I, D components

* Record of a small video of the car in the simulator

## How the final hyperparameters were chosen
* If the system must remain online, one tuning method is to first set  K <sub>i</sub> and K <sub>d</sub> values to zero. Increase the K <sub>p</sub> until the output of the loop oscillates, then the K <sub>p</sub> should be set to approximately half of that value for a "quarter amplitude decay" type response. Then increase K <sub></sub> untill any offset is corrected in sufficient time for the process. However, too much K <sub>i</sub> will cause instability. Finally, increase K <sub>d</sub>, if required, until the loop is acceptably quick to reach its reference after a load disturbance. However, too much K <sub>d</sub> will cause excessive response and overshoot. A fast PID loop tuning usually overshoots slightly to reach the setpoint more quickly; however, some systems cannot accept overshoot, in which case an overdamped closed-loop system is required, which will require a K <sub>p</sub> setting significantly less than half that of the K <sub>p</sub> setting that was causing oscillation.

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

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)





