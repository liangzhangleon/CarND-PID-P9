# CarND-Controls-PID
**Author: Liang Zhang**
Self-Driving Car Engineer Nanodegree Program

[//]: # (References)
[video]: ./result.ogv
[wiki]: https://en.wikipedia.org/wiki/PID_controller

The goals / steps of this project are the following:
* implement PID control algorithm.
* parameter tunning/optimization using manual tunning, twiddle or other methods.
* describe how the parameters are tuned.
---
## Description for the effect of the P, I, D components
* **P** stands for Proportional
The proportional term produces an output value that is proportional to the current cross track error(CTE) value. The proportional response can be adjusted by multiplying the error by a constant K<sub>p</sub>.
A large K<sub>p</sub> results in a large change in the output for a given change in CTE. If K<sub>p</sub> is too high, the system can become unstable (overshooting). In contrast, a small K<sub>p</sub> results in a small output response to a large CTE, and a less responsive or less sensitive controller. If K<sub>p</sub> is too low, the control action may be too small when responding to error changes.

* **I** stands for Integral
The contribution from the integral term is proportional to both the magnitude of CTE and the duration of CTE. The integral in a PID controller is the sum of the instantaneous error over time and gives the accumulated offset that should have been corrected previously. The accumulated error is then multiplied by K<sub>i</sub> and added to the controller output.
The integral term accelerates the movement of the process towards reference value and eliminates the residual steady-state error that occurs with a pure proportional controller. However, since the integral term responds to accumulated errors from the past, it can cause the present value to overshoot the reference value.

* **D** stands for Derivative
The derivative of the process error is calculated by determining the slope of the error over time and multiplying this rate of change by the derivative gain K<sub>d</sub>. 
A derivative term does not consider the error (meaning it cannot bring it to zero: a pure D controller cannot bring the system to its reference value), but the rate of change of error, trying to bring this rate to zero. It aims at flattening the error trajectory and reduces overshoot.

Note that I used the Wiki page ![PID controller][wiki] as a reference for this section.
## How the final hyper-parameters were chosen
I manually tunned the hyper-parameters.  The tunning process can be divided into two steps with respect to two different throttle values. In the following is the detailed description.
* Step 1 (throttle = 0.3) : I first set  K<sub>i</sub> and K<sub>d</sub> values to zero. I then increase the K<sub>p</sub> until the car can drive for a short period with moderate overshooting. Then I set K<sub>p</sub> to approximately half of that value and start to tune K<sub>i</sub> until the offset can be corrected on the road except sharp turns. Finally, I tuned K<sub>d</sub> to reduce overshots.  

Here is the list of parameters I tried.
|No. | P | I | D |
|:---:|:---:|:---:|:---:|
| 1 | 0.001 | 0 | 0 | 
| 2 | 0.01 | 0 | 0 |
| 3 | 0.1 | 0 | 0 |
| 4 | 0.05 | 0 | 0 |
| 5 | 0.025 | 0 | 0 |
| 6 | 0.025 | 0.1 | 0 |
| 7 | 0.025 | 0.01 | 0 |
| 8 | 0.025 | 0.001 | 0 |
| 9 | 0.025 | 0.0001 | 0 |
| 10 | 0.025 | 0.0002 | 0 |
| 11 | 0.025 | 0.0002 | 0.1 |
| 12 | 0.025 | 0.0002 | 1 |
| 13 | 0.025 | 0.0002 | 2 |
| 14 | 0.025 | 0.0002 | 4 |

* Step 2  (throttle = 0.6 * fabs(1 - fabs(cte))) : In this step, I increased the base value of throttle as 0.6. In addition, I added a parameter fabs(1 - fabs(cte)) to decrease throttle when the cross track error is large. For a larger throttle value, I found I need to increase K<sub>p</sub> and K<sub>i</sub> so that the reaction time for correcting offset is reasonable at sharp turns. 

Here is the list of parameters I tried.
|No. | P | I | D |
|:---:|:---:|:---:|:---:|
| 1 | 0.035 | 0.0002 | 4 |
| 2 | 0.045 | 0.0002 | 4 |
| 3 | 0.065 | 0.0002 | 4 |
| 4 | 0.08 | 0.0002 | 4 |
| 5 | 0.08 | 0.00025 | 4 |
| 6 | 0.08 | 0.00025 | 3 |
| 7 | 0.08 | 0.00025 | 2 |
| 8 | 0.09 | 0.00025 | 2 |
| 9 | 0.1 | 0.00025 | 2 |
| 10 | 0.1 | 0.00025 | 2.2 |
## Result
[//]: # (References)
[video]: ./result.ogv
![Link to the result video][video]

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





