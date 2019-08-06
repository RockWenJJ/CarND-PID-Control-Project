# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Introduction
In this project, I have implemented a PID controller of a self-driving car by manual tuning the hyperparameters and it has been successfully tested in the simulator. There are two controllers, one controls the steering angle while the other
controls the throttle. CTE(cross-track error) was used for steering angle control and SPE(speed error) for throttle control. SPE is defined by the error between the current speed and the target speed. Target speed for this simulator is 40mph.
Steering angle, speed and CTE is provided by the simulator via local websocket.

### Requirements
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

## Results
The final result is shown as below.
![output](output.gif)

## Reflections
* Effect of P,I,D components
    * Proportional: For both steering angle and throttle controls, the "P" component had the most observable effect on the car's behavior.
    It caused the car to steer proportional to the car's distance from the lane center, which means if the car is too far away from the lane center,
    then the steering angle will be large to make the car back to the center. Likewise, for the throttle control, when the current speed is far less than
    the target speed, the throttle is large. 
    * Differential: The "D" component counteracts the "P" component's tendency to ring and overshoot the lane center. A properly tuned "D" component can make
    the car to approach the lane center more smoothly. 
    * Integral: The "I" component counteracts a bias in the CTE to prevent the P-D controller from reaching the center line.
 * How the final hyperparameters were chosen
    * The final hyperparameters were chosen by manual tuning. Firstly, the hyperparameters of throttle controller were tuned to make the car drive at a reasonable
    speed. Then, the "P" component of steering angle controller was adjusted from 0.05 to 0.16, with a step of 0.01 for each adjustment. The "I" component was then
    tuned from 0.001 to 0.003, with a step of 0.001 for each adjustment. The "D" component was tuned to 2.01 from 0.1.
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

