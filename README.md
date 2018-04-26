 CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies
* cmake >= 3.5 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSocket
    cd uWebSockets
    git checkout e94b6e1
    ``
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

## The Model
A motion model is used to imitate the vehicle's behaviour. While a dynamic model would describe the vehicle's behaviour in reality, a kinematic motion model offers a practical representation of the same. The state variables used to describe the car are as follows:
 - x,y : give the position of the car
 - psi : gives the orientation of the car
 - v : describes the velocity of the car
Given a car and waypoints for it to follow, MPC uses a polynomial to fit the waypoints, then predicts it's control inputs to follow the predicted trajectory. In our given model, we assume a two input control as follows:
 - delta : gives the steering angle of the vehicle (constrained between -25 degrees to 25 degrees)
 - acc : gives the throttle response, where a negative acceleration signifies braking ( from -1  to 1)
 *Note : Constrains are used to keep real world limitations in check*
 
 Additionally, two error parameters were maintained to track how closely the trajectory was followed. The errors used were as follows:
  - cte : cross track error : gave how far from the car was from the trajectory
  - epsi : gives the error in orientation of the vehicle
  
 Our state is thus given by -> state : (x, y, psi, v, cte, epsi) while the actuators are given by -> control : (delta, acc)
 
 The following equations were used to update the state:
  - x_new = x + v * cos(psi) * dt
  - y_new = y + v * sin(psi) * dt
  - psi_new = psi + (v / Lf) * delta * dt
  - v_new = v + acc * dt
  - cte_new = cte + v * sin(psi) + dt
  - epsi_new = epsi + (v / Lf) * delta * dt
  where,
  Lf = distance of front axle from centre of gravity
  cte = difference in prediction and actual trajectory => f(x) - y
  epsi = psi - psi_des, where psi_des is given by arctan(f'(x)) 
  dt = elapsed duration
 
Using the given state and actuators and applying the above equations, we have a model that represents our car and can be used to predict future control inputs.

## Timestep Length and Elapsed Duration

Timestep Length is used to decide how far into the future the predictions are made

Elapsed Duration is the duration of each timestep

For the given situation, a timestep length of N = 25 and an elapsed duration of dt = 0.05 was used. These values gave a step size that was small enough to make predictions with reduced oscillations and a low error. 
Both classroom material and trial and error were used to reach this value. Other values tried were N = 10 and dt = 0.1, N = 15 and dt 0.3.
## Polynomial Fitting and MPC Preprocessing
The waypoints are presented in Global Coordinates, while the simulator receives signals in the vehicle coordinates. A conversion process is hence carried out, so that the car becomes origin as is the case for vehicle coordinates. With this conversion, the x, y and psi values become 0 when converted to vehicle coordinates.(Lines 106 to 113)

For fitting the waypoints in a trajectory, a 3rd degree polynomial was used as it was seen sufficient to accomodate curves of the given path.

## Model Predictive Control with Latency
Latency can be regarded as the internal delay that occurs between an actuator command being issued and the actual change in control. This has been reflected in the system using a delay of 100ms.

MPC deals with latency in the following manner. It simulates the given state through a latency till actual change is reached in the control. The state at that point is then used as input to the system. 

This is implemented in the code by updating the state variables for a timestep equal to latency. The new state achieved is then used as input for the controller function.(Lines 100 to 103)
