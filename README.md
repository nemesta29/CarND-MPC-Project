 CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5 * All OSes: [click here for installation instructions](https://cmake.org/install/)
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

## The Model
A motion model is used to imitate the vehicle's behaviour. While a dynamic model would describe the vehicle's behaviour in reality, a kinematic motion model offers a practical representation of the same. The state variables used to describe the car are as follows:
 - x,y : give the position of the car
 - psi : gives the orientation of the car
 - v : describes the velocity of the car
Given a car and waypoints for it to follow, MPC uses a polynomial to fit the waypoints, then predicts it's control inputs to follow the predicted trajectory. In our given model, we assume a two input control as follows:
 - delta : gives the steering angle of the vehicle (constrained between -25 degrees to 25 degrees)
 - a : gives the throttle response, where a negative acceleration signifies braking ( from -1  to 1)
 *Note : Constrains are used to keep real world limitations in check*
 
 Additionally, two error parameters were maintained to track how closely the trajectory was followed. The errors used were as follows:
  - cte : cross track error : gave how far from the car was from the trajectory
  - epsi : gives the error in orientation of the vehicle
 
 Our state is thus given by -> state : (x, y, psi, v, cte, epsi)
 Our actuators are given by -> control : (delta, acc)
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
 
Using the given state and actuators and applying the above equations, we have a model that represents our car and can be used to predict future control inputs.

## 
