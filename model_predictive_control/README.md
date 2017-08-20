# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Model

Global kinematic model was used in this model predictive controller.

The state consists of X position, Y position, psi angle, v speed, Cross track error (distance between vehicle and it's trajectory) and Error in PSI (difference between the vehicle angle and the trajectory angle).

The actuators for the model are delta (for steering angle) and acceleration, which is the output of the model after solving it given the model constraints which are described below.

The number of time steps in the future are determined by N (value of 15) and T (1) where dt is 1 in this case.
The values of N and T were picked by trial and error, with the following guidelines:
 - If the model is trying to predict too long time in the future, it takes a while to compute and all of them are being then thrown away
 - If the model is trying to predict too short in the future, then it doesn't have any basis of what will happen after the next step which leads to bad prediction and the car usually ends up driving off track

A value in the middle is the best to avoid too much computation and being short sighted which was adjusted with trial and error.

In order to account for the 100ms delay, this_thread::sleep_for(chrono::milliseconds(100)); was added before sending the steer and acceleration to the simulator.
In the model predictive controller, the x position sent to the state is advanced by 100 ms by doing: future_x = v * dt (where dt is 0.1) in order to account for this.

## Code Structure

Main does the following:

1. As the simulator is running, it gets a 'telemetry' event which has the following information:

  a. (ptsx, ptsy) which are the map coordinates
  b. (px, py) which are the vehicle coordinates
  c. psi which is the angle
  d. psi_unity
  e. v which is the speed

2. Create ptsxv and ptsyv which are the map coordinates
3. Convert them into vehicle space coordinates by doing the following calculation:

  a. x = ptsxv - px
  b. y = ptsyv - py
  c. new_x_coordinate = x * cos(psi) + y * sin(psi)
  d. new_y_coordinate = -x * sin(psi) + y * cos(psi)

4. Fit ptsxv and ptsyv into 3rd degree polynomial and get the coefficients.
5. Get cross track error by evaluating the polynomial with the coefficients from the previous step and at x = 0 and multiply it be -1.
6. Calculate psi error (epsi) by evaluating atan(coefficients[1]) and multiplying it by -1.
7. Create the current state from (px, py, psi, v, cte, epsi)
8. Solve given the current state and the constraints (handled by the MPC module)
9. Get the steer value and the throttle value
10. Display the MPC predicted trajectory and the reference points

MPC module does the following:

1. Get the current state from the input (x, y, psi, v, cte, epsi)
2. Define the number of constraints to be 6
3. Set the upper bound and lower bound for:

    a. all variables (big negative number, big positive number)
    b. delta to be [-25 degrees to 25 degrees]
    c. accelerator to be [-1 to 1]

4. Compute the solution from the constraints using the following:

    a. Define the cost to accumulate the following:

        1. power(velocity difference to reference velocity of 50, 2) => Will keep velocity close to 0
        2. power(cte difference to reference cte of 0, 2) => Will keep CTE close to 0
        3. 2 * power(epsi difference to reference epsi of 0, 2) => Will keep EPSI close to 0
        4. power(delta, 2)
        5. power(acceleration, 2)
        6. 200 * power(delta difference, 2) => Will make changes in delta happen more slowly and smoothly
        7. 10 * power(acceleration difference, 2) => Will make changes in acceleration happen more slowly and smoothly
        8. power(psi difference, 2) => This will make changes in psi happen more slowly and smoothly
        9. power(cte difference, 2) => This will make changes in cte happen more slowly to avoid having the car jerk around

    b. Copy over the state for the next N times
    c. Create all the constraints of the following:

        1. X constraint: x1 - (x0 + v0 * cos(psi0) * dt)
        2. Y constraint: y1 - (y0 + v0 * sin(psi0) * dt)
        3. PSI Constraint: psi1 - (psi0 + v0 * delta0 / Lf * dt)
        4. V Constraint: v1 - (v0 + a0 * dt)
        5. CTE Constraint: cte1 - ((f0 - y0) + (v0 * sin(epsi0) * dt))
        6. EPSI Constraint: epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt)

        where f0 = coeffs[0] + (coeffs[1] * x0) + (coeffs[2] * pow(x0,2)) + (coeffs[3] * pow(x0,3))
        and psides0 = atan(coeffs[1] + (2 * coeffs[2] * x0) + (3 * coeffs[3]* pow(x0,2) ))


5. Get the delta and acceleration from the solution

## Simulation

The following shows simulation using this Model Predictive Controller.
The green line represents the predicted path while the yellow line represents the ground truth.

## Credit

Tips have been followed from https://github.com/hortovanyi/CarND-MPC-Project

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
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
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
