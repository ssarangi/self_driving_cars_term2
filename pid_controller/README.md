# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program
---

# PID Controller

This project creates a PID controller. The following are the elements of the PID controller:

1. Proportional to Cross Track Error (CTE): proportional to - alpha1 * CTE in reference to where we want to be. The P component makes sure that we approach the Y value of where we want to be.
2. Integral to Cross Track Error (CTE): proportional to - alpha2 * [summation of all CTE at all time frames in reference to where we want to be]. The I component is used to guard against the drift error.
3. Differential to Cross Track Error (CTE): proportional to - alpha3 * [difference in CTE in reference to where we want to be]. The D component ensures that we have less oscillations when reaching the Y value of where we want to be.

This project was an alternate method of doing Udacity's Term 1 Project 3 where a car steering angle was being predicted using Deep Learning. The basic idea of a PID controller is that an error is measured and based on that some correction is applied. This process continues in a loop till a stable set of values are found which work for the controller.

# Idea
The idea for this project was fairly simple. There is a simulator which udacity developed which had to be used for this. The code connects to the simulator via a socket connection and then sends the calculated steering values to the simulator. The simulator then returns the CTE (Cross Track Error) or in simple terms how much the car has deviated from the center of the lane. This value is used to apply the correction in this case. Since PID is a well known and well explained topic I won't explain it here but I will rather focus on the steps I took to complete this project.

## Equation
![alt text](http://brettbeauregard.com/blog/wp-content/uploads/2011/03/pidalgorithm.png)

Let's define 3 terms to explain this.
current_error -> Defines the Error at the current timestep
previous_error -> Defines the Error at the previous timestep
cumulative_error -> Defines the Error accumulated over all the timesteps till now

So simply putting the equation for PID its as follows:
Error = -Kp * current_error - Kd * (current_error - previous_error) - Ki * (cumulative_error)

## Effect of P, I, D
# P

The P component compensates for the proportional error in the controller. Since its a direct proportion of the error in the current time step as error increases over time it causes the car to have a cyclic movement (going from side to side) after some time.
[![alt text](https://img.youtube.com/vi/r1lT29YvIvs/0.jpg)](https://youtu.be/r1lT29YvIvs)

# I

The I component mainly tries to compensate for drift error which is negligible in this case. However, a higher value for I causes the car to drift much vigorously since it gets multiplied by the accumulated error.
[![alt text](https://img.youtube.com/vi/YY-pS-Wp46c/0.jpg)](https://youtu.be/YY-pS-Wp46c)

# D

The D component mainly tries to reduce the diff in CTE in order to be able to reach the desired location without overshooting in both directions as it minimizes the CTE. However, its influence is low and hence a higher D value is needed to compensate since it only takes the error difference between current and previous timestep.
[![alt text](https://img.youtube.com/vi/aKFcGgmHbqM/0.jpg)](https://youtu.be/aKFcGgmHbqM)


## Choosing Parameters (Kp, Ki, Kd)
Choosing the values for Kp, Ki & Kd are the gist of this project. Once we have the right values for these 3 parameters, the car runs nicely on the road. So there are multiple ways in which this can be deduced.

### Manual Tuning
Using manual tuning, the parameters which worked for me were
* steering_Kp = 0.1;
* steering_Ki = 0.0001;
* steering_Kd = 4.0;

This process involved manually tuning all the parameters by hand by modifying one parameter and keeping others constant.

### Twiddle Algorithm
The other method I used was the Twiddle Algorithm. The twiddle algorithm is a very intuitive algorithm which helps in parameter tuning. The algorithm starts by initializing values to some known values and then modifying them by some factor, first increasing the parameter value, if it increases, then decrease the value by subtracting it and so on and so forth. Twiddle was explained in the lectures for Term 2 so I decided to try out this algorithm.

Because of the way the simulator was setup, the code was a little tricky to write because once the parameters were changed it wasn't straight forward to just run the car on the simulator and measure the combined error as explained in the lectures. Hence after modifications to suit the algorithm to this simulator I started my experiments.

One of my biggest hurdles was choosing the initial parameters. I took the approach in the lectures and applied p = [0, 0, 0] and dp = [1, 1, 1] and this caused the algorithm to never converge. Rather it caused it to go in circles and circles.

<b>I later realized that since the integral term and the proportional error were very high those parameters should be very low as compared to the differential term.</b>

Taking this approach, I used a very simple rule to initialize the values.

* Kd = (some value)
* Kp = 0.01 * Kd
* Ki = 0.01 * Kp

So p = [Kp, Ki, Kd]

Similarly for the dp parameter which controls how much of movement should happen while tuning the parameters I chose to use the same ratio of the parameters themselves.

```c++
double dp_Kd = 0.5 * Kd;
m_dp = {(Kp / dp_Kd) * Kp, dp_Kd, (Ki / dp_Kd) * Ki};
```

So the values for Kd have been halved and the rest of them are in the same ratio as the original parameters.

The other main thing to consider was the number of timesteps for error measurement. I used 2500 timesteps of error measurement.

# Results:
To enable Steering Twiddle, turn the <b>use_steering_twiddle</b> parameter to <b>true</b> in main.cpp
To enable Throttle Twiddle, turn the <b>use_throttle_twiddle</b> parameter to <b>true</b> in main.cpp

# Steering Angle:
Starting from
* Kp = 0.1
* Ki = 0.001
* Kd = 10

The final convergence for the steering angle was at
* Kp = 0.098
* Ki = 0.00100017
* Kd = 20.5

with a constant throttle of 0.3

# Throttle:
Due to lack of time I couldn't get the throttle PID to converge and hence that's a part TODO for the enhancement to this project.

---

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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
