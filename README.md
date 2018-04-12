# Overview

The main goal of the project is to implement PID controller in C++ to steer the self driving car around the track in a [simulator](https://github.com/udacity/self-driving-car-sim/releases). A PID (Proportional, Integral, Derivative) controller is a control loop feedback controller which is widely used in different control systems.These three controllers (P,I,D) are combined in such a way that it produces a control signal. This is how the vehicle uses steering, throttle, and brake to move through the world, executing a trajectory created by the path planning block.

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

## PID Controller components

### P Component

Before discussing about the P component we should define Cross Track Error (CTE). A cross track error is a distance of the vehicle from trajectory. In theory it’s best suited to control the car by steering in proportion to Cross Track Error(CTE).

P component of the PID controller sets the steering angle in proportion to CTE with a proportional factor tau.

```
-tau * cte
```
In other words, the P, or "proportional", component had the most directly observable effect on the car’s behavior. It causes the car to steer proportional (and opposite) to the car’s distance from the lane center(CTE) - if the car is far to the right it steers hard to the left, if it’s slightly to the left it steers slightly to the right.

### D Component

It is the differential component of the controller which helps to take temporal derivative of error. This means when the car turned enough to reduce the error, it will help not to overshoot through the x axis.

In other words, the D, or "differential", component counteracts the P component’s tendency to ring and overshoot the center line. A properly tuned D parameter will cause the car to approach the center line smoothly without ringing.

```
diff_cte = cte - prev_cte
prev_cte = cte
- tau_d * diff_cte
```

### I Component

It is the integral or sum of error to deal with systematic biases.

In other words, the I, or "integral", component counteracts a bias in the CTE which prevents the P-D controller from reaching the center line. This bias can take several forms, such as a steering drift , but I believe that in this particular implementation the I component particularly serves to reduce the CTE around curves.

```
int_cte += cte
tau_i * int_cte
```
And combination of these we can get PID controller to control the steering value.

```
cte = robot.y
diff_cte = cte - prev_cte
prev_cte = cte
int_cte += cte
steer = -tau_p * cte - tau_d * diff_cte - tau_i * int_cte
```

Parameter optimisation can be done manually or using Twiddle algorithm. 

Pseudocode for implementing the Twiddle algorithm is as follows:

```
function(tol=0.2) {
    p = [0, 0, 0]
    dp = [1, 1, 1]
    best_error = move_robot()
    loop untill sum(dp) > tol
        loop until the length of p using i
            p[i] += dp[i]
            error = move_robot()

            if err < best_err
                best_err = err
                dp[i] *= 1.1
            else
                p[i] -= 2 * dp[i]
                error = move_robot()

                if err < best_err
                    best_err = err
                    dp[i] *= 1.1
                else
                    p[i] += dp[i]
                    dp[i] *= 0.9
    return p
}
```

## How I chose the parameters

### P value

In the first run I chose to tune the parameters manually and then use those manually tuned parameters as a starting point for Twiddle algorithm. First I increased the proportional factor until the car in the simulator starts to oscillate around the center of the lane. I started with 0.01 and ended at 0.05. I got the P value with oscillating behaviour when I set the value to 0.05 and I and D set to zero.

### D value

I started to increase the differential parameter until the transient oscillation was sufficient fast. It was kind of a fine-tuning to not over-damp or under-damp the car control. After finding a suitable paramter that shows a good performance in the simulator I started to increase the integral parameter by very small amounts. I found the D value that stops the oscillating behaviour which is set to 1.5 along with 0.05 for P and zero for I.

### I value

As described above the I-parameter compensates the bias in the CTE, e.g. due to mechanical issues. Since the car in the simulator does not have a certain mechanical bias, therefore I kept it very small 0.0001 at the beginning.

The following animation perfectly describes the approach for manually tuning the PID parameters: 
![alt Text](Imgs/P_I_D_effect.gif)

source: https://www.wikiwand.com/en/PID_controller


## Using Twiddle algorithm

I have taken these manually tuned parameters as the initial values for Twiddle algorithm and ran it for few iterations to let it come up with fine tuned parameters. I modified the main.cpp to implement Twiddle algorithm. When twiddle variable set to true, simulator runs the car with the P,I,D values till the maximum steps set initially and go through the twiddle algorithm. After competition of each iteration, simulator reset to initial stage and car starts from the beginning of the track to maximum steps. This process continuous until tol value below the allowed value.

Finally I got the optimised P,I,D values as below: 

```
0.072685 0.000181674 1.42873
```

[Here](https://youtu.be/k8zeks4MPZM) is a link to the video that I recorded while car was run autonomously using these fine tuned parameters

