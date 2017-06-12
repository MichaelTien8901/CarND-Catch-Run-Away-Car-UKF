# Run Away Robot with Unscented Kalman Filter Bonus Challenge Starter Code
Self-Driving Car Engineer Nanodegree Program

---

Overview

This repository contains all the code needed to complete the Bonus Challenge: Cathc the Run Away Car with Unscented Kalman Filter.

Project Introduction

In this project, not only do you implement an UKF, but also use it to catch an escaped car driving in a circular path. 
The run away car will be being sensed by a stationary sensor, that is able to measure both noisy lidar and radar data. The capture vehicle will need to use these measurements to close in on the run away car. To capture the run away car the capture vehicle needs to come within .1 unit distance of its position. However the capture car and the run away car have the same max velocity, so if the capture vehicle wants to catch the car, it will need to predict where the car will be ahead of time.

Running the Code

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

mkdir build 

cd build 

cmake .. make 

./UnscentedKF

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, ukf.h, and main.cpp which will use some stragety to catch the car, just going to the cars current esimtated position will not be enough since the capture vehicle is not fast enough. There are a number of different strageties you can use to try to catch the car, but all will likely involve prediciting where the car will be in the future which the UKF can do. Also remember that the run away car is simplying moving a circular path without any noise in its movements.


Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program



// current noiseless position state of the capture vehicle, called hunter

["hunter_x"]

["hunter_y"]

["hunter_heading"]

// get noisy lidar and radar measurments from the run away car.

["lidar_measurement"]

["radar_measurement"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["turn"] <= the desired angle of the capture car "hunter" no limit for the anlge

["dist"] <= the desired distance to move the capture car "hunter" can't move faster than run away car



## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4
* uWebSocketIO

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF 

## Math Formula
### Formula 1
Solve 
$$a1x + b1y + c1 = 0$$
$$a2x + b2y + c2 = 0$$
for $$x, y$$
Use [wolframeaplpha](https://www.wolframalpha.com/input/?i=a1x+%2B+b1y+%2B+c1+%3D+0,+a2x+%2B+b2y+%2B+c2+%3D+0,+solve+x,+y) to find the solution.
Results,
$$x=\frac{b2c1-b1c2}{a2b1-a1b2}$$ and $$y=\frac{a2c1-a1c2}{a1b2-a2b1}$$ and $$a2b1 \neq a1b2$$ and $$b1\neq0$$
$$x=-\frac{c1}{a1}$$ and $$y=\frac{a2c1-a1c2}{a1b2}$$ and $$b1=0$$ and $$a1\neq 0$$ and $$b2\neq 0$$
$$x=-\frac{c2}{a2}$$ and $$b2=0$$ and $$b1=0$$ and $$c2\neq0$$ and $$a1=\frac{a2c1}{c2}$$ and $$a2c1\neq0$$
$$y=-\frac{a2x+c2}{b2}$$ and $$c1=b1=a1=0$$ and $$b2\neq0$$ and $$c2\neq0$$
$$y=-\frac{a1x+c1}{b1}$$ and $$c2=b2=a2=0$$ and $$b1\neq0$$ and $$c1\neq0$$


### Formula 2

Simplify $$(a-x)^2 + (b-y)^2 - (c-x)^2 - (d-y)^2 = 0$$
Use [wolframeapha](https://www.wolframalpha.com/input/?i=(a-x)%5E2+%2B+(b-y)%5E2+-+(c-x)%5E2+-+(d-y)%5E2+%3D+0,+simplify)
Results,
$$(2a-2c)x+(2b-2d)y+(c^2+d^2-a^2-b^2)=0$$

### Find Circle center with 3 points of circle

If coordinate of 3 points are $$(a, b)$$, $$(c, d)$$, $$(e, f)$$
and center of of circle is $$(x, y)$$
Radius $$r^2 = (a-x)^2+(b-y)^2 = (c-x)^2+(d-y)^2$$
Radius $$r^2 = (a-x)^2+(b-y)^2= (e-x)^2+(f-y)^2$$

We can use formula 2 to simplify both equations and use formula 1 to solve x, y, center of circle.

### Strategy for Interception
 * Direct the hunter car to center of circle
 * At the center of circle, the distance between and point of circle is equal.  That is, we can calculate $$\Delta t$$ if velocity v is known.  
 * $$radius d = v * \Delta t$$
 * Since d = radius, $$v^2 = v_x^2 + v_y^2$$, we can calculate $$\Delta t$$.
 * From any 2 points, we can calculate $$\theta$$ at each points.  From the time difference from these 2 points, we can know $$\dot \theta = \frac{\theta 1 - \theta 2}{t2-t1}$$. 
 * $$\Delta \theta = \dot \theta * \Delta t$$
 