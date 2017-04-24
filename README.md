# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.


## Lidar and Radar Data Fusion Pipline

<img src="https://github.com/alexxucui/CarND-Unscented-Kalman-Filter-Project/blob/master/img/ukfpipelane.PNG" width="700">

## UKF Evaluation (RMSE)

For Sample Data 1:

| RMSE |     R    |     L    |     RL    |
|:----:|:--------:|:--------:|:---------:|
|  px  | 0.147402 | 0.126846 |  0.066121 |
|  py  |  0.18935 | 0.137202 | 0.0752295 |
|  vx  |  0.61559 | 0.714981 |  0.566673 |
|  vy  | 0.632704 | 0.637848 |  0.570433 |

For Sample Data 2:

| RMSE |     R    |     L    |    RL    |
|:----:|:--------:|:--------:|:--------:|
|  px  |    N/A   | 0.213169 | 0.184595 |
|  py  |    N/A   | 0.186715 | 0.180058 |
|  vx  |    N/A   | 0.291684 | 0.223678 |
|  vy  |    N/A   | 0.259848 |  0.24774 |

N/A: LLT failed!
R: Radar L: Lidar

## Compare with EKF

For Sample Data 1:

| RMSE |     R    |     L    |     RL    |
|:----:|:--------:|:--------:|:---------:|
|  px  | 0.130167 | 0.105896 | 0.0651649 |
|  py  | 0.134765 | 0.107821 | 0.0605378 |
|  vx  | 0.613422 | 0.724494 |  0.54319  |
|  vy  | 0.581875 | 0.638341 |  0.544191 |

For Sample Data 2:

| RMSE |     R    |     L    |    RL    |
|:----:|:--------:|:--------:|:--------:|
|  px  |  1.57139 | 0.217995 | 0.185465 |
|  py  | 0.812246 | 0.194325 | 0.190254 |
|  vx  | 0.938841 |  0.93745 | 0.476509 |
|  vy  |  1.16265 | 0.833882 | 0.810787 |


## Visulization 
Output data:   `/data`
Images: `/visuliazation`

Kalman Filter Esitmate - Measurement - Ground Truth

Sample Data 1 - RL
![](https://github.com/alexxucui/CarND-Extended-Kalman-Filter-Project/blob/master/visualization/sample1%20-%20LR.png)

Sample Data 1 - R
![](https://github.com/alexxucui/CarND-Extended-Kalman-Filter-Project/blob/master/visualization/sample1%20-%20R.png)

Sample Data 1 - L
![](https://github.com/alexxucui/CarND-Extended-Kalman-Filter-Project/blob/master/visualization/sample1%20-%20L.png)

Sample Data 2 - RL
![](https://github.com/alexxucui/CarND-Extended-Kalman-Filter-Project/blob/master/visualization/sample2%20-%20LR.png)

Sample Data 2 - R
![](https://github.com/alexxucui/CarND-Extended-Kalman-Filter-Project/blob/master/visualization/sample2%20-%20R.png)

Sample Data 2 - L
![](https://github.com/alexxucui/CarND-Extended-Kalman-Filter-Project/blob/master/visualization/sample2%20-%20L.png)

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/f437b8b0-f2d8-43b0-9662-72ac4e4029c1)
for instructions and the project rubric.
