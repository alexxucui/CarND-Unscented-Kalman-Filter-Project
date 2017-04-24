# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

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

## Lidar and Radar Data Fusion Pipline

<img src="https://github.com/alexxucui/CarND-Unscented-Kalman-Filter-Project/blob/master/img/ukfpipelane.PNG" width="700">

<img src="https://github.com/alexxucui/CarND-Unscented-Kalman-Filter-Project/blob/master/img/process.PNG" width="700">


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
![](https://github.com/alexxucui/CarND-Unscented-Kalman-Filter-Project/blob/master/visualization/output1_RL.png)

Sample Data 1 - NIS
![](https://github.com/alexxucui/CarND-Unscented-Kalman-Filter-Project/blob/master/visualization/data1_NIS_radar.png)

Sample Data 2 - RL
![](https://github.com/alexxucui/CarND-Unscented-Kalman-Filter-Project/blob/master/visualization/output2_RL.png)

Sample Data 2 - NIS
![](https://github.com/alexxucui/CarND-Unscented-Kalman-Filter-Project/blob/master/visualization/data2_NIS_radar.png)

## Conclusions

* From RMSE table and compared with EKF project, the UKF model performs on par in sample data 1 but improves in sample data 2, expecially in vx and vy esitmation.
* From the visulization, UKF model predicts better in a non-linear movement, especally during a sharp turn where it doesn't overshoot. From NIS consistency check, the model is consistent for sample data 1 but slighly over-estimate the uncertainty for sample data 2.
