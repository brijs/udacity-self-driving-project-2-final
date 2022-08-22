# Writeup: Track 3D-Objects Over Time

Please use this starter template to answer the following questions:

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?

See [below](#step-1-extended-kalman-filter) for implementation summary & results of the four steps.

**Challenges** - The project was for the most part a repeat of the individual exercises, so it wasn't so difficult in general. The part that took me some time was to really grasp the EKF intuition & equations (vs just blindly implementing the equations). I had to watch the videos again & also refer to resources (outsdie of udacity) to improve my understanding. Also made a summary table below:
<details>
  <summary><pre><strong>EKF summary table</strong></pre> (click to expand)</summary>

```
+======================+=======================+======================================+=============+======================================+
| Thing                | Type                  | Description                          | Shape       | Init                                 |
|                      |                       |                                      | (for 3D)    |                                      |
+======================+=======================+======================================+=============+======================================+
|                      |                       | State of Track                       | dim_state=6 | (x, y, z, Vx, Vy, Vz)                |
+----------------------+-----------------------+--------------------------------------+-------------+--------------------------------------+
| x                    | Vector                | State(Position & Velocity)           | (6,1)       |                                      |
| P                    | Covariance Matrix     | Model or Process Noise               | (6,6)       | high for vx,vy,vz                    |
|                      |                       |                                      |             |                                      |
+======================+=======================+======================================+=============+======================================+
|                      |                       | EKF: Prediction Step                 |             |                                      |
+----------------------+-----------------------+--------------------------------------+-------------+--------------------------------------+
| f,F                  | func, Mapping Matrix  | State Transition Model               | (6,6)       | Fixed per formula, params: dt        |
| Q                    | Covariance Matrix     | deviation from constant velocity     | (6,6)       | Fixed . params: q, dt                |
| x,P                  |                       | x, P <= Predicted                    |             |                                      |
|                      |                       |                                      |             |                                      |
+======================+=======================+======================================+=============+======================================+
|                      |                       | Measurement, Noise & Residual        |             | Vx,Vy,Vz are zeroes, so shape        |
|                      |                       | dim_measurement = 3 (or 2, camera)   |             | is (3, ..) instead of (6, ..)        |
+----------------------+-----------------------+--------------------------------------+-------------+--------------------------------------+
| z                    | Vector                | Measurement                          | (3,1) Lidar | (2,1) for Image                      |
| h(x) Output: Lidar   | func returning Vector | Map Veh x(6,1) => sensor coord       | (3,1) Lidar | fixed, return identity * x           |
| h(x) Output: Image   | func returning Vector | Map Veh x(6,1) => sensor coord       | (2,1) Image | non-linear, using c_i,c_j,fi,fj & x  |
| gamma                | Vector                | Residual ie Measurement error        | (3,1) Lidar | (2,1) for Image                      |
| R                    | Covariance Marix      | Noise in Lidar / Cameras             | (3,3) Lidar | sigma_lidar_x, y, z                  |
| S                    | Covariance Matrix     | Residual Noise                       | (3,3)       |                                      |
| _veh_to_sensor_coord |                       |                                      | (4,4)       | homgonenous, Rot + Trans             |
|                      |                       |                                      |             |                                      |
+======================+=======================+======================================+=============+======================================+
|                      |                       | EKF: Update Step                     |             |                                      |
+----------------------+-----------------------+--------------------------------------+-------------+--------------------------------------+
| H_J                  | Mapping Matrix        | Measurement Model (for Image Sensor) | (2,6)       | jacobian on left & x; rest zeroes    |
| H                    | Mapping Matrix        | Measurement Model (for Lidar Sensor) | (3,6)       | fixed, identity on left; rest zeroes |
| K                    | Matrix                | Kalman gain                          | (6,3)       | (6,2) for Image                      |
|                      |                       | = state(rows) x measurement(col)     |             | : "diagonal" matrixl; rest zeroes    |
|                      |                       |                                      |             |                                      |
+----------------------+-----------------------+--------------------------------------+-------------+--------------------------------------+
``` 
</details>


### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 

For the purposes of Tracking, fusing measurements from both sensors should help with improving the Tracking performance. Using multiple sensors combines their individual strengths (performance in various conditions such as environments, lighting, occlusions, weather, slope of roads & terrain etc), and provides better reliability in general.
 
For the specific Sequence and range of frames that we processed, the RMSE for all 3 Tracks was reduced when both Lidar and Camera measurements were used (vs when only Lidar was used).

### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?

In a real-life setup, a sensor fusion system is more complex, will consume more power and will be more costly (hardware) due to the additional suite of hardware/sensors. It's important to tune the parameters correctly for the individual sensors such as the variance values by proper testing & calibrations.


### 4. Can you think of ways to improve your tracking results in the future?

- Fine tune various parameters (variance), based on actual testing & calibration of sensors
- Experiment with various values of thresholds used in Track Management
- Use different Association algorithms such as GNN & PDA.


## Step 1: Extended Kalman Filter

The following were implemented as part of the EKF
- Covariance matrix: `Q`
- State Transition Model: `F` 
- Measurement noise & covariance matrix: `gamma`, `S`
- `predict()` and `update()` methods

The output generated for a simple single-target scenario is shown below. 
The computed RMSE is **0.32**.

<img src="screenshots/step1_rmse.jpg" width="450"></img>
<img src="screenshots/step1_single_track_plot.jpg" width="350"></img>
<figcaption>Figure 1: Single Track Plot & RMSE</figcaption>


## Step 2: Track Initialization & Management 

Relevant notes related to the implementation:
- `veh_to_sens` mapping matrix is used (in `Track.init()`), so that the Track related code is generalized across sensor types
- Introduced `tenative` state and threshold
- RMSE plot below : **0.78**
- For frames [65,100], below is a summary of interesting events

| Frame # |                   Event                    |                  Notes                  |
| ------- | ------------------------------------------ | --------------------------------------- |
| 67      | Track 0 is initialized                     |                                         |
| 68      | Track 0 is marked tentative                | threshold=0.1                           |
| 71      | Track 0 is confirmed                       | threshold=0.8                           |
| 72      | Track 0 score reaches 1.0                  | score reaches 1.0 and stays there       |
| 78-97   | No track<>meas association. No KF.update() | Pxx,Pyy keep increasing                 |
| 97      | Track 0 is deleted                         | Pxx (& Pyy) reach 9.94 and exceed max_P |

<img src="screenshots/step2_rmse.jpg" width="600"></img>
<figcaption>Figure 2: Single Track RMSE</figcaption>


## Step 3: Data Association - Measurements to Tracks

<img src="screenshots/step3_rmse.jpg" width="600"></img>
<figcaption>Figure 3: Multi Track RMSE</figcaption>


## Step 4: Sensor Fusion - with Camera

The relevant camera related functions are implemented
 - Sensor's `in_fov()` method (both camera and lidar)
 - Camera Sensor's `get_hx()` function returning non-linear vector
 - Initializing Camera measurement's `z` and `R` values

The output video of the final run is shown below:

https://user-images.githubusercontent.com/1574336/185986568-5041652d-2066-4aee-b14a-1e79002dbbab.mp4
<figcaption>Video 1: Multi Track (3) being tracked</figcaption>
