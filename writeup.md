# Writeup: Track 3D-Objects Over Time

Please use this starter template to answer the following questions:

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?


### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 


### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?


### 4. Can you think of ways to improve your tracking results in the future?



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