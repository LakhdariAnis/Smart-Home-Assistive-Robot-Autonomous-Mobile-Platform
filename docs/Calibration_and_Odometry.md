# Sensor Calibration and Odometry

## Overview
This document details the mathematical verification of the robot's movement, focusing on sensor calibration and odometry. It covers the ground truth speed verification process using physical measurements to resolve monocular scale ambiguity, as well as the intrinsic calibration of the Raspberry Pi Camera.

## Ground Truth Speed Verification
Monocular SLAM algorithms (like ORB-SLAM3 tracking a single camera) inherently suffer from scale ambiguity—they can determine the shape of a trajectory but not its physical size in meters. To resolve this, we established a physical ground truth to constrain and rescale the SLAM output.

### Methodology
Using `test_car_speed.py` and `calibrate_120m.py`, we conducted physical tests to measure the real-world performance of the motor control signals:
1. A precise track of known length was laid out on the operating surface.
2. The robot was commanded to drive forward at a stable, specified PWM duty cycle.
3. The time taken to traverse the track was recorded over multiple iterative runs to account for battery voltage drop and mechanical friction.

### Results & SLAM Scale Constraint
Based on the empirical data gathered, the **average speed was determined to be 0.2367 m/s**.

**Application:** 
This average speed acts as our primary metric scale constraint. When the IMU/Odometry module or SLAM bridge runs, this constant (`AVERAGE_SPEED = 0.2367`) is used to integrate the position over time ($position += \text{AVERAGE\_SPEED} \times \Delta t \times \text{heading\_vector}$). By comparing this metric dead-reckoning distance against the arbitrary distance units produced by ORB-SLAM3, we apply a scale factor (e.g., converting SLAM map coordinates to meters), enabling accurate target-to-map navigation in the 2D UI.

## Camera Intrinsic Calibration
ORB-SLAM3 requires highly accurate camera intrinsic parameters to project 2D image pixels into 3D space. Any uncorrected distortion significantly degrades tracking accuracy over time.

### Methodology
The intrinsic parameters were calculated using the `camera_calibration.py` script:
1. We utilized a standard **9x6 checkerboard** pattern of known square dimensions.
2. The camera captured a broad dataset of images, observing the checkerboard from various extreme angles, tilts, and distances to cover the camera's entire Field of View (FOV).
3. OpenCV's `calibrateCamera` function processed the image set, detecting the corners and computing the focal lengths ($f_x$, $f_y$), principal point ($c_x$, $c_y$), and radial/tangential distortion coefficients ($k_1, k_2, p_1, p_2, k_3$).

### Results & Implications
The iterative calibration process successfully minimized the **RMS (Root Mean Square) reprojection error to 0.3297 px**.

**Why this matters:**
*   **Sub-centimeter tracking:** Achieving an RMS error well below 0.5 pixels represents a high-quality calibration. It ensures that when ORB-SLAM3 extracts FAST features, their geometrical mapping matches reality with sub-centimeter precision, keeping the point cloud dense and structurally sound.
*   **Distortion Mapping:** The calculated distortion parameters correctly un-warp the raw frames before they enter the SLAM pipeline, preventing "bowed" walls and artificial drift during rotations. These values populate the `picam.yaml` configuration file for edge processing.
