# Raspberry Pi Edge Node: Low-Level Firmware & Sensor Stack

This directory contains the edge-computing firmware running directly on the Raspberry Pi. The stack is engineered for low-latency data acquisition, robust sensor fusion, and precise motor actuation to support real-time SLAM and autonomous navigation.

## 📷 Vision Pipeline (`camera_zmq.py`)
The imaging stack leverages `picamera2` to interface directly with the IMX219 sensor, capturing uncompressed **BGR888** frames natively required by OpenCV. 
To balance network bandwidth and image quality for PC-side ORB-SLAM3, the pipeline implements a **JPEG encoding strategy** (typically Quality 80). This maintains the distinct, high-frequency edge and corner features critical for ORB feature extraction while allowing the video stream to publish at a consistent 30 FPS over a ZeroMQ (PUB) socket without network bottlenecking.

## 🧭 Sensor Reliability & Odometry (`imu_zmq.py`)
Inertial tracking relies on an MPU6050 6-DOF IMU polled at 50Hz via I2C. A major challenge with low-cost MEMS IMUs is double-integration drift. To combat this, the firmware implements **Hysteresis ZUPT (Zero Velocity Update)**. 
By continuously calculating an acceleration variance window, the system accurately detects when the robot is physically stationary and instantly hard-clamps the velocity vector to absolute zero. This entirely eliminates stationary tracking drift. While moving, heading (yaw) is derived via a complementary filter (alpha=0.98) on the Z-axis gyroscope, forming a reliable fallback odometry stream broadcasted via structured JSON.

## ⚙️ Motor Control Stack (`car.py` & `car_node.py`)
The actuation layer drives a 4-wheel differential setup utilizing dual L298N H-bridge motor controllers. It features granular speed regulation driven by software PWM running natively at **1000Hz**. 

**Hardware GPIO Mapping (BCM):**
*   **Front Right (FR):** ENA = 12 (PWM), IN1 = 17, IN2 = 27
*   **Front Left (FL):** ENB = 13 (PWM), IN3 = 22, IN4 = 23
*   **Rear Left (RL):** ENA2 = 18 (PWM), IN5 = 24, IN6 = 25
*   **Rear Right (RR):** ENB2 = 19 (PWM), IN7 = 5, IN8 = 6

This optimized topology allows for precise thresholding and differential steering, enabling smooth translations and in-place rotations crucial for closing physical SLAM loops.

## 🐍 Python Dependencies
Hardware interaction and network messaging require:
```bash
pip install pyzmq smbus2 opencv-python RPi.GPIO Flask
```
