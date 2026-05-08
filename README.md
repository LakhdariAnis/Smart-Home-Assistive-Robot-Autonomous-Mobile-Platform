# Smart Home Assistive Robot: Autonomous Mobile Platform

## Executive Summary: Distributed Computation Offloading

This project demonstrates a highly optimized **Distributed Computation Offloading** architecture designed for autonomous robotics. By decoupling low-level hardware interactions from computationally expensive algorithms, the system maximizes performance across constrained and high-end hardware.

### The Thin Client: Raspberry Pi (Edge Node)
The Raspberry Pi acts as a dedicated **Thin Client** for raw sensor data acquisition and motor actuation. Operating close to the metal, it handles:
- **Camera Streaming:** Capturing and compressing frames from the Pi Camera at **30fps**.
- **Odometry & Telemetry:** Reading raw IMU data (MPU6050) at **50Hz**.
- **Motor Control:** Translating high-level navigation commands into precise PWM signals for a 4-wheel drive system and 4-DOF robotic arm.

### The Inference Engine: PC (High-Performance Node)
To overcome the limitations of the edge processor, all heavy mathematical lifting is offloaded to a high-performance PC acting as the **Inference Engine**:
- **Visual SLAM:** Consumes the 30fps camera stream to run **ORB-SLAM3** mapping and localization.
- **Sensor Fusion:** Merges SLAM coordinates with the 50Hz IMU stream for robust trajectory estimation.
- **Path Planning:** Calculates real-time navigation matrices and issues sub-second control commands back to the edge node.

## Hardware-Software Interface

At the boundary between physical sensors and distributed networks lies a robust hardware-software interface. 
- **I2C to Network Packets:** Hardware signals from the MPU6050 accelerometer and gyroscope are read via the **I2C bus**, calibrated in real-time, and serialized into JSON payloads.
- **ZeroMQ (ZMQ) Backbone:** These packets, along with compressed JPEG frames, are broadcast over a local network using **ZeroMQ (ZMQ)**. This guarantees high-throughput, low-latency asynchronous message passing, bridging raw physical measurements with the software inference engine.

## Collaborative Integration: The Physical Agent

This mobile platform does not exist in isolation. It serves as the physical agent for a wider **Smart Home / 3D Web ecosystem**.
- **Bridging the Digital and Physical:** By rendering spatial data into a 3D web interface, users can visualize the robot's real-time position within their smart home.
- **Interactive Command:** Users interact with the 3D map to define waypoints or tasks, which are asynchronously sent as navigation targets to the robot.
- **Extensible Workflows:** Whether inspecting an area via the camera stream or retrieving an object utilizing the 4-DOF arm, the robot translates digital intentions into tangible physical actions within the collaborative ecosystem.
