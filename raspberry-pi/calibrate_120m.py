#!/usr/bin/env python3
import zmq
import json
import math
import sys

# --- ZMQ Configuration ---
ZMQ_ADDR = "tcp://localhost:5556"

def get_latest_odom():
    """Connects, grabs one fresh message, and closes to ensure no stale data."""
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.connect(ZMQ_ADDR)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")

    # Wait for a fresh message
    try:
        msg = sock.recv_string(flags=zmq.NOBLOCK) # clear buffer
    except zmq.Again:
        pass

    msg = sock.recv_string()
    data = json.loads(msg)

    sock.close()
    ctx.term()
    return data["x"], data["y"]

def calculate_dist(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# --- Main Logic ---
print("\n" + "═"*45)
print("  IMU ODOMETRY DISTANCE MEASURER")
print("  Ensure imu_odom.py is running!")
print("═"*45)

try:
    # 1. Capture Start Point
    input("\n[STEP 1] Place car at START line.\nPress ENTER to lock start position...")
    x_start, y_start = get_latest_odom()
    print(f"📍 Start Locked: x={x_start:.3f}, y={y_start:.3f}")

    # 2. Driving Phase
    print("\n[STEP 2] DRIVE the car now.")
    input("Press ENTER once you have reached the STOP line...")
    x_end, y_end = get_latest_odom()
    print(f"📍 End Locked:   x={x_end:.3f}, y={y_end:.3f}")

    # 3. Calculation
    measured_dist = calculate_dist(x_start, y_start, x_end, y_end)

    print("\n" + "─"*45)
    print(f"  CALCULATED DISTANCE: {measured_dist:.4f} meters")
    print(f"  CALCULATED DISTANCE: {measured_dist * 100:.2f} cm")
    print("─"*45)

    # 4. Optional Accuracy Check
    real_dist_str = input("\nEnter actual distance traveled in meters (or skip): ")
    if real_dist_str:
        real_dist = float(real_dist_str)
        error = abs(real_dist - measured_dist)
        accuracy = (1 - (error / real_dist)) * 100 if real_dist > 0 else 0
        print(f"\n📈 Accuracy: {accuracy:.2f}%")
        print(f"📏 Error: {error*100:.2f} cm")

except KeyboardInterrupt:
    print("\nExiting...")
    sys.exit()
except Exception as e:
    print(f"\n❌ Error: {e}")
    print("Is your imu_odom.py broadcasting on port 5556?")

