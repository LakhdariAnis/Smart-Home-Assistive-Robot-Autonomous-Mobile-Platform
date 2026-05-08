#!/usr/bin/env python3
"""
slam_bridge.py — Task 5 fusion
Reads SLAM pose from ZMQ:5557 (slam_reader)
Reads IMU fallback from ZMQ:5556 (imu_odom.py on Pi)
Publishes unified pose on ZMQ:5558
"""

import zmq, json, time, threading, math

PI_IP    = "raspberrypi.local"   # change to Pi's IP if .local doesn't work
IMU_PORT  = 5556
SLAM_PORT = 5557
OUT_PORT  = 5558

# ── Shared state ─────────────────────────────────────────────────────────────
imu  = {"x": 0.0, "y": 0.0, "heading_deg": 0.0, "moving": False}
slam = {"x": 0.0, "y": 0.0, "z": 0.0, "ok": False, "seq": -1}
lock = threading.Lock()

# ── Thread: read IMU from Pi ──────────────────────────────────────────────────
def imu_thread():
    ctx  = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.setsockopt(zmq.RCVHWM, 2)
    sock.connect(f"tcp://{PI_IP}:{IMU_PORT}")
    print(f"[IMU ] Connected tcp://{PI_IP}:{IMU_PORT}")
    while True:
        try:
            d = json.loads(sock.recv_string())
            with lock:
                imu["x"]           = d["x"]
                imu["y"]           = d["y"]
                imu["heading_deg"] = d["heading_deg"]
                imu["moving"]      = d["moving"]
        except Exception as e:
            print(f"[IMU ] Error: {e}")

# ── Thread: read SLAM pose from slam_reader ───────────────────────────────────
def slam_thread():
    ctx  = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.setsockopt(zmq.RCVHWM, 2)
    sock.connect(f"tcp://localhost:{SLAM_PORT}")
    print(f"[SLAM] Connected tcp://localhost:{SLAM_PORT}")
    while True:
        try:
            d = json.loads(sock.recv_string())
            with lock:
                slam["ok"]  = d.get("ok", False)
                slam["seq"] = d.get("seq", -1)
                if slam["ok"]:
                    slam["x"] = d["x"]
                    slam["y"] = d["y"]
                    slam["z"] = d["z"]
        except Exception as e:
            print(f"[SLAM] Error: {e}")

# ── Main: fuse and publish ────────────────────────────────────────────────────
def main():
    threading.Thread(target=imu_thread,  daemon=True).start()
    threading.Thread(target=slam_thread, daemon=True).start()

    ctx  = zmq.Context()
    pub  = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://*:{OUT_PORT}")
    print(f"[OUT ] Publishing unified pose on port {OUT_PORT}")
    print()
    print(f"{'SOURCE':<8} {'x(m)':>8} {'y(m)':>8} {'hdg°':>8} {'moving':>7}")
    print("─" * 46)

    interval = 0.1   # 100ms

    while True:
        t0 = time.monotonic()

        with lock:
            slam_ok      = slam["ok"]
            slam_x       = slam["x"]
            slam_y       = slam["y"]
            imu_x        = imu["x"]
            imu_y        = imu["y"]
            imu_hdg      = imu["heading_deg"]
            imu_moving   = imu["moving"]

        if slam_ok:
            source = "SLAM"
            out_x  = slam_x
            out_y  = slam_y
            # SLAM doesn't give heading directly — use IMU heading always
            out_hdg = imu_hdg
        else:
            source = "IMU"
            out_x  = imu_x
            out_y  = imu_y
            out_hdg = imu_hdg

        msg = json.dumps({
            "source":      source,
            "x":           round(out_x,   4),
            "y":           round(out_y,   4),
            "heading_deg": round(out_hdg, 2),
            "moving":      imu_moving,
            "ts_ms":       int(time.time() * 1000),
        })
        pub.send_string(msg)

        print(
            f"[{source:<6}] {out_x:>+8.3f} {out_y:>+8.3f} "
            f"{out_hdg:>7.1f}° {'MOV' if imu_moving else 'STOP':>7}",
            end="\r"
        )

        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, interval - elapsed))

if __name__ == "__main__":
    main()
