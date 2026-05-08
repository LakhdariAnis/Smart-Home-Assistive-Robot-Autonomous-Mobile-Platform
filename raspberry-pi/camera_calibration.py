import cv2
import numpy as np
import zmq
import threading
import time

# ─── CONFIG ──────────────────────────────────────────────────────────────────
BOARD_SIZE      = (9, 6)
SQUARE_MM       = 24.0
ZMQ_ADDRESS     = "tcp://10.93.217.191:5555"
SAVE_PATH       = "camera_calib.npz"
DETECT_EVERY    = 8
DETECT_SCALE    = 1
TARGET_CAPTURES = 32
PRUNE_THRESHOLD = 0.3        # drop frames with per-image error above this
# ─────────────────────────────────────────────────────────────────────────────

objp = np.zeros((BOARD_SIZE[0] * BOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:BOARD_SIZE[0], 0:BOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_MM

obj_pts, img_pts = [], []

# ── ZMQ receive in background thread ─────────────────────────────────────────
latest_frame = None
frame_lock   = threading.Lock()

def recv_loop():
    global latest_frame
    ctx    = zmq.Context()
    socket = ctx.socket(zmq.SUB)
    socket.connect(ZMQ_ADDRESS)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    socket.setsockopt(zmq.RCVTIMEO, 3000)

    while not stop_event.is_set():
        try:
            buf = socket.recv()
            arr = np.frombuffer(buf, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is not None:
                with frame_lock:
                    latest_frame = img
        except zmq.Again:
            print("[WARN] No frame for 3 s — is the Pi still streaming?")
    socket.close()
    ctx.term()

stop_event = threading.Event()
t = threading.Thread(target=recv_loop, daemon=True)
t.start()

print("Waiting for first frame…")
while True:
    with frame_lock:
        if latest_frame is not None:
            break
    time.sleep(0.05)
print(f"Stream OK. Target: {TARGET_CAPTURES} captures.")
print("Controls:  SPACE = capture  |  Q = quit early\n")

count      = 0
frame_idx  = 0
found      = False
corners    = None

while True:
    with frame_lock:
        frame = latest_frame.copy() if latest_frame is not None else None

    if frame is None:
        time.sleep(0.01)
        continue

    H, W = frame.shape[:2]
    frame_idx += 1

    # ── detection ──────────────────────────────────────────────────────────
    if frame_idx % DETECT_EVERY == 0:
        small = cv2.resize(frame, (0, 0), fx=DETECT_SCALE, fy=DETECT_SCALE)
        gray  = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        flags = cv2.CALIB_CB_FAST_CHECK
        found, corners_small = cv2.findChessboardCorners(gray, BOARD_SIZE, flags)

        if found:
            corners = corners_small / DETECT_SCALE
        else:
            corners = None

    # ── draw minimalist UI ────────────────────────────────────────────────
    disp = frame.copy()

    if found and corners is not None:
        cv2.drawChessboardCorners(disp, BOARD_SIZE, corners, found)
        msg   = f"FOUND! Press SPACE | Captured: {count}/{TARGET_CAPTURES}"
        color = (0, 255, 0)
    else:
        msg   = f"Searching... | Captured: {count}/{TARGET_CAPTURES}"
        color = (0, 165, 255)

    cv2.putText(disp, msg, (10, H - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.imshow("Camera Calibration", disp)
    key = cv2.waitKey(1) & 0xFF

    # ── SPACE: capture ────────────────────────────────────────────────────
    if key == ord(' ') and found and corners is not None:
        gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        crit      = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2  = cv2.cornerSubPix(gray_full, corners, (11, 11), (-1, -1), crit)

        obj_pts.append(objp)
        img_pts.append(corners2)
        count += 1
        print(f"  ✓ Captured #{count:2d}")

        flash = disp.copy()
        cv2.rectangle(flash, (0, 0), (W, H), (0, 255, 0), 12)
        cv2.addWeighted(flash, 0.4, disp, 0.6, 0, disp)
        cv2.imshow("Camera Calibration", disp)
        cv2.waitKey(300)

        found   = False
        corners = None

        if count >= TARGET_CAPTURES:
            print("\nAll shots done!")
            break

    elif key == ord('q'):
        print("\nQuitting early…")
        break

# ── cleanup ───────────────────────────────────────────────────────────────────
stop_event.set()
cv2.destroyAllWindows()

# ── helpers ───────────────────────────────────────────────────────────────────
def compute_errors(obj_pts, img_pts, rvecs, tvecs, K, dist):
    errors = []
    for i in range(len(obj_pts)):
        projected, _ = cv2.projectPoints(obj_pts[i], rvecs[i], tvecs[i], K, dist)
        err = cv2.norm(img_pts[i], projected, cv2.NORM_L2) / len(projected)
        errors.append(err)
    return errors

def print_results(label, rms, K, dist, errors):
    print(f"\n{'='*52}")
    print(f"  [{label}]")
    print(f"  RMS re-projection error : {rms:.4f} px")
    print(f"  Target: < 0.5 px  (excellent < 0.3 | good < 0.5 | ok < 1.0)")
    print(f"{'='*52}")
    print(f"  Camera matrix K:\n{K}")
    print(f"  Distortion coeffs: {dist.ravel()}")
    print(f"  Per-image errors: min={min(errors):.3f}  max={max(errors):.3f}  mean={np.mean(errors):.3f}")
    bad = [(i, e) for i, e in enumerate(errors) if e > 1.0]
    if bad:
        print(f"  [!] {len(bad)} frame(s) with error > 1.0 px")

# ── calibrate ─────────────────────────────────────────────────────────────────
if count < 15:
    print(f"\n[!] Only {count} captures — need at least 15. Re-run.")
else:
    h, w = frame.shape[:2]
    print(f"\nRunning calibration on {count} frames…")

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_pts, img_pts, (w, h), None, None
    )
    errors = compute_errors(obj_pts, img_pts, rvecs, tvecs, K, dist)
    print_results("Initial calibration", rms, K, dist, errors)

    # ── prune bad frames and recalibrate ──────────────────────────────────
    good_obj = [obj_pts[i] for i, e in enumerate(errors) if e <= PRUNE_THRESHOLD]
    good_img = [img_pts[i] for i, e in enumerate(errors) if e <= PRUNE_THRESHOLD]
    dropped  = count - len(good_obj)

    print(f"\nPruning frames with error > {PRUNE_THRESHOLD} px …")
    print(f"  Kept {len(good_obj)}/{count} frames  ({dropped} dropped)")

    if len(good_obj) >= 15:
        rms2, K2, dist2, rvecs2, tvecs2 = cv2.calibrateCamera(
            good_obj, good_img, (w, h), None, None
        )
        errors2 = compute_errors(good_obj, good_img, rvecs2, tvecs2, K2, dist2)
        print_results("After pruning", rms2, K2, dist2, errors2)

        if rms2 < rms:
            print(f"\n✓ Pruning improved RMS: {rms:.4f} → {rms2:.4f} px")
            K_final, dist_final, rms_final = K2, dist2, rms2
        else:
            print(f"\n  Pruning did not improve RMS — keeping original.")
            K_final, dist_final, rms_final = K, dist, rms
    else:
        print(f"  [!] Not enough frames after pruning ({len(good_obj)}) — keeping original.")
        K_final, dist_final, rms_final = K, dist, rms

    # ── save ──────────────────────────────────────────────────────────────
    np.savez(SAVE_PATH, K=K_final, dist=dist_final, rms=rms_final, image_size=(w, h))
    print(f"\nSaved → {SAVE_PATH}  (RMS: {rms_final:.4f} px)")

    # ── undistorted preview ────────────────────────────────────────────────
    print("Undistorted preview (any key to close)…")
    new_K, roi = cv2.getOptimalNewCameraMatrix(K_final, dist_final, (w, h), 1, (w, h))
    undist     = cv2.undistort(frame, K_final, dist_final, None, new_K)
    side       = np.hstack([frame, undist])
    cv2.putText(side, "Original",    (20,   35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
    cv2.putText(side, "Undistorted", (w+20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255,   0), 2)
    cv2.imshow("Result — any key to close", side)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
