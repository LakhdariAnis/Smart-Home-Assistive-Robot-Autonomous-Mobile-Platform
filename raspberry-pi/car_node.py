import cv2
import zmq
import threading
import time
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import libcamera

# ── GPIO ─────────────────────────────────────────────────────
ENA = 12;  IN1 = 17; IN2 = 27
ENB = 13;  IN3 = 22; IN4 = 23
ENA2 = 18; IN5 = 24; IN6 = 25
ENB2 = 19; IN7 = 5;  IN8 = 6

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
all_pins = [ENA,IN1,IN2,IN3,IN4,ENB,ENA2,IN5,IN6,IN7,IN8,ENB2]
GPIO.setup(all_pins, GPIO.OUT, initial=GPIO.LOW)

pwm_fr = GPIO.PWM(ENA,  1000); pwm_fr.start(0)
pwm_fl = GPIO.PWM(ENB,  1000); pwm_fl.start(0)
pwm_rl = GPIO.PWM(ENA2, 1000); pwm_rl.start(0)
pwm_rr = GPIO.PWM(ENB2, 1000); pwm_rr.start(0)

def set_motors(fr_f, fr_b, fl_f, fl_b, rl_f, rl_b, rr_f, rr_b,
               spd_fr=100, spd_fl=100, spd_rl=100, spd_rr=100):
    pwm_fr.ChangeDutyCycle(spd_fr); pwm_fl.ChangeDutyCycle(spd_fl)
    pwm_rl.ChangeDutyCycle(spd_rl); pwm_rr.ChangeDutyCycle(spd_rr)
    GPIO.output(IN1, fr_f); GPIO.output(IN2, fr_b)
    GPIO.output(IN3, fl_f); GPIO.output(IN4, fl_b)
    GPIO.output(IN5, rl_f); GPIO.output(IN6, rl_b)
    GPIO.output(IN7, rr_f); GPIO.output(IN8, rr_b)

def apply(cmd):
    if cmd == b"FORWARD":
        set_motors(0,1, 0,1, 0,1, 0,1)
    elif cmd == b"BACKWARD":
        set_motors(1,0, 1,0, 1,0, 1,0)
    elif cmd == b"LEFT":
        set_motors(0,1, 1,0, 1,0, 0,1)
    elif cmd == b"RIGHT":
        set_motors(1,0, 0,1, 0,1, 1,0)
    elif cmd == b"STOP":
        set_motors(0,0, 0,0, 0,0, 0,0,
                   spd_fr=0, spd_fl=0, spd_rl=0, spd_rr=0)

# ── ZMQ ──────────────────────────────────────────────────────
context = zmq.Context()

# Camera stream out
pub = context.socket(zmq.PUB)
pub.bind("tcp://*:5555")

# Command listener in
cmd_sock = context.socket(zmq.REP)
cmd_sock.bind("tcp://*:5556")

# ── Camera ───────────────────────────────────────────────────
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "BGR888"},
    raw={"size": (1640, 1232)},
    transform=libcamera.Transform(hflip=1, vflip=1)
)
picam2.configure(config)
picam2.start()

# ── Command thread ────────────────────────────────────────────
def command_thread():
    print("Command listener ready on port 5556")
    while True:
        try:
            cmd = cmd_sock.recv()
            print(f"CMD: {cmd}")
            apply(cmd)
            if cmd != b"STOP":
                time.sleep(0.18)
                apply(b"STOP")
            cmd_sock.send(b"OK")
        except Exception as e:
            print(f"CMD error: {e}")
            cmd_sock.send(b"ERR")

threading.Thread(target=command_thread, daemon=True).start()

# ── Camera stream loop ────────────────────────────────────────
print("Streaming on port 5555 — Ctrl+C to stop")
try:
    while True:
        frame = picam2.capture_array()
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        pub.send(buf.tobytes())
except KeyboardInterrupt:
    print("Stopped.")
finally:
    apply(b"STOP")
    GPIO.cleanup()
    picam2.stop()
    pub.close()
    cmd_sock.close()
    context.term()

