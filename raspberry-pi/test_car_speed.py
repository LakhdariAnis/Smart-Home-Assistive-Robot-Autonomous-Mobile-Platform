import RPi.GPIO as GPIO
import time

# ── Pin definitions (same as car.py) ─────────────────────────
ENA = 12; IN1 = 22; IN2 = 23    # FR
ENB = 13; IN3 = 17; IN4 = 27    # FL
ENA2 = 18; IN5 = 5;  IN6 = 6   # RL
ENB2 = 19; IN7 = 24; IN8 = 25  # RR

# ── Config ────────────────────────────────────────────────────
RUN_DURATION = 3        # seconds per run (change as needed)
NUM_RUNS     = 5        # number of test runs
DUTY_CYCLE   = 100      # full speed (same as car.py default)
PWM_FREQ     = 1000     # Hz

# ── GPIO Setup ────────────────────────────────────────────────
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
all_pins = [ENA, IN1, IN2, IN3, IN4, ENB, ENA2, IN5, IN6, IN7, IN8, ENB2]
GPIO.setup(all_pins, GPIO.OUT, initial=GPIO.LOW)

pwm_fr = GPIO.PWM(ENA,  PWM_FREQ); pwm_fr.start(0)
pwm_fl = GPIO.PWM(ENB,  PWM_FREQ); pwm_fl.start(0)
pwm_rl = GPIO.PWM(ENA2, PWM_FREQ); pwm_rl.start(0)
pwm_rr = GPIO.PWM(ENB2, PWM_FREQ); pwm_rr.start(0)

def forward(duty=DUTY_CYCLE):
    pwm_fr.ChangeDutyCycle(duty)
    pwm_fl.ChangeDutyCycle(duty)
    pwm_rl.ChangeDutyCycle(duty)
    pwm_rr.ChangeDutyCycle(duty)
    GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN4, GPIO.LOW)
    GPIO.output(IN5, GPIO.HIGH); GPIO.output(IN6, GPIO.LOW)
    GPIO.output(IN7, GPIO.HIGH); GPIO.output(IN8, GPIO.LOW)

def stop():
    pwm_fr.ChangeDutyCycle(0)
    pwm_fl.ChangeDutyCycle(0)
    pwm_rl.ChangeDutyCycle(0)
    pwm_rr.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW); GPIO.output(IN4, GPIO.LOW)
    GPIO.output(IN5, GPIO.LOW); GPIO.output(IN6, GPIO.LOW)
    GPIO.output(IN7, GPIO.LOW); GPIO.output(IN8, GPIO.LOW)

def cleanup():
    stop()
    for pwm in [pwm_fr, pwm_fl, pwm_rl, pwm_rr]:
        pwm.stop()
    GPIO.cleanup()

# ── Test ──────────────────────────────────────────────────────
print("=" * 45)
print("   Car Speed Test")
print(f"   Duty cycle : {DUTY_CYCLE}%  |  Duration: {RUN_DURATION}s")
print(f"   Runs       : {NUM_RUNS}")
print("=" * 45)

speeds = []

try:
    for run in range(1, NUM_RUNS + 1):
        input(f"\nRun {run}/{NUM_RUNS} — Press ENTER to start...")
        print(f"  ▶ Running for {RUN_DURATION}s...")
        forward()
        time.sleep(RUN_DURATION)
        stop()
        print("  ■ Stopped.")

        while True:
            try:
                dist = float(input("  Enter distance travelled (metres): "))
                if dist < 0:
                    print("  Distance must be non-negative. Try again.")
                    continue
                break
            except ValueError:
                print("  Invalid input. Enter a number.")

        speed = dist / RUN_DURATION
        speeds.append(speed)
        print(f"  → Run {run} speed: {speed:.4f} m/s  ({speed*100:.2f} cm/s)")

    print("\n" + "=" * 45)
    print("   Results")
    print("=" * 45)
    for i, s in enumerate(speeds, 1):
        print(f"  Run {i}: {s:.4f} m/s")
    avg = sum(speeds) / len(speeds)
    print(f"\n  Average speed : {avg:.4f} m/s")
    print(f"                  {avg*100:.2f} cm/s")
    print(f"                  {avg*3.6:.4f} km/h")
    print("=" * 45)

except KeyboardInterrupt:
    print("\nAborted by user.")

finally:
    cleanup()
    print("GPIO cleaned up.")

