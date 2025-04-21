import RPi.GPIO as GPIO
import time
import threading

# Define GPIO pins for motor 1
Dir_pin1 = 20
Step_pin1 = 21

# Define GPIO pins for motor 2
Step_pin2 = 17
Dir_pin2 = 27

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(Dir_pin1, GPIO.OUT)
GPIO.setup(Step_pin1, GPIO.OUT)
GPIO.setup(Dir_pin2, GPIO.OUT)
GPIO.setup(Step_pin2, GPIO.OUT)

# Shared variables
speed_lock = threading.Lock()
left_motor_speed_rpm = 0  # Motor speed in RPM
right_motor_speed_rpm = 0  # Motor speed in RPM
running = True

def run_motor_continuously():
    global left_motor_speed_rpm, right_motor_speed_rpm, running

    while running:
        with speed_lock:
            left_speed = left_motor_speed_rpm
            right_speed = right_motor_speed_rpm
        # Prevents CPU from running at 100%
        if left_speed == 0 and right_speed == 0:
            time.sleep(0.1)
            continue

        # Calculate delay between steps
        steps_per_rev = 200  # Usually 200 steps per rev (adjust if different)
        left_delay = 60.0 / (steps_per_rev * left_speed * 2)  # Half-period for pulses
        right_delay = 60.0 / (steps_per_rev * right_speed * 2)  

        GPIO.output(Step_pin1, GPIO.HIGH)
        time.sleep(left_delay)
        GPIO.output(Step_pin1, GPIO.LOW)
        time.sleep(left_delay)

        GPIO.output(Step_pin2, GPIO.HIGH)
        time.sleep(right_delay)
        GPIO.output(Step_pin2, GPIO.LOW)
        time.sleep(right_delay)

# Set direction: true = backwards, false = forwards
GPIO.output(Dir_pin1, False)
GPIO.output(Dir_pin2, True)

def set_motor_speed(left_speed_rpm, right_speed_rpm):
    global left_motor_speed_rpm, right_motor_speed_rpm
    #TODO: Check CW and CCW for both motors
    # Determine direction of rotation
    if left_speed_rpm > 0:
        direction = 'CW'
    else:
        direction = 'CCW'
        left_speed_rpm = -left_speed_rpm
    if right_speed_rpm > 0:
        direction = 'CW'
    else:
        direction = 'CCW'
        right_speed_rpm = -right_speed_rpm

    with speed_lock:
        left_motor_speed_rpm = left_speed_rpm
        right_motor_speed_rpm = right_speed_rpm
        GPIO.output(Dir_pin1, GPIO.HIGH if direction == 'CW' else GPIO.LOW)
        GPIO.output(Dir_pin2, GPIO.HIGH if direction == 'CW' else GPIO.LOW)

try:
    motor_thread = threading.Thread(target=run_motor_continuously, daemon=True)
    motor_thread.start()
finally:
    running = False
    motor_thread.join()
    GPIO.cleanup()
    print("Cleaned up GPIO.")







GPIO.cleanup()
