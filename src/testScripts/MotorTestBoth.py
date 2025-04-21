import RPi.GPIO as GPIO
import time
import threading

# Define GPIO pins for motor 1
Dir_pin1 = 20
Step_pin1 = 21

# Define GPIO pins for motor 2  
Dir_pin2 = 27
Step_pin2 = 17

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(Dir_pin1, GPIO.OUT)
GPIO.setup(Step_pin1, GPIO.OUT)
GPIO.setup(Dir_pin2, GPIO.OUT)
GPIO.setup(Step_pin2, GPIO.OUT)

# Shared variables
speed_lock1 = threading.Lock()
speed_lock2 = threading.Lock()
motor1_speed_rpm = 0  # Motor 1 speed in RPM
motor2_speed_rpm = 0  # Motor 2 speed in RPM

running = True

def run_motor1_continuously():
    global motor1_speed_rpm, running

    while running:
        with speed_lock1:
            speed = motor1_speed_rpm
  
        # Prevents CPU from running at 100%
        if speed == 0:
            time.sleep(0.1)
            continue

        # Calculate delay between steps
        steps_per_rev = 200  # Usually 200 steps per rev (adjust if different)
        delay = 60.0 / (steps_per_rev * speed * 2)  # Half-period for pulses

        GPIO.output(Step_pin1, GPIO.HIGH)
        print("Stepping motor 1")
        time.sleep(delay)
        GPIO.output(Step_pin1, GPIO.LOW)
        time.sleep(delay)

def run_motor2_continuously():
    global motor2_speed_rpm, running

    while running:
        with speed_lock2:
            speed = motor2_speed_rpm
  
        # Prevents CPU from running at 100%
        if speed == 0:
            time.sleep(0.1)
            continue

        # Calculate delay between steps
        steps_per_rev = 200  # Usually 200 steps per rev (adjust if different)
        delay = 60.0 / (steps_per_rev * speed * 2)  # Half-period for pulses

        GPIO.output(Step_pin2, GPIO.HIGH)
        print("Stepping motor 2")
        time.sleep(delay)
        GPIO.output(Step_pin2, GPIO.LOW)
        time.sleep(delay)

# Set initial direction for both motors
GPIO.output(Dir_pin1, False)
GPIO.output(Dir_pin2, False)

def set_motor1_speed(speed_rpm):
    global motor1_speed_rpm
    # Determine direction of rotation
    if speed_rpm > 0:
        direction = 'CW'
    else:
        direction = 'CCW'
        speed_rpm = -speed_rpm

    with speed_lock1:
        motor1_speed_rpm = speed_rpm
        GPIO.output(Dir_pin1, GPIO.HIGH if direction == 'CW' else GPIO.LOW)
        print("Setting motor 1 direction to", direction)

def set_motor2_speed(speed_rpm):
    global motor2_speed_rpm
    # Determine direction of rotation
    if speed_rpm > 0:
        direction = 'CW'
    else:
        direction = 'CCW'
        speed_rpm = -speed_rpm

    with speed_lock2:
        motor2_speed_rpm = speed_rpm
        GPIO.output(Dir_pin2, GPIO.HIGH if direction == 'CW' else GPIO.LOW)
        print("Setting motor 2 direction to", direction)

if __name__ == "__main__":
    try:
        # Set initial direction
        GPIO.output(Dir_pin1, False)
        GPIO.output(Dir_pin2, False)
        
        # Start motor control threads
        motor1_thread = threading.Thread(target=run_motor1_continuously, daemon=True)
        motor1_thread.start()
        
        motor2_thread = threading.Thread(target=run_motor2_continuously, daemon=True)
        motor2_thread.start()
        
        print("Motor test started. Enter speed in RPM (negative for reverse, 0 to stop, Ctrl+C to exit):")
        
        # Main control loop
        while True:
            try:
                motor1_speed = float(input("Motor 1 Speed (RPM): "))
                
                
                motor2_speed = float(input("Motor 2 Speed (RPM): "))
                set_motor1_speed(motor1_speed)
                set_motor2_speed(motor2_speed)

                
                # direction1 = "reverse" if motor1_speed < 0 else "forward"
                # status1 = "stopped" if motor1_speed == 0 else f"running at {abs(motor1_speed)} RPM {direction1}"
                # print(f"Motor 1: {status1}")
                
                # direction2 = "reverse" if motor2_speed < 0 else "forward"
                # status2 = "stopped" if motor2_speed == 0 else f"running at {abs(motor2_speed)} RPM {direction2}"
                # print(f"Motor 2: {status2}")
                
            except ValueError:
                print("Please enter a valid number")
                
    except KeyboardInterrupt:
        print("\nExiting program...")
    finally:
        # Clean up
        running = False
        time.sleep(0.5)  # Give the threads time to exit
        GPIO.cleanup()
        print("Cleaned up GPIO.")
