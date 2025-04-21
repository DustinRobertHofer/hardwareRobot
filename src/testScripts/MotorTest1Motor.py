import RPi.GPIO as GPIO
import time
import threading

# Define GPIO pins for motor 1
Dir_pin1 = 20
Step_pin1 = 21


# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(Dir_pin1, GPIO.OUT)
GPIO.setup(Step_pin1, GPIO.OUT)


# Shared variables
speed_lock = threading.Lock()
motor_speed_rpm = 0  # Motor speed in RPM

running = True

def run_motor_continuously():
    global motor_speed_rpm, running

    while running:
        with speed_lock:
            speed = motor_speed_rpm
  
        # Prevents CPU from running at 100%
        if speed == 0:
            time.sleep(0.1)
            continue

        # Calculate delay between steps
        steps_per_rev = 200  # Usually 200 steps per rev (adjust if different)
        delay = 60.0 / (steps_per_rev * speed * 2)  # Half-period for pulses

        GPIO.output(Step_pin1, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(Step_pin1, GPIO.LOW)
        time.sleep(delay)


# Set direction: true = backwards, false = forwards
GPIO.output(Dir_pin1, False)

def set_motor_speed(speed_rpm):
    global motor_speed_rpm
    #TODO: Check CW and CCW for both motors
    # Determine direction of rotation
    if speed_rpm > 0:
        direction = 'CW'
    else:
        direction = 'CCW'
        speed_rpm = -speed_rpm

    with speed_lock:
        motor_speed_rpm = speed_rpm
        GPIO.output(Dir_pin1, GPIO.HIGH if direction == 'CW' else GPIO.LOW)

if __name__ == "__main__":
    try:
        # Set initial direction
        GPIO.output(Dir_pin1, False)
        
        # Start motor control thread
        motor_thread = threading.Thread(target=run_motor_continuously, daemon=True)
        motor_thread.start()
        
        print("Motor test started. Enter speed in RPM (negative for reverse, 0 to stop, Ctrl+C to exit):")
        
        # Main control loop
        while True:
            try:
                speed = float(input("Speed (RPM): "))
                set_motor_speed(speed)
                if speed == 0:
                    print("Motor stopped")
                else:
                    print(f"Motor running at {abs(speed)} RPM {'reverse' if speed < 0 else 'forward'}")
            except ValueError:
                print("Please enter a valid number")
                
    except KeyboardInterrupt:
        print("\nExiting program...")
    finally:
        # Clean up
        running = False
        time.sleep(0.5)  # Give the thread time to exit
        GPIO.cleanup()
        print("Cleaned up GPIO.")







GPIO.cleanup()
