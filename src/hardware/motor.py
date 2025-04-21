import RPi.GPIO as GPIO
import time
import threading
from robotConfig import MOTOR_PINS, MOTION_PARAMS        

class Motors:
    def __init__(self):
        # Define GPIO pins for motor 1
        self.Dir_pin1 = MOTOR_PINS['left_motor']['dir_pin']
        self.Step_pin1 = MOTOR_PINS['left_motor']['step_pin']

        # Define GPIO pins for motor 2  
        self.Dir_pin2 = MOTOR_PINS['right_motor']['dir_pin']
        self.Step_pin2 = MOTOR_PINS['right_motor']['step_pin']

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.Dir_pin1, GPIO.OUT)
        GPIO.setup(self.Step_pin1, GPIO.OUT)
        GPIO.setup(self.Dir_pin2, GPIO.OUT)
        GPIO.setup(self.Step_pin2, GPIO.OUT)

        # Shared variables
        self.speed_lock1 = threading.Lock()
        self.speed_lock2 = threading.Lock()
        self.motor1_speed_rpm = 0  # Motor 1 speed in RPM
        self.motor2_speed_rpm = 0  # Motor 2 speed in RPM
        self.motor1_steps = 0
        self.motor2_steps = 0
        self.direction1 = 'CW'
        self.direction2 = 'CW'
        self.log_counter1 = 0  # Counter for reducing log frequency
        self.log_counter2 = 0  # Counter for reducing log frequency
       
        self.running = True
        self.motor1_thread = None
        self.motor2_thread = None

    def start(self):
        """Start the motor control threads"""
        self.running = True
        self.motor1_thread = threading.Thread(target=self.run_motor1_continuously, daemon=True)
        self.motor2_thread = threading.Thread(target=self.run_motor2_continuously, daemon=True)
        self.motor1_thread.start()
        self.motor2_thread.start()
        
    def cleanup(self):
        """Stop threads and cleanup GPIO"""
        self.running = False
        if self.motor1_thread and self.motor1_thread.is_alive():
            self.motor1_thread.join(timeout=1.0)
        if self.motor2_thread and self.motor2_thread.is_alive():
            self.motor2_thread.join(timeout=1.0)
        GPIO.cleanup()

    def set_speed(self, motor_name, speed_rpm):
        if motor_name == 'left_motor':
            self.set_motor1_speed(speed_rpm)
        elif motor_name == 'right_motor':
            self.set_motor2_speed(speed_rpm)
            
    def get_rotations(self, motor_name):
        if motor_name == 'left_motor':
            return self.motor1_steps / MOTION_PARAMS['steps_per_revolution']
        elif motor_name == 'right_motor':
            return self.motor2_steps / MOTION_PARAMS['steps_per_revolution']


    def run_motor1_continuously(self):
        while self.running:
            with self.speed_lock1:
                speed = self.motor1_speed_rpm
    
            # Prevents CPU from running at 100%
            if speed == 0:
                time.sleep(0.1)
                continue

            # Calculate delay between steps
            steps_per_rev = MOTION_PARAMS['steps_per_revolution']  # Usually 200 steps per rev (adjust if different)
            delay = 60.0 / (steps_per_rev * speed * 2)  # Half-period for pulses

            GPIO.output(self.Step_pin1, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.Step_pin1, GPIO.LOW)
            time.sleep(delay)

            # step logging
            if self.direction1 == 'CCW':
                self.motor1_steps += 1
            else:
                self.motor1_steps -= 1
                
            # Print steps less frequently (every 100 steps)
            self.log_counter1 += 1
            if self.log_counter1 >= 100:
                #print(f"Motor 1 steps: {self.motor1_steps}")
                self.log_counter1 = 0

    def run_motor2_continuously(self):
        while self.running:
            with self.speed_lock2:
                speed = self.motor2_speed_rpm
    
            # Prevents CPU from running at 100%
            if speed == 0:
                time.sleep(0.1)
                continue

            # Calculate delay between steps
            steps_per_rev = MOTION_PARAMS['steps_per_revolution']  # Usually 200 steps per rev (adjust if different)
            delay = 60.0 / (steps_per_rev * speed * 2)  # Half-period for pulses

            GPIO.output(self.Step_pin2, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.Step_pin2, GPIO.LOW)
            time.sleep(delay)

            # step logging
            if self.direction2 == 'CW':
                self.motor2_steps += 1
            else:
                self.motor2_steps -= 1
                
            # Print steps less frequently (every 100 steps)
            self.log_counter2 += 1
            if self.log_counter2 >= 100:
                #print(f"Motor 2 steps: {self.motor2_steps}")
                self.log_counter2 = 0


    def set_motor1_speed(self, speed_rpm):
        # Determine direction of rotation
        if speed_rpm > 0:
            self.direction1 = 'CCW'
        else:
            self.direction1 = 'CW'
            speed_rpm = -speed_rpm

        with self.speed_lock1:
            self.motor1_speed_rpm = speed_rpm
            GPIO.output(self.Dir_pin1, GPIO.HIGH if self.direction1 == 'CW' else GPIO.LOW)

    def set_motor2_speed(self, speed_rpm):
        # Determine direction of rotation
        if speed_rpm > 0:
            self.direction2 = 'CW'
        else:
            self.direction2 = 'CCW'
            speed_rpm = -speed_rpm

        with self.speed_lock2:
            self.motor2_speed_rpm = speed_rpm
            GPIO.output(self.Dir_pin2, GPIO.HIGH if self.direction2 == 'CW' else GPIO.LOW)

# Singleton instance of Motors
_motors_instance = None

class Motor:
    """
    Single motor interface to provide compatibility with motionController.
    This class wraps the Motors class for individual motor control.
    """
    def __init__(self, motor_name):
        """
        Initialize a motor.
        
        Args:
            motor_name (str): 'left_motor' or 'right_motor'
        """
        global _motors_instance
        
        # Create shared Motors instance if it doesn't exist
        if _motors_instance is None:
            _motors_instance = Motors()
            _motors_instance.start()
            
        self.motors = _motors_instance
        self.motor_name = motor_name
        self.rpm_to_rad_per_sec = 2 * 3.14159 / 60  # Conversion factor
        
        # Track previous rotation value to calculate deltas
        self.previous_rotation = self.motors.get_rotations(self.motor_name) 
           
    def set_velocity(self, velocity_rad_per_sec):
        """
        Set motor velocity in radians per second.
        
        Args:
            velocity_rad_per_sec (float): Velocity in radians per second
        """
        # Convert rad/s to RPM
        rpm = velocity_rad_per_sec / self.rpm_to_rad_per_sec
        self.motors.set_speed(self.motor_name, rpm)
        
    def stop(self):
        """Stop the motor."""
        self.motors.set_speed(self.motor_name, 0)
        
    def get_distance(self):
        """Get distance traveled since last call (in rotations)."""
        current_rotation = self.motors.get_rotations(self.motor_name)
        delta_rotation = current_rotation - self.previous_rotation
        self.previous_rotation = current_rotation
        #convert to meters
        distance = delta_rotation * MOTION_PARAMS['wheel_radius']
        return distance
        
    def cleanup(self):
        """Clean up resources."""
        # This does nothing for individual motors as cleanup is handled by the shared Motors instance
        pass





