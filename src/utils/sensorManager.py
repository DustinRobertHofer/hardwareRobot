import time
from hardware.laserRange import LaserRangeFinder
from hardware.compass import Compass

from robotConfig import SENSOR_PINS
class SensorManager:
    """Sensor manager for Raspberry Pi hardware implementation."""
    
    def __init__(self, sensor_mode):
        """Initialize all sensors for Raspberry Pi."""
        # Initialize sensors
        if sensor_mode['Three_lasers']:
            self.laser_range_forward = LaserRangeFinder(SENSOR_PINS['laser_range_forward'])
            self.laser_range_left = LaserRangeFinder(SENSOR_PINS['laser_range_left'])
            self.laser_range_right = LaserRangeFinder(SENSOR_PINS['laser_range_right'])
        elif sensor_mode['One_laser']:
            self.laser_range_forward = LaserRangeFinder(SENSOR_PINS['laser_range_forward'])
        elif sensor_mode['No_lasers']:
            self.laser_range_forward = None
            self.laser_range_left = None
            self.laser_range_right = None
        self.compass = Compass(SENSOR_PINS['compass'])
        
        # Store latest sensor data
        self.sensor_data = {}
        self.last_update_time = 0
        self.update_interval = 0.05  # 50ms update interval
        
        # Track wheel data (this would be obtained from the motors)
        self.wheel_data = {
            'left_distance': 0,
            'right_distance': 0,
            'delta_left': 0,
            'delta_right': 0,
            'forward_motion': 0
        }
        
        # Perform initial sensor update
        self.update()
        
    def update(self):
        """Update all sensor readings."""
        current_time = time.time()
        
        # Only update at specified interval
        if current_time - self.last_update_time < self.update_interval:
            return
            
        self.last_update_time = current_time
        
        # Get laser range finder reading
        if self.laser_range_forward:
            distance_forward = self.laser_range_forward.get_distance()
        else:
            distance_forward = None
            
        if self.laser_range_left:
            distance_left = self.laser_range_left.get_distance()
        else:
            distance_left = None
        if self.laser_range_right:
            distance_right = self.laser_range_right.get_distance()
        else:
            distance_right = None
        
        # Get compass heading
        compass_heading = self.compass.get_bearing()
        
        # Combine all sensor data
        self.sensor_data = {
            'wheel_data': self.wheel_data,
            'compass_heading': compass_heading
        }
        
        # Only include distance data if sensor is available
        if distance_forward is not None:
            self.sensor_data['distance_forward'] = distance_forward
        else:
            self.sensor_data['distance_forward'] = 10.0
        if distance_left is not None:
            self.sensor_data['distance_left'] = distance_left
        else:
            self.sensor_data['distance_left'] = 10.0
        if distance_right is not None:
            self.sensor_data['distance_right'] = distance_right
        else:
            self.sensor_data['distance_right'] = 10.0
        

    def set_wheel_data(self, left_motor, right_motor):
        """Update wheel data from motor ."""
        # Get delta distances from motor 
        delta_left = left_motor.get_delta_distance()
        delta_right = right_motor.get_delta_distance()
        
        # Calculate forward motion (average of wheels)
        forward_motion = (delta_left + delta_right) / 2
        
        # Update wheel data
        self.wheel_data = {
            'left_distance': left_motor.get_distance(),
            'right_distance': right_motor.get_distance(),
            'delta_left': delta_left,
            'delta_right': delta_right,
            'forward_motion': forward_motion
        }
    
    def get_sensor_data(self):
        """Return the latest sensor readings."""
        return self.sensor_data
    
    def calibrate_compass(self):
        """Calibrate the digital compass."""
        self.compass.calibrate()
    
    def cleanup(self):
        """Clean up all sensor resources."""
        self.laser_range_forward.cleanup()
        self.laser_range_left.cleanup()
        self.laser_range_right.cleanup()
        self.compass.cleanup() 