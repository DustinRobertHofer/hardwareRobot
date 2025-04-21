from math import cos, sin, pi



class State:
    def __init__(self):
        """Initialize state tracking system"""
        self.x = 0.0  # X position in meters
        self.y = 0.0  # Y position in meters
        self.theta = 0.0  # Heading in radians
        
        # Parameters for turn detection
        self.prev_theta = 0.0
        self.TURN_THRESHOLD = 0.01  # About 0.57 degrees in radians
        

        # Default obstacle data
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')

    def update(self, sensor_data):
        """Update state estimation using sensor data"""
        # Get sensor readings
        wheel_data = sensor_data['wheel_data']
        compass_heading = sensor_data['compass_heading']
        
        # Update heading
        self.prev_theta = self.theta
        self.theta = self.normalize_angle(compass_heading)
        
        # Check if turning
        is_turning = abs(self.theta - self.prev_theta) > self.TURN_THRESHOLD
        
        # Get distance data directly from motors if available
        forward_motion = 0
        if wheel_data['delta_left'] and wheel_data['delta_right']:
            # Use wheel_data deltas that were already captured in SensorManager
            # This prevents calling get_distance() multiple times which resets internal tracking
            delta_left = wheel_data['delta_left']
            delta_right = wheel_data['delta_right']
            forward_motion = (delta_left + delta_right) / 2
            print(f"Forward motion: {forward_motion}")
        else:
            # Fall back to sensor data if motors not set
            print("No motors set")
            forward_motion = wheel_data['forward_motion']
            
        # Only update position if not turning
        if not is_turning and forward_motion != 0:
            self.x += forward_motion * cos(self.theta)
            self.y += forward_motion * sin(self.theta)
            
        # Update obstacle information
        if 'distance_forward' in sensor_data:
            self.obstacle_distance = sensor_data['distance_forward']
            self.obstacle_detected = self.obstacle_distance < 0.5  # Consider obstacles closer than 0.5m
        else:
            self.obstacle_detected = False
            self.obstacle_distance = float('inf')
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        if angle > pi:
            angle -= 2 * pi
        elif angle < -pi:
            angle += 2 * pi
        return angle
            
    def get_position(self):
        """Return current estimated position and heading"""
        return {
            'x': self.x,
            'y': self.y,
            'theta': self.theta,
            'obstacle_detected': self.obstacle_detected,
            'obstacle_distance': self.obstacle_distance
        }
        
    def reset(self):
        """Reset position estimation to origin"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_theta = 0.0
