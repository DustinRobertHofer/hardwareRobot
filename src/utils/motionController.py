from hardware.motor import Motor
from robotConfig import MOTION_PARAMS, ROBOT_PARAMS

class MotionController:
    """Motion controller for Raspberry Pi using gpiozero-controlled motors."""
    
    def __init__(self):
        """Initialize motion control system with motors."""
        # No need to initialize GPIO with gpiozero
        
        # Initialize motors
        self.left_motor = Motor('left_motor')
        self.right_motor = Motor('right_motor')
        
        # Load physical parameters from config
        self.wheel_radius = ROBOT_PARAMS['wheel_radius']
        self.wheel_base = ROBOT_PARAMS['wheel_distance']
        
        # Speed limits
        self.max_linear_speed = MOTION_PARAMS['max_linear_speed']
        self.max_angular_speed = MOTION_PARAMS['max_angular_speed']
        
        print("Motion controller initialized")
        
    def execute_command(self, command):
        """Execute a movement command based on its type."""
        if command is None:
            self.stop()
            print("No command to execute, stopping robot")
            return
            
        if command['type'] == 'move':
            self.set_velocity(command['linear_velocity'], command['angular_velocity'])
        elif command['type'] == 'stop':
            self.stop()
            
    def set_velocity(self, linear_velocity, angular_velocity):
        """Set robot linear and angular velocity."""
        # Apply velocity limits
        linear_velocity = self._limit_value(linear_velocity, -self.max_linear_speed, self.max_linear_speed)
        angular_velocity = self._limit_value(angular_velocity, -self.max_angular_speed, self.max_angular_speed)
            
        # Convert to wheel velocities using differential drive kinematics
        left_speed = (linear_velocity + angular_velocity * self.wheel_base / 2) / self.wheel_radius
        right_speed = (linear_velocity - angular_velocity * self.wheel_base / 2) / self.wheel_radius
        
        # Set motor velocities
        self.left_motor.set_velocity(left_speed)
        self.right_motor.set_velocity(right_speed)
        
    def stop(self):
        """Stop all robot motion by setting wheel velocities to zero."""
        self.left_motor.stop()
        self.right_motor.stop()
    
    def get_wheel_data(self):
        """Get wheel data from motors."""
        left_distance = self.left_motor.get_distance()
        right_distance = self.right_motor.get_distance()
        forward_distance = (left_distance + right_distance) / 2
        return {
            'left_distance': left_distance,
            'right_distance': right_distance,
            'forward_distance': forward_distance,
            'delta_left': left_distance,
            'delta_right': right_distance,
            'forward_motion': forward_distance
        }
    
    def _limit_value(self, value, min_value, max_value):
        """Limit a value to specified range."""
        return max(min_value, min(value, max_value))
    
    def get_distance(self):
        """Get distance traveled by the robot."""
        return self.left_motor.get_distance()
    
    def cleanup(self):
        """Clean up resources."""
        self.stop()
        # Import here to avoid circular imports
        from hardware.motor import _motors_instance
        if _motors_instance is not None:
            _motors_instance.cleanup() 
