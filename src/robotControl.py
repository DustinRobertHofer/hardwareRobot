import time
import signal
from utils.state import State  # Reuse existing state class
from utils.navigator import Navigator  # Reuse existing navigator class
from utils.motionController import MotionController
from utils.sensorManager import SensorManager
from utils.pathPlanner import generate_cleaning_path, estimate_cleaning_time
from robotConfig import DEFAULT_CLEANING_AREA, SYSTEM_PARAMS, NAVIGATION_PARAMS, SENSOR_MODE

class RobotController:
    """Main robot controller for Raspberry Pi hardware implementation."""
    
    def __init__(self):
        """Initialize the robot controller with all subsystems."""
        print("Initializing Raspberry Pi Robot Controller...")
        
        # Initialize core components
        self.sensor_manager = SensorManager(SENSOR_MODE)
        self.state = State()  # Pass None for robot and 0 for timestep (not used in hardware)
        self.motion_controller = MotionController()
        self.navigator = Navigator()  # Pass None for robot and 0 for timestep (not used in hardware)
        
        # Set up motor references for state
        left_motor, right_motor = self.motion_controller.get_motors()
        #self.state.set_motors(left_motor, right_motor)
        
        # Set the cleaning area and navigation parameters
        self.navigator.waypoint_threshold = NAVIGATION_PARAMS['waypoint_threshold']
        self.navigator.heading_threshold = NAVIGATION_PARAMS['heading_threshold']
        self.navigator.TURN_THRESHOLD = NAVIGATION_PARAMS['turn_threshold']
        
        # Initialize state variables
        self.cleaning_path = None
        self.boundary_points = DEFAULT_CLEANING_AREA
        self.is_running = False
        self.last_update_time = 0
        self.control_loop_interval = SYSTEM_PARAMS['control_loop_interval']
        
        # Register signal handlers for clean shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        print("Robot controller initialization complete")
    
    def _signal_handler(self):
        """Handle system signals for clean shutdown."""
        print("\nShutting down robot...")
        self.cleanup()
        print("Goodbye!")
        exit(0)
    
    def setup(self):
        """Perform necessary setup operations before starting cleaning."""
        print(f"Setting up robot with boundary: {self.boundary_points}")
        
        # Generate cleaning path
        self.cleaning_path = generate_cleaning_path(self.boundary_points)
        self.navigator.set_path(self.cleaning_path)
        
        # Estimate cleaning time
        estimated_time = estimate_cleaning_time(self.cleaning_path)
        print(f"Generated cleaning path with {len(self.cleaning_path)} waypoints")
        print(f"Estimated cleaning time: {int(estimated_time/60)} minutes {int(estimated_time%60)} seconds")
        
        # Calibrate compass if needed
        print("Would you like to calibrate the digital compass? (y/n)")
        response = input().lower()
        if response == 'y':
            self.sensor_manager.calibrate_compass()
    
    def _check_obstacles(self, sensor_data):
        """Check for obstacles using sensor data and handle if needed."""
        if 'distance' not in sensor_data and SENSOR_MODE['Three_lasers']:
            print("Warning: No laser range finder data available")
            return False
            
        # Stop if obstacle is too close
        if sensor_data['distance_forward'] < NAVIGATION_PARAMS['safe_distance'] or sensor_data['distance_left'] < NAVIGATION_PARAMS['safe_side_distance'] or sensor_data['distance_right'] < NAVIGATION_PARAMS['safe_side_distance']:
            print(f"OBSTACLE DETECTED at {sensor_data['distance_forward']:.2f}m! Stopping robot.")
            self.motion_controller.stop()
            return True
        
        return False
    
    def step(self):
        """Main control loop - process one iteration of robot control."""
        # Limit update rate to avoid excessive CPU usage
        current_time = time.time()
        if current_time - self.last_update_time < self.control_loop_interval:
            time.sleep(0.001)  # Small sleep to avoid CPU hogging
            return False
        
        self.last_update_time = current_time
        
        # Get motor objects for encoder data
        left_motor, right_motor = self.motion_controller.get_motors()
        
        # Update sensor readings
        self.sensor_manager.set_wheel_data(left_motor, right_motor)
        self.sensor_manager.update()
        sensor_data = self.sensor_manager.get_sensor_data()
        
        # Update state - State will get distance data directly from motors
        self.state.update(sensor_data)
        current_state = self.state.get_position()
        
        # Check for obstacles
        if self._check_obstacles(sensor_data):
            return False
        
        # Get and execute navigation commands
        nav_command = self.navigator.get_next_command(current_state)
        self.motion_controller.execute_command(nav_command)
        
        # Display current state periodically (every second)
        if int(current_time) != int(self.last_update_time):
            print(f"Position: x={current_state['x']:.2f}m, y={current_state['y']:.2f}m, heading={current_state['theta']*180/3.14159:.1f}°")
        
        # Check if cleaning is complete
        if nav_command['type'] == 'stop':
            print("Path completed. Cleaning operation finished!")
            return True
        
        return False
    
    def start(self):
        """Start the cleaning operation."""
        if self.is_running:
            print("Robot is already running")
            return
        
        self.is_running = True
        print("Starting cleaning operation...")
        
        try:
            # Main control loop
            while self.is_running:
                if self.step():
                    self.is_running = False
                    print("Cleaning operation completed successfully")
                    break
        except KeyboardInterrupt:
            print("\nOperation interrupted by user")
            self.is_running = False
        except Exception as e:
            print(f"Error during operation: {e}")
            self.is_running = False
        finally:
            self.cleanup()
    
    def stop(self):
        """Stop the cleaning operation."""
        self.is_running = False
        self.motion_controller.stop()
        print("Cleaning operation stopped")
    
    def cleanup(self):
        """Perform any necessary cleanup operations."""
        print("Cleaning up resources...")
        self.motion_controller.stop()
        self.motion_controller.cleanup()
        self.sensor_manager.cleanup()

        print("Cleanup complete")
        
    def set_boundary(self, boundary_points):
        """Set a new boundary for cleaning."""
        self.boundary_points = boundary_points
        print(f"New boundary set with {len(boundary_points)} points")


def main():
    """Main function to start the robot."""
    print("Pressure Washing Robot - Raspberry Pi Version")
    print("--------------------------------------------")
    
    # Create and setup controller
    controller = RobotController()
    controller.setup()
    
    # Wait for user to start
    print("\nPress Enter to start cleaning, or type 'q' to quit")
    user_input = input()
    
    if user_input.lower() != 'q':
        # Start the robot
        controller.start()
    else:
        controller.cleanup()
        print("Exiting without starting. Goodbye!")


if __name__ == "__main__":
    main() 
