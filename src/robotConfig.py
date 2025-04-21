# Raspberry Pi Hardware Configuration for Robot Control

# Sensor Mode
SENSOR_MODE = {
    'Three_lasers': False,
    'One_laser': True,
    'No_lasers': False
}

# Motor pins configuration
MOTOR_PINS = {
    'left_motor': {
        'dir_pin': 17,     # Direction pin for left motor
        'step_pin': 27,     # Step pin for left motor
    },
    'right_motor': {
        'dir_pin': 20,     # Direction pin for right motor
        'step_pin': 21,     # Step pin for right motor
    }
}

# Sensor pins configuration
SENSOR_PINS = {
    'laser_range_forward': {
        'rx_pin': None,     # Serial RX pin for forward laser range finder
        'tx_pin': None,     # Serial TX pin for forward laser range finder
        'uart_port': '/dev/ttyAMA0',  # Hardware UART port 
        'i2c_address': 0x62   # I2C address if using I2C interface
    },
    'laser_range_left': {
        'rx_pin': None,     # Serial RX pin for left laser range finder
        'tx_pin': None,     # Serial TX pin for left laser range finder
        'uart_port': None,  # Using I2C for side sensors
        'i2c_address': 0x63   # I2C address - different from forward sensor
    },
    'laser_range_right': {
        'rx_pin': None,     # Serial RX pin for right laser range finder
        'tx_pin': None,     # Serial TX pin for right laser range finder
        'uart_port': None,  # Using I2C for side sensors
        'i2c_address': 0x64   # I2C address - different from other sensors
    },
    'compass': {
        'scl_pin': 3,       # I2C SCL pin for digital compass
        'sda_pin': 2,       # I2C SDA pin for digital compass
        'address': 0x1E     # I2C address for HMC5883L digital compass
    }
}

# Robot physical parameters
ROBOT_PARAMS = {
    'wheel_radius': 0.08,         # Wheel radius in meters
    'wheel_distance': 0.40244,    # Distance between wheels in meters
    'cleaning_unit_diameter': 0.3048,  # meters (12 inches)
    'path_overlap': 0.1016,       # meters (4 inches)
}

# Motion parameters
MOTION_PARAMS = {
    'max_linear_speed': 0.6,      # meters/second
    'max_angular_speed': 0.5,     # radians/second
    'pwm_frequency': 1000,        # PWM frequency in Hz
    'steps_per_revolution': 200,  # Steps per revolution for the motor
}

# Navigation parameters
NAVIGATION_PARAMS = {
    'waypoint_threshold': 0.1,    # meters (distance to consider waypoint reached)
    'heading_threshold': 0.05,    # radians (angle to consider heading aligned)
    'turn_threshold': 0.1,        # radians (angle to consider turn in place)
    'safe_distance': 0.3,         # meters (minimum safe distance to obstacles)
    'safe_side_distance': 0.2     # meters (minimum safe distance to side obstacles)
}

# System parameters
SYSTEM_PARAMS = {
    'control_loop_interval': 0.05,  # seconds (50ms)
    'sensor_update_interval': 0.05  # seconds (50ms)
}

# Default cleaning area (in meters) - rectangular area
DEFAULT_CLEANING_AREA = [
    {'x': 0.0, 'y': 0.0},    # Starting point
    {'x': 2.8, 'y': 0.0},    # Right edge
    {'x': 2.8, 'y': 3.0},    # Top-right corner
    {'x': 0.0, 'y': 3.0},    # Top-left corner
] 