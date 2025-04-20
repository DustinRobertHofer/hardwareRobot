# Pressure Washing Robot

An autonomous robot system designed to perform pressure washing operations on flat surfaces. Built with a Raspberry Pi for control and navigation.

## Overview

This project implements a complete robotic system for autonomous pressure washing operations. The robot navigates a specified cleaning area using path planning algorithms and maintains its position through sensor feedback.

## Features

- Autonomous navigation and path planning
- Obstacle detection and avoidance
- Real-time position tracking and state estimation
- Configurable cleaning area boundaries
- Stepper motor control for precise movement
- Sensor fusion with compass and range finders
- Cleaning path optimization for efficient coverage

## Hardware Requirements

- Raspberry Pi (3B+ or 4 recommended)
- Stepper motors with drivers
- Digital compass (HMC5883L)
- Laser range finders
- Rotary encoders for wheel odometry
- Pressure washing system components
- Power supply system
- Chassis and mounting components

## Software Architecture

The project follows a modular architecture:

- **Main Control** (`robotControl.py`): Main robot controller that coordinates all subsystems
- **Configuration** (`robotConfig.py`): Central configuration for pins, parameters, and settings
- **Hardware Interfaces**:
  - `motor.py`: Stepper motor control interface
  - `compass.py`: Digital compass interface
  - `laserRange.py`: Laser range finder interface
- **Utilities**:
  - `state.py`: Robot state estimation
  - `navigator.py`: Navigation and waypoint following
  - `motionController.py`: Motion control and motor commands
  - `sensorManager.py`: Sensor data acquisition and processing
  - `pathPlanner.py`: Cleaning path generation algorithms

## Setup and Installation

1. Clone this repository to your Raspberry Pi
2. Install required dependencies:
   ```
   pip install RPi.GPIO smbus2 numpy
   ```
3. Connect hardware components according to pin configurations in `robotConfig.py`
4. Run the main control program:
   ```
   python src/robotControl.py
   ```

## Usage

1. Power on the robot and ensure all connections are secure
2. Run the main control program
3. The robot will prompt for compass calibration (follow on-screen instructions)
4. Press Enter to start the cleaning operation
5. The robot will autonomously navigate the cleaning area
6. Press Ctrl+C to safely stop the operation at any time

## Configuration

Modify `robotConfig.py` to customize:
- Pin assignments for motors and sensors
- Robot physical parameters
- Motion control parameters
- Navigation thresholds
- Default cleaning area boundaries

## Safety Considerations

- Always ensure the pressure washing system is properly configured
- Test in a controlled environment before deployment
- Ensure proper waterproofing of electronic components
- Monitor the robot during operation, especially in the initial testing phase
- Implement emergency stop mechanisms

