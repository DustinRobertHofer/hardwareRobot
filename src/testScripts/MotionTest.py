import sys
import time
import signal
import os

# Add the src directory to the Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(script_dir, '..'))
sys.path.append(src_dir)

from utils.motionController import MotionController

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("Stopping motors and cleaning up...")
    if motion_controller:
        motion_controller.cleanup()
    sys.exit(0)

# Register signal handler for graceful shutdown
signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":
    motion_controller = None
    try:
        print("Initializing motion controller...")
        motion_controller = MotionController()
        
        print("Moving forward at 0.1 m/s...")
        motion_controller.set_velocity(0.1, 0)  # Forward at 0.1 m/s
        time.sleep(3)
        
        print("Turning right...")
        motion_controller.set_velocity(0.05, 0.5)  # Forward while turning right
        time.sleep(3)
        
        print("Turning left...")
        motion_controller.set_velocity(0.05, -0.5)  # Forward while turning left
        time.sleep(3)
        
        print("Stopping...")
        motion_controller.stop()
        time.sleep(1)
        
        print("Test complete. Cleaning up...")
        motion_controller.cleanup()
        
    except Exception as e:
        print(f"Error: {e}")
        if motion_controller:
            motion_controller.cleanup() 