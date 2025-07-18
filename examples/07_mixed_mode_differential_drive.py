#!/usr/bin/env python3
"""
Mixed Mode and Differential Drive Example for Basicmicro motor controllers.

This example demonstrates controlling a differential drive robot using:
- Basic mixed mode commands (forward/backward/left/right)
- Mixed duty cycle control
- Mixed speed control
- Mixed speed with acceleration control
- Mixed position control
"""

import time
import logging
import argparse
import math
from basicmicro import Basicmicro

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Basicmicro mixed mode and differential drive example')
    parser.add_argument('-p', '--port', type=str, required=True, help='Serial port (e.g., /dev/ttyACM0, COM3)')
    parser.add_argument('-b', '--baud', type=int, default=38400, help='Baud rate (default: 38400)')
    parser.add_argument('-a', '--address', type=lambda x: int(x, 0), default=0x80, 
                        help='Controller address (default: 0x80)')
    args = parser.parse_args()
    
    logger.info(f"Connecting to controller at {args.port}, baud rate {args.baud}")
    
    try:
        with Basicmicro(args.port, args.baud) as controller:
            # Read firmware version
            version_result = controller.ReadVersion(args.address)
            if version_result[0]:
                logger.info(f"Connected to controller with firmware version: {version_result[1]}")
            else:
                logger.error("Failed to read firmware version!")
                return
            
            # Reset encoders to zero
            controller.ResetEncoders(args.address)
            
            # 1. Basic Mixed Mode Commands (Legacy API)
            logger.info("\n=== BASIC MIXED MODE COMMANDS ===")
            logger.info("These commands use the traditional 0-127 value range for speed/turn")
            
            # Drive forward
            logger.info("Driving forward for 2 seconds")
            controller.ForwardMixed(args.address, 60)  # Value 0-127
            time.sleep(2)
            controller.ForwardMixed(args.address, 0)  # Stop
            
            # Drive backward
            logger.info("Driving backward for 2 seconds")
            controller.BackwardMixed(args.address, 60)  # Value 0-127
            time.sleep(2)
            controller.BackwardMixed(args.address, 0)  # Stop
            
            # Turn right
            logger.info("Turning right for 2 seconds")
            controller.TurnRightMixed(args.address, 60)  # Value 0-127
            time.sleep(2)
            controller.TurnRightMixed(args.address, 0)  # Stop
            
            # Turn left
            logger.info("Turning left for 2 seconds")
            controller.TurnLeftMixed(args.address, 60)  # Value 0-127
            time.sleep(2)
            controller.TurnLeftMixed(args.address, 0)  # Stop
            
            # Diagonal movement (forward + right)
            logger.info("Diagonal movement (forward + right) for 2 seconds")
            controller.ForwardMixed(args.address, 60)  # Value 0-127
            controller.TurnRightMixed(args.address, 30)  # Value 0-127
            time.sleep(2)
            controller.ForwardMixed(args.address, 0)  # Stop
            controller.TurnRightMixed(args.address, 0)  # Stop
            
            # 2. Mixed Duty Cycle Control
            logger.info("\n=== MIXED DUTY CYCLE CONTROL ===")
            logger.info("Using DutyM1M2 for direct control of both motors")
            
            # Both motors forward at different speeds
            logger.info("Both motors forward, M1 faster than M2 for 2 seconds")
            # Values are in range -32767 to +32767
            controller.DutyM1M2(args.address, 16384, 8192)  # M1=50%, M2=25%
            time.sleep(2)
            controller.DutyM1M2(args.address, 0, 0)  # Stop
            
            # Rotate in place (one motor forward, one backward)
            logger.info("Rotating in place (right turn) for 2 seconds")
            controller.DutyM1M2(args.address, 8192, -8192)  # M1 forward, M2 backward
            time.sleep(2)
            controller.DutyM1M2(args.address, 0, 0)  # Stop
            
            # 3. Mixed Speed Control (with encoders)
            logger.info("\n=== MIXED SPEED CONTROL ===")
            logger.info("Using SpeedM1M2 for precise speed control")
            
            # Reset encoders
            controller.ResetEncoders(args.address)
            
            # Both motors at different speeds
            logger.info("Both motors at different speeds for 3 seconds")
            controller.SpeedM1M2(args.address, 1000, 750)  # counts per second
            
            # Monitor encoder values
            for _ in range(6):
                encoders = controller.GetEncoders(args.address)
                speeds = controller.GetSpeeds(args.address)
                if encoders[0] and speeds[0]:
                    logger.info(f"Encoders - M1: {encoders[1]}, M2: {encoders[2]}")
                    logger.info(f"Speeds - M1: {speeds[1]}, M2: {speeds[2]}")
                time.sleep(0.5)
            
            controller.SpeedM1M2(args.address, 0, 0)  # Stop
            
            # 4. Driving in a Circle
            logger.info("\n=== DRIVING IN A CIRCLE ===")
            logger.info("Using SpeedM1M2 for precise control")
            
            # Reset encoders
            controller.ResetEncoders(args.address)
            
            # Set different speeds for a circular path
            # The radius of the circle depends on the speed difference between motors
            logger.info("Driving in a circle for 5 seconds")
            controller.SpeedM1M2(args.address, 1500, 1000)  # Outer wheel faster
            
            # Monitor for 5 seconds
            for _ in range(10):
                encoders = controller.GetEncoders(args.address)
                if encoders[0]:
                    logger.info(f"Encoders - M1: {encoders[1]}, M2: {encoders[2]}")
                time.sleep(0.5)
            
            controller.SpeedM1M2(args.address, 0, 0)  # Stop
            
            # 5. Controlled Acceleration
            logger.info("\n=== MIXED SPEED WITH ACCELERATION CONTROL ===")
            
            # Reset encoders
            controller.ResetEncoders(args.address)
            
            # Accelerate both motors to different speeds
            logger.info("Accelerating both motors with controlled rates")
            accel = 500  # acceleration in counts/second²
            speed1 = 1500  # target speed for motor 1
            speed2 = 1200  # target speed for motor 2
            
            controller.SpeedAccelM1M2(args.address, accel, speed1, speed2)
            
            # Monitor acceleration
            for _ in range(6):
                speeds = controller.GetSpeeds(args.address)
                if speeds[0]:
                    logger.info(f"Speeds - M1: {speeds[1]}, M2: {speeds[2]}")
                time.sleep(0.5)
            
            # Decelerate to stop
            logger.info("Decelerating to stop")
            controller.SpeedAccelM1M2(args.address, accel, 0, 0)
            time.sleep(3)  # Allow time to decelerate
            
            # 6. Square Path Demonstration
            logger.info("\n=== SQUARE PATH DEMONSTRATION ===")
            logger.info("Driving in a square pattern using speed control")
            
            # Reset encoders
            controller.ResetEncoders(args.address)
            
            # Define parameters for square
            side_duration = 3  # seconds per side
            speed = 1200  # counts per second
            turn_duration = 1.5  # seconds per turn
            
            # Drive in a square: forward, right, forward, right, forward, right, forward
            for i in range(4):  # 4 sides of the square
                # Go straight
                logger.info(f"Side {i+1}: Going straight")
                controller.SpeedM1M2(args.address, speed, speed)
                time.sleep(side_duration)
                
                # Stop
                controller.SpeedM1M2(args.address, 0, 0)
                time.sleep(0.5)
                
                # Turn right (90 degrees)
                logger.info(f"Side {i+1}: Turning right")
                controller.SpeedM1M2(args.address, speed, -speed)  # Rotate in place
                time.sleep(turn_duration)
                
                # Stop
                controller.SpeedM1M2(args.address, 0, 0)
                time.sleep(0.5)
            
            # 7. Mixed Position Control
            logger.info("\n=== MIXED POSITION CONTROL ===")
            
            # Reset encoders
            controller.ResetEncoders(args.address)
            
            # Move both motors to specific positions
            position1 = 5000  # encoder counts
            position2 = 3000  # encoder counts
            buffer = 0  # don't buffer command
            
            logger.info(f"Moving motors to positions: M1={position1}, M2={position2}")
            controller.PositionM1M2(args.address, position1, position2, buffer)
            
            # Monitor until move completes
            while True:
                buffer_status = controller.ReadBuffers(args.address)
                encoders = controller.GetEncoders(args.address)
                
                if encoders[0]:
                    logger.info(f"Encoders - M1: {encoders[1]}, M2: {encoders[2]}")
                
                # Exit when buffer shows command is complete
                if buffer_status[0] and buffer_status[1] == 0:
                    logger.info("Position move complete")
                    break
                    
                time.sleep(0.5)
            
            # Return to home position
            logger.info("Returning to home position")
            controller.PositionM1M2(args.address, 0, 0, buffer)
            
            # Wait for move to complete
            while True:
                buffer_status = controller.ReadBuffers(args.address)
                if buffer_status[0] and buffer_status[1] == 0:
                    logger.info("Returned to home position")
                    break
                time.sleep(0.5)
            
            logger.info("Mixed mode and differential drive example completed")
            
    except Exception as e:
        logger.error(f"Error during test: {str(e)}")

if __name__ == "__main__":
    main()