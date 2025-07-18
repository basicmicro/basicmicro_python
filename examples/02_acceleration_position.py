#!/usr/bin/env python3
"""
Acceleration and Position Control Example for Basicmicro motor controllers.

This example demonstrates:
- Acceleration control with speed
- Position control
- Speed-controlled movements to positions
"""

import time
import logging
import argparse
from basicmicro import Basicmicro

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Basicmicro acceleration and position control example')
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
            
            # 1. Speed with Acceleration Control
            logger.info("\n=== SPEED WITH ACCELERATION CONTROL ===")
            
            # Gradually accelerate motor 1
            logger.info("Motor 1: Accelerate to 2000 counts/sec with acceleration of 500 counts/sec^2")
            controller.SpeedAccelM1(args.address, 500, 2000)
            
            # Monitor for 5 seconds
            for _ in range(10):
                enc = controller.ReadEncM1(args.address)
                speed = controller.ReadSpeedM1(args.address)
                if enc[0] and speed[0]:
                    logger.info(f"Encoder: {enc[1]}, Speed: {speed[1]} counts/sec")
                time.sleep(0.5)
            
            # Decelerate by setting a new speed of 0 with same acceleration
            logger.info("Decelerating to stop...")
            controller.SpeedAccelM1(args.address, 500, 0)
            time.sleep(3)  # Wait for deceleration
            
            # Reset encoders
            controller.ResetEncoders(args.address)
            time.sleep(1)
            
            # 2. Speed, Distance, and Acceleration
            logger.info("\n=== SPEED, DISTANCE, AND ACCELERATION ===")
            
            # Move motor 1 to a specific distance with controlled speed and acceleration
            distance = 10000  # encoder counts
            speed = 1500  # counts per second 
            accel = 800   # counts per second^2
            buffer = 0    # 0 = don't buffer command
            
            logger.info(f"Moving motor 1 to position {distance} with speed {speed} and accel {accel}")
            controller.SpeedAccelDistanceM1(args.address, accel, speed, distance, buffer)
            
            # Monitor until move completes
            while True:
                # Read buffer status - tells us if the command is still executing
                buffer_status = controller.ReadBuffers(args.address)
                enc = controller.ReadEncM1(args.address)
                
                if enc[0]:
                    logger.info(f"Encoder: {enc[1]}, Buffer: {buffer_status[1]}")
                
                # Exit when buffer shows command is complete
                if buffer_status[0] and buffer_status[1] == 0:
                    logger.info("Movement complete")
                    break
                    
                time.sleep(0.5)
            
            time.sleep(1)
            
            # 3. Position Control
            logger.info("\n=== ABSOLUTE POSITION CONTROL ===")
            
            # Move to positions in sequence
            positions = [5000, 8000, 3000, 0]
            
            for pos in positions:
                buffer = 0  # don't buffer command
                logger.info(f"Moving to absolute position: {pos}")
                controller.PositionM1(args.address, pos, buffer)
                
                # Wait for move to complete
                while True:
                    buffer_status = controller.ReadBuffers(args.address)
                    enc = controller.ReadEncM1(args.address)
                    
                    if enc[0]:
                        logger.info(f"Encoder: {enc[1]}, Buffer: {buffer_status[1]}")
                    
                    # Exit when buffer shows command is complete
                    if buffer_status[0] and buffer_status[1] == 0:
                        logger.info(f"Reached position {pos}")
                        break
                        
                    time.sleep(0.5)
                
                time.sleep(1)  # Pause between positions
            
            # 4. Speed-Controlled Position Movement
            logger.info("\n=== SPEED-CONTROLLED POSITION MOVEMENT ===")
            
            # Move to position with specified speed
            position = 10000
            speed = 1000
            buffer = 0
            
            logger.info(f"Moving to position {position} with speed {speed}")
            controller.SpeedPositionM1(args.address, speed, position, buffer)
            
            # Monitor until move completes
            while True:
                buffer_status = controller.ReadBuffers(args.address)
                enc = controller.ReadEncM1(args.address)
                
                if enc[0]:
                    logger.info(f"Encoder: {enc[1]}, Buffer: {buffer_status[1]}")
                
                # Exit when buffer shows command is complete
                if buffer_status[0] and buffer_status[1] == 0:
                    logger.info("Movement complete")
                    break
                    
                time.sleep(0.5)
            
            # Move back to zero position
            logger.info("Moving back to home position (0)")
            controller.SpeedPositionM1(args.address, speed, 0, buffer)
            
            # Wait for move to complete
            while True:
                buffer_status = controller.ReadBuffers(args.address)
                if buffer_status[0] and buffer_status[1] == 0:
                    logger.info("Returned to home position")
                    break
                time.sleep(0.5)
            
            logger.info("Acceleration and position control examples completed")
            
    except Exception as e:
        logger.error(f"Error during test: {str(e)}")

if __name__ == "__main__":
    main()