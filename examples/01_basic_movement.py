#!/usr/bin/env python3
"""
Basic Movement Example for Basicmicro motor controllers.

This example demonstrates the fundamental movement commands:
- Setting duty cycle (analog to PWM)
- Speed control
- Mixed mode control for differential drive
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
    parser = argparse.ArgumentParser(description='Basicmicro motor controller basic movement example')
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
            
            # 1. Duty Cycle Control (Simple)
            logger.info("\n=== DUTY CYCLE CONTROL ===")
            
            # Motor 1 forward at 25% duty
            logger.info("Motor 1: 25% forward for 2 seconds")
            controller.DutyM1(args.address, 8192)  # 8192 = 25% of 32767 (max)
            time.sleep(2)
            controller.DutyM1(args.address, 0)  # Stop
            
            # Motor 2 backward at 25% duty
            logger.info("Motor 2: 25% backward for 2 seconds")
            controller.DutyM2(args.address, -8192)  # -8192 = 25% backward
            time.sleep(2)
            controller.DutyM2(args.address, 0)  # Stop
            
            # Both motors simultaneously
            logger.info("Both motors: M1 forward, M2 backward for 2 seconds")
            controller.DutyM1M2(args.address, 8192, -8192)
            time.sleep(2)
            controller.DutyM1M2(args.address, 0, 0)  # Stop both
            
            time.sleep(1)  # Pause between tests
            
            # 2. Speed Control (Requires encoders)
            logger.info("\n=== SPEED CONTROL ===")
            
            # First, we'll reset the encoders to zero
            controller.ResetEncoders(args.address)
            
            # Motor 1 at 1000 counts per second
            logger.info("Motor 1: 1000 encoder counts/sec for 3 seconds")
            controller.SpeedM1(args.address, 1000)
            for _ in range(6):
                enc = controller.ReadEncM1(args.address)
                speed = controller.ReadSpeedM1(args.address)
                if enc[0] and speed[0]:
                    logger.info(f"Encoder: {enc[1]}, Speed: {speed[1]} counts/sec")
                time.sleep(0.5)
            controller.SpeedM1(args.address, 0)  # Stop
            
            # Reset encoders again
            controller.ResetEncoders(args.address)
            time.sleep(1)
            
            # Both motors with different speeds
            logger.info("Both motors: M1=800, M2=-600 counts/sec for 3 seconds")
            controller.SpeedM1M2(args.address, 800, -600)
            for _ in range(6):
                enc1 = controller.ReadEncM1(args.address)
                enc2 = controller.ReadEncM2(args.address)
                if enc1[0] and enc2[0]:
                    logger.info(f"M1 Encoder: {enc1[1]}, M2 Encoder: {enc2[1]}")
                time.sleep(0.5)
            controller.SpeedM1M2(args.address, 0, 0)  # Stop both
            
            time.sleep(1)  # Pause between tests
            
            # 3. Mixed Mode Control (For differential drive robots)
            logger.info("\n=== MIXED MODE CONTROL ===")
            
            # Forward movement
            logger.info("Mixed mode: Forward for 2 seconds")
            controller.ForwardMixed(args.address, 64)  # ~50% speed (0-127 range)
            time.sleep(2)
            controller.ForwardMixed(args.address, 0)  # Stop
            
            # Turning in place
            logger.info("Mixed mode: Turn right for 2 seconds")
            controller.TurnRightMixed(args.address, 64)  # ~50% turn rate
            time.sleep(2)
            
            logger.info("Mixed mode: Turn left for 2 seconds")
            controller.TurnLeftMixed(args.address, 64)  # ~50% turn rate
            time.sleep(2)
            
            # Stop all movements
            controller.ForwardMixed(args.address, 0)
            controller.TurnRightMixed(args.address, 0)
            
            logger.info("Movement examples completed")
            
    except Exception as e:
        logger.error(f"Error during test: {str(e)}")

if __name__ == "__main__":
    main()