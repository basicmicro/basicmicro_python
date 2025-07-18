#!/usr/bin/env python3
"""
Basic control example for the Basicmicro library.

This example demonstrates how to initialize the controller, control motors,
read encoders, and properly close the connection.
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
    parser = argparse.ArgumentParser(description='Basicmicro motor controller example')
    parser.add_argument('-p', '--port', type=str, required=True, help='Serial port (e.g., /dev/ttyACM0, COM3)')
    parser.add_argument('-b', '--baud', type=int, default=38400, help='Baud rate (default: 38400)')
    parser.add_argument('-a', '--address', type=lambda x: int(x, 0), default=0x80, 
                        help='Controller address (default: 0x80)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose debug logging')
    args = parser.parse_args()
    
    # Enable debug logging if requested
    if args.verbose:
        logging.getLogger('basicmicro').setLevel(logging.DEBUG)
    
    logger.info(f"Connecting to controller at {args.port}, baud rate {args.baud}")
    
    # Using context manager for safe resource handling
    with Basicmicro(args.port, args.baud, verbose=args.verbose) as controller:
        # Read firmware version
        version_result = controller.ReadVersion(args.address)
        if version_result[0]:
            logger.info(f"Connected to controller with firmware version: {version_result[1]}")
        else:
            logger.error("Failed to read firmware version!")
            return
        
        # Read battery voltage
        voltage_result = controller.ReadMainBatteryVoltage(args.address)
        if voltage_result[0]:
            logger.info(f"Main battery voltage: {voltage_result[1]/Basicmicro.VOLTAGE_SCALE}V")
        
        # Run a simple motor control sequence
        try:
            # Reset encoders
            controller.ResetEncoders(args.address)
            logger.info("Encoders reset to zero")
            
            # Move motor 1 at half speed
            logger.info("Moving motor 1 forward at half speed for 2 seconds")
            controller.DutyM1(args.address, 16384)  # Half speed forward
            
            # Monitor encoder and speed for 2 seconds
            start_time = time.time()
            while time.time() - start_time < 2.0:
                enc = controller.ReadEncM1(args.address)
                speed = controller.ReadSpeedM1(args.address)
                
                if enc[0] and speed[0]:
                    logger.info(f"Encoder: {enc[1]}, Speed: {speed[1]} counts/sec")
                
                time.sleep(0.2)
            
            # Stop motor
            logger.info("Stopping motor")
            controller.DutyM1(args.address, 0)
            
            # Wait for motor to stop completely
            time.sleep(0.5)
            
            # Move motor in reverse
            logger.info("Moving motor 1 backward at quarter speed for 2 seconds")
            controller.DutyM1(args.address, -8192)  # Quarter speed backward
            
            # Monitor encoder and speed for 2 seconds
            start_time = time.time()
            while time.time() - start_time < 2.0:
                enc = controller.ReadEncM1(args.address)
                speed = controller.ReadSpeedM1(args.address)
                
                if enc[0] and speed[0]:
                    logger.info(f"Encoder: {enc[1]}, Speed: {speed[1]} counts/sec")
                
                time.sleep(0.2)
            
            # Final stop
            logger.info("Stopping motor")
            controller.DutyM1(args.address, 0)
            
        except KeyboardInterrupt:
            logger.info("Test interrupted by user")
            # Make sure to stop motors on interruption
            controller.DutyM1(args.address, 0)
            controller.DutyM2(args.address, 0)
        
        logger.info("Test sequence completed")

if __name__ == "__main__":
    main()