#!/usr/bin/env python3
"""
PID Configuration Example for Basicmicro motor controllers.

This example demonstrates:
- Reading and setting velocity PID parameters
- Reading and setting position PID parameters
- Testing PID performance with movement commands
- Reading encoder status and position/speed errors
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
    parser = argparse.ArgumentParser(description='Basicmicro PID configuration example')
    parser.add_argument('-p', '--port', type=str, required=True, help='Serial port (e.g., /dev/ttyACM0, COM3)')
    parser.add_argument('-b', '--baud', type=int, default=38400, help='Baud rate (default: 38400)')
    parser.add_argument('-a', '--address', type=lambda x: int(x, 0), default=0x80, 
                        help='Controller address (default: 0x80)')
    parser.add_argument('--test-run', action='store_true', help='Run test movements with PID changes')
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
            
            # 1. Reading Current PID Values
            logger.info("\n=== READING CURRENT PID VALUES ===")
            
            # Read velocity PID parameters for motor 1
            vel_pid_m1 = controller.ReadM1VelocityPID(args.address)
            if vel_pid_m1[0]:
                logger.info("Motor 1 Velocity PID Parameters:")
                logger.info(f"  P: {vel_pid_m1[1]:.6f}")
                logger.info(f"  I: {vel_pid_m1[2]:.6f}")
                logger.info(f"  D: {vel_pid_m1[3]:.6f}")
                logger.info(f"  QPPS: {vel_pid_m1[4]}")  # Quadrature Pulses Per Second (max speed)
            else:
                logger.error("Failed to read Motor 1 Velocity PID parameters")
            
            # Read velocity PID parameters for motor 2
            vel_pid_m2 = controller.ReadM2VelocityPID(args.address)
            if vel_pid_m2[0]:
                logger.info("Motor 2 Velocity PID Parameters:")
                logger.info(f"  P: {vel_pid_m2[1]:.6f}")
                logger.info(f"  I: {vel_pid_m2[2]:.6f}")
                logger.info(f"  D: {vel_pid_m2[3]:.6f}")
                logger.info(f"  QPPS: {vel_pid_m2[4]}")
            else:
                logger.error("Failed to read Motor 2 Velocity PID parameters")
            
            # Read position PID parameters for motor 1
            pos_pid_m1 = controller.ReadM1PositionPID(args.address)
            if pos_pid_m1[0]:
                logger.info("Motor 1 Position PID Parameters:")
                logger.info(f"  P: {pos_pid_m1[1]:.6f}")
                logger.info(f"  I: {pos_pid_m1[2]:.6f}")
                logger.info(f"  D: {pos_pid_m1[3]:.6f}")
                logger.info(f"  MaxI: {pos_pid_m1[4]}")
                logger.info(f"  Deadzone: {pos_pid_m1[5]}")
                logger.info(f"  MinPos: {pos_pid_m1[6]}")
                logger.info(f"  MaxPos: {pos_pid_m1[7]}")
            else:
                logger.error("Failed to read Motor 1 Position PID parameters")
            
            # 2. Setting New PID Values
            if args.test_run:
                logger.info("\n=== SETTING NEW PID VALUES ===")
                
                # Example velocity PID values for Motor 1
                # NOTE: These are example values only and may need tuning for your specific system
                new_p = 1.0
                new_i = 0.5
                new_d = 0.25
                qpps = vel_pid_m1[4] if vel_pid_m1[0] else 10000  # Keep existing QPPS or use default
                
                logger.info(f"Setting Motor 1 Velocity PID: P={new_p}, I={new_i}, D={new_d}, QPPS={qpps}")
                success = controller.SetM1VelocityPID(args.address, new_p, new_i, new_d, qpps)
                
                if success:
                    logger.info("Successfully set Motor 1 Velocity PID parameters")
                    
                    # Verify the changes
                    new_pid = controller.ReadM1VelocityPID(args.address)
                    if new_pid[0]:
                        logger.info("New Motor 1 Velocity PID Parameters:")
                        logger.info(f"  P: {new_pid[1]:.6f}")
                        logger.info(f"  I: {new_pid[2]:.6f}")
                        logger.info(f"  D: {new_pid[3]:.6f}")
                        logger.info(f"  QPPS: {new_pid[4]}")
                else:
                    logger.error("Failed to set Motor 1 Velocity PID parameters")
                
                # Example position PID values for Motor 1
                # NOTE: These are example values only and may need tuning for your specific system
                pos_p = 10.0
                pos_i = 0.5
                pos_d = 1.0
                max_i = 50  # Maximum integral windup
                deadzone = 10  # Encoder count deadzone
                min_pos = -1000000  # Minimum position value
                max_pos = 1000000   # Maximum position value
                
                logger.info(f"Setting Motor 1 Position PID: P={pos_p}, I={pos_i}, D={pos_d}")
                success = controller.SetM1PositionPID(
                    args.address, pos_p, pos_i, pos_d, max_i, deadzone, min_pos, max_pos
                )
                
                if success:
                    logger.info("Successfully set Motor 1 Position PID parameters")
                    
                    # Verify the changes
                    new_pos_pid = controller.ReadM1PositionPID(args.address)
                    if new_pos_pid[0]:
                        logger.info("New Motor 1 Position PID Parameters:")
                        logger.info(f"  P: {new_pos_pid[1]:.6f}")
                        logger.info(f"  I: {new_pos_pid[2]:.6f}")
                        logger.info(f"  D: {new_pos_pid[3]:.6f}")
                        logger.info(f"  MaxI: {new_pos_pid[4]}")
                        logger.info(f"  Deadzone: {new_pos_pid[5]}")
                        logger.info(f"  MinPos: {new_pos_pid[6]}")
                        logger.info(f"  MaxPos: {new_pos_pid[7]}")
                else:
                    logger.error("Failed to set Motor 1 Position PID parameters")
                
                # 3. Testing Velocity PID Performance
                logger.info("\n=== TESTING VELOCITY PID PERFORMANCE ===")
                
                # Reset encoders
                controller.ResetEncoders(args.address)
                
                # Set motor to a specific speed
                test_speed = 2000  # counts per second
                logger.info(f"Running Motor 1 at speed {test_speed} to test PID...")
                controller.SpeedM1(args.address, test_speed)
                
                # Monitor performance for 5 seconds
                for i in range(10):
                    # Read encoder position and speed
                    enc = controller.ReadEncM1(args.address)
                    speed = controller.ReadSpeedM1(args.address)
                    
                    # Read error values
                    errors = controller.GetSpeedErrors(args.address)
                    
                    if enc[0] and speed[0] and errors[0]:
                        logger.info(f"Time: {i*0.5}s, Position: {enc[1]}, Speed: {speed[1]}, Error: {errors[1]}")
                    
                    time.sleep(0.5)
                
                # Stop motor
                controller.SpeedM1(args.address, 0)
                time.sleep(1)
                
                # 4. Testing Position PID Performance
                logger.info("\n=== TESTING POSITION PID PERFORMANCE ===")
                
                # Reset encoders
                controller.ResetEncoders(args.address)
                
                # Move to a target position
                target_pos = 5000
                buffer = 0  # don't buffer command
                logger.info(f"Moving Motor 1 to position {target_pos} to test Position PID...")
                controller.PositionM1(args.address, target_pos, buffer)
                
                # Monitor until move completes
                while True:
                    buffer_status = controller.ReadBuffers(args.address)
                    enc = controller.ReadEncM1(args.address)
                    pos_errors = controller.GetPosErrors(args.address)
                    
                    if enc[0] and pos_errors[0]:
                        logger.info(f"Position: {enc[1]}, Error: {pos_errors[1]}, Buffer: {buffer_status[1]}")
                    
                    # Exit when position is reached
                    if buffer_status[0] and buffer_status[1] == 0:
                        logger.info(f"Reached position {target_pos}")
                        break
                        
                    time.sleep(0.2)
                
                # Move back to home position
                logger.info("Moving back to home position...")
                controller.PositionM1(args.address, 0, buffer)
                
                # Wait for move to complete
                while True:
                    buffer_status = controller.ReadBuffers(args.address)
                    if buffer_status[0] and buffer_status[1] == 0:
                        logger.info("Returned to home position")
                        break
                    time.sleep(0.2)
            
            logger.info("PID configuration example completed")
            
    except Exception as e:
        logger.error(f"Error during test: {str(e)}")

if __name__ == "__main__":
    main()