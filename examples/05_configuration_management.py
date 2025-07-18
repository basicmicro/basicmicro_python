#!/usr/bin/env python3
"""
Configuration Management Example for Basicmicro motor controllers.

This example demonstrates:
- Reading and setting controller configuration
- Reading and setting encoder modes
- Reading and writing to non-volatile memory (NVM)
- Setting and reading serial numbers
- Setting and getting default speeds and accelerations
- Working with emergency stop features
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
    parser = argparse.ArgumentParser(description='Basicmicro configuration management example')
    parser.add_argument('-p', '--port', type=str, required=True, help='Serial port (e.g., /dev/ttyACM0, COM3)')
    parser.add_argument('-b', '--baud', type=int, default=38400, help='Baud rate (default: 38400)')
    parser.add_argument('-a', '--address', type=lambda x: int(x, 0), default=0x80, 
                        help='Controller address (default: 0x80)')
    parser.add_argument('--modify', action='store_true', help='Enable modification of controller settings')
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
            
            # 1. Reading Current Configuration
            logger.info("\n=== CURRENT CONTROLLER CONFIGURATION ===")
            
            # Read current configuration value
            config = controller.GetConfig(args.address)
            if config[0]:
                logger.info(f"Controller configuration: 0x{config[1]:04X}")
                
                # Interpret configuration bits (this will depend on your specific controller)
                # These are example bits only - check your controller documentation
                logger.info("Configuration settings:")
                logger.info(f"  RC Mode: {(config[1] & 0x01) != 0}")
                logger.info(f"  Serial Mode: {(config[1] & 0x02) != 0}")
                logger.info(f"  Differential Input: {(config[1] & 0x04) != 0}")
                logger.info(f"  Mixing Enabled: {(config[1] & 0x08) != 0}")
                logger.info(f"  Exponential: {(config[1] & 0x10) != 0}")
                logger.info(f"  MCU Timeout: {(config[1] & 0x20) != 0}")
            
            # Read encoder modes
            enc_modes = controller.ReadEncoderModes(args.address)
            if enc_modes[0]:
                logger.info(f"Encoder 1 Mode: {enc_modes[1]}")
                logger.info(f"Encoder 2 Mode: {enc_modes[2]}")
                
                # Interpret encoder modes
                for i, mode in enumerate([enc_modes[1], enc_modes[2]], 1):
                    mode_str = "Unknown"
                    if mode == 0:
                        mode_str = "Quadrature"
                    elif mode == 1:
                        mode_str = "Absolute"
                    logger.info(f"  Encoder {i} Mode Type: {mode_str}")
            
            # Read default speeds and accelerations
            def_speeds = controller.GetDefaultSpeeds(args.address)
            if def_speeds[0]:
                logger.info(f"Default Speed M1: {def_speeds[1]}")
                logger.info(f"Default Speed M2: {def_speeds[2]}")
                
            def_accels = controller.GetDefaultAccels(args.address)
            if def_accels[0]:
                logger.info(f"Default Accel M1: {def_accels[1]}")
                logger.info(f"Default Accel M2: {def_accels[2]}")
                logger.info(f"Default Decel M1: {def_accels[3]}")
                logger.info(f"Default Decel M2: {def_accels[4]}")
            
            # Read serial number
            serial_num = controller.GetSerialNumber(args.address)
            if serial_num[0]:
                logger.info(f"Controller Serial Number: {serial_num[1]}")
            
            # Read e-stop lock status
            estop_lock = controller.GetEStopLock(args.address)
            if estop_lock[0]:
                lock_state = "Unknown"
                if estop_lock[1] == Basicmicro.ESTOP_HW_RESET:
                    lock_state = "Hardware reset required"
                elif estop_lock[1] == Basicmicro.ESTOP_AUTO_RESET:
                    lock_state = "Automatic reset"
                elif estop_lock[1] == Basicmicro.ESTOP_SW_RESET:
                    lock_state = "Software reset enabled"
                    
                logger.info(f"E-Stop Lock State: 0x{estop_lock[1]:02X} ({lock_state})")
            
            # 2. Modifying Configuration (if enabled)
            if args.modify:
                logger.info("\n=== MODIFYING CONTROLLER CONFIGURATION ===")
                logger.warning("WARNING: Some settings may affect communication or controller behavior.")
                
                # Example: Set encoder mode for motor 1
                logger.info("Setting Encoder 1 to Quadrature mode")
                result = controller.SetM1EncoderMode(args.address, 0)  # 0 = Quadrature mode
                if result:
                    logger.info("Successfully set Encoder 1 mode")
                else:
                    logger.error("Failed to set Encoder 1 mode")
                
                # Example: Set default acceleration
                accel_value = 1000  # acceleration value
                logger.info(f"Setting default acceleration for Motor 1 to {accel_value}")
                result = controller.SetM1DefaultAccel(args.address, accel_value)
                if result:
                    logger.info("Successfully set default acceleration")
                else:
                    logger.error("Failed to set default acceleration")
                
                # Example: Set a new serial number
                # Note: This is just an example - you probably don't want to change
                # the serial number on a production controller
                if False:  # Disabled by default for safety
                    new_serial = "EXAMPLE-SN-123456"
                    logger.info(f"Setting new serial number: {new_serial}")
                    result = controller.SetSerialNumber(args.address, new_serial)
                    if result:
                        logger.info("Successfully set new serial number")
                    else:
                        logger.error("Failed to set serial number")
                
                # Example: Save settings to non-volatile memory
                if False:  # Disabled by default for safety
                    logger.info("Saving settings to non-volatile memory (NVM)")
                    result = controller.WriteNVM(args.address)
                    if result:
                        logger.info("Successfully saved settings to NVM")
                    else:
                        logger.error("Failed to save settings to NVM")
                
                # Verify our changes
                logger.info("\n=== VERIFYING CONFIGURATION CHANGES ===")
                
                # Read encoder modes again
                enc_modes = controller.ReadEncoderModes(args.address)
                if enc_modes[0]:
                    logger.info(f"Encoder 1 Mode: {enc_modes[1]}")
                    logger.info(f"Encoder 2 Mode: {enc_modes[2]}")
                
                # Read default acceleration again
                def_accels = controller.GetDefaultAccels(args.address)
                if def_accels[0]:
                    logger.info(f"Default Accel M1: {def_accels[1]}")
            
            # 3. Emergency Stop Functionality
            logger.info("\n=== EMERGENCY STOP FUNCTIONALITY ===")
            
            # IMPORTANT: Demonstration only. In a real system, you would
            # trigger E-Stop in response to safety-critical conditions.
            if False:  # Disabled by default for safety
                
                # First, let's start a motor
                logger.info("Starting motor 1 at low speed...")
                controller.DutyM1(args.address, 4096)  # ~12.5% forward
                time.sleep(2)
                
                # Now trigger emergency stop
                logger.info("Triggering emergency stop...")
                # Simulate emergency stop (depends on controller configuration)
                # This typically involves setting a specific pin on the controller
                
                # Check status after E-Stop
                logger.info("Checking status after E-Stop...")
                status = controller.GetStatus(args.address)
                if status[0]:
                    logger.info(f"State: 0x{status[2]:08X}")
                    logger.info(f"PWM Motor 1: {status[7]}")
                
                # Reset E-Stop
                logger.info("Resetting E-Stop...")
                result = controller.ResetEStop(args.address)
                if result:
                    logger.info("Successfully reset E-Stop")
                else:
                    logger.error("Failed to reset E-Stop")
                    
                # Ensure motor is stopped regardless
                controller.DutyM1(args.address, 0)
            
            logger.info("Configuration management example completed")
            
    except Exception as e:
        logger.error(f"Error during test: {str(e)}")

if __name__ == "__main__":
    main()