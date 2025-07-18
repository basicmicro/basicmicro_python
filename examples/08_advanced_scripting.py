#!/usr/bin/env python3
"""
Advanced Scripting and Asynchronous Control Example for Basicmicro controllers.

This example demonstrates:
- Script control (start/stop)
- Setting script autorun parameters
- Working with buffer commands
- Multi-threaded motor control
- Using threading for simultaneous sensor monitoring
- Creating complex movement sequences
"""

import time
import logging
import argparse
import threading
from typing import Dict, Any, List
from basicmicro import Basicmicro

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Global flag for stopping threads
stop_threads = False

def monitor_thread(controller, address):
    """Thread function to continuously monitor controller status."""
    logger.info("Monitor thread started")
    
    try:
        while not stop_threads:
            # Read voltages
            main_batt = controller.ReadMainBatteryVoltage(address)
            
            # Read temperatures
            temps = controller.GetTemps(address)
            
            # Read currents
            currents = controller.ReadCurrents(address)
            
            # Read encoder positions
            encoders = controller.GetEncoders(address)
            
            # Build status string
            status = "STATUS:"
            if main_batt[0]:
                status += f" Batt={main_batt[1]/10.0:.1f}V"
            if temps[0]:
                status += f" Temp1={temps[1]/10.0:.1f}°C Temp2={temps[2]/10.0:.1f}°C"
            if currents[0]:
                status += f" Curr1={currents[1]}mA Curr2={currents[2]}mA"
            if encoders[0]:
                status += f" Enc1={encoders[1]} Enc2={encoders[2]}"
            
            logger.info(status)
            
            # Check for errors
            error = controller.ReadError(address)
            if error[0] and error[1] != 0:
                logger.warning(f"ERROR DETECTED: 0x{error[1]:08X}")
            
            # Sleep between readings
            time.sleep(1)
    
    except Exception as e:
        logger.error(f"Error in monitor thread: {str(e)}")
    
    logger.info("Monitor thread stopped")

def execute_sequence(controller, address, sequence):
    """Execute a sequence of movement commands.
    
    Args:
        controller: The Basicmicro controller object
        address: The controller address
        sequence: List of command dictionaries
    """
    for i, cmd in enumerate(sequence):
        logger.info(f"Executing command {i+1}/{len(sequence)}: {cmd['description']}")
        
        # Execute the appropriate command based on type
        if cmd['type'] == 'duty':
            controller.DutyM1M2(address, cmd['m1'], cmd['m2'])
        elif cmd['type'] == 'speed':
            controller.SpeedM1M2(address, cmd['m1'], cmd['m2'])
        elif cmd['type'] == 'speed_accel':
            controller.SpeedAccelM1M2(address, cmd['accel'], cmd['m1'], cmd['m2'])
        elif cmd['type'] == 'position':
            controller.PositionM1M2(address, cmd['m1'], cmd['m2'], 0)
            
            # Wait for position command to complete
            if cmd.get('wait_complete', False):
                while True:
                    buffer_status = controller.ReadBuffers(address)
                    if buffer_status[0] and buffer_status[1] == 0:
                        break
                    time.sleep(0.1)
        
        # Wait for specified duration
        if 'duration' in cmd:
            time.sleep(cmd['duration'])
    
    # Stop motors at end of sequence
    controller.DutyM1M2(address, 0, 0)

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Basicmicro advanced scripting example')
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
            
            # Reset encoders
            controller.ResetEncoders(args.address)
            
            # 1. Script Control
            logger.info("\n=== SCRIPT CONTROL ===")
            
            # Check script autorun settings
            script_autorun = controller.GetScriptAutoRun(args.address)
            if script_autorun[0]:
                logger.info(f"Script autorun setting: {script_autorun[1]}ms")
                
                if script_autorun[1] > 0:
                    logger.info(f"Script is set to autorun after {script_autorun[1]}ms")
                else:
                    logger.info("Script autorun is disabled")
            
            # Start/stop script demonstration
            # NOTE: This assumes the controller has a script already loaded
            # If there is no script, these commands will have no effect
            if False:  # Disabled by default for safety
                logger.info("Starting controller script")
                result = controller.StartScript(args.address)
                if result:
                    logger.info("Script started successfully")
                else:
                    logger.warning("Failed to start script")
                
                # Let the script run for a few seconds
                logger.info("Letting script run for 5 seconds...")
                time.sleep(5)
                
                # Stop the script
                logger.info("Stopping controller script")
                result = controller.StopScript(args.address)
                if result:
                    logger.info("Script stopped successfully")
                else:
                    logger.warning("Failed to stop script")
            
            # 2. Multi-threaded Monitoring
            logger.info("\n=== MULTI-THREADED MONITORING ===")
            
            # Reset global flag
            global stop_threads
            stop_threads = False
            
            # Start monitoring thread
            logger.info("Starting monitoring thread")
            monitor = threading.Thread(target=monitor_thread, args=(controller, args.address))
            monitor.daemon = True
            monitor.start()
            
            # 3. Complex Movement Sequence
            logger.info("\n=== COMPLEX MOVEMENT SEQUENCE ===")
            
            # Define a movement sequence
            # Each entry is a command dictionary with parameters
            sequence = [
                {
                    'type': 'speed_accel',
                    'accel': 500,
                    'm1': 1000,
                    'm2': 1000,
                    'duration': 3,
                    'description': 'Forward acceleration'
                },
                {
                    'type': 'speed_accel',
                    'accel': 500,
                    'm1': 1500,
                    'm2': 800,
                    'duration': 4,
                    'description': 'Wide right turn'
                },
                {
                    'type': 'speed_accel',
                    'accel': 500,
                    'm1': 800,
                    'm2': 1500,
                    'duration': 4,
                    'description': 'Wide left turn'
                },
                {
                    'type': 'speed_accel',
                    'accel': 300,
                    'm1': 0,
                    'm2': 0,
                    'duration': 2,
                    'description': 'Gentle stop'
                },
                {
                    'type': 'speed',
                    'm1': 800,
                    'm2': -800,
                    'duration': 2.5,
                    'description': 'Rotate 180 degrees'
                },
                {
                    'type': 'position',
                    'm1': 5000,
                    'm2': 5000,
                    'wait_complete': True,
                    'description': 'Move to specific position'
                },
                {
                    'type': 'position',
                    'm1': 0,
                    'm2': 0,
                    'wait_complete': True,
                    'description': 'Return to home position'
                }
            ]
            
            # Execute the sequence
            execute_sequence(controller, args.address, sequence)
            
            # 4. Asynchronous Buffer Commands
            logger.info("\n=== ASYNCHRONOUS BUFFER COMMANDS ===")
            
            # Reset encoders
            controller.ResetEncoders(args.address)
            
            # Example: Queue multiple movement commands
            # Each movement will execute in sequence without PC intervention
            logger.info("Queuing multiple buffered commands")
            
            # Command 1: Move forward 3000 counts at speed 1000
            buffer1 = 1  # Use buffer
            controller.SpeedDistanceM1M2(
                args.address, 
                1000, 3000,  # M1 speed and distance
                1000, 3000,  # M2 speed and distance
                buffer1
            )
            
            # Command 2: Rotate right 90 degrees
            buffer2 = 1  # Use buffer
            controller.SpeedDistanceM1M2(
                args.address, 
                1000, 1500,   # M1 speed and distance (outer wheel)
                -1000, 1500,  # M2 speed and distance (inner wheel)
                buffer2
            )
            
            # Command 3: Move forward 3000 counts at speed 1000
            buffer3 = 1  # Use buffer
            controller.SpeedDistanceM1M2(
                args.address, 
                1000, 3000,  # M1 speed and distance
                1000, 3000,  # M2 speed and distance
                buffer3
            )
            
            # Command 4: Return to home (reverse of the previous movements)
            # This command will execute after all the above are complete
            buffer4 = 0  # Final command, don't buffer
            controller.SpeedDistanceM1M2(
                args.address, 
                1000, 7500,   # M1 speed and distance (reverse of total distance)
                -1000, 7500,  # M2 speed and distance (reverse of total distance)
                buffer4
            )
            
            # Monitor buffer status until all commands complete
            logger.info("Monitoring buffer status while commands execute")
            while True:
                # Read buffer status
                buffer_status = controller.ReadBuffers(args.address)
                
                if buffer_status[0]:
                    logger.info(f"Buffer status: {buffer_status[1]}")
                    
                    # If buffer is empty, all commands are complete
                    if buffer_status[1] == 0:
                        logger.info("All buffered commands completed")
                        break
                
                time.sleep(0.5)
            
            # 5. Stop monitoring thread
            logger.info("\n=== STOPPING MONITORING ===")
            stop_threads = True
            logger.info("Waiting for monitoring thread to stop...")
            monitor.join(timeout=2)
            
            logger.info("Advanced scripting example completed")
            
    except Exception as e:
        logger.error(f"Error during test: {str(e)}")
        # Make sure to stop all threads on error
        stop_threads = True

if __name__ == "__main__":
    main()