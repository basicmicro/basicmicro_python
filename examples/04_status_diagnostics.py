#!/usr/bin/env python3
"""
Status and Diagnostics Example for Basicmicro motor controllers.

This example demonstrates:
- Reading controller status
- Reading voltage and temperature
- Reading current and PWM values
- Reading encoder status
- Checking for errors
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
    parser = argparse.ArgumentParser(description='Basicmicro status and diagnostics example')
    parser.add_argument('-p', '--port', type=str, required=True, help='Serial port (e.g., /dev/ttyACM0, COM3)')
    parser.add_argument('-b', '--baud', type=int, default=38400, help='Baud rate (default: 38400)')
    parser.add_argument('-a', '--address', type=lambda x: int(x, 0), default=0x80, 
                        help='Controller address (default: 0x80)')
    parser.add_argument('--monitor', action='store_true', help='Continuously monitor status')
    parser.add_argument('--motor-test', action='store_true', help='Run a motor test during monitoring')
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
            
            # 1. Basic Status Information
            logger.info("\n=== BASIC STATUS INFORMATION ===")
            
            # Read battery voltage
            main_batt = controller.ReadMainBatteryVoltage(args.address)
            logic_batt = controller.ReadLogicBatteryVoltage(args.address)
            
            if main_batt[0] and logic_batt[0]:
                logger.info(f"Main battery voltage: {main_batt[1]/10.0}V")
                logger.info(f"Logic battery voltage: {logic_batt[1]/10.0}V")
            
            # Read temperatures
            temp1 = controller.ReadTemp(args.address)
            temp2 = controller.ReadTemp2(args.address)
            
            if temp1[0]:
                logger.info(f"Temperature 1: {temp1[1]/10.0}°C")
            if temp2[0]:
                logger.info(f"Temperature 2: {temp2[1]/10.0}°C")
            
            # Read error status
            error = controller.ReadError(args.address)
            if error[0]:
                logger.info(f"Error status: 0x{error[1]:08X}")
                if error[1] == 0:
                    logger.info("No errors detected")
                else:
                    # Interpret error bits - this will depend on your specific controller
                    logger.warning(f"Error detected! Code: 0x{error[1]:08X}")
                    # Here you could add specific error code interpretation
            
            # Read encoder status
            enc_status = controller.GetEncStatus(args.address)
            if enc_status[0]:
                logger.info(f"Encoder 1 status: 0x{enc_status[1]:02X}")
                logger.info(f"Encoder 2 status: 0x{enc_status[2]:02X}")
            
            # 2. Detailed Status (GetStatus command)
            logger.info("\n=== DETAILED STATUS INFORMATION ===")
            
            status = controller.GetStatus(args.address)
            if status[0]:
                logger.info("Controller Status:")
                logger.info(f"  Tick: {status[1]}")
                logger.info(f"  State: 0x{status[2]:08X}")
                logger.info(f"  Temperature 1: {status[3]/10.0}°C")
                logger.info(f"  Temperature 2: {status[4]/10.0}°C")
                logger.info(f"  Main Battery: {status[5]/10.0}V")
                logger.info(f"  Logic Battery: {status[6]/10.0}V")
                logger.info(f"  PWM Motor 1: {status[7]}")
                logger.info(f"  PWM Motor 2: {status[8]}")
                logger.info(f"  Current Motor 1: {status[9]}mA")
                logger.info(f"  Current Motor 2: {status[10]}mA")
                logger.info(f"  Encoder 1 Value: {status[11]}")
                logger.info(f"  Encoder 2 Value: {status[12]}")
                logger.info(f"  Setpoint Speed 1: {status[13]}")
                logger.info(f"  Setpoint Speed 2: {status[14]}")
                logger.info(f"  Speed 1: {status[15]}")
                logger.info(f"  Speed 2: {status[16]}")
                logger.info(f"  Speed Error 1: {status[17]}")
                logger.info(f"  Speed Error 2: {status[18]}")
                logger.info(f"  Position Error 1: {status[19]}")
                logger.info(f"  Position Error 2: {status[20]}")
            
            # 3. Read Current and PWM values directly
            logger.info("\n=== CURRENT AND PWM VALUES ===")
            
            currents = controller.ReadCurrents(args.address)
            if currents[0]:
                logger.info(f"Motor 1 current: {currents[1]}mA")
                logger.info(f"Motor 2 current: {currents[2]}mA")
            
            pwms = controller.ReadPWMs(args.address)
            if pwms[0]:
                # Convert to percentage for better readability
                pwm1_pct = pwms[1] * 100.0 / Basicmicro.MAX_DUTY
                pwm2_pct = pwms[2] * 100.0 / Basicmicro.MAX_DUTY
                logger.info(f"Motor 1 PWM: {pwms[1]} ({pwm1_pct:.1f}%)")
                logger.info(f"Motor 2 PWM: {pwms[2]} ({pwm2_pct:.1f}%)")
                            
            # 4. Continuous Monitoring
            if args.monitor:
                logger.info("\n=== STARTING CONTINUOUS MONITORING ===")
                logger.info("Press Ctrl+C to stop monitoring")
                
                # Start motor test if requested
                if args.motor_test:
                    logger.info("Starting motor test sequence during monitoring")
                    controller.DutyM1(args.address, 8192)  # 25% forward
                
                try:
                    while True:
                        # Read basic status
                        main_batt = controller.ReadMainBatteryVoltage(args.address)
                        currents = controller.ReadCurrents(args.address)
                        temps = controller.GetTemps(args.address)
                        speeds = controller.GetSpeeds(args.address)
                        encoders = controller.GetEncoders(args.address)
                        
                        # Log status in a compact format
                        status_line = "STATUS:"
                        if main_batt[0]:
                            status_line += f" Batt={main_batt[1]/10.0:.1f}V"
                        if temps[0]:
                            status_line += f" Temp1={temps[1]/10.0:.1f}°C Temp2={temps[2]/10.0:.1f}°C"
                        if currents[0]:
                            status_line += f" Curr1={currents[1]}mA Curr2={currents[2]}mA"
                        if speeds[0]:
                            status_line += f" Speed1={speeds[1]} Speed2={speeds[2]}"
                        if encoders[0]:
                            status_line += f" Enc1={encoders[1]} Enc2={encoders[2]}"
                            
                        logger.info(status_line)
                        
                        # Check for errors
                        error = controller.ReadError(args.address)
                        if error[0] and error[1] != 0:
                            logger.warning(f"ERROR DETECTED: 0x{error[1]:08X}")
                        
                        # Sleep between readings
                        time.sleep(1)
                
                except KeyboardInterrupt:
                    logger.info("Monitoring stopped by user")
                    if args.motor_test:
                        # Stop motors
                        controller.DutyM1(args.address, 0)
            
            logger.info("Status and diagnostics example completed")
            
    except Exception as e:
        logger.error(f"Error during test: {str(e)}")

if __name__ == "__main__":
    main()