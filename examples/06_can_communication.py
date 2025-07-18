#!/usr/bin/env python3
"""
CAN Bus Communication Example for Basicmicro motor controllers.

This example demonstrates:
- Sending CAN packets
- Receiving CAN packets
- Reading CAN buffer state
- Working with CANopen dictionary
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
    parser = argparse.ArgumentParser(description='Basicmicro CAN communication example')
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
            
            # 2. Sending CAN Packets
            logger.info("\n=== SENDING CAN PACKETS ===")
            
            # Example: Send a simple packet
            # Parameters:
            # - CAN ID (11-bit standard ID)
            # - RTR flag (Remote Transmission Request)
            # - Data bytes (up to 8 bytes)
            can_id = 0x123
            rtr = 0  # 0 = normal packet, 1 = remote request
            data = [0x01, 0x02, 0x03, 0x04]  # Example data payload
            
            logger.info(f"Sending CAN packet - ID: 0x{can_id:03X}, Data: {data}")
            result = controller.CANPutPacket(args.address, can_id, rtr, data)
            
            if result:
                logger.info("CAN packet sent successfully")
            else:
                logger.error("Failed to send CAN packet")
            
            # Example: Send a Remote Transmission Request (RTR) packet
            # An RTR packet requests data from another node
            can_id = 0x321
            rtr = 1  # RTR flag
            data = []  # Empty data for RTR
            
            logger.info(f"Sending RTR packet - ID: 0x{can_id:03X}")
            result = controller.CANPutPacket(args.address, can_id, rtr, data)
            
            if result:
                logger.info("RTR packet sent successfully")
            else:
                logger.error("Failed to send RTR packet")
            
            # 3. Receiving CAN Packets
            logger.info("\n=== RECEIVING CAN PACKETS ===")
            
            # Read all available packets
            while True:
                packet = controller.CANGetPacket(args.address)
                
                if packet[0]:
                    if packet[1]:
                        can_id = packet[2]
                        rtr = packet[3]
                        length = packet[4]
                        data = packet[5][:length]  # Only include valid data bytes
                        
                        # Format data bytes as hex
                        data_hex = " ".join([f"0x{b:02X}" for b in data])
                        
                        logger.info(f"Received packet {i+1}:")
                        logger.info(f"  ID: 0x{can_id:03X}")
                        logger.info(f"  RTR: {rtr}")
                        logger.info(f"  Length: {length}")
                        logger.info(f"  Data: {data_hex}")
                    else:   
                        logger.info(f"No more CAN packets available")
                        break
                else:
                    logger.error(f"Failed to read packet {i+1}")
            
            # Optional: Wait for incoming packets (timeout after 5 seconds)
            logger.info("\n=== WAITING FOR INCOMING CAN PACKETS ===")
            logger.info("Listening for incoming CAN packets for 5 seconds...")
            
            start_time = time.time()
            packet_count = 0

            # Read all available packets
            while True:
                packet = controller.CANGetPacket(args.address)
                
                if packet[0]:
                    if packet[1]:
                        can_id = packet[2]
                        rtr = packet[3]
                        length = packet[4]
                        data = packet[5][:length]
                        
                        # Format data bytes as hex
                        data_hex = " ".join([f"0x{b:02X}" for b in data])
                        
                        logger.info(f"Received packet:")
                        logger.info(f"  ID: 0x{can_id:03X}")
                        logger.info(f"  RTR: {rtr}")
                        logger.info(f"  Length: {length}")
                        logger.info(f"  Data: {data_hex}")
                        
                        packet_count += 1
                
                # Wait a bit before checking again
                time.sleep(0.1)
                if time.time()-start_time>5:
                    break
            
            logger.info(f"Listening complete. Received {packet_count} packets.")
            
            # 4. CANopen Dictionary Operations
            logger.info("\n=== CANOPEN DICTIONARY OPERATIONS ===")
            
            # Example: Read from local CANopen dictionary
            index = 0x1000  # Example: Device Type
            subindex = 0
            
            logger.info(f"Reading from CANopen dictionary - Index: 0x{index:04X}, Subindex: {subindex}")
            result = controller.CANOpenReadLocalDict(args.address, index, subindex)
            
            if result[0]:
                value = result[1]
                size = result[2]
                value_type = result[3]
                op_result = result[4]
                
                logger.info(f"Read successful:")
                logger.info(f"  Value: 0x{value:08X}")
                logger.info(f"  Size: {size}")
                logger.info(f"  Type: {value_type}")
                logger.info(f"  Result Code: 0x{op_result:08X}")
            else:
                logger.error("Failed to read from CANopen dictionary")
            
            # Example: Write to local CANopen dictionary
            # Note: This is just an example - only write to dictionary entries
            # that are writable and that you understand
            if False:  # Disabled by default for safety
                index = 0x2000  # Example: Application-specific parameter
                subindex = 1
                value = 0x12345678
                size = 4  # 4 bytes (32-bit)
                
                logger.info(f"Writing to CANopen dictionary - Index: 0x{index:04X}, Subindex: {subindex}, Value: 0x{value:08X}")
                result = controller.CANOpenWriteLocalDict(args.address, index, subindex, value, size)
                
                if result[0]:
                    op_result = result[1]
                    logger.info(f"Write successful, result code: 0x{op_result:08X}")
                else:
                    logger.error("Failed to write to CANopen dictionary")
            
            logger.info("CAN communication example completed")
            
    except Exception as e:
        logger.error(f"Error during test: {str(e)}")
        logger.exception("Exception details:")

if __name__ == "__main__":
    main()