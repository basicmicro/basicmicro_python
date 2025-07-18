"""
Basicmicro Interface Class for controlling Basicmicro motor controllers.
"""

import time
import random
import serial
import struct
import logging
from typing import Tuple, List, Dict, Any, Optional, Union, Callable

from basicmicro.commands import Commands
from basicmicro.utils import initialize_crc_table, calc_mixed
from basicmicro.types import *
from basicmicro.exceptions import PacketTimeoutError, CommunicationError, BasicmicroError

logger = logging.getLogger(__name__)


class Basicmicro:
    """
    Basicmicro Interface Class for controlling Basicmicro motor controllers.
    
    This class provides a comprehensive interface for communicating with
    Basicmicro motor controllers using the Basicmicro packet serial mode. It
    supports all major functions including motor control, encoder reading,
    and configuration settings.
    
    Basic usage:
    
        # Enable logging
        import logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )

        # Initialize the controller
        dev = Basicmicro("/dev/ttyACM0", 38400)  # Port and baud rate
        dev.Open()

        # Simple motor control
        address = 0x80  # Default address
        dev.DutyM1(address, 16384)  # Half speed forward for motor 1
        dev.DutyM2(address, -8192)  # Quarter speed backward for motor 2

        # Read encoder values
        enc1 = dev.ReadEncM1(address)
        if enc1[0]:  # Check if read was successful
            print(f"Encoder 1 count: {enc1[1]}")

        # Set velocity PID values
        dev.SetM1VelocityPID(address, kp=1.0, ki=0.5, kd=0.25, qpps=44000)
    
    See the Basicmicro user manual(s) for detailed command descriptions and parameters.
    """

    # Constants to improve readability and reduce magic numbers
    MAX_RETRY_COUNT = 3
    CRC_POLYNOMIAL = 0x1021
    SUCCESS = 1
    FAILURE = 0

    DEFAULT_ADDRESS = 0x80
    MIN_ADDRESS = 0x80
    MAX_ADDRESS = 0x87
    NVM_COMMIT_KEY = 0xE22EAB7A
    RESTORE_DEFAULTS_KEY = 0xE22EAB7A # Often the same as NVM key
    PID_FLOAT_SCALE_VEL = 65536.0
    PID_FLOAT_SCALE_POS = 1024.0
    LR_FLOAT_SCALE = 0x1000000  # For Set/Get M1/M2 LR
    VOLTAGE_SCALE = 10.0
    TEMP_SCALE = 10.0
    MAX_DUTY = 32767
    MIN_DUTY = -32767

    ESTOP_AUTO_RESET = 0x55
    ESTOP_SW_RESET = 0xAA
    ESTOP_HW_RESET = 0x00

    """
    RoboClaw Error and Warning Status Bit Definitions
    Translated from C #defines to Python constants
    """

    # Error Status Bits (errorStatus.lo - lower 16 bits)
    ERROR_NONE = 0x0000
    ERROR_ESTOP = 0x0001
    ERROR_TEMP = 0x0002
    ERROR_TEMP2 = 0x0004
    ERROR_MBATHIGH = 0x0008
    ERROR_LBATHIGH = 0x0010
    ERROR_LBATLOW = 0x0020

    # Error Status Bits (errorStatus.hi - upper 16 bits)
    ERROR_SPEED1 = 0x0100
    ERROR_SPEED2 = 0x0200
    ERROR_POS1 = 0x0400
    ERROR_POS2 = 0x0800
    ERROR_CURRENTM1 = 0x1000
    ERROR_CURRENTM2 = 0x2000
    ERROR_MBATLOW = 0x4000
    ERROR_MBATHIGH_HYST = 0x8000

    # Warning Status Bits (warnStatus.lo - lower 16 bits)
    WARN_NONE = 0x0000
    WARN_OVERCURRENTM1 = 0x0001
    WARN_OVERCURRENTM2 = 0x0002
    WARN_MBATHIGH = 0x0004
    WARN_MBATLOW = 0x0008
    WARN_TEMP = 0x0010
    WARN_TEMP2 = 0x0020
    WARN_S4 = 0x0040  # HOME, LIMITF, LIMITR, SAFESTOP, OFF
    WARN_S5 = 0x0080  # HOME, LIMITF, LIMITR, SAFESTOP, OFF

    # Warning Status Bits (warnStatus.hi - upper 16 bits)
    WARN_SPEED1 = 0x0100
    WARN_SPEED2 = 0x0200
    WARN_POS1 = 0x0400
    WARN_POS2 = 0x0800
    WARN_CAN = 0x1000
    WARN_BOOT = 0x2000
    WARN_OVERREGENM1 = 0x4000
    WARN_OVERREGENM2 = 0x8000

    # Error descriptions for better debugging
    ERROR_DESCRIPTIONS = {
        ERROR_ESTOP: "Emergency Stop",
        ERROR_TEMP: "Temperature Sensor 1 Error",
        ERROR_TEMP2: "Temperature Sensor 2 Error",
        ERROR_MBATHIGH: "Main Battery Voltage Too High",
        ERROR_LBATHIGH: "Logic Battery Voltage Too High",
        ERROR_LBATLOW: "Logic Battery Voltage Too Low",
        ERROR_SPEED1: "Motor 1 Speed Error",
        ERROR_SPEED2: "Motor 2 Speed Error",
        ERROR_POS1: "Motor 1 Position Error",
        ERROR_POS2: "Motor 2 Position Error",
        ERROR_CURRENTM1: "Motor 1 Current Error",
        ERROR_CURRENTM2: "Motor 2 Current Error",
        ERROR_MBATLOW: "Main Battery Voltage Too Low",
        ERROR_MBATHIGH_HYST: "Main Battery Voltage Too High (Hysteresis)",
    }

    WARNING_DESCRIPTIONS = {
        WARN_OVERCURRENTM1: "Motor 1 Overcurrent",
        WARN_OVERCURRENTM2: "Motor 2 Overcurrent",
        WARN_MBATHIGH: "Main Battery Voltage High Warning",
        WARN_MBATLOW: "Main Battery Voltage Low Warning",
        WARN_TEMP: "Temperature Warning",
        WARN_TEMP2: "Temperature 2 Warning",
        WARN_S4: "S4 Signal Warning (HOME/LIMITF/LIMITR/SAFESTOP/OFF)",
        WARN_S5: "S5 Signal Warning (HOME/LIMITF/LIMITR/SAFESTOP/OFF)",
        WARN_SPEED1: "Motor 1 Speed Warning",
        WARN_SPEED2: "Motor 2 Speed Warning",
        WARN_POS1: "Motor 1 Position Warning",
        WARN_POS2: "Motor 2 Position Warning",
        WARN_CAN: "CAN Bus Warning",
        WARN_BOOT: "Boot Warning",
        WARN_OVERREGENM1: "Motor 1 Over-Regeneration Warning",
        WARN_OVERREGENM2: "Motor 2 Over-Regeneration Warning",
    }

    def __init__(
        self, 
        comport: str, 
        rate: int, 
        timeout: float = 0.01, 
        retries: int = 2, 
        verbose: bool = False
    ) -> None:
        """Initializes the Basicmicro interface.
    
        Args:
            comport: The COM port to use (e.g., 'COM3' on Windows, '/dev/ttyACM0' on Linux)
            rate: The baud rate for the serial communication
            timeout: The timeout for serial communication in seconds
            retries: The number of retries for communication operations
            verbose: Enable detailed debug logging (sets logging level to DEBUG)
        """
        # Configure logger verbosity
        if verbose and logger.level > logging.DEBUG:
            logger.setLevel(logging.DEBUG)

        logger.debug(f"Initializing Basicmicro interface: port={comport}, rate={rate}, timeout={timeout}, retries={retries}")

        self._ST_Power = -1
        self._ST_Turn = -1
        self.comport = comport
        self.rate = rate
        self.timeout = timeout
        self._trystimeout = retries
        self._crc = 0
        self._port = None  # Initialize as None to handle property access before open

        self._connected = False

        # Pre-compute CRC table for faster CRC calculations
        self._CRC_TABLE = initialize_crc_table(self.CRC_POLYNOMIAL)

    def __enter__(self) -> 'Basicmicro':
        """
        Context manager enter method for use with 'with' statement.
    
        Returns:
            Basicmicro: The Basicmicro instance
            
        Raises:
            RuntimeError: If connection cannot be established
        """
        if not self.Open():
            logger.error("Failed to open connection in context manager")
            raise RuntimeError("Failed to open serial connection")
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit method for use with 'with' statement."""
        logger.debug("Exiting context manager, closing connection")
        self.close()

    def Open(self) -> bool:
        """Opens and configures the serial connection to the controller.
    
        Returns:
            bool: True if connection successful, False otherwise
    
        Raises:
            serial.SerialException: If there are issues with the serial port
            ValueError: If port parameters are invalid
        """
        try:
            # Close port if already open
            self.close()

            # Configure and open serial port
            self._port = serial.Serial(
                port=self.comport,
                baudrate=self.rate,
                timeout=1,
                write_timeout=1,
                inter_byte_timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )

            # Ensure port is open
            if not self._port.is_open:
                self._port.open()

            logger.debug("Serial port opened, clearing buffers")

            # Clear buffers
            try:
                self._port.reset_input_buffer()
                self._port.reset_output_buffer()
            except Exception as e:
                logger.warning(f"Error while clearing buffers: {str(e)}")

            self._connected = True
            return True
        
        except (serial.SerialException, ValueError) as e:
            logger.error(f"Error opening serial port: {str(e)}")
            self.close()
            return False
        except Exception as e:
            logger.error(f"Unexpected error opening serial port: {str(e)}")
            self.close()
            return False

    def reconnect(self) -> bool:
        """Attempts to close and reopen the serial connection.
        
        Returns:
            bool: True if reconnection successful, False otherwise
        """
        logger.info(f"Attempting to reconnect to {self.comport}")
        self.close()
        return self.Open()
    
    def close(self) -> None:
        """Closes the serial connection to the controller with improved cleanup."""
        logger.info(f"Closing connection to {self.comport}")
        try:
            if hasattr(self, '_port') and self._port is not None:
                try:
                    if self._port.is_open:
                        self._port.close()
                        logger.debug("Serial port successfully closed")
                except Exception as e:
                    logger.error(f"Error closing serial port: {str(e)}")
        except Exception as e:
            logger.error(f"Unexpected error during port cleanup: {str(e)}")
        finally:
            self._port = None
            self._connected = False
        
    def _is_port_ready(self) -> bool:
        """Checks if port exists and is open without raising exceptions.
        Returns True if the port is ready for operations.
        """
        return hasattr(self, '_port') and self._port is not None and self._port.is_open
    
    @property
    def is_connected(self) -> bool:
        """Returns the current connection status.
        
        Returns:
            bool: True if connected and port is ready
        """
        return self._connected and self._is_port_ready()
        
# CRC and core communication methods
    def crc_clear(self) -> None:
        """Clears the CRC value."""
        self._crc = 0
        
    def crc_update(self, data: int) -> None:
        """Updates the CRC value with the given data.
    
        Args:
            data: The data to update the CRC with
        """
        # Use a faster lookup table approach for CRC calculation
        self._crc = ((self._crc << 8) ^ self._CRC_TABLE[(self._crc >> 8) ^ (data & 0xFF)]) & 0xFFFF

    def _sendcommand(self, address: int, command: int) -> None:
        """Sends a command to the controller.
    
        Args:
            address: The address of the controller (0x80-0x87)
            command: The command to send
    
        Raises:
            CommunicationError: If sending the command fails due to serial port issues
        """
        if not self.is_connected:
            logger.error("Cannot send command: port not ready")
            raise CommunicationError("Serial port not open or not initialized")
        
        logger.debug(f"Sending command: address=0x{address:02x}, command=0x{command:02x}")
        self.crc_clear()
        data = [address, command]
        self.crc_update(data[0])
        self.crc_update(data[1])
        try:
            self._port.write(bytes(data))
        except Exception as e:
            logger.error(f"Failed to send command: {str(e)}")
            raise CommunicationError(f"Failed to send command: {str(e)}")
        
    def _readbyte(self) -> Tuple[bool, int]:
        """Reads a byte from the controller.
        
        Returns:
            Tuple[bool, int]: (success, value)
                success: True if read successful
                value: The byte value
        """
        if not self.is_connected:
            logger.debug("Cannot read byte: port not ready")
            return (False, 0)
        
        try:
            data = bytearray(self._port.read(1))
            if len(data) == 1:
                val = data[0] & 0xFF
                self.crc_update(val)
                return (True, val)  
            return (False, 0)
        except Exception as e:
            logger.error(f"Error reading byte: {str(e)}")
            return (False, 0)
        
    def _readword(self) -> Tuple[bool, int]:
        """Reads a 16-bit word from the controller.
        
        Returns:
            Tuple[bool, int]: (success, value)
                success: True if read successful
                value: The word value
        """
        if not self.is_connected:
            logger.debug("Cannot read word: port not ready")
            return (False, 0)
        
        try:
            data = bytearray(self._port.read(2))
            if len(data) == 2:
                self.crc_update(data[0])
                self.crc_update(data[1])
                return (True, (data[0] << 8) | data[1])
            return (False, 0)
        except Exception as e:
            logger.error(f"Error reading word: {str(e)}")
            return (False, 0)

    def _readlong(self) -> Tuple[bool, int]:
        """Reads a 32-bit long value from the controller.
        
        Returns:
            Tuple[bool, int]: (success, value)
                success: True if read successful
                value: The long value
        """
        if not self.is_connected:
            logger.debug("Cannot read long: port not ready")
            return (False, 0)
        
        try:
            data = bytearray(self._port.read(4))
            if len(data) == 4:
                self.crc_update(data[0])
                self.crc_update(data[1])
                self.crc_update(data[2])
                self.crc_update(data[3])
                return (True, (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3])
            return (False, 0)
        except Exception as e:
            logger.error(f"Error reading long: {str(e)}")
            return (False, 0)

    def _readslong(self) -> Tuple[bool, int]:
        """Reads a signed 32-bit long value from the controller.
        
        Returns:
            Tuple[bool, int]: (success, value)
                success: True if read successful
                value: The signed long value
        """
        val = self._readlong()
        if val[0]:
            if val[1] & 0x80000000:
                return (val[0], val[1] - 0x100000000)
            return (val[0], val[1])
        return (False, 0)

    def _writebyte(self, val: int) -> None:
        """Writes a byte to the controller.
        
        Args:
            val: The byte value to write
            
        Raises:
            CommunicationError: If writing the byte fails
        """
        if not self.is_connected:
            logger.debug("Cannot write byte: port not ready")
            raise CommunicationError("Serial port not connected")
        
        data = bytearray([val & 0xFF])
        self.crc_update(data[0])
        try:
            self._port.write(data)
        except Exception as e:
            logger.error(f"Error writing byte: {str(e)}")
            raise CommunicationError(f"Error writing byte: {str(e)}")

    def _writesbyte(self, val: int) -> None:
        """Writes a signed byte to the controller.
        
        Args:
            val: The signed byte value to write
        """
        self._writebyte(val)

    def _writeword(self, val: int) -> None:
        """Writes a 16-bit word to the controller.
        
        Args:
            val: The word value to write
            
        Raises:
            CommunicationError: If writing the word fails
        """
        if not self.is_connected:
            logger.debug("Cannot write word: port not ready")
            raise CommunicationError("Serial port not connected")
        
        data = bytearray([(val >> 8) & 0xFF, val & 0xFF])
        self.crc_update(data[0])
        self.crc_update(data[1])
        try:
            self._port.write(data)
        except Exception as e:
            logger.error(f"Error writing word: {str(e)}")
            raise CommunicationError(f"Error writing word: {str(e)}")
        
    def _writesword(self, val: int) -> None:
        """Writes a signed 16-bit word to the controller.
        
        Args:
            val: The signed word value to write
        """
        self._writeword(val)

    def _writelong(self, val: int) -> None:
        """Writes a 32-bit long value to the controller.
        
        Args:
            val: The long value to write
            
        Raises:
            CommunicationError: If writing the long fails
        """
        if not self.is_connected:
            logger.debug("Cannot write long: port not ready")
            raise CommunicationError("Serial port not connected")
        
        data = bytearray([(val >> 24) & 0xFF, (val >> 16) & 0xFF, (val >> 8) & 0xFF, val & 0xFF])
        self.crc_update(data[0])
        self.crc_update(data[1])
        self.crc_update(data[2])
        self.crc_update(data[3])
        try:
            self._port.write(data)
        except Exception as e:
            logger.error(f"Error writing long: {str(e)}")
            raise CommunicationError(f"Error writing long: {str(e)}")

    def _writeslong(self, val: int) -> None:
        """Writes a signed 32-bit long value to the controller.
        
        Args:
            val: The signed long value to write
        """
        self._writelong(val)

    def _write(self, address: int, cmd: int, *args, **kwargs) -> bool:
        """
        Generic write method that sends a command to the controller with variable arguments.

        This method sends the address, command, optional data values, and CRC16 checksum to
        the controller. If no arguments are provided, it only sends the address, command, 
        and CRC16.

        Args:
            address: The address of the controller (0x80-0x87)
            cmd: The command to send
            *args: Variable number of arguments to write
            **kwargs: Keyword arguments that specify how to write each argument
                - types (list or str): Specifies the data type for each argument
                    Supported types: 'byte', 'sbyte', 'word', 'sword', 'long', 'slong'
                    Can be provided as a comma-separated string or a list
                    Must match the number of arguments in *args

        Returns:
            bool: True if successful, False otherwise
        
        Raises:
            ValueError: If number of type specifications doesn't match number of arguments
            ValueError: If an unsupported type is specified
        """
        if not self.is_connected:
            logger.error("Cannot perform write operation: not connected")
            return False
        
        logger.debug(f"Write: address=0x{address:02x}, cmd=0x{cmd:02x}, args={args}")
        # Determine the types of arguments
        arg_types = kwargs.get('types', ['byte'] * len(args))
        if isinstance(arg_types, str):
            arg_types = arg_types.split(',')

        if len(arg_types) != len(args):
            raise ValueError(f"Number of type specifications ({len(arg_types)}) must match number of arguments ({len(args)})")

        # Start retry loop
        for attempt in range(self._trystimeout):
            try:
                # Send command (address and command)
                self._sendcommand(address, cmd)

                # Write each argument according to its type
                for arg, arg_type in zip(args, arg_types):
                    arg_type = arg_type.lower()
                    if arg_type == 'byte':
                        self._writebyte(arg)
                    elif arg_type == 'sbyte':
                        self._writesbyte(arg)
                    elif arg_type == 'word':
                        self._writeword(arg)
                    elif arg_type == 'sword':
                        self._writesword(arg)
                    elif arg_type == 'long':
                        self._writelong(arg)
                    elif arg_type == 'slong':
                        self._writeslong(arg)
                    else:
                        raise ValueError(f"Unsupported type: {arg_type}")

                # Write checksum and verify we received acknowledgment
                if self._writechecksum():
                    return True
                    
                logger.debug(f"Attempt {attempt+1}/{self._trystimeout} failed for cmd 0x{cmd:02x}")
                
            except serial.SerialTimeoutException as e:
                logger.debug(f"Serial timeout on attempt {attempt+1}: {str(e)}")
            except Exception as e:
                logger.warning(f"Error on write attempt {attempt+1}: {str(e)}")

        logger.warning(f"Write failed after {self._trystimeout} attempts: address=0x{address:02x}, cmd=0x{cmd:02x}")

        # Raise timeout exception after all retries are exhausted
        raise PacketTimeoutError(f"Timeout writing to address 0x{address:02x}, cmd 0x{cmd:02x} after {self._trystimeout} attempts")
        
        return False

    def _writechecksum(self) -> bool:
        """Writes 16-bit CRC and reads one byte (ack) from the controller.
        
        Returns:
            bool: True if successful
        """
        logger.debug(f"Writing checksum: 0x{self._crc & 0xFFFF:04x}")
        self._writeword(self._crc & 0xFFFF)
        val = self._readbyte()
        if val[0]:
            return True
        logger.debug("No acknowledgment received")
        return False

    def _read(self, address: int, cmd: int, **kwargs) -> Tuple[bool, ...]:
        """
        Generic read method that reads data from the controller based on specified types.

        This method sends a command to the controller and reads back data types
        specified in the types parameter, verifying the CRC16 checksum.

        Args:
            address: The address of the controller (0x80-0x87)
            cmd: The command to send
            **kwargs: Keyword arguments that specify how to read data
                - types (list or str): Specifies the data types to read
                    Supported types: 'byte', 'sbyte', 'word', 'sword', 'long', 'slong'
                    Can be provided as a comma-separated string or a list
                - retry_on_error (bool): Whether to retry on error, defaults to True

        Returns:
            Tuple[bool, ...]: (success, *values)
                success: True if read successful
                *values: The values read according to the specified types
            
        Raises:
            ValueError: If an unsupported type is specified
        """
        if not self.is_connected:
            logger.error("Cannot perform read operation: not connected")
            return tuple([self.FAILURE] + [0] * len(kwargs.get('types', [])))
        
        logger.debug(f"Read: address=0x{address:02x}, cmd=0x{cmd:02x}")
        retry_on_error = kwargs.get('retry_on_error', True)
        arg_types = kwargs.get('types', [])
        if isinstance(arg_types, str):
            arg_types = arg_types.split(',')

        def read_value(arg_type: str) -> Tuple[bool, int]:
            if arg_type == 'byte':
                return self._readbyte()
            elif arg_type == 'sbyte':
                val = self._readbyte()
                return (val[0], val[1] - 0x100 if val[0] and val[1] & 0x80 else val[1])
            elif arg_type == 'word':
                return self._readword()
            elif arg_type == 'sword':
                val = self._readword()
                return (val[0], val[1] - 0x10000 if val[0] and val[1] & 0x8000 else val[1])
            elif arg_type == 'long':
                return self._readlong()
            elif arg_type == 'slong':
                return self._readslong()
            else:
                raise ValueError(f"Unsupported type: {arg_type}")

        trys = self._trystimeout if retry_on_error else 1
        while trys:
            try:
                self._port.flushInput()
            except Exception as e:
                logger.warning(f"Error flushing input buffer: {str(e)}")
                # Continue trying despite the error
            
            self._sendcommand(address, cmd)
            result = [self.SUCCESS]

            for arg_type in arg_types:
                val = read_value(arg_type)
                if not val[0]:
                    break
                result.append(val[1])
            else:
                crc = self._readchecksumword()
                if crc[0] and self._crc & 0xFFFF == crc[1] & 0xFFFF:
                    return tuple(result)

            trys -= 1

        logger.warning(f"Read failed after {self._trystimeout} attempts: address=0x{address:02x}, cmd=0x{cmd:02x}")
        
        if retry_on_error:
            # Only raise the timeout exception if retries were enabled and all failed
            raise PacketTimeoutError(f"Timeout reading from address 0x{address:02x}, cmd 0x{cmd:02x} after {self._trystimeout} attempts")
            
        return tuple([self.FAILURE] + [0] * len(arg_types))

    def _readchecksumword(self) -> Tuple[bool, int]:
        """Reads a 16-bit checksum word from the controller.

        Returns:
            Tuple[bool, int]: (success, checksum)
                success: True if read successful
                checksum: The checksum value
        """
        logger.debug("Reading checksum word")
        try:
            # Try to read 2 bytes from the serial port
            data = bytearray(self._port.read(2))

            # Check if we received exactly 2 bytes (complete checksum)
            if len(data) == 2:
                # Combine the bytes into a 16-bit word (big-endian)
                # First byte is high byte, second is low byte
                checksum = ((data[0] & 0xFF) << 8) | (data[1] & 0xFF)
                return (self.SUCCESS, checksum)

            # If we didn't get 2 bytes, the read failed
            logger.debug("Failed to read checksum: incomplete data")
            return (self.FAILURE, 0)
        except (serial.SerialException, ValueError) as e:
            # Handle specific exceptions related to serial communication
            logger.error(f"Serial error while reading checksum: {str(e)}")
            return (self.FAILURE, 0)
        except Exception as e:
            # Catch any other unexpected exceptions
            logger.error(f"Unexpected error reading checksum: {str(e)}")
            return (self.FAILURE, 0)

    def _ST_Single(self, cmd: int, address: int, power: int) -> bool:
        """Utility Function for Stubs. Sets the power for a single motor.
        
        Args:
            cmd: The command to send
            address: The address of the controller
            power: The power value to set
        
        Returns:
            bool: True if successful
        """
        self._ST_Power = -128
        self._ST_Turn = -128

        power = power & 0x7F      
        if cmd == Commands.M17BIT or cmd == Commands.M27BIT:
            if power == 0: 
                power = 1  # Keep Fwd/Bwd power range symmetric
            power = (power * 2) - 128
        if cmd == Commands.M1BACKWARD or cmd == Commands.M2BACKWARD:
            power = -power

        # power = +-127 at this point
            
        duty = power * self.MAX_DUTY / 127
        if cmd == Commands.M1FORWARD or cmd == Commands.M1BACKWARD or cmd == Commands.M17BIT:
            return self.DutyAccelM1(address, 0, duty)
        else:
            return self.DutyAccelM2(address, 0, duty)

    def _ST_Mixed(self, cmd: int, address: int, power: int) -> bool:
        """Utility Function for Stubs. Sets the power and turn for mixed mode.
        
        Args:
            cmd: The command to send
            address: The address of the controller
            power: The power value to set
        
        Returns:
            bool: True if successful
        """
        power = power & 0x7F
        if cmd < Commands.MIXEDFB:
            # Regular mode calculation
            if cmd & 0x1: 
                power = -power
        else:
            # 7bit mode calculation
            if power == 0: 
                power = 1  # Keep Fwd/Bwd power range symmetric (eg +-127 from center)
            power = (power * 2) - 128

        # temp == +-127 at this point
            
        if cmd == Commands.MIXEDRIGHT or cmd == Commands.MIXEDLEFT or cmd == Commands.MIXEDLR:
            self._ST_Turn = power
        else:
            self._ST_Power = power

        if self._ST_Power != -128 and self._ST_Turn != -128:
            duties = calc_mixed(self._ST_Power * self.MAX_DUTY / 127, self._ST_Turn * self.MAX_DUTY / 127)
            return self.DutyM1M2(address, duties[0], duties[1])
        return False  # Both power and turn commands must be used at least once. Will return false until then
    
    def SendRandomData(self, cnt: int) -> None:
        """Sends random data to the controller. Used for testing only
    
        Args:
            cnt: The number of random bytes to send
        """
        if not self.is_connected:
            logger.error("Cannot send random data: not connected")
            return

        try:
            for _ in range(0, cnt):
                byte = random.getrandbits(8)
                self._port.write(bytes([byte & 0xFF]))
            return
        except Exception as e:
            logger.error(f"Error sending random data: {str(e)}")   

    def decode_error_status(error_status):
        """
        Decode a 16-bit error status value into human-readable error messages.
        
        Args:
            error_status (int): 16-bit error status value from RoboClaw
            
        Returns:
            tuple: (has_errors, error_list)
                has_errors (bool): True if any errors are present
                error_list (list): List of error description strings
        """
        errors = []
        
        if error_status == ERROR_NONE:
            return False, ["No errors"]
        
        # Check each error bit
        for error_bit, description in ERROR_DESCRIPTIONS.items():
            if error_status & error_bit:
                errors.append(f"ERROR: {description}")
        
        return len(errors) > 0, errors


    def decode_warning_status(warning_status):
        """
        Decode a 16-bit warning status value into human-readable warning messages.
        
        Args:
            warning_status (int): 16-bit warning status value from RoboClaw
            
        Returns:
            tuple: (has_warnings, warning_list)
                has_warnings (bool): True if any warnings are present
                warning_list (list): List of warning description strings
        """
        warnings = []
        
        if warning_status == WARN_NONE:
            return False, ["No warnings"]
        
        # Check each warning bit
        for warning_bit, description in WARNING_DESCRIPTIONS.items():
            if warning_status & warning_bit:
                warnings.append(f"WARNING: {description}")
        
        return len(warnings) > 0, warnings


    def decode_full_status(combined_status):
        """
        Decode a 32-bit combined status value from ReadError command.
        High 16 bits = warnings, Low 16 bits = errors
        
        Args:
            combined_status (int): 32-bit status value from RoboClaw ReadError command
            
        Returns:
            tuple: (has_errors, has_warnings, error_list, warning_list)
                has_errors (bool): True if any errors are present
                has_warnings (bool): True if any warnings are present
                error_list (list): List of error description strings
                warning_list (list): List of warning description strings
        """
        # Extract error bits (low 16 bits)
        error_bits = combined_status & 0xFFFF
        
        # Extract warning bits (high 16 bits)
        warning_bits = (combined_status >> 16) & 0xFFFF
        
        # Decode errors and warnings
        has_errors, error_list = decode_error_status(error_bits)
        has_warnings, warning_list = decode_warning_status(warning_bits)
        
        return has_errors, has_warnings, error_list, warning_list


    def analyze_roboclaw_status(controller, address):
        """
        Read and analyze both error and warning status from RoboClaw ReadError command.
        
        Args:
            controller: Basicmicro controller object
            address (int): RoboClaw address
            
        Returns:
            dict: Status analysis results
        """
        result = {
            'combined_status_raw': 0,
            'error_status_raw': 0,
            'warning_status_raw': 0,
            'has_errors': False,
            'has_warnings': False,
            'errors': [],
            'warnings': [],
            'read_success': False
        }
        
        try:
            # Read combined error/warning status
            status_result = controller.ReadError(address)
            if status_result[0]:  # Success
                result['combined_status_raw'] = status_result[1]
                result['error_status_raw'] = status_result[1] & 0xFFFF
                result['warning_status_raw'] = (status_result[1] >> 16) & 0xFFFF
                
                # Decode the combined status
                has_errors, has_warnings, error_list, warning_list = decode_combined_status(status_result[1])
                
                result['has_errors'] = has_errors
                result['has_warnings'] = has_warnings
                result['errors'] = error_list
                result['warnings'] = warning_list
                result['read_success'] = True
            else:
                result['errors'] = ["Failed to read error status"]
                
        except Exception as e:
            result['errors'] = [f"Exception reading status: {str(e)}"]
        
        return result

    def print_status_analysis(status_result):
        """
        Print a formatted analysis of RoboClaw status.
        
        Args:
            status_result (dict): Result from analyze_roboclaw_status()
        """
        print(f"=== RoboClaw Status Analysis ===")
        print(f"Combined Status Raw: 0x{status_result['combined_status_raw']:08X}")
        print(f"Error Bits (Low 16):  0x{status_result['error_status_raw']:04X}")
        print(f"Warning Bits (High 16): 0x{status_result['warning_status_raw']:04X}")
        print()
        
        if status_result['has_errors']:
            print("ERRORS DETECTED:")
            for error in status_result['errors']:
                print(f"  • {error}")
            print()
        else:
            print("✓ No errors detected")
            print()
        
        if status_result['has_warnings']:
            print("WARNINGS DETECTED:")
            for warning in status_result['warnings']:
                print(f"  • {warning}")
            print()
        else:
            print("✓ No warnings detected")
            print()
    
    # Deprecated functions preserved for backward compatibility
    def ForwardM1(self, address: int, val: int) -> bool:        
        """
        Sets the power for motor 1 to move forward.

        Args:
            address: The address of the controller.
            val: The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Single(Commands.M1FORWARD, address, val)

    def BackwardM1(self, address: int, val: int) -> bool:
        """
        Sets the power for motor 1 to move backward.

        Args:
            address: The address of the controller.
            val: The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Single(Commands.M1BACKWARD, address, val)

    def SetMinVoltageMainBattery(self, address: int, val: int) -> bool:
        """Deprecated: Sets the minimum voltage for the main battery.

        Args:
            address: The address of the controller.
            val: The minimum voltage value to set.

        Returns:
            bool: Always returns False.
        """
        return False  # Deprecated

    def SetMaxVoltageMainBattery(self, address: int, val: int) -> bool:
        """Deprecated: Sets the maximum voltage for the main battery.

        Args:
            address: The address of the controller.
            val: The maximum voltage value to set.

        Returns:
            bool: Always returns False.
        """
        return False  # Deprecated

    def ForwardM2(self, address: int, val: int) -> bool:
        """
        Sets the power for motor 2 to move forward.

        Args:
            address: The address of the controller.
            val: The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Single(Commands.M2FORWARD, address, val)

    def BackwardM2(self, address: int, val: int) -> bool:
        """
        Sets the power for motor 2 to move backward.

        Args:
            address: The address of the controller.
            val: The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Single(Commands.M2BACKWARD, address, val)

    def ForwardBackwardM1(self, address: int, val: int) -> bool:
        """
        Sets the power for motor 1 to move forward or backward in 7-bit mode.

        Args:
            address: The address of the controller.
            val: The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Single(Commands.M17BIT, address, val)

    def ForwardBackwardM2(self, address: int, val: int) -> bool:
        """
        Sets the power for motor 2 to move forward or backward in 7-bit mode.

        Args:
            address: The address of the controller.
            val: The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Single(Commands.M27BIT, address, val)

    def ForwardMixed(self, address: int, val: int) -> bool:
        """
        Sets the power for mixed mode to move forward.

        Args:
            address: The address of the controller.
            val: The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Mixed(Commands.MIXEDFORWARD, address, val)

    def BackwardMixed(self, address: int, val: int) -> bool:
        """
        Sets the power for mixed mode to move backward.

        Args:
            address: The address of the controller.
            val: The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Mixed(Commands.MIXEDBACKWARD, address, val)

    def TurnRightMixed(self, address: int, val: int) -> bool:
        """
        Sets the power for mixed mode to turn right.

        Args:
            address: The address of the controller.
            val: The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Mixed(Commands.MIXEDRIGHT, address, val)

    def TurnLeftMixed(self, address: int, val: int) -> bool:
        """
        Sets the power for mixed mode to turn left.

        Args:
            address: The address of the controller.
            val: The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Mixed(Commands.MIXEDLEFT, address, val)

    def ForwardBackwardMixed(self, address: int, val: int) -> bool:
        """
        Sets the power for mixed mode to move forward or backward.

        Args:
            address: The address of the controller.
            val: The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Mixed(Commands.MIXEDFB, address, val)

    def LeftRightMixed(self, address: int, val: int) -> bool:
        """
        Sets the power for mixed mode to move left or right.

        Args:
            address: The address of the controller.
            val: The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Mixed(Commands.MIXEDLR, address, val)
        
#Packet Serial Commands
    def SetTimeout(self, address: int, timeout: float) -> bool:
        """
        Sets the timeout for motor 1 encoder.

        Args:
            address: The address of the controller.
            timeout: The timeout value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETTIMEOUT, int(timeout * 10), types=["byte"])

    def GetTimeout(self, address: int) -> TimeoutResult:
        """
        Reads the timeout for motor 1 encoder.

        Args:
            address: The address of the controller.

        Returns:
            TimeoutResult: (success, timeout)
                success: True if read successful.
                timeout: The timeout value.
        """
        val = self._read(address, Commands.GETTIMEOUT, types=["byte"])
        if val[0]:
            return (True, float(val[1]) / 10)
        return (False, 0)

    def ReadEncM1(self, address: int) -> EncoderResult:
        """
        Reads the encoder count for motor 1.

        Args:
            address: The address of the controller.

        Returns:
            EncoderResult: (success, count, status)
                success: True if read successful.
                count: The encoder count.
                status: The status byte.
        """
        return self._read(address, Commands.GETM1ENC, types=["long", "byte"])

    def ReadEncM2(self, address: int) -> EncoderResult:
        """
        Reads the encoder count for motor 2.

        Args:
            address: The address of the controller.

        Returns:
            EncoderResult: (success, count, status)
                success: True if read successful.
                count: The encoder count.
                status: The status byte.
        """
        return self._read(address, Commands.GETM2ENC, types=["long", "byte"])

    def ReadSpeedM1(self, address: int) -> SpeedResult:
        """
        Reads the speed for motor 1.

        Args:
            address: The address of the controller.

        Returns:
            SpeedResult: (success, speed, status)
                success: True if read successful.
                speed: The speed value.
                status: The status byte.
        """
        return self._read(address, Commands.GETM1SPEED, types=["long", "byte"])

    def ReadSpeedM2(self, address: int) -> SpeedResult:
        """
        Reads the speed for motor 2.

        Args:
            address: The address of the controller.

        Returns:
            SpeedResult: (success, speed, status)
                success: True if read successful.
                speed: The speed value.
                status: The status byte.
        """
        return self._read(address, Commands.GETM2SPEED, types=["long", "byte"])

    def ResetEncoders(self, address: int) -> bool:
        """
        Resets the encoders for both motors.

        Args:
            address: The address of the controller.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.RESETENC)

    def ReadVersion(self, address: int) -> VersionResult:
        """
        Reads the firmware version of the controller.

        This method attempts to read the firmware version string from the controller
        with multiple retries if needed. It parses the raw byte response into a string
        and verifies the checksum.

        Args:
            address: The address of the controller (0x80-0x87)

        Returns:
            VersionResult: (success, version)
                success: True if read successful
                version: The firmware version as a string
        """
        logger.debug(f"Reading firmware version from address=0x{address:02x}")
        for _ in range(self._trystimeout):
            try:
                self._port.flushInput()
            except Exception as e:
                logger.warning(f"Error flushing input buffer during version read: {str(e)}")
                # Continue trying despite the error
            self._sendcommand(address, Commands.GETVERSION)
            version = []
            passed = True
            for _ in range(48):
                try:
                    data = self._port.read(1)
                    if data:
                        self.crc_update(data[0])
                        if data[0] == 0:
                            break
                        version.append(chr(data[0]))
                    else:
                        logger.debug("Timeout while reading version string")
                        passed = False
                        break
                except Exception as e:
                    logger.debug(f"Error reading version character: {str(e)}")
                    passed = False
                    break
            if passed:
                crc = self._readchecksumword()
                if crc[0] and self._crc & 0xFFFF == crc[1] & 0xFFFF:
                    return (True, ''.join(version))
                else:
                    logger.debug(f"CRC check failed: received=0x{crc[1]:04x}, calculated=0x{self._crc & 0xFFFF:04x}")

            logger.debug("Retrying version read after short delay")
            time.sleep(0.01)
    
        logger.warning(f"Failed to read version from address=0x{address:02x} after {self._trystimeout} attempts")
        return (False, "")

    def SetEncM1(self, address: int, cnt: int) -> bool:
        """
        Sets the encoder count for motor 1.

        Args:
            address: The address of the controller.
            cnt: The encoder count to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETM1ENCCOUNT, cnt, types=["long"])

    def SetEncM2(self, address: int, cnt: int) -> bool:
        """
        Sets the encoder count for motor 2.

        Args:
            address: The address of the controller.
            cnt: The encoder count to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETM2ENCCOUNT, cnt, types=["long"])

    def ReadMainBatteryVoltage(self, address: int) -> ReadResult:
        """
        Reads the main battery voltage.

        Args:
            address: The address of the controller.

        Returns:
            ReadResult: (success, voltage)
                success: True if read successful.
                voltage: The main battery voltage.
        """
        return self._read(address, Commands.GETMBATT, types=["word"])

    def ReadLogicBatteryVoltage(self, address: int) -> ReadResult:
        """
        Reads the logic battery voltage.

        Args:
            address: The address of the controller.

        Returns:
            ReadResult: (success, voltage)
                success: True if read successful.
                voltage: The logic battery voltage.
        """
        return self._read(address, Commands.GETLBATT, types=["word"])

    def SetMinVoltageLogicBattery(self, address: int, val: int) -> bool:
        """
        Deprecated: Sets the minimum voltage for the logic battery.

        Args:
            address: The address of the controller.
            val: The minimum voltage value to set.

        Returns:
            bool: Always returns False.
        """
        return False  # deprecated

    def SetMaxVoltageLogicBattery(self, address: int, val: int) -> bool:
        """
        Deprecated: Sets the maximum voltage for the logic battery.

        Args:
            address: The address of the controller.
            val: The maximum voltage value to set.

        Returns:
            bool: Always returns False.
        """
        return False  # deprecated

    def SetM1VelocityPID(self, address: int, p: float, i: float, d: float, qpps: int) -> bool:
        """
        Sets the velocity PID constants for motor 1.

        Args:
            address: The address of the controller.
            p: The proportional constant.
            i: The integral constant.
            d: The derivative constant.
            qpps: The speed in quadrature pulses per second.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETM1PID, int(d * self.PID_FLOAT_SCALE_VEL), int(p * self.PID_FLOAT_SCALE_VEL), int(i * self.PID_FLOAT_SCALE_VEL), qpps, types=["long", "long", "long", "long"])

    def SetM2VelocityPID(self, address: int, p: float, i: float, d: float, qpps: int) -> bool:
        """
        Sets the velocity PID constants for motor 2.

        Args:
            address: The address of the controller.
            p: The proportional constant.
            i: The integral constant.
            d: The derivative constant.
            qpps: The speed in quadrature pulses per second.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETM2PID, int(d * self.PID_FLOAT_SCALE_VEL), int(p * self.PID_FLOAT_SCALE_VEL), int(i * self.PID_FLOAT_SCALE_VEL), qpps, types=["long", "long", "long", "long"])

    def ReadISpeedM1(self, address: int) -> SpeedResult:
        """
        Reads the instantaneous speed for motor 1.

        Args:
            address: The address of the controller.

        Returns:
            SpeedResult: (success, speed, status)
                success: True if read successful.
                speed: The instantaneous speed value.
                status: The status byte.
        """
        return self._read(address, Commands.GETM1ISPEED, types=["long", "byte"])

    def ReadISpeedM2(self, address: int) -> SpeedResult:
        """
        Reads the instantaneous speed for motor 2.

        Args:
            address: The address of the controller.

        Returns:
            SpeedResult: (success, speed, status)
                success: True if read successful.
                speed: The instantaneous speed value.
                status: The status byte.
        """
        return self._read(address, Commands.GETM2ISPEED, types=["long", "byte"])
        
    def DutyM1(self, address: int, val: int) -> bool:
        """
        Sets the duty cycle for motor 1.

        Args:
            address: The address of the controller.
            val: The duty cycle value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.M1DUTY, val, types=["sword"])

    def DutyM2(self, address: int, val: int) -> bool:
        """
        Sets the duty cycle for motor 2.

        Args:
            address: The address of the controller.
            val: The duty cycle value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.M2DUTY, val, types=["sword"])

    def DutyM1M2(self, address: int, m1: int, m2: int) -> bool:
        """
        Sets the duty cycle for both motors simultaneously.

        Args:
            address: The address of the controller (0x80-0x87)
            m1: The duty cycle value for motor 1 (-32767 to +32767)
                Positive values rotate forward, negative values rotate backward
            m2: The duty cycle value for motor 2 (-32767 to +32767)
                Positive values rotate forward, negative values rotate backward

        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.MIXEDDUTY, m1, m2, types=["sword", "sword"])

    def SpeedM1(self, address: int, val: int) -> bool:
        """
        Sets the speed for motor 1.

        Args:
            address: The address of the controller.
            val: The speed value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.M1SPEED, val, types=["slong"])

    def SpeedM2(self, address: int, val: int) -> bool:
        """
        Sets the speed for motor 2.

        Args:
            address: The address of the controller.
            val: The speed value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.M2SPEED, val, types=["slong"])

    def SpeedM1M2(self, address: int, m1: int, m2: int) -> bool:
        """
        Sets the speed for both motors.

        Args:
            address: The address of the controller.
            m1: The speed value for motor 1.
            m2: The speed value for motor 2.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.MIXEDSPEED, m1, m2, types=["slong", "slong"])

    def SpeedAccelM1(self, address: int, accel: int, speed: int) -> bool:
        """
        Sets the acceleration and speed for motor 1.

        Args:
            address: The address of the controller.
            accel: The acceleration value to set.
            speed: The speed value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.M1SPEEDACCEL, accel, speed, types=["long", "slong"])

    def SpeedAccelM2(self, address: int, accel: int, speed: int) -> bool:
        """
        Sets the acceleration and speed for motor 2.

        Args:
            address: The address of the controller.
            accel: The acceleration value to set.
            speed: The speed value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.M2SPEEDACCEL, accel, speed, types=["long", "slong"])

    def SpeedAccelM1M2(self, address: int, accel: int, speed1: int, speed2: int) -> bool:
        """
        Sets the acceleration and speed for both motors.

        Args:
            address: The address of the controller.
            accel: The acceleration value to set.
            speed1: The speed value for motor 1.
            speed2: The speed value for motor 2.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.MIXEDSPEEDACCEL, accel, speed1, speed2, types=["long", "slong", "slong"])

    def SpeedDistanceM1(self, address: int, speed: int, distance: int, buffer: int) -> bool:
        """
        Sets the speed and distance for motor 1.

        Args:
            address: The address of the controller.
            speed: The speed value to set.
            distance: The distance value to set.
            buffer: The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.M1SPEEDDIST, speed, distance, buffer, types=["slong", "long", "byte"])

    def SpeedDistanceM2(self, address: int, speed: int, distance: int, buffer: int) -> bool:
        """
        Sets the speed and distance for motor 2.

        Args:
            address: The address of the controller.
            speed: The speed value to set.
            distance: The distance value to set.
            buffer: The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.M2SPEEDDIST, speed, distance, buffer, types=["slong", "long", "byte"])

    def SpeedDistanceM1M2(self, address: int, speed1: int, distance1: int, speed2: int, distance2: int, buffer: int) -> bool:
        """
        Sets the speed and distance for both motors.

        Args:
            address: The address of the controller.
            speed1: The speed value for motor 1.
            distance1: The distance value for motor 1.
            speed2: The speed value for motor 2.
            distance2: The distance value for motor 2.
            buffer: The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.MIXEDSPEEDDIST, speed1, distance1, speed2, distance2, buffer, types=["slong", "long", "slong", "long", "byte"])

    def SpeedAccelDistanceM1(self, address: int, accel: int, speed: int, distance: int, buffer: int) -> bool:
        """
        Sets the acceleration, speed, and distance for motor 1.

        Args:
            address: The address of the controller.
            accel: The acceleration value to set.
            speed: The speed value to set.
            distance: The distance value to set.
            buffer: The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.M1SPEEDACCELDIST, accel, speed, distance, buffer, types=["long", "slong", "long", "byte"])

    def SpeedAccelDistanceM2(self, address: int, accel: int, speed: int, distance: int, buffer: int) -> bool:
        """
        Sets the acceleration, speed, and distance for motor 2.

        Args:
            address: The address of the controller.
            accel: The acceleration value to set.
            speed: The speed value to set.
            distance: The distance value to set.
            buffer: The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.M2SPEEDACCELDIST, accel, speed, distance, buffer, types=["long", "slong", "long", "byte"])

    def SpeedAccelDistanceM1M2(self, address: int, accel: int, speed1: int, distance1: int, speed2: int, distance2: int, buffer: int) -> bool:
        """
        Sets the acceleration, speed, and distance for both motors.

        Args:
            address: The address of the controller.
            accel: The acceleration value to set.
            speed1: The speed value for motor 1.
            distance1: The distance value for motor 1.
            speed2: The speed value for motor 2.
            distance2: The distance value for motor 2.
            buffer: The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.MIXEDSPEEDACCELDIST, accel, speed1, distance1, speed2, distance2, buffer, types=["long", "slong", "long", "slong", "long", "byte"])

    def ReadBuffers(self, address: int) -> BuffersResult:
        """
        Reads the buffer status.

        Args:
            address: The address of the controller.

        Returns:
            BuffersResult: (success, buffer1, buffer2)
                success: True if read successful.
                buffer1: The status of buffer 1.
                buffer2: The status of buffer 2.
        """
        return self._read(address, Commands.GETBUFFERS, types=["byte", "byte"])

    def ReadPWMs(self, address: int) -> PWMsResult:
        """
        Reads the PWM values.

        Args:
            address: The address of the controller.

        Returns:
            PWMsResult: (success, pwm1, pwm2)
                success: True if read successful.
                pwm1: The PWM value for motor 1.
                pwm2: The PWM value for motor 2.
        """
        val = self._read(address, Commands.GETPWMS, types=["word", "word"])
        if val[0]:
            pwm1 = val[1]
            pwm2 = val[2]
            if pwm1 & 0x8000:
                pwm1 -= 0x10000
            if pwm2 & 0x8000:
                pwm2 -= 0x10000
            return (True, pwm1, pwm2)
        return (False, 0, 0)

    def ReadCurrents(self, address: int) -> CurrentsResult:
        """
        Reads the current values.

        Args:
            address: The address of the controller.

        Returns:
            CurrentsResult: (success, current1, current2)
                success: True if read successful.
                current1: The current value for motor 1.
                current2: The current value for motor 2.
        """
        val = self._read(address, Commands.GETCURRENTS, types=["word", "word"])
        if val[0]:
            cur1 = val[1]
            cur2 = val[2]
            if cur1 & 0x8000:
                cur1 -= 0x10000
            if cur2 & 0x8000:
                cur2 -= 0x10000
            return (True, cur1, cur2)
        return (False, 0, 0)

    def SpeedAccelM1M2_2(self, address: int, accel1: int, speed1: int, accel2: int, speed2: int) -> bool:
        """
        Sets the acceleration and speed for both motors with different accelerations.

        Args:
            address: The address of the controller.
            accel1: The acceleration value for motor 1.
            speed1: The speed value for motor 1.
            accel2: The acceleration value for motor 2.
            speed2: The speed value for motor 2.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.MIXEDSPEED2ACCEL, accel1, speed1, accel2, speed2, types=["long", "slong", "long", "slong"])

    def SpeedAccelDistanceM1M2_2(self, address: int, accel1: int, speed1: int, distance1: int, accel2: int, speed2: int, distance2: int, buffer: int) -> bool:
        """
        Sets the acceleration, speed, and distance for both motors with different accelerations.

        Args:
            address: The address of the controller.
            accel1: The acceleration value for motor 1.
            speed1: The speed value for motor 1.
            distance1: The distance value for motor 1.
            accel2: The acceleration value for motor 2.
            speed2: The speed value for motor 2.
            distance2: The distance value for motor 2.
            buffer: The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.MIXEDSPEED2ACCELDIST, accel1, speed1, distance1, accel2, speed2, distance2, buffer, types=["long", "slong", "long", "slong", "long", "long", "byte"])

    def DutyAccelM1(self, address: int, accel: int, duty: int) -> bool:
        """
        Sets the acceleration and duty cycle for motor 1.

        Args:
            address: The address of the controller.
            accel: The acceleration value to set.
            duty: The duty cycle value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.M1DUTYACCEL, duty, accel, types=["sword", "long"])

    def DutyAccelM2(self, address: int, accel: int, duty: int) -> bool:
        """
        Sets the acceleration and duty cycle for motor 2.

        Args:
            address: The address of the controller.
            accel: The acceleration value to set.
            duty: The duty cycle value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.M2DUTYACCEL, duty, accel, types=["sword", "long"])

    def DutyAccelM1M2(self, address: int, accel1: int, duty1: int, accel2: int, duty2: int) -> bool:
        """
        Sets the acceleration and duty cycle for both motors.

        Args:
            address: The address of the controller.
            accel1: The acceleration value for motor 1.
            duty1: The duty cycle value for motor 1.
            accel2: The acceleration value for motor 2.
            duty2: The duty cycle value for motor 2.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.MIXEDDUTYACCEL, duty1, accel1, duty2, accel2, types=["sword", "long", "sword", "long"])
        
    def ReadM1VelocityPID(self, address: int) -> PIDResult:
        """
        Reads the velocity PID constants for motor 1.

        Args:
            address: The address of the controller.

        Returns:
            PIDResult: (success, p, i, d, qpps)
                success: True if read successful.
                p: The proportional constant.
                i: The integral constant.
                d: The derivative constant.
                qpps: The speed in quadrature pulses per second.
        """
        data = self._read(address, Commands.READM1PID, types=["long", "long", "long", "long"])
        if data[0]:
            return (True, data[1] / self.PID_FLOAT_SCALE_VEL, data[2] / self.PID_FLOAT_SCALE_VEL, data[3] / self.PID_FLOAT_SCALE_VEL, data[4])
        return (False, 0, 0, 0, 0)

    def ReadM2VelocityPID(self, address: int) -> PIDResult:
        """
        Reads the velocity PID constants for motor 2.

        Args:
            address: The address of the controller.

        Returns:
            PIDResult: (success, p, i, d, qpps)
                success: True if read successful.
                p: The proportional constant.
                i: The integral constant.
                d: The derivative constant.
                qpps: The speed in quadrature pulses per second.
        """
        data = self._read(address, Commands.READM2PID, types=["long", "long", "long", "long"])
        if data[0]:
            return (True, data[1] / self.PID_FLOAT_SCALE_VEL, data[2] / self.PID_FLOAT_SCALE_VEL, data[3] / self.PID_FLOAT_SCALE_VEL, data[4])
        return (False, 0, 0, 0, 0)

    def SetMainVoltages(self, address: int, min_voltage: int, max_voltage: int, auto_offset: int) -> bool:
        """
        Sets the main battery voltage limits.

        Args:
            address: The address of the controller.
            min_voltage: The minimum voltage value to set.
            max_voltage: The maximum voltage value to set.
            auto_offset: The auto offset value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETMAINVOLTAGES, min_voltage, max_voltage, auto_offset, types=["word", "word", "byte"])
        
    def SetLogicVoltages(self, address: int, min_voltage: int, max_voltage: int) -> bool:
        """
        Sets the logic battery voltage limits.

        Args:
            address: The address of the controller.
            min_voltage: The minimum voltage value to set.
            max_voltage: The maximum voltage value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETLOGICVOLTAGES, min_voltage, max_voltage, types=["word", "word"])
        
    def ReadMinMaxMainVoltages(self, address: int) -> VoltageResult:
        """
        Reads the main battery voltage limits.

        Args:
            address: The address of the controller.

        Returns:
            VoltageResult: (success, min, max, auto_offset)
                success: True if read successful.
                min: The minimum voltage value.
                max: The maximum voltage value.
                auto_offset: The auto offset value.
        """
        return self._read(address, Commands.GETMINMAXMAINVOLTAGES, types=["word", "word", "byte"])

    def ReadMinMaxLogicVoltages(self, address: int) -> Tuple[bool, int, int]:
        """
        Reads the logic battery voltage limits.

        Args:
            address: The address of the controller.

        Returns:
            Tuple[bool, int, int]: (success, min, max)
                success: True if read successful.
                min: The minimum voltage value.
                max: The maximum voltage value.
        """
        return self._read(address, Commands.GETMINMAXLOGICVOLTAGES, types=["word", "word"])

    def SetM1PositionPID(self, address: int, kp: float, ki: float, kd: float, kimax: int, deadzone: int, min_pos: int, max_pos: int) -> bool:
        """
        Sets the position PID constants for motor 1.

        Args:
            address: The address of the controller.
            kp: The proportional constant.
            ki: The integral constant.
            kd: The derivative constant.
            kimax: The integral limit.
            deadzone: The deadzone value.
            min_pos: The minimum position value.
            max_pos: The maximum position value.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETM1POSPID, int(kd * self.PID_FLOAT_SCALE_POS), int(kp * self.PID_FLOAT_SCALE_POS), int(ki * self.PID_FLOAT_SCALE_POS), kimax, deadzone, min_pos, max_pos, types=["long", "long", "long", "long", "long", "long", "long"])

    def SetM2PositionPID(self, address: int, kp: float, ki: float, kd: float, kimax: int, deadzone: int, min_pos: int, max_pos: int) -> bool:
        """
        Sets the position PID constants for motor 2.

        Args:
            address: The address of the controller.
            kp: The proportional constant.
            ki: The integral constant.
            kd: The derivative constant.
            kimax: The integral limit.
            deadzone: The deadzone value.
            min_pos: The minimum position value.
            max_pos: The maximum position value.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETM2POSPID, int(kd * self.PID_FLOAT_SCALE_POS), int(kp * self.PID_FLOAT_SCALE_POS), int(ki * self.PID_FLOAT_SCALE_POS), kimax, deadzone, min_pos, max_pos, types=["long", "long", "long", "long", "long", "long", "long"])

    def ReadM1PositionPID(self, address: int) -> PositionPIDResult:
        """
        Reads the position PID constants for motor 1.

        Args:
            address: The address of the controller.

        Returns:
            PositionPIDResult: (success, kp, ki, kd, kimax, deadzone, min, max)
                success: True if read successful.
                kp: The proportional constant.
                ki: The integral constant.
                kd: The derivative constant.
                kimax: The integral limit.
                deadzone: The deadzone value.
                min: The minimum position value.
                max: The maximum position value.
        """
        data = self._read(address, Commands.READM1POSPID, types=["long", "long", "long", "long", "long", "long", "long"])
        if data[0]:
            return (True, data[1] / self.PID_FLOAT_SCALE_POS, data[2] / self.PID_FLOAT_SCALE_POS, data[3] / self.PID_FLOAT_SCALE_POS, data[4], data[5], data[6], data[7])
        return (False, 0, 0, 0, 0, 0, 0, 0)
        
    def ReadM2PositionPID(self, address: int) -> PositionPIDResult:
        """
        Reads the position PID constants for motor 2.

        Args:
            address: The address of the controller.

        Returns:
            PositionPIDResult: (success, kp, ki, kd, kimax, deadzone, min, max)
                success: True if read successful.
                kp: The proportional constant.
                ki: The integral constant.
                kd: The derivative constant.
                kimax: The integral limit.
                deadzone: The deadzone value.
                min: The minimum position value.
                max: The maximum position value.
        """
        data = self._read(address, Commands.READM2POSPID, types=["long", "long", "long", "long", "long", "long", "long"])
        if data[0]:
            return (True, data[1] / self.PID_FLOAT_SCALE_POS, data[2] / self.PID_FLOAT_SCALE_POS, data[3] / self.PID_FLOAT_SCALE_POS, data[4], data[5], data[6], data[7])
        return (False, 0, 0, 0, 0, 0, 0, 0)

    def SpeedAccelDeccelPositionM1(self, address: int, accel: int, speed: int, deccel: int, position: int, buffer: int) -> bool:
        """
        Sets the acceleration, speed, deceleration, and position for motor 1.

        Args:
            address: The address of the controller.
            accel: The acceleration value to set.
            speed: The speed value to set.
            deccel: The deceleration value to set.
            position: The position value to set.
            buffer: The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.M1SPEEDACCELDECCELPOS, accel, speed, deccel, position, buffer, types=["long", "long", "long", "long", "byte"])

    def SpeedAccelDeccelPositionM2(self, address: int, accel: int, speed: int, deccel: int, position: int, buffer: int) -> bool:
        """
        Sets the acceleration, speed, deceleration, and position for motor 2.

        Args:
            address: The address of the controller.
            accel: The acceleration value to set.
            speed: The speed value to set.
            deccel: The deceleration value to set.
            position: The position value to set.
            buffer: The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.M2SPEEDACCELDECCELPOS, accel, speed, deccel, position, buffer, types=["long", "long", "long", "long", "byte"])

    def SpeedAccelDeccelPositionM1M2(self, address: int, accel1: int, speed1: int, deccel1: int, position1: int, accel2: int, speed2: int, deccel2: int, position2: int, buffer: int) -> bool:
        """
        Sets the acceleration, speed, deceleration, and position for both motors.

        Args:
            address: The address of the controller.
            accel1: The acceleration value for motor 1.
            speed1: The speed value for motor 1.
            deccel1: The deceleration value for motor 1.
            position1: The position value for motor 1.
            accel2: The acceleration value for motor 2.
            speed2: The speed value for motor 2.
            deccel2: The deceleration value for motor 2.
            position2: The position value for motor 2.
            buffer: The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.MIXEDSPEEDACCELDECCELPOS, accel1, speed1, deccel1, position1, accel2, speed2, deccel2, position2, buffer, types=["long", "long", "long", "long", "long", "long", "long", "long", "byte"])

    def SetM1DefaultAccel(self, address: int, accel: int, decel: int) -> bool:
        """
        Sets the default acceleration and deceleration for motor 1.
    
        Args:
            address: The address of the controller.
            accel: The default acceleration value to set.
            decel: The default deceleration value to set.
        
        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETM1DEFAULTACCEL, accel, decel, types=["long", "long"])

    def SetM2DefaultAccel(self, address: int, accel: int, decel: int) -> bool:
        """
        Sets the default acceleration and deceleration for motor 2.
    
        Args:
            address: The address of the controller.
            accel: The default acceleration value to set.
            decel: The default deceleration value to set.
        
        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETM2DEFAULTACCEL, accel, decel, types=["long", "long"])

    def SetM1DefaultSpeed(self, address: int, speed: int) -> bool:
        """
        Sets the default speed for motor 1.
    
        Args:
            address: The address of the controller.
            speed: The default speed value to set.
        
        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETM1DEFAULTSPEED, speed, types=["word"])

    def SetM2DefaultSpeed(self, address: int, speed: int) -> bool:
        """
        Sets the default speed for motor 2.
    
        Args:
            address: The address of the controller.
            speed: The default speed value to set.
        
        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETM2DEFAULTSPEED, speed, types=["word"])

    def GetDefaultSpeeds(self, address: int) -> Tuple[bool, int, int]:
        """
        Reads the default speeds for both motors.

        Args:
            address: The address of the controller.

        Returns:
            Tuple[bool, int, int]: (success, default_speed1, default_speed2)
                success: True if read successful.
                default_speed1: The default speed for motor 1.
                default_speed2: The default speed for motor 2.
        """
        return self._read(address, Commands.GETDEFAULTSPEEDS, types=["word", "word"])
        
    def GetStatus(self, address: int) -> StatusResult:
        """
        Reads the status of the controller.

        Args:
            address: The address of the controller.

        Returns:
            StatusResult: Complex tuple with controller status information
                success: True if read successful.
                tick: The tick value.
                state: The state value.
                temp1: The temperature value 1.
                temp2: The temperature value 2.
                mbat: The main battery voltage.
                lbat: The logic battery voltage.
                pwm1: The PWM value for motor 1.
                pwm2: The PWM value for motor 2.
                cur1: The current value for motor 1.
                cur2: The current value for motor 2.
                enc1: The encoder value for motor 1.
                enc2: The encoder value for motor 2.
                speedS1: The speed setpoint for motor 1.
                speedS2: The speed setpoint for motor 2.
                speed1: The speed value for motor 1.
                speed2: The speed value for motor 2.
                speederror1: The speed error for motor 1.
                speederror2: The speed error for motor 2.
                poserror1: The position error for motor 1.
                poserror2: The position error for motor 2.
        """
        return self._read(address, Commands.GETSTATUS, types=["long", "long", "word", "word", "word", "word", "word", "word", "word", "word", "long", "long", "long", "long", "long", "long", "word", "word", "word", "word"])

    def SetPinFunctions(self, address: int, S3mode: int, S4mode: int, S5mode: int, D1mode: int, D2mode: int) -> bool:
        """
        Sets the functions of pins S3, S4, and S5.

        Args:
            address: The address of the controller.
            S3mode: The mode to set for pin S3.
            S4mode: The mode to set for pin S4.
            S5mode: The mode to set for pin S5.
            D1mode: The mode to set for pin CTRL1.
            D2mode: The mode to set for pin CTRL2.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETPINFUNCTIONS, S3mode, S4mode, S5mode, D1mode, D2mode, types=["byte", "byte", "byte", "byte", "byte"])

    def ReadPinFunctions(self, address: int) -> PinFunctionsResult:
        """
        Reads the functions of pins S3, S4, and S5.

        Args:
            address: The address of the controller.

        Returns:
            PinFunctionsResult: (success, S3mode, S4mode, S5mode)
                success: True if read successful.
                S3mode: The mode of pin S3.
                S4mode: The mode of pin S4.
                S5mode: The mode of pin S5.
                D1mode: The mode of pin CTRL1.
                D2mode: The mode of pin CTRL2.
        """
        return self._read(address, Commands.GETPINFUNCTIONS, types=["byte", "byte", "byte", "byte", "byte"])

    def SetCtrlSettings(self, address: int, S1revdeadband: int, S1fwddeadband: int, S1revlimit: int, S1fwdlimit: int, S1rangecenter: int, S1rangemin: int, S1rangemax: int, S2revdeadband: int, S2fwddeadband: int, S2revlimit: int, S2fwdlimit: int, S2rangecenter: int, S2rangemin: int, S2rangemax: int) -> bool:
        """
        Sets the deadband values.

        Args:
            address: The address of the controller.
            S1revdeadband: Reverse deadband value (0-255)
            S1fwddeadband: Forward deadband value (0-255)
            S1revlimit: Reverse Limit value, RC:0-3000,Analog(0-2047) 
            S1fwdlimit: Forward Limit value, RC:0-3000,Analog(0-2047) 
            S1rangecenter: Input Center
            S1rangemin: Input Minimum
            S1rangemax: Input Maximum
            S2revdeadband: Reverse deadband value (0-255)
            S2fwddeadband: Forward deadband value (0-255)
            S2revlimit: Reverse Limit value, RC:0-3000,Analog(0-2047) 
            S2fwdlimit: Forward Limit value, RC:0-3000,Analog(0-2047) 
            S2rangecenter: Input Center
            S2rangemin: Input Minimum
            S2rangemax: Input Maximum

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETCTRLSETTINGS, S1revdeadband,
                                                              S1fwddeadband,
                                                              S1revlimit,
                                                              S1fwdlimit, 
                                                              S1rangecenter,
                                                              S1rangemin,
                                                              S1rangemax,
                                                              S2revdeadband,
                                                              S2fwddeadband,
                                                              S2revlimit,
                                                              S2fwdlimit,
                                                              S2rangecenter,
                                                              S2rangemin,
                                                              S2rangemax,
                                                              types=["byte", "byte", "word", "word", "word", "word", "word", "byte", "byte", "word", "word", "word", "word", "word"])

    def GetCtrlSettings(self, address: int) -> DeadBandResult:
        """
        Reads the deadband values.

        Args:
            address: The address of the controller.

        Returns:
            DeadBandResult: (success, min, max)
                success: True if read successful.
                S1revdeadband: Reverse deadband value (0-255)
                S1fwddeadband: Forward deadband value (0-255)
                S1revlimit: Reverse Limit value, RC:0-3000,Analog(0-2047) 
                S1fwdlimit: Forward Limit value, RC:0-3000,Analog(0-2047) 
                S1rangecenter: Input Center
                S1rangemin: Input Minimum
                S1rangemax: Input Maximum
                S2revdeadband: Reverse deadband value (0-255)
                S2fwddeadband: Forward deadband value (0-255)
                S2revlimit: Reverse Limit value, RC:0-3000,Analog(0-2047) 
                S2fwdlimit: Forward Limit value, RC:0-3000,Analog(0-2047) 
                S2rangecenter: Input Center
                S2rangemin: Input Minimum
                S2rangemax: Input Maximum
        """
        return self._read(address, Commands.GETCTRLSETTINGS, types=["byte", "byte", "word", "word", "word", "word", "word", "byte", "byte", "word", "word", "word", "word", "word"])

    def GetEncoders(self, address: int) -> EncodersResult:
        """
        Reads the encoder values for both motors.

        Args:
            address: The address of the controller.

        Returns:
            EncodersResult: (success, enc1, enc2)
                success: True if read successful.
                enc1: The encoder value for motor 1.
                enc2: The encoder value for motor 2.
        """
        return self._read(address, Commands.GETENCODERS, types=["long", "long"])

    def GetISpeeds(self, address: int) -> ISpeedsResult:
        """
        Reads the instantaneous speeds for both motors.

        Args:
            address: The address of the controller.

        Returns:
            ISpeedsResult: (success, speed1, speed2)
                success: True if read successful.
                speed1: The instantaneous speed for motor 1.
                speed2: The instantaneous speed for motor 2.
        """
        return self._read(address, Commands.GETISPEEDS, types=["long", "long"])
     
    def RestoreDefaults(self, address: int) -> bool:
        """
        Restores the default settings.

        Args:
            address: The address of the controller.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.RESTOREDEFAULTS, self.RESTORE_DEFAULTS_KEY, types=["long"])

    def GetDefaultAccels(self, address: int) -> AccelsResult:
        """
        Reads the default accelerations for both motors.

        Args:
            address: The address of the controller.

        Returns:
            AccelsResult: (success, accel1, accel2, accel3, accel4)
                success: True if read successful.
                accel1: The default acceleration for motor 1.
                decel1: The default deceleration for motor 1.
                accel2: The default acceleration for motor 2.
                decel2: The default deceleration for motor 2.
        """
        return self._read(address, Commands.GETDEFAULTACCELS, types=["long", "long", "long", "long"])

    def ReadTemp(self, address: int) -> TempResult:
        """
        Reads the temperature from the first sensor.

        Args:
            address: The address of the controller.

        Returns:
            TempResult: (success, temperature)
                success: True if read successful.
                temperature: The temperature value.
        """
        return self._read(address, Commands.GETTEMP, types=["word"])

    def ReadTemp2(self, address: int) -> TempResult:
        """
        Reads the temperature from the second sensor.

        Args:
            address: The address of the controller.

        Returns:
            TempResult: (success, temperature)
                success: True if read successful.
                temperature: The temperature value.
        """
        return self._read(address, Commands.GETTEMP2, types=["word"])

    def ReadError(self, address: int) -> ErrorResult:
        """
        Reads the error status.

        Args:
            address: The address of the controller.

        Returns:
            ErrorResult: (success, error)
                success: True if read successful.
                error: The error status.
        """
        return self._read(address, Commands.GETERROR, types=["long"])

    def ReadEncoderModes(self, address: int) -> EncoderModesResult:
        """
        Reads the encoder modes for both motors.

        Args:
            address: The address of the controller.

        Returns:
            EncoderModesResult: (success, mode1, mode2)
                success: True if read successful.
                mode1: The encoder mode for motor 1.
                mode2: The encoder mode for motor 2.
        """
        return self._read(address, Commands.GETENCODERMODE, types=["byte", "byte"])

    def SetM1EncoderMode(self, address: int, mode: int) -> bool:
        """
        Sets the encoder mode for motor 1.

        Args:
            address: The address of the controller.
            mode: The encoder mode to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETM1ENCODERMODE, mode, types=["byte"])

    def SetM2EncoderMode(self, address: int, mode: int) -> bool:
        """
        Sets the encoder mode for motor 2.

        Args:
            address: The address of the controller.
            mode: The encoder mode to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETM2ENCODERMODE, mode, types=["byte"])

    def WriteNVM(self, address: int) -> bool:
        """
        Saves the active settings to non-volatile memory (NVM).

        Args:
            address: The address of the controller.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.WRITENVM, self.NVM_COMMIT_KEY, types=["long"])

    def ReadNVM(self, address: int) -> bool:
        """
        Restores the settings from non-volatile memory (NVM).

        Args:
            address: The address of the controller.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.READNVM)

    def SetSerialNumber(self, address: int, serial_number: str) -> bool:
        """Sets the device serial number (36 bytes).

        Args:
            address: Controller address (0x80 to 0x87)
            serial_number: Serial number string (36 characters max)

        Returns:
            bool: True if successful, False otherwise

        Notes:
            - Serial number will be padded with null bytes if less than 36 bytes
            - If longer than 36 bytes, it will be truncated
            - Only the specified length of characters will be displayed when read back
    
        Raises:
            ValueError: If serial_number is not a string
        """
        logger.info(f"Setting serial number for address 0x{address:02x} to '{serial_number}'")

        if not isinstance(serial_number, str):
            logger.error("Serial number must be a string")
            raise ValueError("Serial number must be a string")

        # Truncate if longer than 36 bytes
        if len(serial_number) > 36:
            logger.warning(f"Serial number too long, truncating to 36 characters")
            serial_number = serial_number[:36]
        
        # Pad or truncate to exactly 36 bytes
        serial_bytes = bytes([len(serial_number)]) + serial_number.encode('ascii').ljust(36, b'\0')

        for _ in range(self._trystimeout):
            self._sendcommand(address, Commands.SETSERIALNUMBER)
            for i in range(37):
                self._writebyte(serial_bytes[i])

            if self._writechecksum():
                return True

        logger.error("Failed to set serial number after multiple attempts")
        return False

    def GetSerialNumber(self, address: int) -> SerialNumberResult:
        """Reads the device serial number (36 bytes).

        Args:
            address: Controller address (0x80 to 0x87)

        Returns:
            SerialNumberResult: (success, serial_number)
                success: True if read successful
                serial_number: Serial number string with null characters stripped
        """
        logger.info(f"Reading serial number for address 0x{address:02x}")

        for _ in range(self._trystimeout):
            try:
                self._port.flushInput()
            except Exception as e:
                logger.warning(f"Error flushing input buffer during serial number read: {str(e)}")
                # Continue trying despite the error
            self._sendcommand(address, Commands.GETSERIALNUMBER)

            # Read 1 byte for the count of characters used by the serial number
            cnt = self._readbyte()
            if not cnt[0]:
                logger.debug("Failed to read count byte")
                continue

            # Read 36 bytes for the serial number
            serial_bytes = bytearray()
            read_error = False
            for _ in range(36):
                val = self._readbyte()
                if not val[0]:
                    logger.debug("Failed to read serial byte")
                    read_error = True
                    break
                serial_bytes.append(val[1])

            if read_error:
                continue

            if len(serial_bytes) == 36:
                crc = self._readchecksumword()
                if crc[0] and self._crc & 0xFFFF == crc[1] & 0xFFFF:
                    # Use only the number of characters specified by count
                    # Strip null characters for readability
                    serial_str = serial_bytes[:cnt[1]].decode('ascii').rstrip('\0')
                    return (True, serial_str)
                else:
                    logger.debug(f"CRC check failed: received=0x{crc[1]:04x}, calculated=0x{self._crc & 0xFFFF:04x}")
    
            logger.debug(f"Retrying serial number read after attempt {_+1}")

        logger.error(f"Failed to read serial number after {self._trystimeout} attempts")
        return (False, '')

    def SetConfig(self, address: int, config: int) -> bool:
        """
        Sets the configuration of the controller.

        Args:
            address: The address of the controller (0x80-0x87)
            config: The configuration value to set (16-bit)
                Bit meanings vary by controller model - see controller documentation

        Returns:
            bool: True if successful
            
        Warnings:
            - If control mode is changed from packet serial mode, communications will be lost!
            - If baudrate of packet serial mode is changed, communications will be lost!
        """
        return self._write(address, Commands.SETCONFIG, config, types=["word"])

    def GetConfig(self, address: int) -> ConfigResult:
        """
        Reads the configuration.

        Args:
            address: The address of the controller.

        Returns:
            ConfigResult: (success, config)
                success: True if read successful.
                config: The configuration value.
        """
        return self._read(address, Commands.GETCONFIG, types=["word"])

    def GetEncStatus(self, address: int) -> EncStatusResult:
        """Reads encoder error statuses.
            
        Args:
            address: Controller address (0x80 to 0x87)
                
        Returns:
            EncStatusResult: (success, enc1status, enc2status)
                success: True if read successful
                enc1status: Encoder 1 error flags
                enc2status: Encoder 2 error flags
        """
        return self._read(address, Commands.GETENCSTATUS, types=["byte", "byte"])

    def SetAuto1(self, address: int, value: int) -> bool:
        """Sets auto mode 1 value.
            
        Args:
            address: Controller address (0x80 to 0x87)
            value: Auto mode configuration
                
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.SETAUTO1, value, types=["long"])

    def SetAuto2(self, address: int, value: int) -> bool:
        """Sets auto mode 2 value.
            
        Args:
            address: Controller address (0x80 to 0x87)
            value: Auto mode configuration
                
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.SETAUTO2, value, types=["long"])

    def GetAutos(self, address: int) -> AutosResult:
        """Reads auto mode values.
            
        Args:
            address: Controller address (0x80 to 0x87)
                
        Returns:
            AutosResult: (success, auto1, auto2)
                success: True if read successful
                auto1: Auto mode 1 value
                auto2: Auto mode 2 value
        """
        return self._read(address, Commands.GETAUTOS, types=["long", "long"])

    def GetSpeeds(self, address: int) -> SpeedsResult:
        """Reads current speed values for both motors.
    
        Args:
            address: Controller address (0x80 to 0x87)
        
        Returns:
            SpeedsResult: (success, speed1, speed2)
                success: True if read successful
                speed1: Current speed of motor 1 (32-bit)
                speed2: Current speed of motor 2 (32-bit)
        """
        return self._read(address, Commands.GETSPEEDS, types=["long", "long"])

    def SetSpeedErrorLimit(self, address: int, limit1: int, limit2: int) -> bool:
        """Sets speed error limits.
            
        Args:
            address: Controller address (0x80 to 0x87)
            limit1: Motor 1 speed error limit
            limit2: Motor 2 speed error limit
                
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.SETSPEEDERRORLIMIT, limit1, limit2, types=["word", "word"])

    def GetSpeedErrorLimit(self, address: int) -> ErrorLimitResult:
        """Reads speed error limits.
            
        Args:
            address: Controller address (0x80 to 0x87)
                
        Returns:
            ErrorLimitResult: (success, limit1, limit2)
                success: True if successful
                limit1: Motor 1 speed error limit
                limit2: Motor 2 speed error limit
        """
        return self._read(address, Commands.GETSPEEDERRORLIMIT, types=["word", "word"])

    def GetSpeedErrors(self, address: int) -> SpeedErrorsResult:
        """Reads current speed errors.
            
        Args:
            address: Controller address (0x80 to 0x87)
                
        Returns:
            SpeedErrorsResult: (success, error1, error2)
                success: True if successful
                error1: Motor 1 speed error
                error2: Motor 2 speed error
        """
        return self._read(address, Commands.GETSPEEDERRORS, types=["word", "word"])

    def PositionM1(self, address: int, position: int, buffer: int) -> bool:
        """Commands motor 1 to absolute position.
            
        Args:
            address: Controller address (0x80 to 0x87)
            position: Target position value
            buffer: The buffer value to set.
                
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.M1POS, position, buffer, types=["long", "byte"])

    def PositionM2(self, address: int, position: int, buffer: int) -> bool:
        """Commands motor 2 to absolute position.
            
        Args:
            address: Controller address (0x80 to 0x87)
            position: Target position value
            buffer: The buffer value to set.
                
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.M2POS, position, buffer, types=["long", "byte"])

    def PositionM1M2(self, address: int, position1: int, position2: int, buffer: int) -> bool:
        """Commands both motors to positions simultaneously.
            
        Args:
            address: Controller address (0x80 to 0x87)
            position1: Motor 1 target position
            position2: Motor 2 target position
            buffer: The buffer value to set.
                
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.MIXEDPOS, position1, position2, buffer, types=["long", "long", "byte"])

    def SpeedPositionM1(self, address: int, speed: int, position: int, buffer: int) -> bool:
        """Commands motor 1 position with speed.
    
        Args:
            address: Controller address (0x80 to 0x87)
            speed: Maximum speed
            position: Target position
            buffer: The buffer value to set.
        
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.M1SPEEDPOS, speed, position, buffer, types=["long", "long", "byte"])

    def SpeedPositionM2(self, address: int, speed: int, position: int, buffer: int) -> bool:
        """Commands motor 2 position with speed.
    
        Args:
            address: Controller address (0x80 to 0x87)
            speed: Maximum speed (32-bit)
            position: Target position (32-bit)
            buffer: The buffer value to set.
        
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.M2SPEEDPOS, speed, position, buffer, types=["long", "long", "byte"])

    def SpeedPositionM1M2(self, address: int, speed1: int, position1: int, speed2: int, position2: int, buffer: int) -> bool:
        """Commands both motors with speed and position.
    
        Args:
            address: Controller address (0x80 to 0x87)
            speed1: Motor 1 speed
            position1: Motor 1 position
            speed2: Motor 2 speed
            position2: Motor 2 position
            buffer: The buffer value to set.
        
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.MIXEDSPEEDPOS, speed1, position1, speed2, position2, buffer, types=["long", "long", "long", "long", "byte"])

    def PercentPositionM1(self, address: int, position: int, buffer: int) -> bool:
        """Commands motor 1 to a percent position.
    
        Args:
            address: Controller address (0x80 to 0x87)
            position: Target position as percentage (-32767 to +32767)
            buffer: The buffer value to set.
        
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.M1PPOS, position, buffer, types=["sword", "byte"])

    def PercentPositionM2(self, address: int, position: int, buffer: int) -> bool:
        """Commands motor 2 to a percent position.
    
        Args:
            address: Controller address (0x80 to 0x87)
            position: Target position as percentage (-32767 to +32767)
            buffer: The buffer value to set.
        
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.M2PPOS, position, buffer, types=["sword", "byte"])

    def PercentPositionM1M2(self, address: int, position1: int, position2: int, buffer: int) -> bool:
        """Commands both motors to percent positions.
    
        Args:
            address: Controller address (0x80 to 0x87)
            position1: Motor 1 target position percentage (-32767 to +32767)
            position2: Motor 2 target position percentage (-32767 to +32767)
            buffer: The buffer value to set.
        
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.MIXEDPPOS, position1, position2, buffer, types=["sword", "sword", "byte"])
        
    def SetPosErrorLimit(self, address: int, limit1: int, limit2: int) -> bool:
        """Sets position error limits for both motors.
    
        Args:
            address: Controller address (0x80 to 0x87)
            limit1: Motor 1 position error limit (0 to 65535)
            limit2: Motor 2 position error limit (0 to 65535)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.SETPOSERRORLIMIT, limit1, limit2, types=["word", "word"])

    def GetPosErrorLimit(self, address: int) -> ErrorLimitResult:
        """Reads position error limits.
    
        Args:
            address: Controller address (0x80 to 0x87)
        
        Returns:
            ErrorLimitResult: (success, limit1, limit2)
                success: True if read successful
                limit1: Motor 1 position error limit
                limit2: Motor 2 position error limit
        """
        return self._read(address, Commands.GETPOSERRORLIMIT, types=["word", "word"])

    def GetPosErrors(self, address: int) -> PositionErrorsResult:
        """Reads current position errors.
    
        Args:
            address: Controller address (0x80 to 0x87)
        
        Returns:
            PositionErrorsResult: (success, error1, error2)
                success: True if read successful
                error1: Motor 1 position error
                error2: Motor 2 position error
        """
        return self._read(address, Commands.GETPOSERRORS, types=["word", "word"])

    def SetOffsets(self, address: int, offset1: int, offset2: int) -> bool:
        """Sets voltage offsets.
    
        Args:
            address: Controller address (0x80 to 0x87)
            offset1: MBat voltage offset (0 to 255)
            offset2: LBat voltage offset (0 to 255)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.SETOFFSETS, offset1, offset2, types=["byte", "byte"])

    def GetOffsets(self, address: int) -> OffsetsResult:
        """Reads voltage offsets.
    
        Args:
            address: Controller address (0x80 to 0x87)
        
        Returns:
            OffsetsResult: (success, offset1, offset2)
                success: True if read successful
                offset1: MBat voltage offset
                offset2: LBat voltage offset
        """
        results = self._read(address, Commands.GETOFFSETS, types=["byte", "byte"])
        mbatoffset = struct.unpack('b', struct.pack('B', results[1]))[0]
        lbatoffset = struct.unpack('b', struct.pack('B', results[2]))[0]
        return (results[0],mbatoffset,lbatoffset)

    def SetM1LR(self, address: int, L: float, R: float) -> bool:
        """Sets motor 1 Inductance/Resistance.
    
        Args:
            address: Controller address (0x80 to 0x87)
            L: Inductance in Henries
            R: Resistance in Ohms
        
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.SETM1LR, int(L * self.LR_FLOAT_SCALE), int(R * self.LR_FLOAT_SCALE), types=["long", "long"])

    def SetM2LR(self, address: int, L: float, R: float) -> bool:
        """Sets motor 2 Inductance/Resistance.
    
        Args:
            address: Controller address (0x80 to 0x87)
            L: Inductance in Henries
            R: Resistance in Ohms
        
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.SETM2LR, int(L * self.LR_FLOAT_SCALE), int(R * self.LR_FLOAT_SCALE), types=["long", "long"])

    def GetM1LR(self, address: int) -> LRResult:
        """Reads motor 1 Inductance/Resistance.
    
        Args:
            address: Controller address (0x80 to 0x87)
        
        Returns:
            LRResult: (success, L, R)
                success: True if read successful
                L: Inductance in Henries
                R: Resistance in Ohms
        """
        data = self._read(address, Commands.GETM1LR, types=["long", "long"])
        if data[0]:
            return (True, float(data[1]) / self.LR_FLOAT_SCALE, float(data[2]) / self.LR_FLOAT_SCALE)
        return (False, 0, 0)

    def GetM2LR(self, address: int) -> LRResult:
        """Reads motor 2 Inductance/Resistance.
    
        Args:
            address: Controller address (0x80 to 0x87)
        
        Returns:
            LRResult: (success, L, R)
                success: True if read successful
                L: Inductance in Henries
                R: Resistance in Ohms
        """
        data = self._read(address, Commands.GETM2LR, types=["long", "long"])
        if data[0]:
            return (True, float(data[1]) / self.LR_FLOAT_SCALE, float(data[2]) / self.LR_FLOAT_SCALE)
        return (False, 0, 0)

    def GetVolts(self, address: int) -> VoltsResult:
        """Reads main and logic battery voltages.
    
        Args:
            address: Controller address (0x80 to 0x87)
        
        Returns:
            VoltsResult: (success, mbat, lbat)
                success: True if read successful
                mbat: Main battery voltage in tenths of a volt
                lbat: Logic battery voltage in tenths of a volt
        """
        val = self._read(address, Commands.GETVOLTS, types=["word", "word"])
        if val[0]:
            return (True, val[1], val[2])
        return (False, 0, 0)

    def GetTemps(self, address: int) -> Tuple[bool, int, int]:
        """Reads temperature sensor values.
    
        Args:
            address: Controller address (0x80 to 0x87)
        
        Returns:
            Tuple[bool, int, int]: (success, temp1, temp2)
                success: True if read successful
                temp1: Temperature sensor 1 in tenth degrees Celsius
                temp2: Temperature sensor 2 in tenth degrees Celsius
        """
        return self._read(address, Commands.GETTEMPS, types=["word", "word"])

    def SetAuxDutys(self, address: int, duty1: int, duty2: int, duty3: int, duty4: int, duty5: int) -> bool:
        """Sets auxiliary PWM duty cycles.
        
        Args:
            address: Controller address (0x80 to 0x87)
            duty1-duty5: Duty cycle values (-32767 to +32767)
            
        Returns:
            bool: True if successful, False otherwise
        """
        return self._write(address, Commands.SETAUXDUTYS, duty1, duty2, duty3, duty4, duty5, types=["word", "word", "word", "word", "word"])

    def GetAuxDutys(self, address: int) -> Tuple[bool, int, int, int, int, int]:
        """Reads auxiliary PWM duty cycles.
        
        Args:
            address: Controller address (0x80 to 0x87)
            
        Returns:
            Tuple[bool, int, int, int, int, int]: (success, duty1, duty2, duty3, duty4, duty5)
                success: True if read successful
                duty1-5: Current duty cycle values (-32767 to +32767)
        """
        return self._read(address, Commands.GETAUXDUTYS, types=["word", "word", "word", "word", "word"])

    def SetM1MaxCurrent(self, address: int, maxi: int, mini: int) -> bool:
        """
        Sets the maximum and minimum current limits for motor 1.

        Args:
            address: The address of the controller.
            maxi: The maximum current limit.
            mini: The minimum current limit.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETM1MAXCURRENT, maxi, mini, types=["long", "long"])

    def SetM2MaxCurrent(self, address: int, maxi: int, mini: int) -> bool:
        """
        Sets the maximum and minimum current limits for motor 2.

        Args:
            address: The address of the controller.
            maxi: The maximum current limit.
            mini: The minimum current limit.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETM2MAXCURRENT, maxi, mini, types=["long", "long"])

    def ReadM1MaxCurrent(self, address: int) -> MaxCurrentResult:
        """
        Reads the maximum and minimum current limits for motor 1.

        Args:
            address: The address of the controller.

        Returns:
            MaxCurrentResult: (success, maxi, mini)
                success: True if read successful.
                maxi: The maximum current limit.
                mini: The minimum current limit.
        """
        return self._read(address, Commands.GETM1MAXCURRENT, types=["long", "long"])

    def ReadM2MaxCurrent(self, address: int) -> MaxCurrentResult:
        """
        Reads the maximum and minimum current limits for motor 2.

        Args:
            address: The address of the controller.

        Returns:
            MaxCurrentResult: (success, maxi, mini)
                success: True if read successful.
                maxi: The maximum current limit.
                mini: The minimum current limit.
        """
        return self._read(address, Commands.GETM2MAXCURRENT, types=["long", "long"])
        
    def SetDOUT(self, address: int, index: int, action: int) -> bool:
        """Sets the digital output.
        
        Args:
            address: Controller address (0x80 to 0x87)
            index: Output index (0 to 255)
            action: Action to perform (0 to 255)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.SETDOUT, index, action, types=["byte", "byte"])

    def GetDOUTS(self, address: int) -> DOUTSResult:
        """Gets the digital outputs.
    
        Args:
            address: Controller address (0x80 to 0x87)
    
        Returns:
            DOUTSResult: (success, count, actions)
                success: True if read successful
                count: Number of actions
                actions: List of actions performed
        """
        logger.info(f"Reading digital outputs for address 0x{address:02x}")

        for _ in range(self._trystimeout):
            try:
                self._port.flushInput()
            except Exception as e:
                logger.warning(f"Error flushing input buffer during DOUTS read: {str(e)}")
                # Continue trying despite the error
            self._sendcommand(address, Commands.GETDOUTS)
        
            count = self._readbyte()
            if not count[0]:
                logger.debug("Failed to read count byte")
                continue
        
            expected_count = count[1]
            actions = []
            read_success = True # Flag to track if all reads succeeded

            for _ in range(expected_count):
                # Call _readbyte() only ONCE per loop
                action_byte_result = self._readbyte()

                if not action_byte_result[0]:
                    # If reading this specific byte failed
                    logger.debug(f"Failed to read action byte number {_ + 1}")
                    read_success = False
                    break # Stop trying to read more bytes for this attempt

                # If successful, append the value (index [1])
                actions.append(action_byte_result[1])

            # Check if we broke out early OR if the loop finished but didn't get all bytes
            # (The second condition is less likely with the break, but good paranoia)
            if not read_success or len(actions) != expected_count:
                logger.debug(f"Failed to read all expected {expected_count} action bytes successfully.")
                continue # Go to the next iteration of the outer retry loop

            # If we reach here, all 'expected_count' bytes were read successfully
            # Now, proceed to read and check the CRC
            crc = self._readchecksumword()
            if crc[0] and self._crc & 0xFFFF == crc[1] & 0xFFFF:
                # Success! Return the data.
                return (True, expected_count, actions)
            else:
                # Checksum failed or reading checksum failed
                if not crc[0]:
                    logger.debug("Failed to read checksum word")
                else:
                    logger.debug(f"CRC check failed: received=0x{crc[1]:04x}, calculated=0x{self._crc & 0xFFFF:04x}")
                continue # Go to the next iteration of the outer retry loop

        logger.error(f"Failed to read digital outputs after {self._trystimeout} attempts")
        return (False, 0, [])

    def SetPriority(self, address: int, priority1: int, priority2: int, priority3: int) -> bool:
        """Sets the priority levels.
        
        Args:
            address: Controller address (0x80 to 0x87)
            priority1: Priority level 1 (0 to 255)
            priority2: Priority level 2 (0 to 255)
            priority3: Priority level 3 (0 to 255)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.SETPRIORITY, priority1, priority2, priority3, types=["byte", "byte", "byte"])

    def GetPriority(self, address: int) -> PriorityResult:
        """Gets the priority levels.
        
        Args:
            address: Controller address (0x80 to 0x87)
        
        Returns:
            PriorityResult: (success, priority1, priority2, priority3)
                success: True if read successful
                priority1: Priority level 1
                priority2: Priority level 2
                priority3: Priority level 3
        """
        return self._read(address, Commands.GETPRIORITY, types=["byte", "byte", "byte"])

    def SetAddressMixed(self, address: int, new_address: int, enable_mixing: int) -> bool:
        """Sets the mixed address.
        
        Args:
            address: Controller address (0x80 to 0x87)
            new_address: New address (0 to 255)
            enable_mixing: Enable mixing (0 or 1)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.SETADDRESSMIXED, new_address, enable_mixing, types=["byte", "byte"])

    def GetAddressMixed(self, address: int) -> AddressMixedResult:
        """Gets the mixed address.
        
        Args:
            address: Controller address (0x80 to 0x87)
        
        Returns:
            AddressMixedResult: (success, new_address, mixed)
                success: True if read successful
                new_address: New address
                mixed: Mixing enabled (0 or 1)
        """
        return self._read(address, Commands.GETADDRESSMIXED, types=["byte", "byte"])

    def SetSignal(
        self, 
        address: int, 
        index: int, 
        signal_type: int, 
        mode: int, 
        target: int, 
        min_action: int, 
        max_action: int, 
        lowpass: int, 
        timeout: int, 
        loadhome: int, 
        min_val: int, 
        max_val: int, 
        center: int, 
        deadband: int, 
        powerexp: int, 
        minout: int, 
        maxout: int, 
        powermin: int, 
        potentiometer: int
    ) -> bool:
        """Sets the signal parameters.
    
        Args:
            address: Controller address (0x80 to 0x87)
            index: Signal index (0 to 255)
            signal_type: Signal type (0 to 255)
            mode: Mode (0 to 255)
            target: Target (0 to 255)
            min_action: Minimum action (0 to 65535)
            max_action: Maximum action (0 to 65535)
            lowpass: Lowpass filter (0 to 255)
            timeout: Timeout (0 to 4294967295)
            loadhome: Load home position (-2147483648 to 2147483647)
            min_val: Minimum value (-2147483648 to 2147483647)
            max_val: Maximum value (-2147483648 to 2147483647)
            center: Center value (-2147483648 to 2147483647)
            deadband: Deadband (0 to 4294967295)
            powerexp: Power exponent (0 to 4294967295)
            minout: Minimum output (0 to 4294967295)
            maxout: Maximum output (0 to 4294967295)
            powermin: Minimum power (0 to 4294967295)
            potentiometer: Potentiometer value (0 to 4294967295)
    
        Returns:
            bool: True if successful
        """
        logger.info(f"Setting signal parameters for address 0x{address:02x}, index={index}")
        logger.debug(f"Signal params: type={signal_type}, mode={mode}, target={target}, min_action={min_action}, max_action={max_action}, lowpass={lowpass}")
    
        params = [
            (index, "byte"), (signal_type, "byte"), (mode, "byte"), (target, "byte"),
            (min_action, "word"), (max_action, "word"), (lowpass, "byte"), (timeout, "long"),
            (loadhome, "slong"), (min_val, "slong"), (max_val, "slong"), (center, "slong"),
            (deadband, "long"), (powerexp, "long"), (minout, "long"), (maxout, "long"),
            (powermin, "long"), (potentiometer, "long")
        ]

        for _ in range(self._trystimeout):
            self._sendcommand(address, Commands.SETSIGNAL)
            for value, dtype in params:
                if dtype == "byte":
                    self._writebyte(value)
                elif dtype == "word":
                    self._writeword(value)
                elif dtype == "long":
                    self._writelong(value)
                elif dtype == "slong":
                    self._writeslong(value)
        
            if self._writechecksum():
                return True
    
        logger.error(f"Failed to set signal parameters for address 0x{address:02x}, index={index} after {self._trystimeout} attempts")
        return False

    def GetSignals(self, address: int) -> SignalsResult:
        """Gets the signal parameters configured in the controller.

        Args:
            address: Controller address (0x80 to 0x87)

        Returns:
            SignalsResult: (success, count, signals)
                success: True if read successful
                count: Number of signals configured
                signals: List of dictionaries containing signal parameters:
                    - type: Signal type (0-255, see controller documentation)
                    - mode: Operating mode (0-255, see controller documentation)
                    - target: Target channel (0-255)
                    - min_action: Minimum action value (0-65535)
                    - max_action: Maximum action value (0-65535)
                    - lowpass: Lowpass filter value (0-255)
                    - timeout: Signal timeout in milliseconds
                    - loadhome: Home position value
                    - min_val: Minimum input value
                    - max_val: Maximum input value
                    - center: Center input value
                    - deadband: Deadband value
                    - powerexp: Power exponent value
                    - minout: Minimum output value
                    - maxout: Maximum output value
                    - powermin: Minimum power value
                    - potentiometer: Potentiometer configuration
        """
        logger.info(f"Reading signal parameters for address 0x{address:02x}")

        for _ in range(self._trystimeout):
            try:
                self._port.flushInput()
            except Exception as e:
                logger.warning(f"Error flushing input buffer during signals read: {str(e)}")
                # Continue trying despite the error
            self._sendcommand(address, Commands.GETSIGNALS)
            count = self._readbyte()
            if not count[0]:
                logger.debug("Failed to read signal count byte")
                continue

            signals = []
            read_error = False

            for i in range(count[1]):
                signal = {
                    'type': self._readbyte(),
                    'mode': self._readbyte(),
                    'target': self._readbyte(),
                    'min_action': self._readword(),
                    'max_action': self._readword(),
                    'lowpass': self._readbyte(),
                    'timeout': self._readlong(),
                    'loadhome': self._readslong(),
                    'min_val': self._readslong(),
                    'max_val': self._readslong(),
                    'center': self._readslong(),
                    'deadband': self._readlong(),
                    'powerexp': self._readlong(),
                    'minout': self._readlong(),
                    'maxout': self._readlong(),
                    'powermin': self._readlong(),
                    'potentiometer': self._readlong()
                }

                if not all([v[0] for v in signal.values()]):
                    logger.debug(f"Failed to read complete signal data for signal {i+1}")
                    read_error = True
                    break

                signals.append({k: v[1] for k, v in signal.items()})

            if read_error:
                continue

            crc = self._readchecksumword()
            if crc[0] and self._crc & 0xFFFF == crc[1] & 0xFFFF:
                return (True, count[1], signals)
            else:
                logger.debug(f"CRC check failed: received=0x{crc[1]:04x}, calculated=0x{self._crc & 0xFFFF:04x}")

        logger.error(f"Failed to read signals from address 0x{address:02x} after {self._trystimeout} attempts")
        return (False, 0, [])
        
    def SetStream(self, address: int, index: int, stream_type: int, baudrate: int, timeout: int) -> bool:
        """Sets the stream parameters.
    
        Args:
            address: Controller address (0x80 to 0x87)
            index: Stream index (0 to 255)
            stream_type: Stream type (0 to 255)
            baudrate: Baudrate (0 to 4294967295)
            timeout: Timeout (0 to 4294967295)
    
        Returns:
            bool: True if successful
        """
        logger.info(f"Setting stream parameters for address 0x{address:02x}, index={index}")
        logger.debug(f"Stream params: type={stream_type}, baudrate={baudrate}, timeout={timeout}")

        for _ in range(self._trystimeout):
            self._sendcommand(address, Commands.SETSTREAM)
            self._writebyte(index)
            self._writebyte(stream_type)
            self._writelong(baudrate)
            self._writelong(timeout)
            if self._writechecksum():
                return True

        logger.error(f"Failed to set stream parameters for address 0x{address:02x}, index={index} after {self._trystimeout} attempts")
        return False

    def GetStreams(self, address: int) -> StreamsResult:
        """Gets the stream parameters configured in the controller.

        Args:
            address: Controller address (0x80 to 0x87)

        Returns:
            StreamsResult: (success, count, streams)
                success: True if read successful
                count: Number of streams configured
                streams: List of dictionaries containing stream parameters:
                    - type: Stream type (0 = disabled, 1 = UART, 2 = I2C, 3 = SPI)
                    - baudrate: Communication baudrate (for UART) or clock rate (for I2C/SPI)
                    - timeout: Communication timeout in milliseconds
        """
        logger.info(f"Reading stream parameters for address 0x{address:02x}")

        for _ in range(self._trystimeout):
            try:
                self._port.flushInput()
            except Exception as e:
                logger.warning(f"Error flushing input buffer during streams read: {str(e)}")
                # Continue trying despite the error
            self._sendcommand(address, Commands.GETSTREAMS)
            count = self._readbyte()
            if not count[0]:
                logger.debug("Failed to read stream count byte")
                continue

            streams = []
            read_error = False

            for i in range(count[1]):
                stream = {
                    'type': self._readbyte(),
                    'baudrate': self._readlong(),
                    'timeout': self._readlong()
                }

                if not all([v[0] for v in stream.values()]):
                    logger.debug(f"Failed to read complete stream data for stream {i+1}")
                    read_error = True
                    break

                streams.append({k: v[1] for k, v in stream.items()})

            if read_error:
                continue

            crc = self._readchecksumword()
            if crc[0] and self._crc & 0xFFFF == crc[1] & 0xFFFF:
                return (True, count[1], streams)
            else:
                logger.debug(f"CRC check failed: received=0x{crc[1]:04x}, calculated=0x{self._crc & 0xFFFF:04x}")

        logger.error(f"Failed to read streams from address 0x{address:02x} after {self._trystimeout} attempts")
        return (False, 0, [])

    def GetSignalsData(self, address: int) -> SignalsDataResult:
        """Gets the current signals data from the controller.

        Args:
            address: Controller address (0x80 to 0x87)

        Returns:
            SignalsDataResult: (success, count, signals_data)
                success: True if read successful
                count: Number of signal data entries
                signals_data: List of dictionaries containing signal data:
                    - command: Current command value
                    - position: Current position value
                    - percent: Current percentage value (0-100%)
                    - speed: Current speed value
                    - speeds: Speed status information
        """
        logger.info(f"Reading signals data from address 0x{address:02x}")
    
        for _ in range(self._trystimeout):
            try:
                self._port.flushInput()
            except Exception as e:
                logger.warning(f"Error flushing input buffer during signals data read: {str(e)}")
                # Continue trying despite the error
            self._sendcommand(address, Commands.GETSIGNALSDATA)
            count = self._readbyte()
            if not count[0]:
                logger.debug("Failed to read signal data count byte")
                continue

            signals_data = []
            read_error = False
        
            for i in range(count[1]):
                signal_data = {}
                signal_data['command'] = self._readlong()
                signal_data['position'] = self._readlong()
                signal_data['percent'] = self._readlong()
                signal_data['speed'] = self._readlong()
                signal_data['speeds'] = self._readlong()

                if not all([v[0] for v in signal_data.values()]):
                    logger.debug(f"Failed to read complete data for signal {i+1}")
                    read_error = True
                    break

                signals_data.append({k: v[1] for k, v in signal_data.items()})

            if read_error:
                continue

            crc = self._readchecksumword()
            if crc[0] and self._crc & 0xFFFF == crc[1] & 0xFFFF:
                return (True, count[1], signals_data)
            else:
                logger.debug(f"CRC check failed: received=0x{crc[1]:04x}, calculated=0x{self._crc & 0xFFFF:04x}")

        logger.error(f"Failed to read signals data from address 0x{address:02x} after {self._trystimeout} attempts")
        return (False, 0, [])
    
    def SetNodeID(self, address: int, nodeid: int) -> bool:
        """Sets the node ID.
        
        Args:
            address: Controller address (0x80 to 0x87)
            nodeid: Node ID (0 to 255)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.SETNODEID, nodeid, types=["byte"])

    def GetNodeID(self, address: int) -> NodeIDResult:
        """Gets the node ID.
        
        Args:
            address: Controller address (0x80 to 0x87)
        
        Returns:
            NodeIDResult: (success, nodeid)
                success: True if read successful
                nodeid: Node ID
        """
        return self._read(address, Commands.GETNODEID, types=["byte"])

    def SetPWMIdle(self, address: int, idledelay1: float, idlemode1: bool, idledelay2: float, idlemode2: bool) -> bool:
        """Sets the PWM idle parameters.
    
        Args:
            address: Controller address (0x80 to 0x87)
            idledelay1: Idle delay 1 (0 to 12.7 seconds)
            idlemode1: Idle mode 1 (True = enable, False = disable)
            idledelay2: Idle delay 2 (0 to 12.7 seconds)
            idlemode2: Idle mode 2 (True = enable, False = disable)
    
        Returns:
            bool: True if successful
        """
        byte1 = (int(idledelay1 * 10) & 0x7F) | (0x80 if idlemode1 else 0x00)
        byte2 = (int(idledelay2 * 10) & 0x7F) | (0x80 if idlemode2 else 0x00)   
        return self._write(address, Commands.SETPWMIDLE, byte1, byte2, types=["byte", "byte"])
        
    def GetPWMIdle(self, address: int) -> PWMIdleResult:
        """Gets the PWM idle parameters.
    
        Args:
            address: Controller address (0x80 to 0x87)
    
        Returns:
            PWMIdleResult: (success, idledelay1, idlemode1, idledelay2, idlemode2)
                success: True if read successful
                idledelay1: Idle delay 1 (0 to 12.7 seconds)
                idlemode1: Idle mode 1 (True = enable, False = disable)
                idledelay2: Idle delay 2 (0 to 12.7 seconds)
                idlemode2: Idle mode 2 (True = enable, False = disable)
        """
        result = self._read(address, Commands.GETPWMIDLE, types=["byte", "byte"])
    
        if result[0]:
            val1 = result[1]
            val2 = result[2]
            idledelay1 = float(val1 & 0x7F) / 10
            idlemode1 = bool(val1 & 0x80)
            idledelay2 = float(val2 & 0x7F) / 10
            idlemode2 = bool(val2 & 0x80)
            return (True, idledelay1, idlemode1, idledelay2, idlemode2)    
        return (False, 0, False, 0, False)
        
    def CANGetESR(self, address: int) -> CANBufferResult:
        """Gets CAN ESR register.
    
        Args:
            address: Controller address (0x80 to 0x87)
    
        Returns:
            CANGetESRResult: (success, count)
                success: True if read successful
                ESR: ESR Register value
        """
        return self._read(address, Commands.CANGETESR, types=["long"])

    def CANPutPacket(self, address: int, cob_id: int, RTR: int, data: List[int]) -> bool:
        """Sends a CAN packet.

        Args:
            address: Controller address (0x80 to 0x87)
            cob_id: CAN object identifier (0 to 2047)
            RTR: Remote Transmission Request (0 = data frame, 1 = remote frame)
            data: List of data bytes (length must be <= 8 bytes)

        Returns:
            bool: True if successful
        
        Notes:
            - Data will be padded with zeros to always send 8 bytes
            - For RTR=1, data length is still needed but data content is ignored
            
        Raises:
            ValueError: If data length is more than 8 bytes
        """
        length = len(data)
        if length > 8:
            raise ValueError("Data length must be no more than 8 bytes")

        # Pad data to 8 bytes with 0s
        padded_data = data.copy()
        padded_data.extend([0] * (8 - length))
    
        # First the command-specific arguments (cob_id, RTR, length)
        # Then all 8 bytes of data (padded with zeros if needed)
        return self._write(address, Commands.CANPUTPACKET, cob_id, RTR, length, 
                          padded_data[0], padded_data[1], padded_data[2], padded_data[3], 
                          padded_data[4], padded_data[5], padded_data[6], padded_data[7],
                          types=["word", "byte", "byte", "byte", "byte", "byte", "byte", "byte", "byte", "byte", "byte"]
                         )

    def CANGetPacket(self, address: int) -> CANPacketResult:
        """Reads a CAN packet from the controller.

        Args:
            address: Controller address (0x80 to 0x87)

        Returns:
            CANPacketResult: (success, cob_id, RTR, length, data)
                success: True if read successful
                valid: True if CAN packet was available
                cob_id: CAN object identifier
                RTR: Remote Transmission Request (0 = data frame, 1 = remote frame)
                length: Length of the data (actual valid bytes)
                data: List of data bytes (always 8 bytes, padded with zeros)
        """
        val = self._read(address, Commands.CANGETPACKET, types=["byte", "word", "byte", "byte", "byte", "byte", "byte", "byte", "byte", "byte", "byte", "byte"])
        if val[0]:
            # First byte (0xFF) is a validation marker for valid packet
            if val[1] == 0xFF:
                cob_id = val[2]
                RTR = val[3]
                length = val[4]
                # Extract data bytes (all 8)
                data = [val[i+5] for i in range(8)]
                return (True, True, cob_id, RTR, length, data)
            else:
                return (True, False, 0, 0, 0, [])
    
        return (False, False, 0, 0, 0, [])

    def CANOpenWriteLocalDict(self, address: int, bNodeID: int, wIndex: int, bSubindex: int, lValue: int, bSize: int) -> Tuple[bool, int]:
        """Writes to the local CANopen dictionary.

        Args:
            address: Controller address (0x80 to 0x87)
            bNodeID: Node number of CANOpen device to be accessed(0 for local dictionary)
            wIndex: Index in the dictionary (16-bit)
            bSubindex: Subindex in the dictionary (8-bit)
            lValue: Value to write (32-bit)
            bSize: Size of the value in bytes (1, 2, or 4)

        Returns:
            Tuple[bool, int]: (success, lResult)
                success: True if successful
                lResult: Result of the write operation (0 = success, non-zero = error code)
        """
        logger.debug(f"Writing to CANopen dictionary at address=0x{address:02x}, index=0x{wIndex:04x}, subindex=0x{bSubindex:02x}, value={lValue}, size={bSize}")

        for _ in range(self._trystimeout):
            self._sendcommand(address, Commands.CANOPENWRITEDICT)
            self._writebyte(bNodeID)
            self._writeword(wIndex)
            self._writebyte(bSubindex)
            self._writelong(lValue)
            self._writebyte(bSize)
        
            if self._writechecksum():
                self.crc_clear()
                lResult = self._readlong()
                if lResult[0]:
                    crc = self._readchecksumword()
                    if crc[0] and self._crc & 0xFFFF == crc[1] & 0xFFFF:
                        return (True, lResult[1])
                    else:
                        logger.debug(f"CRC check failed: received=0x{crc[1]:04x}, calculated=0x{self._crc & 0xFFFF:04x}")
                else:
                    logger.debug("Failed to read result value")
            else:
                logger.debug("Failed to write checksum")

        logger.error(f"Failed to write to CANopen dictionary at address=0x{address:02x}, index=0x{wIndex:04x}, subindex=0x{bSubindex:02x} after {self._trystimeout} attempts")
        return (False, 0)

    def CANOpenReadLocalDict(self, address: int, bNodeID: int, wIndex: int, bSubindex: int) -> CANOpenResult:
        """Reads from the local CANopen dictionary.
    
        Args:
            address: Controller address (0x80 to 0x87)
            bNodeID: Node number of CANOpen device to be accessed(0 for local dictionary)
            wIndex: Index in the dictionary (16-bit)
            bSubindex: Subindex in the dictionary (8-bit)
    
        Returns:
            CANOpenResult: (success, lValue, bSize, bType, lResult)
                success: True if read successful
                lValue: Value read (32-bit)
                bSize: Size of the value in bytes (1, 2, or 4)
                bType: Type of the value (0 = integer, 1 = boolean, 2 = string)
                lResult: Result of the read operation (0 = success, non-zero = error code)
        """
        logger.debug(f"Reading from CANopen dictionary at address=0x{address:02x}, index=0x{wIndex:04x}, subindex=0x{bSubindex:02x}")

        for _ in range(self._trystimeout):
            self._sendcommand(address, Commands.CANOPENREADDICT)
            self._writebyte(bNodeID)
            self._writeword(wIndex)
            self._writebyte(bSubindex)
        
            lValue = self._readlong()
            if not lValue[0]:
                logger.debug("Failed to read lValue")
                continue

            bSize = self._readbyte()
            if not bSize[0]:
                logger.debug("Failed to read bSize")
                continue

            bType = self._readbyte()
            if not bType[0]:
                logger.debug("Failed to read bType")
                continue

            lResult = self._readlong()
            if not lResult[0]:
                logger.debug("Failed to read lResult")
                continue
                
            crc = self._readchecksumword()
            if crc[0] and self._crc & 0xFFFF == crc[1] & 0xFFFF:
                return (True, lValue[1], bSize[1], bType[1], lResult[1])
    
            else:
                logger.debug(f"CRC check failed: received=0x{crc[1]:04x}, calculated=0x{self._crc & 0xFFFF:04x}")
    
        logger.error(f"Failed to read from CANopen dictionary after {self._trystimeout} attempts: address=0x{address:02x}, index=0x{wIndex:04x}, subindex=0x{bSubindex:02x}")
        return (False, 0, 0, 0, 0)

    def ResetEStop(self, address: int) -> bool:
        """Resets the emergency stop.
        
        Args:
            address: Controller address (0x80 to 0x87)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.RESETESTOP)

    def SetEStopLock(self, address: int, state: int) -> bool:
        """Sets the emergency stop lock state.
    
        Args:
            address: Controller address (0x80 to 0x87)
            state: State value:
                - 0x55: Automatic reset (resumes after e-stop condition clears)
                - 0xAA: Software reset (requires ResetEStop command)
                - 0: Hardware reset (requires physical reset)
    
        Returns:
            bool: True if successful
        
        Raises:
            ValueError: If state value is invalid (not 0x55, 0xAA, or 0)
        """
        if state not in [self.ESTOP_AUTO_RESET, self.ESTOP_SW_RESET, self.ESTOP_HW_RESET]:
            raise ValueError("Invalid state value. Must be 0x55 (auto reset), 0xAA (software reset), or 0 (hardware reset).")
    
        return self._write(address, Commands.SETESTOPLOCK, state, types=["byte"])

    def GetEStopLock(self, address: int) -> EStopLockResult:
        """Gets the emergency stop lock state.
        
        Args:
            address: Controller address (0x80 to 0x87)
        
        Returns:
            EStopLockResult: (success, state)
                success: True if read successful
                state: State value (0x55 for automatic reset, 0xAA for software reset, 0 for hardware reset)
        """
        return self._read(address, Commands.GETESTOPLOCK, types=["byte"])

    def SetScriptAutoRun(self, address: int, scriptauto_time: int) -> bool:
        """Sets the script auto run time.
    
        Args:
            address: Controller address (0x80 to 0x87)
            scriptauto_time: Auto run time in milliseconds. 
                - 0: Script does not autorun
                - 100-65535: Script autoruns after this many milliseconds
    
        Returns:
            bool: True if successful
        
        Notes:
            Values less than 100 (except 0) will not autorun the script due to 
            controller limitations.
        
        Raises:
            ValueError: If scriptauto_time is less than 100 and not 0
        """
        if scriptauto_time < 100 and scriptauto_time != 0:
            raise ValueError("Scriptauto_time value is below 100! Script will not autorun.")
        
        return self._write(address, Commands.SETSCRIPTAUTORUN, scriptauto_time, types=["long"])

    def GetScriptAutoRun(self, address: int) -> ScriptAutoRunResult:
        """Gets the script auto run time.
        
        Args:
            address: Controller address (0x80 to 0x87)
        
        Returns:
            ScriptAutoRunResult: (success, scriptauto_time)
                success: True if read successful
                scriptauto_time: Auto run time in milliseconds
        """
        return self._read(address, Commands.GETSCRIPTAUTORUN, types=["long"])

    def StartScript(self, address: int) -> bool:
        """Starts the script.
        
        Args:
            address: Controller address (0x80 to 0x87)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.STARTSCRIPT)

    def StopScript(self, address: int) -> bool:
        """Stops the script.
        
        Args:
            address: Controller address (0x80 to 0x87)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, Commands.STOPSCRIPT)

    def SetPWMMode(self, address: int, mode1: int, mode2: int) -> bool:
        """
        Sets the PWM modes for both motors.

        Args:
            address: The address of the controller.
            mode1: The PWM mode to set for motor 1.
            mode2: The PWM mode to set for motor 2.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.SETPWMMODE, mode1, mode2, types=["byte", "byte"])

    def ReadPWMMode(self, address: int) -> PWMModeResult:
        """
        Reads the PWM modes for both motors.

        Args:
            address: The address of the controller.

        Returns:
            PWMModeResult: (success, mode1, mode2)
                success: True if read successful.
                mode1: The PWM mode for motor 1.
                mode2: The PWM mode for motor 2.
        """
        return self._read(address, Commands.GETPWMMODE, types=["byte", "byte"])

    def ReadEeprom(self, address: int, ee_address: int) -> EEPROMResult:
        """
        Reads a word from the EEPROM.

        Args:
            address: The address of the controller (0x80-0x87)
            ee_address: The EEPROM address to read from (0-255)

        Returns:
            EEPROMResult: (success, value)
                success: True if read successful
                value: The word value read from EEPROM (16-bit) or error code:
                    -0x10000: Timeout error occurred
                    -0x20000: CRC mismatch error
                    -0x30000: Write error occurred
        """
        # Define error codes as constants for better readability
        CRC_MISMATCH = -0x20000
        TIMEOUT_ERROR = -0x10000
        WRITE_ERROR = -0x30000
    
        logger.info(f"Reading EEPROM from controller address=0x{address:02x}, EEPROM address=0x{ee_address:02x}")
    
        for _ in range(self._trystimeout):
            try:
                self._port.flushInput()
            except Exception as e:
                logger.warning(f"Error flushing input buffer during EEPROM read: {str(e)}")
                # Continue trying despite the error
            
            self._sendcommand(address, Commands.READEEPROM)
            self.crc_update(ee_address)

            try:
                self._port.write(bytes([ee_address & 0xFF]))
            except (serial.SerialException, ValueError) as e:
                logger.error(f"Serial communication error while writing EEPROM address: {str(e)}")
                return (self.FAILURE, WRITE_ERROR)
            except Exception as e:
                logger.error(f"Unexpected error during EEPROM address write: {str(e)}")
                return (self.FAILURE, WRITE_ERROR)
        
            # Read response
            val = self._readword()
            if not val[0]:
                logger.debug("Failed to read value from EEPROM")
                continue
        
            # Validate checksum
            crc = self._readchecksumword()
            if not crc[0]:
                logger.debug("Failed to read CRC checksum")
                continue
        
            # Check if CRC matches
            if self._crc & 0xFFFF == crc[1] & 0xFFFF:
                return (self.SUCCESS, val[1])
            else:
                logger.debug(f"CRC mismatch: calculated=0x{self._crc & 0xFFFF:04x}, received=0x{crc[1]:04x}")
                return (self.FAILURE, CRC_MISMATCH)
    
        logger.error(f"Failed to read from EEPROM after {self._trystimeout} attempts: controller address=0x{address:02x}, EEPROM address=0x{ee_address:02x}")
        return (self.FAILURE, TIMEOUT_ERROR)

    def WriteEeprom(self, address: int, ee_address: int, ee_word: int) -> bool:
        """
        Writes a word to the EEPROM.

        Args:
            address: The address of the controller.
            ee_address: The EEPROM address to write to.
            ee_word: The word value to write to EEPROM.

        Returns:
            bool: True if successful.
        """
        return self._write(address, Commands.WRITEEEPROM, ee_address, ee_word >> 8, ee_word & 0xFF, types=["byte", "byte", "byte"])
        
