# Basicmicro Python Library

A modernized Python 3 library for controlling Basicmicro motor controllers using the Basicmicro packet serial mode.

## Features

- Full support for all Basicmicro packet serial commands
- Comprehensive type hints for better IDE integration
- Detailed logging for debugging
- Context manager support for safe resource handling
- Exception handling for robust error management
- Extensive documentation
- Modular file structure for better maintainability

## Installation

### From PyPI (recommended)

```bash
pip install basicmicro
```

### From Source

Clone the repository and install:

```bash
git clone https://github.com/basicmicro/basicmicro_python.git
cd basicmicro_python
pip install -e .
```

### Platform-Specific Instructions

#### Windows

On Windows, you'll need to identify the correct COM port for your controller:

1. Open Device Manager (right-click on Start Menu → Device Manager)
2. Expand "Ports (COM & LPT)"
3. Find your controller (e.g., "USB Serial Device") and note the COM port (e.g., COM3)

Example usage:
```python
controller = Basicmicro("COM3", 38400)
```

#### Linux

On Linux, the serial port is typically in the /dev directory:

```python
controller = Basicmicro("/dev/ttyACM0", 38400)  # or /dev/ttyUSB0
```

You may need to add your user to the 'dialout' group for permission to access serial ports:
```bash
sudo usermod -a -G dialout $USER
```
Then log out and log back in for changes to take effect.

#### macOS

On macOS, the serial port will be in the /dev directory:

```python
controller = Basicmicro("/dev/tty.usbserial-XXXXXXXX", 38400)
```

The exact name will depend on your USB-serial adapter.

## Quick Start

```python
import logging
from basicmicro import Basicmicro

# Enable logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

# Using context manager for automatic resource handling
with Basicmicro("/dev/ttyACM0", 38400) as controller:  # Use "COM3" on Windows
    address = 0x80  # Default address

    # Simple motor control
    controller.DutyM1(address, 16384)  # Half speed forward for motor 1
    controller.DutyM2(address, -8192)  # Quarter speed backward for motor 2

    # Read encoder values
    enc1 = controller.ReadEncM1(address)
    if enc1[0]:  # Check if read was successful
        print(f"Encoder 1 count: {enc1[1]}")

    # Set velocity PID values
    controller.SetM1VelocityPID(address, kp=1.0, ki=0.5, kd=0.25, qpps=44000)
```

## Usage Examples

### Basic Motor Control

```python
from basicmicro import Basicmicro

# Initialize and open connection
controller = Basicmicro("/dev/ttyACM0", 38400)  # Use "COM3" on Windows
controller.Open()

address = 0x80  # Default controller address

# Control motors by duty cycle (-32767 to +32767)
controller.DutyM1(address, 16384)  # 50% forward
controller.DutyM2(address, -8192)  # 25% backward

# Control motors by speed (encoder counts per second)
controller.SpeedM1(address, 1000)  # 1000 counts per second forward
controller.SpeedM2(address, -500)  # 500 counts per second backward

# Control both motors simultaneously
controller.DutyM1M2(address, 8192, -8192)  # Motor 1 forward, Motor 2 backward

# Always close connection when done
controller.close()
```

### Using Context Manager

```python
from basicmicro import Basicmicro

# The context manager automatically closes the connection when done
with Basicmicro("/dev/ttyACM0", 38400) as controller:  # Use "COM3" on Windows
    address = 0x80
    
    # Read battery voltages
    main_batt = controller.ReadMainBatteryVoltage(address)
    logic_batt = controller.ReadLogicBatteryVoltage(address)
    
    if main_batt[0] and logic_batt[0]:
        print(f"Main battery: {main_batt[1]/10.0}V")
        print(f"Logic battery: {logic_batt[1]/10.0}V")
    
    # Read temperatures
    temp = controller.ReadTemp(address)
    if temp[0]:
        print(f"Temperature: {temp[1]/10.0}°C")
```

### Reading Encoders and Speed

```python
from basicmicro import Basicmicro
import time

controller = Basicmicro("/dev/ttyACM0", 38400)  # Use "COM3" on Windows
controller.Open()
address = 0x80

# Reset encoders to zero
controller.ResetEncoders(address)

# Set motor speed
controller.SpeedM1(address, 1000)  # 1000 counts per second

# Monitor encoders and speed
try:
    for _ in range(10):
        enc = controller.ReadEncM1(address)
        speed = controller.ReadSpeedM1(address)
        
        if enc[0] and speed[0]:
            print(f"Encoder: {enc[1]}, Speed: {speed[1]} counts/sec, Status: {enc[2]}")
        
        time.sleep(0.5)
finally:
    controller.DutyM1(address, 0)  # Stop motor
    controller.close()
```

### Setting PID Parameters

```python
from basicmicro import Basicmicro

controller = Basicmicro("/dev/ttyACM0", 38400)  # Use "COM3" on Windows
controller.Open()
address = 0x80

# Set velocity PID parameters
kp = 1.0  # Proportional constant
ki = 0.5  # Integral constant
kd = 0.25  # Derivative constant
qpps = 44000  # Maximum speed in quadrature pulses per second

controller.SetM1VelocityPID(address, kp, ki, kd, qpps)

# Read back the PID settings to verify
result = controller.ReadM1VelocityPID(address)
if result[0]:
    print(f"P: {result[1]}, I: {result[2]}, D: {result[3]}, QPPS: {result[4]}")

controller.close()
```

## Examples

For more detailed examples, check the `examples` directory:

1. Basic Movement: Demonstrates fundamental motor control
2. Acceleration & Position: Shows speed and position control with acceleration
3. PID Configuration: Examples of reading and setting PID parameters
4. Status & Diagnostics: Reading controller status and diagnostic information
5. Configuration Management: Managing controller settings
6. CAN Communication: Using the CAN bus interface
7. Mixed Mode & Differential Drive: Controlling differential drive robots
8. Advanced Scripting: Multi-threaded control and complex sequences

Run examples using:
```bash
python -m examples.01_basic_movement -p COM3  # On Windows
python -m examples.01_basic_movement -p /dev/ttyACM0  # On Linux
```

## Logging

The library uses Python's standard logging module. To enable logging:

```python
import logging

# Configure global logging level
logging.basicConfig(
    level=logging.INFO,  # Set to DEBUG for more detailed output
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

# Enable DEBUG level for just the basicmicro module
logging.getLogger('basicmicro').setLevel(logging.DEBUG)
```

## Documentation

For detailed API documentation, see the [API Reference](./docs/api_reference.md).

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

---

## Supported Hardware

This library works with all current Basicmicro motor controllers:

- **[RoboClaw Motor Controllers](https://www.basicmicro.com/motor-controller)** — Dual-channel brushed DC, 2×7A through 2×300A, up to 60VDC
- **[MCP Industrial Motor Controllers](https://www.basicmicro.com/motor-controller-industrial)** — 30A to 60A continuous at 60VDC, CAN bus, RS-232, MCL scripting

## Resources

- **[Motion Studio](https://www.basicmicro.com/motor-controller-downloads)** — Free configuration and PID tuning software (Windows)
- **[Datasheets, User Manuals & CAD](https://www.basicmicro.com/motor-controller-downloads)** — Datasheets, user manuals, CAD models and drawings
- **[Tutorials & Guides](https://resources.basicmicro.com/)** — Articles, guides, and news
- **[Contact Support](https://www.basicmicro.com/Contact_ep_7.html)** — Technical support and sales inquiries



