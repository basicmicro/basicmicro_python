"""
Basicmicro Interface Package for controlling Basicmicro motor controllers.

This package provides a comprehensive interface for communicating with
Basicmicro motor controllers using the Basicmicro packet serial mode.

Modules:
    controller: Main controller interface class
    commands: Command enumerations for the controller
    utils: Utility functions for CRC calculation and data handling
    exceptions: Custom exceptions for the library
    types: Type definitions for type hinting
"""

from basicmicro.controller import Basicmicro
from basicmicro.commands import Commands

__version__ = "2.0.10"
__all__ = ["Basicmicro", "Commands"]