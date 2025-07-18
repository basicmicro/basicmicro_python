"""
Custom exceptions for the Basicmicro package.
"""

class BasicmicroError(Exception):
    """Base exception class for Basicmicro operations"""
    pass

class CommunicationError(BasicmicroError):
    """Exception raised for errors in the communication with the controller."""
    pass


class PacketTimeoutError(BasicmicroError):
    """Exception raised when a packet transmission times out."""
    pass
