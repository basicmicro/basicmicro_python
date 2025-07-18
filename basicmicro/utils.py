"""
Utility functions for the Basicmicro package.
"""

import logging
from typing import List, Tuple

logger = logging.getLogger(__name__)


def initialize_crc_table(polynomial: int = 0x1021) -> List[int]:
    """Initialize a CRC lookup table for faster CRC calculations.
    
    Pre-computes a 256-element lookup table based on the provided CRC polynomial
    to accelerate CRC calculations during serial communications.
    
    Args:
        polynomial: The CRC polynomial to use (default: 0x1021)
            Common values: 0x1021 (CCITT), 0x8005 (CRC-16)
        
    Returns:
        List[int]: The pre-computed CRC table with 256 16-bit values
    """
    table = [0] * 256
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ polynomial) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
            table[i] = crc
    return table


def calc_mixed(fb: int, lr: int) -> Tuple[int, int]:
    """Calculate mixed mode drive values for differential steering.
    
    This utility function transforms forward/backward and left/right commands
    into individual motor commands for differential drive systems.
    
    Args:
        fb: Forward/backward value (-32767 to +32767)
            Positive = forward, negative = backward
        lr: Left/right value (-32767 to +32767)
            Positive = right, negative = left
    
    Returns:
        Tuple[int, int]: Tuple of mixed mode values (left_motor, right_motor)
            Both values will be in the range -32767 to +32767
    """
    # Force conversion to integers and check if signs are different
    fb = int(fb)
    lr = int(lr)
    
    if (lr < 0) != (fb < 0):  # Signs are different?
        if abs(lr) > abs(fb):
            out1 = -lr
        else:
            out1 = fb
        out0 = fb + lr
    else:
        if abs(fb) > abs(lr):
            out0 = fb
        else:
            out0 = lr
        out1 = fb - lr
    
    return (out0, out1)