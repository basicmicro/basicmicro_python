"""
Type definitions for the Basicmicro package.
    
This module defines common return type annotations used throughout the
Basicmicro package to provide better type hinting for IDEs and static
type checkers.
    
Each type definition represents the return value structure of a corresponding
method in the Basicmicro class. Most return types include a success flag
as the first element, followed by method-specific return values.
    
Type Variables:
    SuccessFlag: Boolean indicating success or failure of an operation
    
Common Return Types:
    ReadResult: Simple read operation result with a value
    EncoderResult: Encoder read result with count and status
    SpeedResult: Speed read result with speed and status
    VoltsResult: Voltage read result with main and logic battery values
    PIDResult: PID parameter read result
    ...and many more specialized result types
"""

from typing import Tuple, List, Dict, Any, Optional, Union, Callable, TypeVar, Generic

# Define common return types
SuccessFlag = bool
ReadResult = Tuple[SuccessFlag, int]
EncoderResult = Tuple[SuccessFlag, int, int]  # success, count, status
SpeedResult = Tuple[SuccessFlag, int, int]  # success, speed, status
VoltsResult = Tuple[SuccessFlag, int, int]  # success, mbat, lbat
CurrentsResult = Tuple[SuccessFlag, int, int]  # success, current1, current2
BuffersResult = Tuple[SuccessFlag, int, int]  # success, buffer1, buffer2
PWMsResult = Tuple[SuccessFlag, int, int]  # success, pwm1, pwm2
TimeoutResult = Tuple[SuccessFlag, float]  # success, timeout
VersionResult = Tuple[SuccessFlag, str]  # success, version
PIDResult = Tuple[SuccessFlag, float, float, float, int]  # success, p, i, d, qpps
PositionPIDResult = Tuple[SuccessFlag, float, float, float, int, int, int, int]  # success, kp, ki, kd, kimax, deadzone, min, max
VoltageResult = Tuple[SuccessFlag, int, int, int]  # success, min, max, auto_offset
StatusResult = Tuple[SuccessFlag, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int]  # complex status
PinFunctionsResult = Tuple[SuccessFlag, int, int, int]  # success, S3mode, S4mode, S5mode
DeadBandResult = Tuple[SuccessFlag, int, int]  # success, min, max
EncodersResult = Tuple[SuccessFlag, int, int]  # success, enc1, enc2
ISpeedsResult = Tuple[SuccessFlag, int, int]  # success, speed1, speed2
AccelsResult = Tuple[SuccessFlag, int, int, int, int]  # success, accel1, accel2, accel3, accel4
TempResult = Tuple[SuccessFlag, int]  # success, temperature
ErrorResult = Tuple[SuccessFlag, int]  # success, error
EncoderModesResult = Tuple[SuccessFlag, int, int]  # success, mode1, mode2
SerialNumberResult = Tuple[SuccessFlag, str]  # success, serial_number
ConfigResult = Tuple[SuccessFlag, int]  # success, config
EncStatusResult = Tuple[SuccessFlag, int, int]  # success, enc1status, enc2status
AutosResult = Tuple[SuccessFlag, int, int]  # success, auto1, auto2
SpeedsResult = Tuple[SuccessFlag, int, int]  # success, speed1, speed2
ErrorLimitResult = Tuple[SuccessFlag, int, int]  # success, limit1, limit2
SpeedErrorsResult = Tuple[SuccessFlag, int, int]  # success, error1, error2
PositionErrorsResult = Tuple[SuccessFlag, int, int]  # success, error1, error2
OffsetsResult = Tuple[SuccessFlag, int, int]  # success, offset1, offset2
LRResult = Tuple[SuccessFlag, float, float]  # success, L, R
MaxCurrentResult = Tuple[SuccessFlag, int, int]  # success, maxi, mini
DOUTSResult = Tuple[SuccessFlag, int, List[int]]  # success, count, actions
PriorityResult = Tuple[SuccessFlag, int, int, int]  # success, priority1, priority2, priority3
AddressMixedResult = Tuple[SuccessFlag, int, int]  # success, new_address, mixed
SignalsResult = Tuple[SuccessFlag, int, List[Dict[str, Any]]]  # success, count, signals
StreamsResult = Tuple[SuccessFlag, int, List[Dict[str, Any]]]  # success, count, streams
SignalsDataResult = Tuple[SuccessFlag, int, List[Dict[str, Any]]]  # success, count, signals_data
NodeIDResult = Tuple[SuccessFlag, int]  # success, nodeid
PWMIdleResult = Tuple[SuccessFlag, float, bool, float, bool]  # success, idledelay1, idlemode1, idledelay2, idlemode2
CANBufferResult = Tuple[SuccessFlag, int]  # success, count
CANPacketResult = Tuple[SuccessFlag, int, int, int, List[int]]  # success, cob_id, RTR, length, data
CANOpenResult = Tuple[SuccessFlag, int, int, int, int]  # success, lValue, bSize, bType, lResult
EStopLockResult = Tuple[SuccessFlag, int]  # success, state
ScriptAutoRunResult = Tuple[SuccessFlag, int]  # success, scriptauto_time
PWMModeResult = Tuple[SuccessFlag, int]  # success, mode
EEPROMResult = Tuple[SuccessFlag, int]  # success, value

# Define command types for type checking
CommandType = int
AddressType = int