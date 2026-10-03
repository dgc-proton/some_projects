import ctypes
from dataclasses import dataclass


@dataclass(frozen=False, kw_only=True)
class MotorSteps:
    """Holds step data for motors."""

    left: ctypes.Array
    right: ctypes.Array
    max_list_length: ctypes.c_int
    list_length: ctypes.c_int
    valid: bool = False
