"""A quick script for testing the motors."""

import cProfile
import ctypes
import io
import itertools
import pstats
import sys
from dataclasses import dataclass, field, fields
from enum import Enum
from pprint import pp
from time import sleep
from typing import Final, Self

import serial

MAX_MOVES: Final[int] = 128

DEBUG_TIME_EX: bool = True


@dataclass(frozen=True, kw_only=True)
class MsgTypeEncoding:
    """Fixed data defining encoding for message types."""

    enq: Final[str]
    ack: Final[str]
    angles: Final[str]
    errorcode: Final[str]


@dataclass(frozen=True, kw_only=True)
class MsgEncoding:
    """Fixed data defining message encoding."""

    start: Final[str]
    end: Final[str]
    angle_sep: Final[str]
    data_sep: Final[str]
    type: MsgTypeEncoding


MSG: Final[MsgEncoding] = MsgEncoding(
    start="$",
    end="%",
    angle_sep="&",
    data_sep=",",
    type=MsgTypeEncoding(enq="?", ack="#", angles=">", errorcode="!"),
)


@dataclass(frozen=False, kw_only=True)
class MotorStepLists:
    """Holds step data for motors."""

    left: ctypes.Array
    right: ctypes.Array
    list_length: int = 0
    valid: bool = False


def main() -> None:
    motor_steps = MotorStepLists(
        left=(ctypes.c_uint * MAX_MOVES)(), right=(ctypes.c_uint * MAX_MOVES)()
    )

    # setup serial connection
    print("Opening serial connection to MCU...")
    try:
        serial_comm: serial.Serial = serial.Serial(
            port="/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_55731323735351303270-if00",
            baudrate=115200,
            timeout=2,
            write_timeout=2,
        )
    except serial.SerialException as ser_exception:
        print(f"Serial port error: {ser_exception}, exiting...")
        sys.exit()
    except PermissionError:
        print("Serial port permission denied (check user permissions), exiting...")
        sys.exit()
    except FileNotFoundError:
        print("Port not found (check device connection), exiting...")
        sys.exit()
    sleep(2)  # allow time for MCU to reset (common Arduino connection issue)
    responses: str = serial_comm.read_all()
    print("Initial messages from the MCU:")
    pp(responses)

    # perform handshake with MCU
    serial_handshake(serial_interface=serial_comm)

    # add test data for motor steps
    test_moves: Final[int] = 128
    assert test_moves <= MAX_MOVES
    for i in range(test_moves):
        motor_steps.left[i] = i
        motor_steps.right[i] = test_moves - i
    motor_steps.list_length = test_moves
    motor_steps.valid = True

    # send steps to MCU
    move_to(
        serial_interface=serial_comm, coord_x=0, coord_y=0, mstep_holder=motor_steps
    )
    sleep(1)
    print("Response to initial move_to() call:")
    pp(str(serial_comm.read_all()))
    print()

    # repeated function calls for profiling
    print("Will now complete code profiling...")
    pr = cProfile.Profile()
    pr.enable()
    for _ in range(100):
        move_to(
            serial_interface=serial_comm, coord_x=0, coord_y=0, mstep_holder=motor_steps
        )
    pr.disable()
    s = io.StringIO()
    results = pstats.Stats(pr, stream=s)
    results.strip_dirs().sort_stats("cumulative").print_stats()
    print(s.getvalue())


def move_to(
    *,
    serial_interface: serial.Serial,
    coord_x: int,
    coord_y: int,
    mstep_holder: MotorStepLists,
) -> bool:
    """Send instructions to the MCU to move the arm to the specified coordinates. Returns success of operation."""
    # NOTE: call to Zaynes function would go here, coords in, step lists out
    if not mstep_holder.valid:
        return False  # step function failed
    # send new data to MCU
    serial_interface.write(
        f"{MSG.start}{MSG.type.angles}{mstep_holder.left[0]:03d}&{mstep_holder.right[0]:03d}".encode(
            encoding="ascii"
        )
    )
    for i in range(1, mstep_holder.list_length):
        serial_interface.write(
            f"{MSG.data_sep}{mstep_holder.left[i]:03d}{MSG.angle_sep}{mstep_holder.right[i]:03d}".encode(
                encoding="ascii"
            )
        )
        serial_interface.flush()
    serial_interface.write(f"{MSG.end}".encode(encoding="ascii"))

    return True


def serial_handshake(*, serial_interface: serial.Serial, num_tries: int = 5) -> bool:
    """Attempts handshake with MCU, returns true if successful else false."""
    serial_interface.reset_input_buffer()  # clear input buffer
    for attempt in range(num_tries):
        print(f"MCU handshake attempt {attempt}...")
        serial_interface.write(
            f"{MSG.start}{MSG.type.enq}{MSG.end}".encode(encoding="ascii")
        )  # send enquire message
        serial_interface.flush()  # force transmission of buffered data
        sleep(1)
        # response: bytes = serial_interface.read_all()
        # if response == f"{MSG.start}{MSG.type.ack}{MSG.end}".encode(encoding="ascii"):
        response: bytes = serial_interface.read_until(
            MSG.start.encode(encoding="ascii")
        )
        response: bytes = serial_interface.read_until(MSG.end.encode(encoding="ascii"))
        if response[:-1] == f"{MSG.type.ack}".encode(encoding="ascii"):
            print("Correct handshake response received")
            return True
        print(f"Incorrect handshake response received:\n{response}")
    return False


if __name__ == "__main__":
    main()
