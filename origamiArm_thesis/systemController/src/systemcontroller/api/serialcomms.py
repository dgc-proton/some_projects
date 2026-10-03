import sys
from pprint import pp
from time import sleep
from typing import Any, Final

import serial

from systemcontroller.api.motorstepclass import MotorSteps
from systemcontroller.api.serialencoding import ERRORCODES, MSG


def connect(*, config: dict[str, Any]) -> serial.Serial:
    """Establishes serial comms with the motor control MCU."""
    TIMEOUT: Final[int] = 5
    print("Opening serial connection to MCU...")
    try:
        serial_comm: serial.Serial = serial.Serial(
            port=config["comms"]["usb_serial_port"],
            baudrate=config["comms"]["baudrate"],
            timeout=config["comms"]["timeout"],
            write_timeout=config["comms"]["write_timeout"],
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
    print(f"connect is waiting {TIMEOUT} seconds")
    sleep(TIMEOUT)  # allow time for MCU to reset then calibrate
    responses: str = serial_comm.read_all()
    print("Initial messages from the MCU:")
    pp(responses)
    return serial_comm


def move_to(*, serial_interface: serial.Serial, mstep_holder: MotorSteps) -> bool:
    """Send lists of motor steps to the MCU to move the arm."""
    # reset the serial buffers so that the next message sent is the list of steps,
    # and the next message received will have been sent by the motorcontroller after
    # it has received this message
    serial_interface.reset_output_buffer()
    serial_interface.reset_input_buffer()
    serial_interface.write(
        f"{MSG.start}{MSG.type.angles}{mstep_holder.left[0]:03d}&{mstep_holder.right[0]:03d}".encode(
            encoding="ascii"
        )
    )
    serial_interface.flush()

    for i in range(1, mstep_holder.list_length.value):
        serial_interface.write(
            f"{MSG.data_sep}{mstep_holder.left[i]:03d}{MSG.angle_sep}{mstep_holder.right[i]:03d}".encode(
                encoding="ascii"
            )
        )
        serial_interface.flush()

    serial_interface.write(f"{MSG.end}".encode(encoding="ascii"))
    serial_interface.flush()

    return True


def handshake(*, serial_interface: serial.Serial, num_tries: int = 5) -> bool:
    """Attempts handshake with MCU, returns true if successful else false."""
    serial_interface.reset_input_buffer()  # clear input buffer
    MAX_READ: Final[int] = 1000  # max bytes to read
    for attempt in range(num_tries):
        print(f"MCU handshake attempt {attempt}...")
        serial_interface.reset_input_buffer()
        serial_interface.reset_output_buffer()
        serial_interface.write(
            f"{MSG.start}{MSG.type.enq}{MSG.end}".encode(encoding="ascii")
        )  # send enquire message
        serial_interface.flush()  # force transmission of buffered data
        sleep(3)
        try:
            response: bytes = serial_interface.read_until(
                MSG.start.encode(encoding="ascii"), size=MAX_READ
            )
            response: bytes = serial_interface.read_until(
                MSG.end.encode(encoding="ascii"), size=MAX_READ
            )
        except serial.SerialException as e:
            print("Comms error waiting to receive message: ", end="")
            print(e)
            continue
        decoded: str = response.decode("ascii")
        if (len(decoded) > 1) and (
            (decoded[0] == MSG.type.ack) or (decoded[1] == MSG.type.ack)
        ):
            print("Correct handshake response received")
            return True
        print(f"Incorrect handshake response received:\n{decoded}")
        sleep(3)
    return False


def receive_position(*, sercom: serial.Serial) -> tuple[int | None, int | None]:
    """Receives current position in absolute steps from the motorcontroller."""
    MAX_READ: Final[int] = 1000  # max bytes to read
    try:
        response: bytes = sercom.read_until(
            MSG.start.encode(encoding="ascii"), size=MAX_READ
        )
        response: bytes = sercom.read_until(
            MSG.end.encode(encoding="ascii"), size=MAX_READ
        )
    except serial.SerialException as e:
        print("Comms error waiting to receive message: ", end="")
        print(e)
        return None, None
    decoded: str = response.decode("ascii")
    # print(f"receive_position got: {decoded}")
    if len(decoded) < 8:
        return None, None
    if decoded[0] != MSG.type.sendpos:
        return None, None
    try:
        left: int = int(decoded[1:4])
        right: int = int(decoded[5:8])
    except ValueError:
        return None, None
    return left, right


def send_msg_enq(*, sercomms: serial.Serial) -> None:
    """Send an enquire type message, resetting buffers first, then flushing and waiting."""
    sercomms.reset_input_buffer()
    sercomms.reset_output_buffer()
    sercomms.write(f"{MSG.start}{MSG.type.enq}{MSG.end}".encode(encoding="ascii"))
    sercomms.flush()
    sleep(0.1)
    return


def receive_ack_error(*, sercom: serial.Serial) -> tuple[str | None, str | None]:
    """Attempts to receive an ack or error message. Blocking.

    Returns (msg type, error description).
    """
    MAX_READ: Final[int] = 20
    try:
        response: bytes = sercom.read_until(
            MSG.start.encode(encoding="ascii"), size=MAX_READ
        )
        response: bytes = sercom.read_until(
            MSG.end.encode(encoding="ascii"), size=MAX_READ
        )
    except serial.SerialException as e:
        print("Comms error waiting to receive message: ", end="")
        print(e)
        return None, None
    decoded: str = response.decode("ascii")
    if len(decoded) < 2:
        return None, None
    match decoded[0]:
        case MSG.type.ack:
            return MSG.type.ack, None
        case MSG.type.errorcode:
            if decoded[1] in ERRORCODES:
                return MSG.type.errorcode, ERRORCODES[decoded[1]]
    return None, None  # if reaches here message was invalid
