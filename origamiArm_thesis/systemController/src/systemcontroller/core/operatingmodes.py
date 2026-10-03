import ctypes
from collections.abc import Sequence
from pprint import pp
from time import sleep
from typing import Any, Final

import cv2 as cv
import numpy as np
import serial

from systemcontroller.api import serialcomms, serialencoding
from systemcontroller.api.motorstepclass import MotorSteps
from systemcontroller.core import annotateframe, aruco
from systemcontroller.core.coreclasses import FrameCoords, LocationData
from systemcontroller.core.visionsetup import FrameCapture
from systemcontroller.utils import userinput
from systemcontroller.utils.tphints import OpenCVframe

# work around to utilise opencv mouse click coordinates
cursor_coords = FrameCoords(x_px=0, y_px=0)


def mode_normal(
    *,
    camerathread: FrameCapture,
    detector: cv.aruco.ArucoDetector,
    kin_lib: ctypes.CDLL,
    puck: LocationData,
    desired_loc: LocationData,
    stp_data: MotorSteps,
    ser: serial.Serial,
    showscreen: bool = False,
) -> None:
    """Run normally with minimum messages etc."""
    SERIAL_TIMEOUT: Final[int] = 4

    ser.timeout = SERIAL_TIMEOUT  # set serial timeout
    last_msg_was_puck_nd = False
    leftmotor_steps: int | None = None
    rightmotor_steps: int | None = None
    detect_result: tuple[int, int] | None

    print("getting initial motor positions from motorcontroller")
    while not (leftmotor_steps and rightmotor_steps):
        serialcomms.send_msg_enq(sercomms=ser)
        leftmotor_steps, rightmotor_steps = serialcomms.receive_position(sercom=ser)
    print("motor controller positions received, launching manual2coordds main loop")

    while True:
        success_steps: bool = False
        frame: OpenCVframe = camerathread.current_frame
        # detect puck
        detect_result = aruco.detect_aruco(
            aframe=frame,
            adetector=detector,
            arcode=5,
        )
        if not detect_result:
            if not last_msg_was_puck_nd:
                print(f"\n{'!! puck not detected !!':^80}\n")
            last_msg_was_puck_nd = True
            if showscreen:
                cv.imshow("Camera", frame)
                cv.waitKey(1)
            continue
        print("puck detected")
        last_msg_was_puck_nd = False
        desired_loc.coords_vis_px_x = detect_result[0]
        desired_loc.coords_vis_px_y = detect_result[1]
        if showscreen:
            annotateframe.circle(
                img=frame,
                coords_xy=(desired_loc.coords_vis_px_x, desired_loc.coords_vis_px_y),
                size_radius=8,
            )
            cv.imshow("Camera", frame)
            cv.waitKey(1)
        # get motor steps from kinematics lib
        success_steps: bool = kin_lib.get_steplists(
            ctypes.byref(stp_data.left),
            ctypes.byref(stp_data.right),
            stp_data.max_list_length,
            ctypes.byref(stp_data.list_length),
            ctypes.c_int(leftmotor_steps),  # left motor current position steps
            ctypes.c_int(rightmotor_steps),  # right motor current position steps
            ctypes.c_int(desired_loc.coords_phys_mm_x),  # desired x
            ctypes.c_int(desired_loc.coords_phys_mm_y),  # desired y
        )
        if not success_steps:
            print(f"\n{'!! kinematics library - movement not possible !!':^80}\n")
            last_msg_was_puck_nd = False
            sleep(0.5)
            continue
        # send coords to the uno
        serialcomms.move_to(serial_interface=ser, mstep_holder=stp_data)
        # get arm positions back
        leftmotor_steps, rightmotor_steps = serialcomms.receive_position(sercom=ser)
        while not (leftmotor_steps and rightmotor_steps):
            serialcomms.send_msg_enq(sercomms=ser)
            leftmotor_steps, rightmotor_steps = serialcomms.receive_position(sercom=ser)


def manual2coords(
    *,
    camerathread: FrameCapture,
    kin_lib: ctypes.CDLL,
    desired_pos: LocationData,
    stp_data: MotorSteps,
    ser: serial.Serial,
) -> None:
    """Go to coordinates provided by user."""
    cv.namedWindow("Camera")
    cv.setMouseCallback("Camera", _onany)
    SERIAL_TIMEOUT: Final[int] = 4
    ser.timeout = SERIAL_TIMEOUT  # set serial timeout
    leftmotor_steps: int | None = None
    rightmotor_steps: int | None = None

    print("getting initial motor positions from motorcontroller")
    while not (leftmotor_steps and rightmotor_steps):
        serialcomms.send_msg_enq(sercomms=ser)
        leftmotor_steps, rightmotor_steps = serialcomms.receive_position(sercom=ser)
    print("motor controller positions received, launching manual2coordds main loop")

    while True:
        success_steps: bool = False
        print("on GUI press [a] to send arm to cursor, or [q] to quit")
        while True:
            # get user input
            frame: OpenCVframe = camerathread.current_frame
            cv.imshow("Camera", frame)
            key: int = cv.waitKey(10)  # required to update GUI properly
            if key == ord("a"):
                desired_pos.coords_vis_px_x = cursor_coords.x_px
                desired_pos.coords_vis_px_y = cursor_coords.y_px
                break
            if key == ord("q"):
                cv.destroyAllWindows()
                return
        annotateframe.circle(
            img=frame,
            coords_xy=(desired_pos.coords_vis_px_x, desired_pos.coords_vis_px_y),
            size_radius=8,
        )
        cv.imshow("Camera", frame)
        cv.waitKey(1)
        # get motor steps from kinematics lib
        print(
            f"asking for steps to: ({desired_pos.coords_phys_mm_x}, {desired_pos.coords_phys_mm_y})",
            end=None,
        )
        print(
            f"Calling kinematics library with motor pos L{leftmotor_steps} R{rightmotor_steps}... ",
            end="",
        )
        success_steps: bool = kin_lib.get_steplists(
            ctypes.byref(stp_data.left),
            ctypes.byref(stp_data.right),
            stp_data.max_list_length,
            ctypes.byref(stp_data.list_length),
            ctypes.c_int(leftmotor_steps),  # current lmotor steps
            ctypes.c_int(rightmotor_steps),  # current rmotor steps
            ctypes.c_int(desired_pos.coords_phys_mm_x),  # desired x
            ctypes.c_int(desired_pos.coords_phys_mm_y),  # desired y
        )
        if not success_steps:
            print("No steps received from kinematics library")
            continue
        # send coords to the uno
        print("Received steps:")
        print("Left")
        for i in range(stp_data.list_length.value):
            pp(stp_data.left[i])
        print("Right")
        for i in range(stp_data.list_length.value):
            pp(stp_data.right[i])
        print("Sending steps to motorcontroller... ", end="")
        serialcomms.move_to(serial_interface=ser, mstep_holder=stp_data)
        print("sent steps to motorcontroller")
        print(
            "checking on response from motorcontroller, press [n] to start polling it"
        )
        leftmotor_steps, rightmotor_steps = None, None
        while not (leftmotor_steps and rightmotor_steps):
            frame = camerathread.current_frame
            annotateframe.circle(
                img=frame,
                coords_xy=(desired_pos.coords_vis_px_x, desired_pos.coords_vis_px_y),
                size_radius=8,
            )
            cv.imshow("Camera", frame)
            key: int = cv.waitKey(1)
            leftmotor_steps, rightmotor_steps = serialcomms.receive_position(sercom=ser)
            if key == ord("n"):
                break
        # get response if not already received
        while not (leftmotor_steps and rightmotor_steps):
            serialcomms.send_msg_enq(sercomms=ser)
            leftmotor_steps, rightmotor_steps = serialcomms.receive_position(sercom=ser)


def mode_visiononly(
    *, camerathread: FrameCapture, detector: cv.aruco.ArucoDetector
) -> None:
    """Display the camera and detection status on a GUI."""
    # complex type hints
    corners_list: Sequence[np.ndarray[Any, np.dtype[np.integer[Any] | np.floating[Any]]]]
    id_list: np.ndarray[Any, np.dtype[np.integer[Any] | np.floating[Any]]]
    # end of complex typehints

    cv.namedWindow("Camera")

    while True:
        frame: OpenCVframe = camerathread.current_frame
        preproc_frame: OpenCVframe = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners_list, id_list, _ = detector.detectMarkers(preproc_frame)
        if id_list is not None:
            for the_id, corners in zip(id_list, corners_list, strict=True):
                corners_unpacked = corners[0].squeeze()
                if the_id == 0:
                    # found the puck
                    puck_x = np.mean(corners_unpacked[:, 0])
                    puck_y = np.mean(corners_unpacked[:, 1])
                    cv.circle(
                        frame,
                        (int(puck_x), int(puck_y)),
                        radius=15,
                        color=(0, 255, 0),
                        thickness=5,
                    )
        cv.imshow("Camera", frame)
        if cv.waitKey(1) == ord("q"):
            break


def mode_coordsonly(*, kin_lib: ctypes.CDLL, buffer_size: int) -> None:
    """Get coordinates to and from from the user, output calculated motor angles."""
    left_array = (ctypes.c_int * buffer_size)()
    right_array = (ctypes.c_int * buffer_size)()
    actual_listsize = ctypes.c_int()
    while True:
        # get positions from user
        print("Input data to send to the kinematics library:")
        success: bool = kin_lib.get_steplists(
            ctypes.byref(left_array),
            ctypes.byref(right_array),
            ctypes.c_int(buffer_size),
            ctypes.byref(actual_listsize),
            ctypes.c_int(userinput.get_int(prompt="Current left motor steps")),
            ctypes.c_int(userinput.get_int(prompt="Current right motor steps")),
            ctypes.c_int(userinput.get_int(prompt="Desired x coordinate")),
            ctypes.c_int(userinput.get_int(prompt="Desired y coordinate")),
        )
        print("** Output from kinematics library:")
        print(f"** Success: {success}")
        print(f"** Actual list size: {actual_listsize}")
        print("** Step values left motor: ")
        for i in range(actual_listsize.value):
            pp(left_array[i])
        print("** Step values right motor: ")
        for i in range(actual_listsize.value):
            pp(right_array[i])
        print("** End of data")
        opt: str = input("q to exit: ")
        if opt in ["q", "Q"]:
            return


def mode_manualstep(*, step_dat: MotorSteps, step_max: int, ser: serial.Serial) -> None:
    """Manually send lists of steps to the motors."""
    leftmotor_steps: int | None = None
    rightmotor_steps: int | None = None

    print("getting initial motor positions from motorcontroller")
    while not (leftmotor_steps and rightmotor_steps):
        serialcomms.send_msg_enq(sercomms=ser)
        leftmotor_steps, rightmotor_steps = serialcomms.receive_position(sercom=ser)
    print("motor controller positions received, launching manualstep main loop")

    while True:
        print(
            "Current motor positions reported by motorcontroller:"
            f"L{leftmotor_steps} R{rightmotor_steps}"
        )
        _step_input(stp_data=step_dat, max_steps=step_max)
        print("Sending step data to MCU...")
        _ = ser.read_all()
        serialcomms.move_to(serial_interface=ser, mstep_holder=step_dat)
        print("sent steps to motorcontroller")
        print("checking on response from motorcontroller, [ctrl c] to start polling it")
        leftmotor_steps, rightmotor_steps = None, None
        while not (leftmotor_steps and rightmotor_steps):
            try:
                leftmotor_steps, rightmotor_steps = serialcomms.receive_position(
                    sercom=ser
                )
            except KeyboardInterrupt:
                break
        # get response if not already received
        while not (leftmotor_steps and rightmotor_steps):
            serialcomms.send_msg_enq(sercomms=ser)
            leftmotor_steps, rightmotor_steps = serialcomms.receive_position(sercom=ser)
        response: str = input("q to exit, anything else to send more data: ")
        if response in ["q", "Q"]:
            break
    return


def side2side_manual(
    *,
    stp_data: MotorSteps,
    ser: serial.Serial,
    lmotor_largest_step: int,
    rmotor_smallest_step: int,
) -> None:
    """Move the arm side to side to test for skipped steps."""
    SERIAL_TIMEOUT: Final[float] = 1.5
    leftmotor_steps: int | None = None
    rightmotor_steps: int | None = None
    ser.timeout = SERIAL_TIMEOUT  # set serial timeout

    print("getting initial motor positions from motorcontroller")
    while not (leftmotor_steps and rightmotor_steps):
        serialcomms.send_msg_enq(sercomms=ser)
        leftmotor_steps, rightmotor_steps = serialcomms.receive_position(sercom=ser)
    print("motor controller positions received, launching side2side_manual main loop")

    left: list[int] = []
    right: list[int] = []

    # calculate the movement step lists
    i: int = 0
    breakpoint()
    # go right
    while (rightmotor_steps > rmotor_smallest_step) and (
        leftmotor_steps < lmotor_largest_step
    ):
        left.append(leftmotor_steps)
        right.append(rightmotor_steps)
        i += 1
        leftmotor_steps -= 1
        rightmotor_steps -= 1
    # go left
    leftmotor_steps += 1
    rightmotor_steps += 1
    while (rightmotor_steps > rmotor_smallest_step) and (
        leftmotor_steps < lmotor_largest_step
    ):
        left.append(leftmotor_steps)
        right.append(rightmotor_steps)
        i += 1
        leftmotor_steps += 1
        rightmotor_steps += 1
    # go right
    leftmotor_steps -= 1
    rightmotor_steps -= 1
    while (rightmotor_steps > rmotor_smallest_step) and (
        leftmotor_steps < lmotor_largest_step
    ):
        left.append(leftmotor_steps)
        right.append(rightmotor_steps)
        i += 1
        leftmotor_steps -= 1
        rightmotor_steps -= 1

    stp_data.list_length = ctypes.c_int(1)

    while True:
        for i in range(len(left)):
            # send coords to the uno
            stp_data.left[0] = left[i]
            stp_data.right[0] = right[i]
            print(
                f"will send steps L{left[i]} R{right[i]} to motorcontroller... ", end=""
            )
            input("press enter to continue")
            serialcomms.move_to(serial_interface=ser, mstep_holder=stp_data)
            print("sent steps to motorcontroller...", end="")
            print("checking response from motorcontroller...", end="")
            leftsteps, rightsteps = None, None
            while not (leftsteps and rightsteps):
                leftsteps, rightsteps = serialcomms.receive_position(sercom=ser)
            print(f"response received: L{leftsteps} R{rightsteps}")


def side2side_auto(
    *,
    stp_data: MotorSteps,
    ser: serial.Serial,
    lmotor_largest_step: int,
    rmotor_smallest_step: int,
) -> None:
    """Move the arm side to side to test for skipped steps."""
    SERIAL_TIMEOUT: Final[float] = 1.5
    leftmotor_steps: int | None = None
    rightmotor_steps: int | None = None
    ser.timeout = SERIAL_TIMEOUT  # set serial timeout

    print("getting initial motor positions from motorcontroller")
    while not (leftmotor_steps and rightmotor_steps):
        serialcomms.send_msg_enq(sercomms=ser)
        leftmotor_steps, rightmotor_steps = serialcomms.receive_position(sercom=ser)
    print("motor controller positions received, launching side2side_auto main loop")

    # calculate the movement step lists
    i: int = 0
    # go right
    while (rightmotor_steps > rmotor_smallest_step) and (
        leftmotor_steps < lmotor_largest_step
    ):
        stp_data.left[i] = leftmotor_steps
        stp_data.right[i] = rightmotor_steps
        i += 1
        leftmotor_steps -= 1
        rightmotor_steps -= 1
    # go left
    leftmotor_steps += 1
    rightmotor_steps += 1
    while (rightmotor_steps > rmotor_smallest_step) and (
        leftmotor_steps < lmotor_largest_step
    ):
        stp_data.left[i] = leftmotor_steps
        stp_data.right[i] = rightmotor_steps
        i += 1
        leftmotor_steps += 1
        rightmotor_steps += 1
    # go right
    leftmotor_steps -= 1
    rightmotor_steps -= 1
    while (rightmotor_steps > rmotor_smallest_step) and (
        leftmotor_steps < lmotor_largest_step
    ):
        stp_data.left[i] = leftmotor_steps
        stp_data.right[i] = rightmotor_steps
        i += 1
        leftmotor_steps -= 1
        rightmotor_steps -= 1

    stp_data.list_length = ctypes.c_int(i)

    while True:
        input("press enter to continue")
        serialcomms.move_to(serial_interface=ser, mstep_holder=stp_data)
        print("sent steps to motorcontroller...", end="")
        print("checking response from motorcontroller...", end="")
        while True:
            try:
                leftsteps, rightsteps = serialcomms.receive_position(sercom=ser)
                print(f"response received: L{leftsteps} R{rightsteps}")
                print("[ctrl c] to advance")
            except KeyboardInterrupt:
                break

def mode_takevideo(
    *,
    camerathread: FrameCapture,
) -> None:
    """Captures video to a file via openCV."""
    cv.namedWindow("Camera")
    camerathread.start_write()
    while True:
        frame: OpenCVframe = camerathread.current_frame
        cv.imshow("Camera", frame)
        if cv.waitKey(1) == ord("q"):
            break
    camerathread.stop_write()
    while not camerathread.recording_stopped:
        print("waiting for video file to write")
        sleep(10)  # wait for recording to stop and file to be written
    cv.destroyAllWindows()
    return


def mode_fulldebug(
    *,
    camerathread: FrameCapture,
    detector: cv.aruco.ArucoDetector,
    kin_lib: ctypes.CDLL,
    puck: LocationData,
    mm_per_pixel_arm: float,
    stp_data: MotorSteps,
    ser: serial.Serial,
) -> None:
    """A bit like normal mode, but with GUI and extra debug info displayed."""
    DEBUG_PAUSE: Final[int] = 2
    SERIAL_TIMEOUT: Final[int] = 20
    ser.timeout = SERIAL_TIMEOUT  # set serial timeout
    leftmotor_pos: int | None = None
    rightmotor_pos: int | None = None

    cv.namedWindow("Camera")

    print("getting initial motor positions from motorcontroller")
    while not (leftmotor_pos and rightmotor_pos):
        ser.write(
            f"{serialencoding.MSG.start}{serialencoding.MSG.type.enq}{serialencoding.MSG.end}".encode(
                encoding="ascii"
            )
        )
        leftmotor_pos, rightmotor_pos = serialcomms.receive_position(sercom=ser)
    print("motor controller positions received, launching manual2coordds main loop")

    while True:
        success_steps: bool = False
        print(f"{DEBUG_PAUSE} second debugging pause...")
        sleep(DEBUG_PAUSE)
        # get freshest frame
        print("attempting to update frame... ", end="")
        frame: OpenCVframe = camerathread.current_frame
        # update GUI
        cv.imshow("Camera", frame)
        cv.waitKey(1)  # required to update GUI properly (wastes 1ms)
        print("frame updated. ", end="")
        puck_coords: FrameCoords | None = aruco.detect_aruco(
            aframe=frame,
            adetector=detector,
            arcode=puck.code,
        )
        if not puck_coords:
            print("puck not detected")
            continue
        # draw puck detection
        annotateframe.circle(img=frame, coords=puck_coords)
        effector_x_next_mm = int(puck_coords.x_px * mm_per_pixel_arm)
        effector_y_next_mm = int(puck_coords.y_px * mm_per_pixel_arm)
        # update GUI
        cv.imshow("Camera", frame)
        cv.waitKey(1)  # required to update GUI properly (wastes 1ms)
        # get motor steps from kinematics lib
        print(
            f"asking for steps to: ({effector_x_next_mm}, {effector_y_next_mm})",
            end=None,
        )
        print("Calling kinematics library... ", end="")
        sleep(0.1)  # kinematics sometimes runs faster than python prints
        success_steps: bool = kin_lib.get_steplists(
            ctypes.byref(stp_data.left),
            ctypes.byref(stp_data.right),
            stp_data.max_list_length,
            ctypes.byref(stp_data.list_length),
            ctypes.c_int(leftmotor_pos),  # current lmotor steps
            ctypes.c_int(rightmotor_pos),  # current rmotor steps
            ctypes.c_int(effector_x_next_mm),  # desired x
            ctypes.c_int(effector_y_next_mm),  # desired y
        )
        print("returned from kinematics library. ")
        if not success_steps:
            print("No steps received from kinematics library")
            continue
        # send coords to the uno
        print("Received steps:")
        print("Left")
        for i in range(stp_data.list_length.value):
            pp(stp_data.left[i])
        print("Right")
        for i in range(stp_data.list_length.value):
            pp(stp_data.right[i])
        print("Sending steps to motorcontroller... ", end="")
        serialcomms.move_to(serial_interface=ser, mstep_holder=stp_data)
        print("Waiting to receive motor controller message... ", end="")
        # get response
        leftmotor_pos, rightmotor_pos = serialcomms.receive_position(sercom=ser)
        while not (leftmotor_pos and rightmotor_pos):
            ser.write(
                f"{serialencoding.MSG.start}{serialencoding.MSG.type.enq}{serialencoding.MSG.end}".encode(
                    encoding="ascii"
                )
            )
            leftmotor_pos, rightmotor_pos = serialcomms.receive_position(sercom=ser)


def _step_input(*, stp_data: MotorSteps, max_steps: int) -> None:
    """Updates stp_data from user keyboard input."""
    print("enter the first pair of motor steps")
    stp_data.left[0] = userinput.get_int(
        prompt="Left steps (absolute)",
        num_max=max_steps,
        num_min=0,
    )
    stp_data.right[0] = userinput.get_int(
        prompt="Right steps (absolute)",
        num_max=max_steps,
        num_min=0,
    )
    i: int = 1
    while i <= stp_data.max_list_length.value:
        print("enter another pair of angles, or ctrl + c to send all angles to MCU")
        try:
            stp_data.left[i] = userinput.get_int(
                prompt="Left steps (absolute)",
                num_max=max_steps,
                num_min=0,
            )
            stp_data.right[i] = userinput.get_int(
                prompt="Right steps (absolute)",
                num_max=max_steps,
                num_min=0,
            )
        except KeyboardInterrupt:
            break
        i += 1
    stp_data.list_length.value = i
    stp_data.valid = True
    print("\nStep data to be sent: ")
    print("Left: ", end="")
    for i in range(stp_data.list_length.value):
        print(f"{stp_data.left[i]} ", end="")
    print("\nRight: ", end="")
    for i in range(stp_data.list_length.value):
        print(f"{stp_data.right[i]} ", end="")
    print()
    return


def _coords_input() -> tuple[int, int]:
    """Get user input for coordinates (in mm)."""
    print("Enter coordinates to go to")
    x_mm: int = userinput.get_int(prompt="x coordinate in mm")
    y_mm: int = userinput.get_int(prompt="y coordinate in mm")
    return x_mm, y_mm


def _onany(cvevent: int, x: int, y: int, flags: int, parameters: Any) -> None:
    """OpenCV callback function to get the coordinates (in pixels) of cursor."""
    cursor_coords.x_px = x
    cursor_coords.y_px = y
