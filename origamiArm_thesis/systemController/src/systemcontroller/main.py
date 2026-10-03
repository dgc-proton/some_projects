import os
import sys

if not __package__:
    # workaround to avoid having to install to jetson to run the code
    package_source_path = os.path.dirname(os.path.dirname(__file__))  # noqa: PTH120
    sys.path.insert(0, package_source_path)

import cProfile
import ctypes
import io
import pstats
from time import sleep
from typing import Any, Final

import cv2 as cv
import numpy as np
import numpy.typing as npt
import serial

import systemcontroller.core.operatingmodes as opmodes
import systemcontroller.core.visionsetup as vsetup
from systemcontroller.api import serialcomms
from systemcontroller.api.motorstepclass import MotorSteps
from systemcontroller.core import annotateframe, aruco, kinematicssetup, yolo
from systemcontroller.core.coreclasses import FrameCoords, LocationData
from systemcontroller.core.visionsetup import FrameCapture
from systemcontroller.utils import calibratecamera
from systemcontroller.utils.config_tools import load_config
from systemcontroller.utils.tphints import OpenCVframe

# work around to utilise opencv mouse click coordinates
cursor_coords2 = FrameCoords(x_px=0, y_px=0)


def main() -> None:
    """The main entry point."""
    success: bool = False
    config: dict[str, Any] = load_config()
    max_moves: Final[int] = config["max_motor_moves"]
    operating_mode: str = config["debug"]["operating_mode"]

    mm_per_pixel_arm: Final[float] = 1 / config["camera"]["pixels_per_mm_arm"]
    mmper_pixel_table: Final[float] = 1 / config["camera"]["pixels_per_mm_table"]

    # Location data
    puck: LocationData = LocationData(
        code=0,
        description="puck",
        mm_per_px=mm_per_pixel_arm,
        coords_vis_px_x=0,
        coords_vis_px_y=0,
    )
    effector: LocationData = LocationData(
        code=1,
        description="effector",
        mm_per_px=mm_per_pixel_arm,
        coords_vis_px_x=0,
        coords_vis_px_y=0,
    )
    motor1: LocationData = LocationData(
        code=7,
        description="motor1",
        mm_per_px=mm_per_pixel_arm,
        coords_vis_px_x=0,
        coords_vis_px_y=0,
    )
    motor2: LocationData = LocationData(
        code=6,
        description="motor2",
        mm_per_px=mm_per_pixel_arm,
        coords_vis_px_x=0,
        coords_vis_px_y=0,
    )
    desired: LocationData = LocationData(
        description="desired location",
        mm_per_px=mm_per_pixel_arm,
        coords_vis_px_x=0,
        coords_vis_px_y=0,
    )

    step_data: MotorSteps = MotorSteps(
        left=(ctypes.c_int * max_moves)(),
        right=(ctypes.c_int * max_moves)(),
        max_list_length=ctypes.c_int(max_moves),
        list_length=ctypes.c_int(0),
    )

    if operating_mode in [
        "normal",
        "manual_step",
        "full_debug",
        "manual2coords",
        "side2side_manual",
        "side2side_auto",
    ]:
        # setup serial comms
        sercomms: serial.Serial = serialcomms.connect(config=config)

    if operating_mode in [
        "normal",
        "vision_only",
        "full_debug",
        "calibrate",
        "manual2coords",
        "takevideo",
        "intrinsics_cal",
    ]:
        # setup camera and vision algorithms
        aruco_detect: cv.aruco.ArucoDetector | None = cv.aruco.ArucoDetector(
            cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),
            cv.aruco.DetectorParameters(),
        )
        if not aruco_detect:
            print("Failed to create aruco detector.")
            sys.exit()
        camthread: FrameCapture = vsetup.setup_camera_opencv(
            configuration=config,
        )

    if operating_mode in ["normal", "vision_only", "full_debug", "manual2coords"]:
        # detect motor positions
        result: tuple[int, int] | None = None
        print("detecting motors...")
        while True:
            frame: OpenCVframe = camthread.current_frame

            result = aruco.detect_aruco(
                aframe=frame,
                adetector=aruco_detect,
                arcode=motor1.code,
            )
            if result:
                motor1.coords_vis_px_x, motor1.coords_vis_px_y = result[0], result[1]
                motor1.been_detected = True

            result = aruco.detect_aruco(
                aframe=frame,
                adetector=aruco_detect,
                arcode=motor2.code,
            )
            if result:
                motor2.coords_vis_px_x, motor2.coords_vis_px_y = result[0], result[1]
                motor2.been_detected = True

            for loc in [motor2, motor1]:
                if loc.been_detected:
                    annotateframe.circle(
                        img=frame,
                        coords_xy=(loc.coords_vis_px_x, loc.coords_vis_px_y),
                    )
                    annotateframe.text(
                        img=frame,
                        coords_xy=(loc.coords_vis_px_x, loc.coords_vis_px_y),
                        txt=loc.description,
                    )
            if config["debug"]["showscreen"]:
                cv.imshow("Detecting motors and end effector...", frame)
                cv.waitKey(1)
            if motor1.been_detected and motor2.been_detected:
                print(
                    "successfully detected motor and effector positions, positions in mm:",
                )
                print(
                    f"motor1({motor1.coords_phys_mm_x:.1f}, {motor1.coords_phys_mm_y:.1f}), "
                    f"motor2({motor2.coords_phys_mm_x:.1f}, {motor2.coords_phys_mm_y:.1f}), "
                    f"effector({effector.coords_phys_mm_x:.1f}, {effector.coords_phys_mm_y:.1f})",
                )
                sleep(1)
                cv.destroyAllWindows()
                break

    if operating_mode in ["normal", "calc_coords_only", "full_debug", "manual2coords"]:
        # setup kinematics calculation shared library, and optionally check workspace with user
        if config["check_workspace_lib"]:
            workspace_lib(camerathread=camthread, mm_p_pixel_arm=mm_per_pixel_arm)
        if motor1.coords_phys_mm_x < motor2.coords_phys_mm_x:
            mleft: LocationData = motor1
            mright: LocationData = motor2
        else:
            mleft: LocationData = motor2
            mright: LocationData = motor1
        kinlib: ctypes.CDLL = kinematicssetup.setup(
            buffer_size=config["max_motor_moves"],
            motorl_xy_mm=(mleft.coords_phys_mm_x, mleft.coords_phys_mm_y),
            motorr_xy_mm=(mright.coords_phys_mm_x, mright.coords_phys_mm_y),
            link_lengths_mm=(
                config["arm"]["linklengths"]["left_mm"],
                config["arm"]["linklengths"]["left_center_mm"],
                config["arm"]["linklengths"]["right_center_mm"],
                config["arm"]["linklengths"]["right_mm"],
            ),
        )

    if config["debug"]["profile_code"]:
        # start code profiling
        prof = cProfile.Profile()
        prof.enable()

    if operating_mode in [
        "normal",
        "manual_step",
        "full_debug",
        "manual2coords",
        "side2side_manual",
        "side2side_auto",
    ]:
        # handshake confirming both this script and the MCU are ready for main routine
        success: bool = serialcomms.handshake(serial_interface=sercomms, num_tries=8)
        if not success:
            print("Handshake with MCU failed")
            sys.exit()

    # launch main operating mode routine
    match operating_mode:
        case "normal":
            try:  # noqa: SIM105
                opmodes.mode_normal(
                    camerathread=camthread,
                    detector=aruco_detect,
                    kin_lib=kinlib,
                    puck=puck,
                    desired_loc=desired,
                    stp_data=step_data,
                    ser=sercomms,
                    showscreen=config["debug"]["showscreen"]
                )
            except KeyboardInterrupt:
                pass  # used as part of profiling execution time, introduces minimal overhead
        case "vision_only":
            opmodes.mode_visiononly(camerathread=camthread, detector=aruco_detect)
        case "calc_coords_only":
            opmodes.mode_coordsonly(
                kin_lib=kinlib,
                buffer_size=config["max_motor_moves"],
            )
        case "manual_step":
            opmodes.mode_manualstep(
                step_dat=step_data,
                step_max=200,
                ser=sercomms,
            )
        case "full_debug":
            opmodes.mode_fulldebug(
                camerathread=camthread,
                detector=aruco_detect,
                kin_lib=kinlib,
                puck=puck,
                mm_per_pixel_arm=mm_per_pixel_arm,
                stp_data=step_data,
                ser=sercomms,
            )
        case "calibrate":
            calibratecamera.px_per_mm(camerathread=camthread, detector=aruco_detect)
        case "manual2coords":
            opmodes.manual2coords(
                camerathread=camthread,
                kin_lib=kinlib,
                desired_pos=desired,
                stp_data=step_data,
                ser=sercomms,
            )
        case "side2side_manual":
            opmodes.side2side_manual(
                stp_data=step_data,
                ser=sercomms,
                lmotor_largest_step=105,
                rmotor_smallest_step=88,
            )
        case "side2side_auto":
            opmodes.side2side_auto(
                stp_data=step_data,
                ser=sercomms,
                lmotor_largest_step=105,
                rmotor_smallest_step=88,
            )
        case "takevideo":
            opmodes.mode_takevideo(camerathread=camthread)
        case "intrinsics_cal":
            calibratecamera.intrinsics_cal(camerathread=camthread)
        case _:
            msg = "operating mode name not matched"
            raise ValueError(msg)

    # process and print code profiling data
    if config["debug"]["profile_code"]:
        prof.disable()
        s = io.StringIO()
        results = pstats.Stats(prof, stream=s)
        results.strip_dirs().sort_stats("cumulative").print_stats()
        print(s.getvalue())


def _onany2(cvevent: int, x: int, y: int, flags: int, parameters: Any) -> None:
    """OpenCV callback function to get the coordinates (in pixels) of cursor."""
    cursor_coords2.x_px = x
    cursor_coords2.y_px = y


def workspace_lib(*, camerathread: FrameCapture, mm_p_pixel_arm: float):
    """Interface to the workspace selector and calculator that Zayne is planning to write."""
    box_corners: list[LocationData] = []
    cv.namedWindow("Camera")
    cv.setMouseCallback("Camera", _onany2)
    print("press a to set the 4 workspace corners")
    while len(box_corners) < 4:
        # get user input
        frame: OpenCVframe = camerathread.current_frame
        for existing_corner in box_corners:
            annotateframe.circle(
                img=frame,
                coords_xy=(
                    existing_corner.coords_vis_px_x,
                    existing_corner.coords_vis_px_y,
                ),
                size_radius=2,
            )
        cv.imshow("Camera", frame)
        key: int = cv.waitKey(10)  # required to update GUI properly
        if key == ord("a"):
            box_corners.append(
                LocationData(
                    description="box corner",
                    mm_per_px=mm_p_pixel_arm,
                    coords_vis_px_x=cursor_coords2.x_px,
                    coords_vis_px_y=cursor_coords2.y_px,
                )
            )
    workspace: npt.ArrayLike = np.zeros(shape=(camerathread.height, camerathread.width))
    raise NotImplementedError  # Zayne having issues with the lib, leaving this code incase needed


if __name__ == "__main__":
    main()
