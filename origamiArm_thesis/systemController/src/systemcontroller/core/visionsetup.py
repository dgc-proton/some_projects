import threading
import time
from importlib import resources
from typing import Any

import cv2 as cv
import numpy as np

from systemcontroller.utils.tphints import OpenCVframe


class FrameCapture(threading.Thread):
    """A thread that constantly contains the most recent camera frame."""

    video_capture: cv.VideoCapture
    video_writer: cv.VideoWriter | None
    current_frame: OpenCVframe
    tmp_frame: OpenCVframe
    width: int
    height: int
    undistort_frames: bool = True
    undistort_mapx: OpenCVframe
    undistort_mapy: OpenCVframe
    recording_stopped: bool = True

    def __init__(self, cvcapture: cv.VideoCapture) -> None:
        """Initialise the thread and start it."""
        success: bool = False
        self.video_capture: cv.VideoCapture = cvcapture
        self.video_writer = None
        assert self.video_capture.isOpened()  # noqa: S101
        while not success:
            # capture the first frame
            success, tmpframe = self.video_capture.read()
        self.height, self.width = tmpframe.shape[:2]
        self._write_log()
        self._load_cal()
        super().__init__(daemon=True)  # initialise parent class
        self.start()  # start thread

    def run(self) -> None:
        """Infinite task loop updating current_frame."""
        success: bool = False
        temp_frame: OpenCVframe
        while True:
            success, temp_frame = self.video_capture.read()
            if not success:
                continue
            if self.undistort_frames:
                self.current_frame = cv.remap(
                    temp_frame,
                    self.undistort_mapx,
                    self.undistort_mapy,
                    cv.INTER_LINEAR,
                )
            else:
                self.current_frame = temp_frame
            if self.video_writer:
                self.video_writer.write(self.current_frame)

    def start_write(self) -> None:
        """Setup video writing, then immediately starts recording."""
        print("setting up video recording")
        self.video_writer = cv.VideoWriter(
            "output.avi",
            fourcc=cv.VideoWriter.fourcc(*"MJPG"),
            fps=self.video_capture.get(cv.CAP_PROP_FPS),
            frameSize=(
                int(self.video_capture.get(cv.CAP_PROP_FRAME_WIDTH)),
                int(self.video_capture.get(cv.CAP_PROP_FRAME_HEIGHT)),
            ),
        )
        self.recording_stopped = False
        print("recording started")


    def stop_write(self) -> None:
        """Finish writing the file."""
        if self.video_writer:
            self.video_writer.release()
        self.recording_stopped = True

    def _load_cal(self) -> None:
        """Load camera calibration (intrinsic camera matrix and undistortion coefficients) data."""
        with resources.open_binary("systemcontroller", "cam_cal.npz") as f:
            cal_data = np.load(f)
            camera_matrix = cal_data["camera_matrix"]
            distortion_coeffs = cal_data["distortion_coeff"]
            rvecs = cal_data["rvecs"]
            tvecs = cal_data["tvecs"]

        ncam_matrix, roi = cv.getOptimalNewCameraMatrix(
            cameraMatrix=camera_matrix,
            distCoeffs=distortion_coeffs,
            imageSize=(self.width, self.height),
            alpha=1,
            newImgSize=(self.width, self.height),
        )
        self.undistort_mapx, self.undistort_mapy = cv.initUndistortRectifyMap(
            cameraMatrix=camera_matrix,
            distCoeffs=distortion_coeffs,
            R=None,
            newCameraMatrix=ncam_matrix,
            size=(self.width, self.height),
            m1type=5,
        )

    def _write_log(self) -> None:
        """Write the camera log file."""
        with open("cam_details.log", mode="w") as f:
            # write camera details out to log file
            print("openCV VideoCapture details:", file=f)
            print(f"FPS: {(self.video_capture.get(cv.CAP_PROP_FPS),)}", file=f)
            print(f"width: {self.video_capture.get(cv.CAP_PROP_FRAME_WIDTH)}", file=f)
            print(f"height: {self.video_capture.get(cv.CAP_PROP_FRAME_HEIGHT)}", file=f)
            print(
                f"frame type: {self.video_capture.get(cv.CAP_PROP_FRAME_TYPE)}", file=f
            )
            print(f"fourcc: {self.video_capture.get(cv.CAP_PROP_FOURCC)}", file=f)
            print(f"backend: {self.video_capture.get(cv.CAP_PROP_BACKEND)}", file=f)
            print(f"bitrate: {self.video_capture.get(cv.CAP_PROP_BITRATE)}", file=f)
            print(
                f"brightness: {self.video_capture.get(cv.CAP_PROP_BRIGHTNESS)}", file=f
            )
            print(f"channel: {self.video_capture.get(cv.CAP_PROP_CHANNEL)}", file=f)
            print(
                f"codec extradata index: {self.video_capture.get(cv.CAP_PROP_CODEC_EXTRADATA_INDEX)}",
                file=f,
            )
            print(
                f"codec pixel format: {self.video_capture.get(cv.CAP_PROP_CODEC_PIXEL_FORMAT)}",
                file=f,
            )
            print(f"contrast: {self.video_capture.get(cv.CAP_PROP_CONTRAST)}", file=f)
            print(f"exposure: {self.video_capture.get(cv.CAP_PROP_EXPOSURE)}", file=f)
            print(f"focus: {self.video_capture.get(cv.CAP_PROP_FOCUS)}", file=f)
            print(f"format: {self.video_capture.get(cv.CAP_PROP_FORMAT)}", file=f)
            print(f"gain: {self.video_capture.get(cv.CAP_PROP_GAIN)}", file=f)
            print(f"gamma: {self.video_capture.get(cv.CAP_PROP_GAMMA)}", file=f)
            print(f"GUID: {self.video_capture.get(cv.CAP_PROP_GUID)}", file=f)
            print(f"hue: {self.video_capture.get(cv.CAP_PROP_HUE)}", file=f)
            print(f"HW device: {self.video_capture.get(cv.CAP_PROP_HW_DEVICE)}", file=f)
            print(f"iso speed: {self.video_capture.get(cv.CAP_PROP_ISO_SPEED)}", file=f)
            print(f"mode: {self.video_capture.get(cv.CAP_PROP_MODE)}", file=f)
            print(f"sharpness: {self.video_capture.get(cv.CAP_PROP_SHARPNESS)}", file=f)


def setup_camera_opencv(*, configuration: dict[str, Any]) -> FrameCapture:
    """Setup the camera, returning the openCV capture object, or None if unsuccessful."""
    cap: cv.VideoCapture | None = cv.VideoCapture(
        configuration["camera"]["device"],
        cv.CAP_V4L2,
        params=[
            cv.CAP_PROP_FRAME_WIDTH,
            configuration["camera"]["width_px"],
            cv.CAP_PROP_FRAME_HEIGHT,
            configuration["camera"]["height_px"],
            cv.CAP_PROP_FOURCC,
            cv.VideoWriter.fourcc(*configuration["camera"]["opencv_fourcc"]),
            cv.CAP_PROP_FPS,
            configuration["camera"]["fps"],
        ],
    )
    camthread = FrameCapture(cvcapture=cap)
    time.sleep(0.1)
    return camthread
