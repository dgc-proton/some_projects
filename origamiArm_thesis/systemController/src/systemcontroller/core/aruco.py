from typing import Any

import cv2 as cv
import numpy as np

from systemcontroller.utils.tphints import OpenCVframe


def detect_aruco(
    *,
    aframe: OpenCVframe,
    adetector: cv.aruco.ArucoDetector,
    arcode: int|None,
) -> tuple[int, int] | None:
    """Detects specified aruco code in a frame."""
    if arcode is None:
        return None
    preproc_frame: OpenCVframe = cv.cvtColor(aframe, cv.COLOR_BGR2GRAY)
    corners_list, id_list, _ = adetector.detectMarkers(preproc_frame)
    if id_list is None:
        return None  # need to guard below against the case of none found
    for the_id, corners in zip(id_list, corners_list, strict=True):
        corners_unpacked = corners[0].squeeze()
        if the_id == arcode:
            return int(np.mean(corners_unpacked[:, 0])), int(np.mean(corners_unpacked[:, 1]))
    return None  # return none if code does not match specified
