from typing import Any

import cv2 as cv
import numpy as np

from systemcontroller.utils.tphints import OpenCVframe


def circle(
    *,
    img: OpenCVframe,
    coords_xy: tuple[int, int],
    size_radius: int = 15,
) -> None:
    """Adds a circle to the opencv frame / image."""
    cv.circle(
        img,
        coords_xy,
        radius=size_radius,
        color=(0, 255, 0),
        thickness=3,
    )
    return


def text(
    *,
    img: OpenCVframe,
    coords_xy: tuple[int, int],
    txt: str,
) -> None:
    cv.putText(
        img,
        text=txt,
        org=coords_xy,
        fontFace=cv.FONT_HERSHEY_SIMPLEX,
        fontScale=1,
        color=(0, 255, 0),
        thickness=2,
    )
    return
