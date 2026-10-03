from pprint import pp
from time import sleep
from typing import Any, Final

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

from systemcontroller.core.visionsetup import FrameCapture


def px_per_mm(*, camerathread: FrameCapture, detector: cv.aruco.ArucoDetector) -> None:
    """Prints calibration value to screen then exits."""
    frame: np.ndarray[Any, np.dtype[np.integer[Any] | np.floating[Any]]]
    preprocessed_frame: np.ndarray[Any, np.dtype[np.integer[Any] | np.floating[Any]]]
    dist_between_mm = float(
        input("Enter the white-space distance between markers in mm: "),
    )
    print(
        "\nInstructions:\n- Place the calibration sheet at the height to be calibrated. "
        "The line drawn by the camera should be exactly between the codes. Press a when correct.\n",
    )
    while True:
        left_detected: bool = False
        right_detected: bool = False
        px_per_mm = None
        frame = camerathread.current_frame
        preprocessed_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners_list, id_list, _ = detector.detectMarkers(preprocessed_frame)
        if id_list is None:
            cv.imshow("Camera", frame)
            cv.waitKey(1)
            continue
        for the_id, corners in zip(
            id_list, corners_list, strict=True
        ):
            corners_x_sorted = np.sort(corners[0].squeeze()[:, 0])
            corners_y_mean = np.mean(corners[0].squeeze()[:, 1])
            match the_id:
                case 48:
                    # left hand marker detected
                    left_x = int(np.mean(corners_x_sorted[-1:]))
                    left_y = int(corners_y_mean)
                    left_detected = True
                case 49:
                    # right hand marker detected
                    right_x = int(np.mean(corners_x_sorted[:1]))
                    right_y = int(corners_y_mean)
                    right_detected = True
            if left_detected and right_detected:
                cv.line(
                    frame,
                    (left_x, left_y),
                    (right_x, right_y),
                    (0, 255, 0),
                    thickness=2,
                )
                cv.line(frame, (1, 1), (1, 1), (0, 0, 0), thickness=2)
                px = np.sqrt((right_x - left_x) ** 2 + (right_y - left_y) ** 2)
                px_per_mm = px / dist_between_mm
        cv.imshow("Camera", frame)
        key: int = cv.waitKey(50)
        if key == ord("a"):
            print(f"Calibration measured {px_per_mm} px per mm")
            sleep(1)
        if key == ord("q"):
            break
    cv.destroyAllWindows()
    return


def intrinsics_cal(*, camerathread: FrameCapture) -> None:
    """Solve for intrinsic camera properties to use in undistorting images.

    Adapted from an example in the openCV documentation.
    """
    num_imgs: Final[int] = 100
    board_rows: Final[int] = 9
    board_cols: Final[int] = 6
    board_layout: Final[tuple[int, int]] = (board_rows, board_cols)
    frame: np.ndarray[Any, np.dtype[np.integer[Any] | np.floating[Any]]]

    camerathread.undistort_frames = False  # disable undistort for calibrating

    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    world_objp = np.zeros((board_rows * board_cols, 3), np.float32)
    world_objp[:, :2] = np.mgrid[0:board_rows, 0:board_cols].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    world_objpoints = []  # 3d point in real world space
    img_points = []  # 2d points in image plane.

    print("Will now take images of the chessboard, important points:")
    print("Camera should be setups in the resolution it will be used at")
    print("Chessboard should always be perfectly flat")
    print("Must get the chessboard in a variety of rotations, and roll angles etc")
    print("Must get many images covering different areas of the frame")
    print(
        "press space to take an image, then a to accept, anything else to retake, q to quit"
    )

    images_taken: int = 0
    finished: bool = False

    # take images and find corners
    while True:
        print(
            f"{images_taken} images taken of target {num_imgs} images, space to capture next"
        )
        while True:
            frame = camerathread.current_frame
            cv.imshow("Camera", frame)
            key: int = cv.waitKey(1)
            if key == ord(" "):
                break
            if key == ord("q"):
                finished = True
                break

        if finished:
            break

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, board_layout, None)

        if not ret:
            print("detection failed")
            cv.imshow("Camera", frame)
            cv.waitKey(500)
            continue

        # If found, add object points, image points (after refining them)
        world_objpoints.append(world_objp)

        corners_refined = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        img_points.append(corners_refined)

        # Draw and display the corners
        cv.drawChessboardCorners(frame, board_layout, corners_refined, ret)
        cv.imshow("Camera", frame)
        cv.waitKey(500)
        images_taken += 1

    cv.destroyAllWindows()

    print("calculating calibration parameters...")
    # carry out calibration with the stored values
    rep_error, camera_matrix, distortion_coeffs, rvecs, tvecs = cv.calibrateCamera(
        world_objpoints, img_points, gray.shape[::-1], None, None
    )

    print("Camera matrix: ")
    pp(camera_matrix)
    print("Distortion coefficients:")
    pp(distortion_coeffs)
    print("rvecs:")
    pp(rvecs)
    print("tvecs:")
    pp(tvecs)
    print(f"Reprojection error (pixels, <0.18 is good): {rep_error:.4f}")
    print("Saving values")

    np.savez(
        "cam_cal.npz",
        rep_error=rep_error,
        camera_matrix=camera_matrix,
        distortion_coeff=distortion_coeffs,
        rvecs=rvecs,
        tvecs=tvecs,
    )

    print("doing a test image")

    # draw lines to show distortion
    height, width = frame.shape[:2]
    cv.line(
        frame,
        pt1=(0, int(height / 4)),
        pt2=(width, int(height / 4)),
        color=(0, 0, 255),
        thickness=10,
    )
    cv.line(
        frame,
        pt1=(0, int(3 * height / 4)),
        pt2=(width, int(3 * height / 4)),
        color=(0, 0, 255),
        thickness=10,
    )
    cv.line(
        frame,
        pt1=(int(width / 4), 0),
        pt2=(int(width / 4), height),
        color=(0, 0, 255),
        thickness=10,
    )
    cv.line(
        frame,
        pt1=(int(3 * width / 4), 0),
        pt2=(int(3 * width / 4), height),
        color=(0, 0, 255),
        thickness=10,
    )

    ncam_matrix, roi = cv.getOptimalNewCameraMatrix(
        cameraMatrix=camera_matrix,
        distCoeffs=distortion_coeffs,
        imageSize=(width, height),
        alpha=0,
        newImgSize=(width, height),
    )
    fixed_alp0 = cv.undistort(
        frame,
        cameraMatrix=camera_matrix,
        distCoeffs=distortion_coeffs,
        dst=None,
        newCameraMatrix=ncam_matrix,
    )

    ncam_matrix, roi = cv.getOptimalNewCameraMatrix(
        cameraMatrix=camera_matrix,
        distCoeffs=distortion_coeffs,
        imageSize=(width, height),
        alpha=1,
        newImgSize=(width, height),
    )
    fixed_alp1 = cv.undistort(
        frame,
        cameraMatrix=camera_matrix,
        distCoeffs=distortion_coeffs,
        dst=None,
        newCameraMatrix=ncam_matrix,
    )

    mapx, mapy = cv.initUndistortRectifyMap(
        cameraMatrix=camera_matrix,
        distCoeffs=distortion_coeffs,
        R=None,
        newCameraMatrix=ncam_matrix,
        size=(width, height),
        m1type=5,
    )
    fixed_map = cv.remap(frame, mapx, mapy, cv.INTER_LINEAR)
    trim_to_roi = False
    if trim_to_roi:
        x, y, w, h = roi
        fixed_map = fixed_map[y : y + h, x : x + w]

    fig1 = plt.figure()
    axs = fig1.subplots(nrows=2, ncols=2)
    axs[0][0].imshow(cv.cvtColor(frame, cv.COLOR_BGR2RGB))
    axs[0][0].set_title("original")
    axs[0][1].imshow(cv.cvtColor(fixed_alp0, cv.COLOR_BGR2RGB))
    axs[0][1].set_title("fixed alpha=0 (only valid pixels retained, slight distortion)")
    axs[1][0].imshow(cv.cvtColor(fixed_alp1, cv.COLOR_BGR2RGB))
    axs[1][0].set_title("fixed alpha=1 (all pixels retained)")
    axs[1][1].imshow(cv.cvtColor(fixed_map, cv.COLOR_BGR2RGB))
    axs[1][1].set_title("fixed using map (optimal for video?)")
    plt.show()
