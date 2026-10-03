import random
import sys

import cv2 as cv
import darknet  # can optionally use the darkhelp library

from systemcontroller.utils.tphints import OpenCVframe


class YOLOdetector:
    """Interface to the darknet C++ framework for running YOLO detection and classification CNNs."""

    def __init__(
        self,
        *,
        cfg_file: str,
        names_file: str,
        weights_file: str,
        datafile: str,
        threshold: float = 0.5,
    ) -> None:
        """Constructor which sets up the CNN."""
        if threshold <= 0 or threshold > 1:
            msg: str = "threshold must be greater than 0, and less than or equal to 1"
            raise ValueError(msg)

        self.threshold = threshold
        darknet.set_verbose(True)
        darknet.show_version_info()

        # load the neural network
        self.network = darknet.load_net_custom(
            cfg_file.encode("ascii"), weights_file.encode("ascii"), 0, 1
        )
        self.class_names = open(names_file).read().splitlines()

        # set colors to use for display boxes
        random.seed(3)  # fixed seed keeps colors the same session to session
        self.colours = darknet.class_colors(self.class_names)

        # there is a bug in darknet library where a function definition conflicts with a
        # darknet argument type; in the library, change
        # "def network_dimensions(net):" to "def get_network_dimensions(net):" as temp fix
        self.network_width, self.network_height, self.network_channels = (
            darknet.get_network_dimensions(self.network)
        )

    def __del__(self) -> None:
        """Destructor to free memory when the object goes out of scope."""
        darknet.free_network_ptr(self.network)

    def frame_inference(self, frame_bgr: OpenCVframe):
        """Detect and classify objects in an openCV frame."""
        # convert from openCV BGR format to RGB for Darknet
        frame_rgb = cv.cvtColor(frame_bgr, cv.COLOR_BGR2RGB)

        # resize the frame to fit the CNN
        frame_resized = cv.resize(
            frame_rgb,
            (self.network_width, self.network_height),
            interpolation=cv.INTER_LINEAR,
        )

        # translate to darknet image structure
        darknet_frame = darknet.make_image(
            self.network_width, self.network_height, self.network_channels
        )
        # darknet.copy_image_from_bytes(darknet_frame, frame_resized.tobytes())

        # carry out detections
        detections = darknet.detect_image(
            self.network, self.class_names, darknet_frame, thresh=self.threshold
        )

        # free memory
        darknet.free_image(darknet_frame)

        # output details to the console
        darknet.print_detections(detections, True)

        # draw detections then display image
        frame_with_detections = darknet.draw_boxes(
            detections, frame_resized, self.colours
        )
        # cv.imshow("Detections", cv.cvtColor(frame_with_detections, cv.COLOR_RGB2BGR))
        # if cv.waitKey() == ord("q"):
        #     pass
        return detections


def test_yolo():
    """Test function for YOLO."""
    prefix = (
        "/home/dv1/Documents/courses/degree_eng_aberdeen/Y4/EG45PE_MEng_Individual_Project"
        "/code/systemController/src/systemcontroller/"
    )
    cfg = prefix + "cnn_files/block_plate_cake_kiwi_1920x1080.cfg"
    names = prefix + "cnn_files/block_plate_cake_kiwi_1920x1080.names"
    weights = prefix + "cnn_files/block_plate_cake_kiwi_1920x1080.weights"
    data = prefix + "cnn_files/block_plate_cake_kiwi_1920x1080.data"

    detector: YOLOdetector = YOLOdetector(
        cfg_file=cfg, names_file=names, weights_file=weights, datafile=data
    )

    prefix = (
        "/home/dv1/Documents/degree_Y4/EG45PE_MEng_Individual_Project/code/darknet/"
        "block_plate_cake_kiwi_1920x1080/video_import_2026-03-18_00-08-51_output_avi/"
    )
    for filename in [
        prefix + "output_frame_002387.jpg",
        prefix + "output_frame_001994.jpg",
        prefix + "output_frame_003184.jpg",
        prefix + "output_frame_004373.jpg",
        prefix + "output_frame_002221.jpg",
        prefix + "output_frame_003380.jpg",
        prefix + "output_frame_002613.jpg",
        prefix + "output_frame_002421.jpg",
        prefix + "output_frame_002847.jpg",
        prefix + "output_frame_003508.jpg",
        prefix + "output_frame_004128.jpg",
        prefix + "output_frame_004458.jpg",
        prefix + "output_frame_002528.jpg",
        prefix + "output_frame_003675.jpg",
        prefix + "output_frame_004264.jpg",
        prefix + "output_frame_004094.jpg",
        prefix + "output_frame_003160.jpg",
        prefix + "output_frame_004292.jpg",
        prefix + "output_frame_004206.jpg",
        prefix + "output_frame_004174.jpg",
        prefix + "output_frame_003463.jpg",
        prefix + "output_frame_001911.jpg",
        prefix + "output_frame_003849.jpg",
        prefix + "output_frame_003113.jpg",
        prefix + "output_frame_003338.jpg",
        prefix + "output_frame_004201.jpg",
        prefix + "output_frame_002145.jpg",
        prefix + "output_frame_004081.jpg",
        prefix + "output_frame_003498.jpg",
        prefix + "output_frame_002177.jpg",
        prefix + "output_frame_002915.jpg",
    ]:
        detector.frame_inference(cv.imread(filename))
    #     frame_bgr = cv.imread(filename)  # load the image
    #     frame_rgb = cv.cvtColor(
    #         frame_bgr, cv.COLOR_BGR2RGB
    #     )  # bgr(openCV)->rgb(darknet)
    #     frame_resized = cv.resize(
    #         frame_rgb, (network_width, network_height), interpolation=cv.INTER_LINEAR
    #     )

    #     # translate to darknet image structure
    #     darknet_frame = darknet.make_image(network_width, network_height, 3)
    #     darknet.copy_image_from_bytes(darknet_frame, frame_resized.tobytes())

    #     # carry out detections
    #     detections = darknet.detect_image(
    #         network, class_names, darknet_frame, thresh=prediction_threshold
    #     )
    #     # free memory
    #     darknet.free_image(darknet_frame)

    #     # output details to the console
    #     darknet.print_detections(detections, True)

    #     # draw detections then display image
    #     frame_with_detections = darknet.draw_boxes(detections, frame_resized, colours)
    #     # cv.imshow("Detections", cv.cvtColor(frame_with_detections, cv.COLOR_RGB2BGR))
    #     # if cv.waitKey() == ord("q"):
    #     #     pass
    #     cv.imwrite("detectionimage.jpg", frame_with_detections)
    #     input("any key to continue")

    # # free memory
    # darknet.free_network_ptr(network)

    sys.exit()
