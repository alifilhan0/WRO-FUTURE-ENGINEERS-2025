from libs.PipeLine import PipeLine
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
import os, sys, ujson, gc, math
from media.media import *
import nncase_runtime as nn
import ulab.numpy as np
import image
import aidemo
import time

# Simple timing context manager for debug
class ScopedTiming:
    def __init__(self, label, enabled=True):
        self.label = label
        self.enabled = enabled

    def __enter__(self):
        if self.enabled:
            self.start = time.ticks_ms()

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.enabled:
            end = time.ticks_ms()
            print(f"[{self.label}] took {end - self.start} ms")

class FaceDetectionApp(AIBase):
    def __init__(
        self,
        kmodel_path,
        model_input_size,
        anchors,
        confidence_threshold=0.5,
        nms_threshold=0.2,
        rgb888p_size=[224,224],
        display_size=[1920,1080],
        debug_mode=0
    ):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.anchors = anchors
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode

        # Setup 2D pre‐processing
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8
        )

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size or self.rgb888p_size
            top, bottom, left, right, _ = letterbox_pad_param(
                self.rgb888p_size, self.model_input_size
            )
            self.ai2d.pad([0,0,0,0, top, bottom, left, right],
                          0, [104,117,123])
            self.ai2d.resize(
                nn.interp_method.tf_bilinear,
                nn.interp_mode.half_pixel
            )
            self.ai2d.build(
                [1,3, ai2d_input_size[1], ai2d_input_size[0]],
                [1,3, self.model_input_size[1], self.model_input_size[0]]
            )

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            out = aidemo.face_det_post_process(
                self.confidence_threshold,
                self.nms_threshold,
                self.model_input_size[1],
                self.anchors,
                self.rgb888p_size,
                results
            )
            return out[0] if out else out

    def draw_result(self, pl):
        """
        Draw a filled rectangle in the middle of the screen.
        """
        with ScopedTiming("display_draw", self.debug_mode > 0):
            pl.osd_img.clear()

            # Get the center coordinates of the screen
            center_x = self.display_size[0] // 2
            center_y = self.display_size[1] // 2

            # Define the size of the rectangle (adjust as needed)
            rect_width = 200
            rect_height = 100

            # Calculate the top-left corner of the rectangle to center it
            x = center_x - rect_width // 2
            y = center_y - rect_height // 2

            # Draw a filled rectangle (green)
            pl.osd_img.draw_rectangle(
                x, y, rect_width, rect_height,
                color=(0, 255, 0, 255),  # Green color
                thickness=-1  # -1 for filled rectangle
            )

    def _set_display_size(self, display_mode):
        if display_mode == "hdmi":
            return [1920, 1080]
        elif display_mode == "lcd":
            return [800, 480]
        else:
            return [1920, 1080]

    def display_size(self):
        return self._display_size


if __name__ == "__main__":
    display_mode = "hdmi"
    rgb888p_size = [1280, 720]

    kmodel_path = "/sdcard/examples/kmodel/face_detection_320.kmodel"
    anchors_path = "/sdcard/examples/utils/prior_data_320.bin"
    anchors = np.fromfile(anchors_path, dtype=np.float).reshape((4200, 4))

    # set up the pipeline
    pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode)
    pl.create()

    # get display size from pipeline
    display_size = pl.display_size

    # init your face‐detector
    face_det = FaceDetectionApp(
        kmodel_path=kmodel_path,
        model_input_size=[320, 320],
        anchors=anchors,
        rgb888p_size=rgb888p_size,
        display_size=display_size,
        debug_mode=0
    )
    face_det.config_preprocess()

    # main loop
    while True:
        with ScopedTiming("total", True):
            frame = pl.get_frame()
            # Directly draw the filled rectangle without any detections
            face_det.draw_result(pl)
            pl.show_image()
            gc.collect()

    # cleanup (never reached in this infinite loop)
    face_det.deinit()
    pl.destroy()
