import time, os, gc, sys, math
from media.sensor import *
from media.display import *
from media.media import *
from collections import namedtuple

DETECT_WIDTH = 640
DETECT_HEIGHT = 480
MIN_SIZE = 120
SIDE_W = 40      # width of the side filled boxes
SIDE_OFFSET = 40 # gap between main box and side boxes

Rect = namedtuple("Rect", ["x", "y", "w", "h", "code", "area", "cx", "cy"])

# Red and green thresholds
thresholds = [
    (30, 100,  15, 127,  15, 127),   # Red   -> code == 1
    (35,  90, -70, -20, -10,  30)    # Green -> code == 2
]

def is_red(rect):   return rect.code == 1
def is_green(rect): return rect.code == 2

def rects_overlap(r1, r2):
    return not (r1.x + r1.w < r2.x or r1.x > r2.x + r2.w or
                r1.y + r1.h < r2.y or r1.y > r2.y + r2.h)

sensor = Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT)
sensor.reset()
sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
sensor.set_pixformat(Sensor.RGB565)

Display.init(Display.VIRT, width=DETECT_WIDTH, height=DETECT_HEIGHT, fps=100)
MediaManager.init()
sensor.run()
fps = time.clock()

green = (0, 255, 0)  # for the “no detection” box

try:
    while True:
        fps.tick()
        os.exitpoint()
        img = sensor.snapshot()

        # 1) Collect all blobs and enforce min size
        all_rects = []
        for blob in img.find_blobs(thresholds,
                                   pixels_threshold=100,
                                   area_threshold=100,
                                   merge=True):
            w = max(blob.w(), MIN_SIZE)
            h = max(blob.h(), MIN_SIZE)
            x = blob.cx() - w // 2
            y = blob.cy() - h // 2
            x = max(0, min(x, DETECT_WIDTH - w))
            y = max(0, min(y, DETECT_HEIGHT - h))
            area = w * h
            all_rects.append(Rect(x, y, w, h, blob.code(), area,
                                  blob.cx(), blob.cy()))

        # 2) Keep only largest in overlaps
        all_rects.sort(key=lambda r: -r.area)
        final_rects = []
        for r in all_rects:
            if all(not rects_overlap(r, other) for other in final_rects):
                final_rects.append(r)

        # 3) Draw them with side boxes
        for rect in final_rects:
            color = (255, 0, 0) if is_red(rect) else (0, 255, 0)
            label = "RED" if is_red(rect) else "GREEN"
            # main box
            img.draw_rectangle([rect.x, rect.y, rect.w, rect.h], color=color)
            img.draw_cross(rect.cx, rect.cy, color=color)
            img.draw_string_advanced(rect.x + 2, rect.y + 2, 32, label)

            # side boxes
            top = rect.y
            height = rect.h
            if is_red(rect):
                # red object: left red, right green
                lx = max(0, rect.x - SIDE_W - SIDE_OFFSET)
                rx = min(DETECT_WIDTH - SIDE_W, rect.x + rect.w + SIDE_OFFSET)
                img.draw_rectangle(lx, top, SIDE_W, height, color=(255,0,0), thickness=120)
                img.draw_rectangle(rx, top, SIDE_W, height, color=(0,255,0), thickness=120)

            elif is_green(rect):
                # green object: left green, right red (swapped)
                lx = max(0, rect.x - SIDE_W - SIDE_OFFSET)
                rx = min(DETECT_WIDTH - SIDE_W, rect.x + rect.w + SIDE_OFFSET)
                img.draw_rectangle(lx, top, SIDE_W, height, color=(0,255,0), thickness=120)
                img.draw_rectangle(rx, top, SIDE_W, height, color=(255,0,0), thickness=120)

        # 4) If nothing detected, draw centered green box using mean_pooled()
        if not final_rects:
            small_img = img.mean_pooled(4, 4)  # Makes a copy.
            x = (img.width() // 2) - (small_img.width() // 2)
            y = (img.height() // 2) - (small_img.height() // 2)
            img.draw_rectangle(x, y,
                               small_img.width(), small_img.height(),
                               color=green, thickness=120)

        Display.show_image(img)
        gc.collect()
        print(fps.fps())

except KeyboardInterrupt:
    print("User stopped")
except BaseException as e:
    print(f"Exception: {e}")
finally:
    sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
