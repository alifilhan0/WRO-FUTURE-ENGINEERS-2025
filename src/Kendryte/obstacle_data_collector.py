import time, os, gc
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA, PWM

DETECT_WIDTH = 640
DETECT_HEIGHT = 480

# ------------------------------
# LAB color thresholds
# ------------------------------
green_threshold = (20, 60, -50, -20, 10, 40)
red_threshold   = (20, 100, 20, 80, 20, 70)

# ------------------------------
# UART setup
# ------------------------------
fpioa = FPIOA()
# Steering UART
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)
uart = UART(UART.UART1, 115200)

# ToF sensors UART
fpioa.set_function(11, FPIOA.UART2_TXD)
fpioa.set_function(12, FPIOA.UART2_RXD)
uart2 = UART(UART.UART2, 115200)

# ------------------------------
# Servo setup (steering)
# ------------------------------
fpioa.set_function(42, FPIOA.PWM0)
S1 = PWM(0, 50, 0, enable=True)

# ------------------------------
# Helper functions
# ------------------------------
def get_sensor_data(command, label):
    uart2.write(command)
    time.sleep_ms(15)
    data = uart2.read(40)
    if data:
        text = data.decode("utf-8", "ignore").strip()
        if text.startswith(label + ":"):
            try:
                return float(text.split(":")[1])
            except:
                return None
    return None

def read_steering(command, label):
    uart.write(command)
    data = uart.read(40)
    if data:
        text = data.decode("utf-8", "ignore").strip()
        if text.startswith(label + ":"):
            try:
                return float(text.split(":")[1])
            except:
                return None
    return None

# ------------------------------
# Initialize camera and display
# ------------------------------
sensor_cam = Sensor(id=0, width=DETECT_WIDTH, height=DETECT_HEIGHT)
sensor_cam.reset()
sensor_cam.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
sensor_cam.set_pixformat(Sensor.RGB565)

Display.init(Display.VIRT, width=DETECT_WIDTH, height=DETECT_HEIGHT, fps=30)
MediaManager.init()
sensor_cam.run()

fps = time.clock()
start_time = time.ticks_ms()

# Open file for writing
f = open('/data/collected.txt', 'a')

# ------------------------------
# Start with MF 30 for first 5 seconds
# ------------------------------
current_mf = "MF 30"
uart2.write(current_mf)
time.sleep(2)  # wait 5 seconds
current_mf = "MF 70"
uart2.write(current_mf)  # now set MF to 70

try:
    while True:
        fps.tick()
        img = sensor_cam.snapshot()

        # ------------------------------
        # Color tracking
        # ------------------------------
        leftgreenx = leftgreeny = rightredx = rightredy = 0

        # Green
        green_blobs = img.find_blobs([green_threshold], pixels_threshold=20, area_threshold=20, merge=True)
        if green_blobs:
            biggest_green = max(green_blobs, key=lambda b: b.w() * b.h())
            x, y, w, h = biggest_green.rect()
            gx = x - 160
            gy = 240 - (y + h)
            if gy < 130:
                leftgreenx, leftgreeny = gx, gy
                img.draw_rectangle((x, y, w, h), color=(0, 255, 0))
                img.draw_cross(biggest_green.cx(), biggest_green.cy(), color=(255, 0, 0))

        # Red
        red_blobs = img.find_blobs([red_threshold], pixels_threshold=50, area_threshold=50, merge=True)
        if red_blobs:
            biggest_red = max(red_blobs, key=lambda b: b.w() * b.h())
            x, y, w, h = biggest_red.rect()
            rx = (x + w) - 160
            ry = 240 - (y + h)
            if ry < 130:
                rightredx, rightredy = rx, ry
                img.draw_rectangle((x, y, w, h), color=(255, 0, 0))
                img.draw_cross(biggest_red.cx(), biggest_red.cy(), color=(0, 255, 0))

        # ------------------------------
        # ToF sensor data
        # ------------------------------
        left = get_sensor_data("TFL", "TFL") or 0
        right = get_sensor_data("TFR", "TFR") or 0
        front = get_sensor_data("TFF", "TFF") or 0
        steering = read_steering("ANG", "ANG") or 0

        # ------------------------------
        # Write data
        # ------------------------------
        line = "{},{},{},{},{},{},{},{}\n".format(
            left, right, front, leftgreenx, leftgreeny, rightredx, rightredy, steering
        )
        f.write(line)
        f.flush()
        print(line.strip())

        # Show image
        Display.show_image(img)
        gc.collect()
        time.sleep_ms(20)

except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    f.close()
    Display.deinit()
    sensor_cam.stop()
    MediaManager.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    print("File saved at /data/collected.txt")
