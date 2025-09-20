from machine import UART, FPIOA, PWM
import time
import nncase_runtime as nn
import ulab.numpy as np

# ------------------------------
# UART setup
# ------------------------------
fpioa = FPIOA()
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)
uart = UART(UART.UART1, 115200)

fpioa.set_function(11, FPIOA.UART2_TXD)
fpioa.set_function(12, FPIOA.UART2_RXD)
uart2 = UART(UART.UART2, 115200)

# ------------------------------
# Servo setup
# ------------------------------
fpioa.set_function(42, FPIOA.PWM0)
S1 = PWM(0, 50, 0, enable=True)

def Servo(servo, angle):
    angle = max(-90, min(90, angle))  # clamp
    duty = (angle-23 + 90) / 180 * 10 + 2.5
    servo.duty(duty)

# ------------------------------
# Sensor data helpers
# ------------------------------
def _clean_number(s):
    allow = "0123456789.-"
    out = "".join(ch for ch in s if ch in allow)
    try:
        return float(out)
    except:
        return None

def get_sensor_data(command, label, timeout_ms=200):
    try:
        uart2.read()  # flush
    except:
        pass
    try:
        uart2.write((command + "\r\n").encode("utf-8"))
    except:
        try:
            uart2.write(command + "\r\n")
        except:
            return None

    start = time.ticks_ms()
    buf = b""
    while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
        chunk = uart2.read(64)
        if chunk:
            buf += chunk
            try:
                text = buf.decode("utf-8", "ignore")
            except:
                text = ""
            lines = text.replace("\r", "\n").split("\n")
            for ln in reversed(lines):
                ln = ln.strip()
                if not ln:
                    continue
                if ln.startswith(label + ":") or ln.startswith(label + "=") or ln.startswith(label + " "):
                    parts = ln.split(":", 1) if ":" in ln else (ln.split("=", 1) if "=" in ln else [label, ln[len(label):]])
                    if len(parts) >= 2:
                        val = _clean_number(parts[1])
                        if val is not None and val > 0:
                            return val
        else:
            time.sleep_ms(5)
    return None

# ------------------------------
# Load KModel
# ------------------------------
try:
    kpu = nn.kpu()
    kpu.load_kmodel("/sdcard/steering3.kmodel")
    print("Model loaded successfully")
    print("Inputs:", kpu.inputs_size(), "Outputs:", kpu.outputs_size())
except Exception as e:
    print("Failed to load model:", e)
    raise e

# ------------------------------
# Predict steering
# ------------------------------
def _try_run_with_array(arr):
    # Force correct 2D shape (1,3)
    arr2d = np.array([[float(arr[0]), float(arr[1]), float(arr[2])]])
    print(f"Feeding model with array {arr2d.shape}: {arr2d}")

    input_tensor = nn.from_numpy(arr2d)  # single argument
    kpu.set_input_tensor(0, input_tensor)
    kpu.run()
    out = kpu.get_output_tensor(0).to_numpy()
    print(f"Raw model output tensor: {out}")
    return float(out.flatten()[0])

def predict_steering(tfl, tfr, tff):
    try:
        steering = _try_run_with_array([tfl, tfr, tff])
        print(f"Raw steering: {steering}")
        # Scale/clamp if needed
        steering=steering*22*0.7/1000
        print(f"Final steering: {steering}")
#        if abs(steering) > 100:
#            steering = steering
#        elif abs(steering) <= 1:
#            steering = steering
        if steering<-22:
            steering=-22
        if steering>22:
            steering=22
        #steering = max(-90, min(90, steering))

        return steering
    except Exception as e:
        print("Model inference failed:", e)
        return 0.0

# ------------------------------
# Main loop
# ------------------------------
#uart2.write("MF 30")
print("Robot running FORWARD...")
last_valid = {"left": 500, "right": 500, "front": 500}
last_infer = 0
steering = 0.0

try:
    uart2.write("MF 75")
    while True:
        front = get_sensor_data("TFF", "TFF", 20)
        left = get_sensor_data("TFL", "TFL", 20)
        right = get_sensor_data("TFR", "TFR", 20)


        if left is None: left = last_valid["left"]
        else: last_valid["left"] = left
        if right is None: right = last_valid["right"]
        else: last_valid["right"] = right
        if front is None: front = last_valid["front"]
        else: last_valid["front"] = front

        now = time.ticks_ms()
        if time.ticks_diff(now, last_infer) >= 50:  # 10 Hz
            try:
                print(f"\nRaw Sensors: L={left}, R={right}, F={front}")
                steering = predict_steering(left, right, front)
                steering=-steering
#                steering=steering/100
#                if steering<-16:
#                    steering=-16
#                if steering>16:
#                    steering=16

                print(f"→ Applied steering: {steering:.2f}°\n")
            except Exception as e:
                print("Inference error:", e)
                steering = 0.0
            last_infer = now

        Servo(S1, steering)
        time.sleep_ms(20)

except KeyboardInterrupt:
    print("Stopping robot...")
    try: uart2.write(b"ST\r\n")
    except: pass
    Servo(S1, 0)
