# WRO Future Engineers 2025 – Team Barakah Brigade

_"Machines may move, but minds make them meaningful."_

---

## I. The Chronicle Begins

At the edge of innovation, where youth meets precision, a story is being written — in copper traces, silicon minds, and code that dares to think.

Welcome to the official documentation of **Team Barakah Brigade**, proudly representing **Bangladesh** in the **WRO Future Engineers 2025** challenge. Our machine is not only a competition entry — it's a product of relentless design, weekend solder burns, deep debugging sessions, and the shared dream of building something that _knows what to do_ when left alone.

---

## II. The Machine: Purpose, Not Parts

Our vehicle is modular, intelligent, and ever-adapting. It uses a hybrid sensor approach to interpret its surroundings and make autonomous driving decisions in real-time — under the constraints and chaos of a real track.

### Core Concepts:

- **Multi-sensor Fusion**: For reliable perception, redundancy, and adaptation.
- **Real-time RTOS Decision Engine**: Powered by RT-Smart, giving predictable, scheduled execution for safety and speed.
- **Microcontroller Offload Architecture**: Peripheral-heavy sensor reading offloaded to a C-programmed sensor hub board.
- **PWM-driven Motion**: Real-time speed and torque adjustments via software PWM on hardware-level pins.
- **Visual Processing Backbone**: Using the OV5647 camera as the main eye of the vehicle.

---

## III. The Team

| Name                  | Role                        | Focus Areas |
|-----------------------|-----------------------------|-------------|
| **Sadnan Adib Khan**  | Team Lead                   | Strategy, PCB design, mechanical layout |
| **Alif Ilhan**        | Embedded Systems & Power    | Firmware, electrical design, integration |
| **Azmain Inqiad Adib**| AI & Algorithm Architect    | Sensor data logic, decision making, tuning |

---

## IV. Repository Layout

```
 t-photos/       # Team portraits, formal and behind-the-scenes
 v-photos/       # Vehicle photos (top, internals, close-ups)
 video/          # Demo video link (inside `video.md`)
 schemes/        # Circuit diagrams and schematic PDFs
 src/            # Firmware and software codebase
 models/         # 3D printable files, CAD assets
 other/          # Supporting documents, external links
```

---

## V. Architecture Overview

### Hardware Components

| Module              | Description |
|---------------------|-------------|
| **Main Controller** | RTOS-enabled board running Python for high-level logic |
| **Sensor Hub**      | I2C-connected microcontroller board running C, managing sensors |
| **Drive System**    | PWM-based motor control for differential steering |
| **Power Delivery**  | Isolated, filtered DC rail for logic and motor subsystems |

#### Sensor Suite

- **ToF Sensors** – Short-range high-accuracy obstacle detection
- **Ultrasonic Modules** – General coverage and redundancy
- **RGB Color Sensor** – Lap detection and marker identification
- **OV5647 Camera** – Vision-based steering (under development)

---

## VI. Software Stack

### RTOS Layer (Python)

- Task scheduling via **RT-Smart**
- Lap counting, obstacle tracking, and response planning
- I2C interfacing with microcontroller board
- Debugging via USB/UART

After we are done with the full code this will be explained in detail.

### MCU Layer (C)

- Direct hardware access for:
  - ToF
  - Ultrasonic
  - RGB sensors
- Real-time reading and packaging of sensor data
- Communication with mainboard via I2C

After we are done with the full code this will be explained in detail.

---

## VII. Development Blog

> This section is meant to be _alive_. All development, test insights, failures, and breakthroughs will be logged here. Entries should follow the format below and be added to `/logs/` as the project evolves.


### Day 2

**Summary:**  
Finalise on the steering method.

**Breakthroughs:**  
We decied to use Ackermann steering mechanism

**Obstacles:**  
We were facing troubles in making the ackerman steering mechanism simulation give expected results.

**Next Steps:**  
Found out that our pivot points were wrong. Fixed it, now the ackerman steering works with good accuracy. Not perfect but it works reliably now.

### Day 3

**Summary:**  
Finalise on the sensors

**Breakthroughs:**  
We decied to use multiple arrays of sensors to improve accuracy

**Obstacles:**  
The board which we are currently working on exposes only one I2C port and only a few PWM pins. Also, we came upon some sensors which were totally undocumented, but looked really good on paper so we went on and collected some of them

**Next Steps:**  > These will be expanded into detailed PDFs, schematics, and circuit notes.
A sensor hub based design approach was taken, a ESPP32-S3-Pico was chosen as an expansion board to connect the sensors, the I2C-only sensors were connected via an I2C Multiplexer PCA9854a. And the PWM based sensors were connected to the ESP32's PWM pins, so we can keep the mainboard's PWM pins to drive motors. This approach was choesn to reduce the delay between getting the information from sensors and acting upon it i.e. sending signals to the motor controller. The undocumented sensors are being reverse engineered and tested.

### Day 8

**Summary:**  
Initial code for object and colour detection

**Breakthroughs:**  
A very rough initial code for detecting coloured objects and predicting the turns.

**Obstacles:**  
Not decided yet on which method to use to attain highest accuracy and efficiency

**Next Steps:**  
Refine the algorithm and start real-time testing.

---

## VIII. Hardware Documentation (Upcoming)

- `schemes/main_circuit.pdf` – Power system, drive control
- `schemes/sensor_bus.pdf` – I2C + sensor connections
- `schemes/mcu_pinout.png` – Sensor MCU pin assignments
- `models/car-chassis-v1.stl` – Chassis print file
- `models/sensor-mount.stl` – Front-facing sensor bracket

---

## IX. Sensor Specifications (To be updated)

| Sensor         | Model     | Range     | Accuracy | Protocol | Address | Connection |
|----------------|-----------|-----------|----------|----------|---------|------------|
| ToF Front      | VL53L0X   | 0–2m      | ±1cm     | I2C      | 0x29    | Sensor Hub I2C |
| Ultrasonic L/R | CS100A   | 2–400cm   | ±3mm     | GPIO     | N/A     | Sensor Hub PWM |
| RGB Bottom     | TCS34725  | Visible spectrum | Color/Light | I2C  | 0x29    | Sensor Hub I2C |
| Camera Module  | OV5647    | 5MP       | –        | CSI      | –       | Mainboard's Camera Interface |

---

## X. Board Specifications (To be updated)

- **Mainboard:**
  - Processor: Model TBD
  - OS: RT-Smart RTOS
  - Communication: I2C Master, UART Debug
  - Interfaces: PWM, GPIO, USB

- **Sensor Hub:**
  - MCU: Model TBD
  - I2C Multiplexer: PCA9548a 
  - Clock: TBD
  - Firmware Lang: C
  - I2C Slave Address: 0x10
  - Ports: GPIO (Ultrasonic), I2C (ToF, RGB)
  - Connection: Mainboard I2C -> Sensor Hub I2C -> Hub's Multiplexer -> Sensor's SDA/SCL pins.
  
  This design is actually very convenient since we offload some of the services to the sensor hub and can directly get processed ouput with a variety of commands we are working on.

---

## XI. Demonstration Video

> Video file link will be published in `/video/video.md`  
> Will include:
> - Initial boot sequence
> - Manual vs autonomous mode (if applicable)
> - Lap counting and obstacle response
> - Debug overlay (optional)

---

## XII. Closing Thoughts

This documentation is not finished — by design.

It is a breathing project log, and the robot it chronicles is still being taught how to see, move, and decide. You are looking not at a submission, but a story in progress.

Stay tuned.

—  
*Team Barakah Brigade, 2025*
