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
- **Real-time RTOS Decision Engine**: Powered by RT-Smart, giving predictable, scheduled execution for safety and speed. Providing an environment 
- **Microcontroller Offload Architecture**: Peripheral-heavy sensor reading offloaded to a C-programmed sensor hub board.
- **PWM-driven Motion**: Real-time speed and torque adjustments via software PWM on hardware-level pins.
- **Visual Processing Backbone**: Using the OV5647 camera as the main eye of the vehicle.
- **Machine Learning assisted motion**: Following the current trends, and picking up with the modern pace, ML assisted motion brings us the edge of driving and transportation technology with automation and reliablity

---

## III. The Team

| Name                  | Role                        | Focus Areas |
|-----------------------|-----------------------------|-------------|
| **Sadnan Adib Khan**  | Team Lead                   | Strategy, PCB design, mechanical layout |
| **Alif Ilhan**        | Embedded Systems & Power    | Firmware, electrical design, sensor integration and bringup |
| **Azmain Inqiad Adib**| AI & Algorithm Architect    | Machine Learning, decision making, and camera vision processing |

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
| **Mainboard** | 01Studios Kendryte K230 CanMV board running Python, Machine learning models for high-level logic |
| **Sensor Hub**      | `ESP32S3 Pico` microcontroller board running ESP-IDF V5.5.1 bundled FreeRTOS, managing sensors and UART communication with mainboard |
| **Drive System**    | PWM-based motor control for differential drive based on `Sparkfun TB6612FNG` driver |
| **Power Delivery**  | Customized for proper power distribution from 12V to 3.3v sensors and 5v modules. Source: Heavy duty 95C battery 12V |
| **Steering System** | Anti-Ackermann steering with `JX-6221-MGI` Servo |

## Sensors and Interfacing
There are two main group of sensors in the whole system. 2x MIPI CSI cameras based on OV5647 connected through the CSI bus in the main board for vision on both sides, and the sensor and data acquisition module which is based on a custom firmware on ESP32-S3 Pico development board. 

### How it works

The cameras are responsible for the vision and are the primary sources of data for the system. And the sensors are operated from an ESP32-S3 daughter board for real-time accurate calculations without adding more load to the main CPU.

### Sensor specifications

**Name: TCS34725**
  - Type: RGB color sensor
  - Communication: I2C, Address: 0x29 through PCA9548A channel 7
  - Purpose: Calculate the amount of laps taken.
  
**Name: VL53L1X**
  - Type: ToF sensor
  - Communication: I2C Address 0x29, through PCA9548A channels 0,3,4,5
  - Range & Blind spots: upto 400cm & 4cm
  - Purpose: Calculate distance of the closest obstruction from the present location.
  The main reason being that this sensor can sample data @ 50Hz, which is a good feature if we want to operate the car in high velocity without reducing accuracy.
  
**Name: OV5647**
  - Type: CSI camera module
  - Communication: MIPI CSI bus ID 0
  - Purpose: The main vision of the car system. Has 120 degree field of vision which is ample for our requirement to provide a field of vision for the system. It is responsible for detecting objects and any artifact on its path and send data to be processed.
  
  

---
## VI. Software Stack


### MCU Layer (C)

- Direct hardware access for:
  - ToF
  - Ultrasonic
  - RGB sensors
- Real-time reading and packaging of sensor data
- Communication with mainboard via I2C

**How it works**

Initial plans for keeping the system as a single processing unit system was discarded and a sub-controller approach was taken. Instead of spending CPU cycles on polling, converting and calculating the sensor data, the ESP32S3 Pico dev boards were tasked with gathering sensor data. The sensors were initialised and communicated with through I2C. VL53L1X ToF sensors were chosen for the main action since their more range, better sensitivity and quick responsiveness. Despite the option of being able to change their addresses with XSHUT pins, a PCA9548A multiplexer module was chosen to reduce design complexiblity, and ToF and color sensors were connected to their respective channels for convenience. The data from ToF sensors were gathered in a continuous looping task utilising the rich features exposed by the ESP-IDF v5.5.1 for the microcontroller. Additionally, the board works as a true companion of the main board by also controlling the main drive motors using it's `MCPWM` hardware.
The task of storing them in a global variable, ready to be transported to the mainboard in demand was done in the following code:-

```
/* Initialise the sensors */
void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    tca9548_init_desc(&i2c_switch, 0x70, 0, CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL);

    const uint8_t channels[4] = {0,4,3,5};
    for(int i=0;i<4;i++){
        tca9548_set_channels(&i2c_switch, BIT(channels[i]));
        vl53l1x_arr[i] = vl53l1x_config(0, CONFIG_I2C_MASTER_SCL, CONFIG_I2C_MASTER_SDA, -1, 0x29, 0);
    }
    for(int i=0;i<4;i++){
        tca9548_set_channels(&i2c_switch, BIT(channels[i]));
        vl53l1x_init(vl53l1x_arr[i]);
        vl53l1x_setROISize(vl53l1x_arr[i], 4, 4);
        //vl53l1x_stopContinuous(vl53l1x_arr[i]);
        vl53l1x_startContinuous(vl53l1x_arr[i], 20000); // 50Hz
    }

    tca9548_set_channels(&i2c_switch, 7);
    TCS_init(TCS, 0, CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL);
    init();
    motor_init();
    xTaskCreatePinnedToCore(uart_slave_task, "uart_slave_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(sensor_task, "sensor_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL, 1);
}

```

the `xTaskCreatePinnedToCore` creates two simultaneously running loops `uart_slave_task` and `sensor_task` to keep gathering data from the sensors and to respond to external requests. The sensor data is collected by

```
void sensor_task(void *pvParameters)
{
    float r = 0.0f, g = 0.0f, b = 0.0f;
    while(1)
    {
        tca9548_set_channels(&i2c_switch, BIT(3));
        vl53_distances[1] = vl53l1x_readSingle(vl53l1x_arr[1], 1); //right

        tca9548_set_channels(&i2c_switch, BIT(4));
        vTaskDelay(pdMS_TO_TICKS(10));
        vl53_distances[0] = vl53l1x_readSingle(vl53l1x_arr[0], 1); //Left

        tca9548_set_channels(&i2c_switch, BIT(5));
        vl53_distances[2] = vl53l1x_readSingle(vl53l1x_arr[2], 1); //front

        tca9548_set_channels(&i2c_switch, BIT(0));
        vl53_distances[3] = vl53l1x_readSingle(vl53l1x_arr[3], 1); //Back


        tca9548_set_channels(&i2c_switch, BIT(7));
        getlap(TCS, r, g, b);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

The task is responsible for saving the information about the surroundings from ToF sensors and the color sensor for a sense of surrouding and counting laps. The output is saved in global variables and is waiting to be sent to the master.

```
/* Handle received UART commands */
void handle_command(char* cmd) {
    char response[64];

    if (strncmp(cmd, "TFL", 3) == 0) {
        snprintf(response, sizeof(response), "TFL:%hu\n", vl53_distances[0]);
    } else if (strncmp(cmd, "TFR", 3) == 0) {
        snprintf(response, sizeof(response), "TFR:%hu\n", vl53_distances[1]);
    } else if (strncmp(cmd, "TFB", 3) == 0) {
        snprintf(response, sizeof(response), "TFB:%hu\n", vl53_distances[3]);
    } else if (strncmp(cmd, "TFF", 3) == 0) {
        snprintf(response, sizeof(response), "TFF:%hu\n", vl53_distances[2]);
    } else if (strncmp(cmd, "MF", 2) == 0) {
        int buf = atoi(cmd + 2);
        motor_set(buf, 2);
        snprintf(response, sizeof(response), "MF %d\n", buf);
    } else if (strncmp(cmd, "ALL", 3)) {
        snprintf(response, sizeof(response), "TFL:%hu TFR:%hu TFB:%hu TFF:%hu\n", vl53_distances[0], vl53_distances[1], vl53_distances[3], vl53_distances[2]); //get all data at once
        vTaskDelay(pdMS_TO_TICKS(30));
    } else if (strncmp(cmd, "MB", 2) == 0) {
        int buf = atoi(cmd + 2);
        motor_set(buf, 1);
        snprintf(response, sizeof(response), "MB %d\n", buf);
    } else if (strncmp(cmd, "STOP", 4) == 0) {
        motor_set(0, 0);
        snprintf(response, sizeof(response), "STOP\n");
    } else if (strncmp(cmd, "LAP", 5) == 0) {
        snprintf(response, sizeof(response), "%d\n", lap);
    } else {
        snprintf(response, sizeof(response), "ERROR:UNKNOWN_CMD\n");
    }
  /* response of the command */
    uart_write_bytes(UART_NUM_1, response, strlen(response)); 
}

void uart_slave_task(void *arg)
{
    char* data = (char*) malloc(RX_BUF_SIZE + 1);
    while(1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 10 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            handle_command(data);
        }
    }
}
```
The MCU board waits for commands from the mainboard and when it receives the command, it just reads from the global variable at that state and stringifies it to be sent over UART. This approach was chosen for its simplicity and flexivlity of usage. We left debug commands for each sensor or option to call them all using the ALL command. Moreover, full control over speed and direction of driver motors are achieved by the `MF %d` and `MB %d` commands. The commands mean MF = Motor forward and MB  = Motor backward. and it epxects a value of 0-100 to be sent with it for speed control.
Then the motor control is done in the following process

```
void motor_init() {
    /* Direction pins */
    gpio_set_direction(DIR_A1, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR_A2, GPIO_MODE_OUTPUT);
    gpio_set_direction(STBY, GPIO_MODE_OUTPUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_A);
  /*MCPWM Hardware init */
    mcpwm_config_t pwm_config = {
        .frequency = 1000,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

/* speed_percent: 0-100, direction: 1=forward, 0=reverse */
void motor_set(int speed_percent, int direction) {
    if(direction == 2) {
        gpio_set_level(DIR_A1, 1);
        gpio_set_level(DIR_A2, 0);
    } else if(direction == 1) {
        gpio_set_level(DIR_A1, 0);
        gpio_set_level(DIR_A2, 1);
    } else if(direction == 0) {
        gpio_set_level(DIR_A1, 0);
        gpio_set_level(DIR_A2, 0);
    }
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed_percent);
}

```

Using the well documented and specialised hardware for motor control, we could control speed of the motor using one PWM pin and Two direction GPIOs with a motor driver. The commands are recieved and the `Sparkfun TB6612FNG` Motor driver handles the motor control from the microcontroller response seamlessly.

This turns the board into a true controller companion board, and this board can be controlled by UART commands by anything out there as long as proper commands are issued to its UART pins at 115200 baud speed.


### RTOS Layer (Python)

- Task scheduling via **RT-Smart**
- Lap counting, obstacle tracking, and response planning
- I2C interfacing with microcontroller board
- Debugging via USB/UART

The Mainboard 01Studios CanMV K230 runs a unique software stack, the little core runing at 800Mhz runs a linux kernel, and the big core running at 1.6GHz runs RT-Smart which exposes rich features like CanMV IDE support which was the main means of programming the bot for the competition. So the software stack of the mainboard runs in the big core over RT-Smart and the linux running smaller core is largely unused.

The board primarily uses UART for communication and a PWM pin for controlling the servo directly.

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

### Day 16
**Summary:**  
Following sensors were sourced
- VL53L1X ToF sensor
- TCS34725 RGB Color sensor
Follwing motor drivers chosen
- Sparkfun TB6612FNG

A dc motor with an axle shaft was chosen to drive a differential drive for our main drive of the car.
Finalise a power distribution board with buck controllers giving 3.3 and 5v from the buck controllers. 12V output direct from the battery

Electrical design finalised. Sensor mounts, camera horn fixed and wheels designed for maximum speed in our given configuration.

### Day 25
***Summary:**__
Machine learning model for Open challenge training has started. Had to convert input tensors to 1D array to feed to the custom model design accepted by the board. Tuning and calibration is still ongoing.
Major documentation of our training process for the model is incoming.

---

## VIII. Hardware Documentation (Upcoming)

- `schemes/main_circuit.pdf` – Power system, drive control
- `schemes/sensor_bus.pdf` – I2C + sensor connections
- `schemes/mcu_pinout.png` – Sensor MCU pin assignments
- `models/car-chassis-v1.stl` – Chassis print file
- `models/sensor-mount.stl` – Front-facing sensor bracket

---


## IX. Board Specifications (To be updated)

- **Mainboard:**
  - Processor: 01Studios Kendryte K230
  - OS: CPU1: RT-Smart RTOS providing CanMV IDE, CPU0: Linux kernel
  - Communication: I2C Master, UART Debug
  - Interfaces: PWM, GPIO, USB

- **Sensor Hub:**
  - Board: Waveshare ESP32-S3 Pico
  - I2C Multiplexer: PCA9548a
  - Firmware SDK: ESP-IDF v5.5.1 with FreeRTOS
  - Protocol: UART. 115200 Baud
  - Ports: I2C (ToF, RGB), PWM(motor control)
  - Connection: Mainboard I2C -> Sensor Hub I2C -> Hub's Multiplexer -> Sensor's SDA/SCL pins.
  
  This design is actually very convenient since we offload some of the services to the sensor hub and can directly get processed ouput with a variety of commands we are working on.

---

## X. Demonstration Video

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
