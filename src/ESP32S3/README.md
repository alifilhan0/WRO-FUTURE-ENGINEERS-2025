# ESP32S3 Sensor hub.


This is the firmware source code for the ESP32s3 board in our system responsible for offloading various tasks from the mainboard, for example taking care of handling data from the sensors. We made it a bit differently, so that this device is can be communicated with defined commands, much akin to a prompt.As of now, the commands are integers 1 to 16 accesses the respective index of the sensor data storage buffer `rx_buffer[]`, and the required data is sent.  This approach was chosen to keep the board versatile, adaptive, and change/testing friendly because we needed to update the sensors or the design of the car. Note that this project is still in development and we need to organize the code for better reliability.
---

## How does it work?

We made the ESP32s3 act as a master for the sensors, which collect the data and ESP32s3 board calculates the distance in this project. The project uses [tca9548](https://github.com/esp-idf-lib/tca9548) I2C multiplexer to add additional ports for sensors with same/similar behavior and same addresses. Commands 1 to 4 mean `VL53L0X` sensor's distance measurement from 1,2,3,4 meaning right front left back respectively. commands 11 to 15 are mirrored in the same way for `VL53L1X`. 

Now, one might ask why did we do this? It is because we originally started with VL53L0X and we were pretty much okay with it. But VL53L1X's feature of sampling distances @50Hz was quite a compelling feature to us to the project if we wanted to increase the car's speed without sacrificing the accuracy of the distance measurement thus achieving a more reliable and safe system even at faster speeds.

In addition to ToF sensors for distance measurement, we did the same with Ultrasonic sensors, however they are not connected through I2C, rather they are connected through PWM to the ESP32s3 board. We decided to adventure with different sensors, and ultrasonic sensors have long been used for the purpose. We used the [esp-idf-lib ultrasonic sensor](https://github.com/esp-idf-lib/ultrasonic) library for this because the library had the temperature compensated distance measurement function `ultrasonic_measure_cm_temp_compensated`.

The ToF libraries were take from 
    1. [ESP32-VL53L0X](https://github.com/revk/ESP32-VL53L0X)
    2. [ESP32-VL53L1X](https://github.com/revk/ESP32-VL53L1X)
    
Distance data acquired from ToFs are in millimeters and for ultrasonic sensors, as the function suggests, returns the data in centimeters. This data is directly transferred to the main board via the I2C bus based on the received command.

Moreover, we have added the [TCS34725](https://github.com/tcs34725) RGB sensor for counting the laps in the path for the challenge. We are conflicted on deciding if this simple task of counting laps be left to be done by this sensor hub or by the main board as the RGB data needs to be processed and match with the track's color lines present in different locations to decide when a lap is completed. Preferring to make the lap calculation in the mainboard for now, subject to change later.
---

## A bit of description about the task handling
Since the system is supposed to be mostly in motion, it is important to have the latest data in the shortest intervals. However doing them in a regular, simple and streamlined flow might make the whole process slower than it needs to be. Hence, we went with the FreeRTOS build of ESP32s3 firmware. It allows us to run multiple tasks at once, and we are running two such tasks. One is `sensor_task` which continuously samples data from the sensor and handles the calculation. And another task, `slave_task` waits for a command in it's I2C slave port. Once a command is received, data of that instant is sent to the master.

We are currently working on a STOP command, which will make the mainboard terminate every process once the required number of laps is complete.
---

## How to work with the code?
We tried to make the main base of the firmware source to be modular and potentially integrate with many other projects, afterall main goal of this board was to be a flexible, adaptive and configurable sensor expansion board. So it is possible to integrate the source with any other projects as long as it is based on ESP-IDF

For requirements, please check out the dependencies for ESP-IDF installation. Here is a step by step guide for beginners to work with the firmware:-
    1. Install ESP-IDF and its dependencies from the official espressif site. Alternatively, esp-idf extension for VSCode is also available. Make sure the dependencies are available in your system before you install the VSCode extension.
    2. After installation, clone UncleRus's esp-idf-lib repo somewhere for the `tca9548`, `ultrasonic sensors`, `tcs34725`. Follow the repo for installation instructions and how to integrate the extra components into the esp-idf project
    3. Make sure to close the default bash prompt of VSCode if you are using the VSCode extension method. Open ESP-IDF terminal from the bottom, it should look like a console icon.
    4. From there run `idf.py menuconfig` Do not change anything, save it by clicking S and Esc and press Q to quit.
    5. The build environment for the project should be ready. run idf.py build for building the firmware, followed by idf.py -p PORT flash to flash it to your ESP32S3. Replace port with the actual port where the ESP32S3 is connected.
    6. Ready to test and tinker!
-
*Team Barakah Brigade, 2025*
