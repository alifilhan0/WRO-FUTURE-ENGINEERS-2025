# WRO Future Engineers 2025 – Team Barakah Brigade

We trained the data for the car utilizing a line follower approach, by making the car move through different routes inside the track, collecting the data and training models with it.

An additional ESP32S3 was to be connected to the steering servo, and data was collected by running Line follower code and an LFR sensor to the front and adding followable lines in the main track.

**Build Instructions:-**
1. Install ESP-IDF.
2. Export the variables and add them to path. Follow the [official documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/index.html#installation) 
We went without the VSCode approach in the terminal by

```
$ cd src/other/Line_follower/src/ESP32S3

/* Connect your ESP32-S3 device */

$ idf.py flash

```

And the ESP32-S3 will be ready to be used as a Line Follower Robot to gather Data.

![alt text](src/other/Line_follower/Line_follower_demo.gif)

**Important:- Make a note of the PIN names in the code. It must match the circuit. All our pins follow the GPXX layout in the official datasheet. Please refer to your own board pinout for reference. Below is an attached table with our pin assingments.**

| Pin   | Function|
|-------|---------|
| GP13-14 | UART RX-TX |
| GP17    | Servo PWM |

*Team Barakah Brigade, 2025*
