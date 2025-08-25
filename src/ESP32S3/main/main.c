/**********************************************************************************
 * 
 * 
 *  Copyright 2025 Alif Ilhan https://github.com/alifilhan0
 *  This is the firmware for esp32s3 sensor daughter board.
 *  I have created two tasks in the firmware, and they will keep updating
 *  the rx buffer with sensor data. And a slave task, which just waits for the master
 *  to send a command, it will respond according to command.
 * 
 * 
 *  TODO:- Add slave writeback to the master functionality
 * 
 * 
 **********************************************************************************/

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <driver/i2c.h>
#include <tca9548.h>
#include <vl53l1x.h>
#include <ultrasonic.h>
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#define MAX_DISTANCE_CM 500

// Ultrasonic pins
#define TRIGGER_GPIO1 36
#define ECHO_GPIO1 35
#define TRIGGER_GPIO2 33
#define ECHO_GPIO2 17
#define TRIGGER_GPIO3 34
#define ECHO_GPIO3 18

// I2C slave pins & address
#define I2C_SLAVE_SDA 13
#define I2C_SLAVE_SCL 14
#define I2C_SLAVE_ADDRESS 0x12
#define I2C_SLAVE_NUM I2C_NUM_1
#define I2C_SLAVE_RX_BUF_LEN 64
#define I2C_SLAVE_TX_BUF_LEN 64

// Global buffers for sensor data
static uint16_t vl53_distances[4] = {0};
static float ultra_distances[3] = {0.0f};

// TCA9548 and VL53L1X
static vl53l1x_t *vl53l1x_arr[4];
static i2c_dev_t i2c_switch = {0};

// Ultrasonic sensors
ultrasonic_sensor_t sensor1 = { .trigger_pin = TRIGGER_GPIO1, .echo_pin = ECHO_GPIO1 };
ultrasonic_sensor_t sensor2 = { .trigger_pin = TRIGGER_GPIO2, .echo_pin = ECHO_GPIO2 };
ultrasonic_sensor_t sensor3 = { .trigger_pin = TRIGGER_GPIO3, .echo_pin = ECHO_GPIO3 };
static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (13)
#define RXD_PIN (14)

#define MOTOR_PWM_UNIT MCPWM_UNIT_0
#define MOTOR_TIMER MCPWM_TIMER_0
#define MOTOR_GEN_A MCPWM_OPR_A
#define MOTOR_GEN_B MCPWM_OPR_B
#define PWM_FREQ_HZ 1000

#define DIR_A1 9
#define DIR_A2 38
#define PWM_A 37
#define STBY 33
// ================= Sensor Task =================
void sensor_task(void *pvParameters)
{
    float distance;

    while(1)
    {
        // ---- Ultrasonics ----
        ultrasonic_measure(&sensor1, MAX_DISTANCE_CM, &distance);
        ultra_distances[0] = distance;

        ultrasonic_measure(&sensor2, MAX_DISTANCE_CM, &distance);
        ultra_distances[1] = distance;

        ultrasonic_measure(&sensor3, MAX_DISTANCE_CM, &distance);
        ultra_distances[2] = distance;

        // ---- VL53L1X ----
        ESP_ERROR_CHECK(tca9548_set_channels(&i2c_switch, BIT(0)));
        vl53_distances[0] = vl53l1x_readSingle(vl53l1x_arr[0], 1);

        ESP_ERROR_CHECK(tca9548_set_channels(&i2c_switch, BIT(2)));
        vl53_distances[1] = vl53l1x_readSingle(vl53l1x_arr[1], 1);

        ESP_ERROR_CHECK(tca9548_set_channels(&i2c_switch, BIT(4)));
        vl53_distances[2] = vl53l1x_readSingle(vl53l1x_arr[2], 1);

        ESP_ERROR_CHECK(tca9548_set_channels(&i2c_switch, BIT(7)));
        vl53_distances[3] = vl53l1x_readSingle(vl53l1x_arr[3], 1);

    }
}

void init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
void motor_init() {
    // Direction pins
    gpio_set_direction(DIR_A1, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR_A2, GPIO_MODE_OUTPUT);
    // Standby pin
    gpio_set_direction(STBY, GPIO_MODE_OUTPUT);
    gpio_set_level(STBY, 1); // enable driver

    // PWM pin
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_A);

    mcpwm_config_t pwm_config = {
        .frequency = 1000,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

// speed_percent: 0-100, direction: 1=forward, 0=reverse
void motor_set(int speed_percent, int direction) {
    if(direction == 1 && speed_percent != 0) {
        gpio_set_level(DIR_A1, 1);
        gpio_set_level(DIR_A2, 0);
    } else if(direction == 0 && speed_percent != 0) {
        gpio_set_level(DIR_A1, 0);
        gpio_set_level(DIR_A2, 1);
    } else if(direction == 0 && speed_percent == 0) {
        gpio_set_level(DIR_A1, 0);
        gpio_set_level(DIR_A2, 0);
    }
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed_percent);
    printf("Motor speed %d%%, direction %d\n", speed_percent, direction);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Handle received UART commands
void handle_command(char* cmd) {
    char response[64];
    if (strncmp(cmd, "TOFLeft", 7) == 0) {
        int tof = vl53_distances[3];
        snprintf(response, sizeof(response), "TOFLeft:%d\n", tof);
    } else if (strncmp(cmd, "TOFRight", 6) == 0) {
        int us = vl53_distances[1];
        snprintf(response, sizeof(response), "TOFRight:%d\n", us);
    } else if (strncmp(cmd, "TOFBack", 6) == 0) {
        int us = vl53_distances[0];
        snprintf(response, sizeof(response), "TOFBack:%d\n", us);
    } else if (strncmp(cmd, "TOFFront", 6) == 0) {
        int us = vl53_distances[2];
        snprintf(response, sizeof(response), "TOFFront:%d\n", us);
    } else if (strncmp(cmd, "USLeft", 6) == 0) {
        float us = ultra_distances[0];
        snprintf(response, sizeof(response), "USLeft:%f\n", us);
    } else if (strncmp(cmd, "USRight", 7) == 0) {
        float us = ultra_distances[2];
        snprintf(response, sizeof(response), "USRight:%f\n", us);
    } else if (strncmp(cmd, "USFront", 7) == 0) {
        float us = ultra_distances[1];
        snprintf(response, sizeof(response), "USFront:%f\n", us);
    } else if (strncmp(cmd, "FORWARD", 7) == 0) {
        motor_set(100, 1);
        snprintf(response, sizeof(response), "MOTOR:FORWARD\n");
    } else if (strncmp(cmd, "BACKWARD", 8) == 0) {
        motor_set(100, 0);
        snprintf(response, sizeof(response), "MOTOR:BACKWARD\n");
    } else if (strncmp(cmd, "FORWARD_SLOW", 12) == 0) {
        motor_set(50, 1);
        snprintf(response, sizeof(response), "MOTOR:FORWARD_SLOW\n");
    } else if (strncmp(cmd, "BACKWARD_SLOW", 13) == 0) {
        motor_set(50, 0);
        snprintf(response, sizeof(response), "MOTOR:BACKWARD_SLOW\n");
    } else if (strncmp(cmd, "STOP", 4) == 0) {
        motor_set(0, 0);
        snprintf(response, sizeof(response), "MOTOR:STOP\n");
    } else {
        snprintf(response, sizeof(response), "ERROR:UNKNOWN_CMD\n");
    }
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

// ================= Main =================
void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    // ---- Initialize TCA9548 ----
    ESP_ERROR_CHECK(tca9548_init_desc(&i2c_switch, 0x70, 0, CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL));

    // ---- Initialize VL53L1X ----
    for(int i=0;i<4;i++){
        vl53l1x_arr[i] = vl53l1x_config(0, CONFIG_I2C_MASTER_SCL, CONFIG_I2C_MASTER_SDA, -1, 0x29, 0);
    }

    // Start each sensor on different TCA channels
    const uint8_t channels[4] = {0,2,4,7};
    for(int i=0;i<4;i++){
        ESP_ERROR_CHECK(tca9548_set_channels(&i2c_switch, BIT(channels[i])));
        vl53l1x_init(vl53l1x_arr[i]);
        vl53l1x_setROISize(vl53l1x_arr[i], 18, 15);
        vl53l1x_stopContinuous(vl53l1x_arr[i]);
        vl53l1x_startContinuous(vl53l1x_arr[i], 20000);
    }

    // ---- Initialize Ultrasonic ----
    ultrasonic_init(&sensor1);
    ultrasonic_init(&sensor2);
    ultrasonic_init(&sensor3);

    // ---- Start Tasks ----
    init();
    motor_init();
    xTaskCreatePinnedToCore(uart_slave_task, "uart_slave_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(sensor_task, "sensor_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL, 1);
}
