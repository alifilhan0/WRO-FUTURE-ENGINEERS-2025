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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "vl53l0x.h"
#include "vl53l1x.h"
#include "ultrasonic.h"
#include "tcs34725.h"
#include "tca9548.h"
#define TAG "MAIN"

// Master side (ESP32 -> TCA9548 + sensors)
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 18
#define I2C_MASTER_SDA_IO 17
#define I2C_MASTER_FREQ_HZ 400000

// Slave side (ESP32 <- external board)
#define I2C_SLAVE_NUM I2C_NUM_1
#define I2C_SLAVE_SCL_IO 10
#define I2C_SLAVE_SDA_IO 11
#define I2C_SLAVE_ADDRESS 0x28
#define I2C_SLAVE_RX_BUF_LEN 128
#define I2C_SLAVE_TX_BUF_LEN 128

#define MAX_ULTRASONIC_SENSOR_NUM 4
#define MAX_TOF_NUM 4
#define RGB_CHANNEL 8
#define SECOND_MUX 0

//Sensor descriptors and configurations
static vl53l0x_t *vl53l0x[MAX_TOF_NUM];
static ESP32_TCS34725 *tcs34725;
static vl53l1x_t *vl53l1x[MAX_TOF_NUM];

static ultrasonic_sensor_t ultra[MAX_ULTRASONIC_SENSOR_NUM] = {
    { .trigger_pin = GPIO_NUM_1, .echo_pin = GPIO_NUM_11 },
    { .trigger_pin = GPIO_NUM_2, .echo_pin = GPIO_NUM_12 },
    { .trigger_pin = GPIO_NUM_3, .echo_pin = GPIO_NUM_13 },
    { .trigger_pin = GPIO_NUM_4, .echo_pin = GPIO_NUM_14 },

};

uint8_t tx_buffer;
uint32_t rx_buffer[16];
/* This rx_buffer is suppposed to store sensor data and will be sent back to master board.
    rx_buffer[0] - rx_buffer[3] will have ToF sensor data from right to behind in clockwise order
    rx_buffer[4] - rx_buffer[7] will have data read from ultrasonic sensors
    rx_buffer[8] - rx_buffer[10] will have red, green and blue respectiively data read from tcs34725 rgb sensor.
    rx_buffer[9] - rx_buffer[15] is reserved for flexible adaptation for alternative sensor debugging without 
    dropping any current sensor
*/
i2c_dev_t *dev;
static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

static void i2c_slave_init(void)
{
    i2c_config_t conf_slave = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = I2C_SLAVE_ADDRESS,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_SLAVE_NUM, &conf_slave));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_SLAVE_NUM, conf_slave.mode,
                                       I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0));
}

void sensor_init()
{
    int i=0;
    
    for(i = 0; i < MAX_TOF_NUM; i++)
    {
        tca9548_set_channels(dev, i);
        vl53l0x[i] = vl53l0x_config(I2C_MASTER_NUM, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, -1, 0x29 + i, 00);
        vl53l0x_init(vl53l0x[i]);
        vl53l0x_stopContinuous(vl53l0x[i]);
        vl53l0x_startContinuous(vl53l0x[i], 20000);
        
    }

    for(i = 0; i < MAX_ULTRASONIC_SENSOR_NUM; i++)
    {
        ultrasonic_init(&ultra[i]);

    }

    
    for(i = 0; i < MAX_TOF_NUM; i++)
    {
        tca9548_set_channels(dev, 4 + i);
        vl53l1x[i] = vl53l1x_config(I2C_MASTER_NUM, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, -1, 0x29 + i, 00);
        vl53l1x_init(vl53l1x[i]);
        vl53l1x_stopContinuous(vl53l1x[i]);
        vl53l1x_startContinuous(vl53l1x[i], 20000);
        
    }

    TCS_init(tcs34725, I2C_MASTER_NUM);

}

static void sensor_task(void *arg)
{
    
   uint32_t distance;
   int i; 
    while (1) {
       
        for(i = 0; i < 4; i++)
        {
            tca9548_set_channels(dev, i);
            rx_buffer[i] = vl53l0x_readRangeContinuousMillimeters(vl53l0x[i]);
        }
        for(i = 0; i < MAX_ULTRASONIC_SENSOR_NUM; i++)
        {
            ultrasonic_measure_cm_temp_compensated(&ultra[i], 4, &distance, 25);
            rx_buffer[4 + i] = distance;
        }
        TCS_getRGB(tcs34725, (float *)rx_buffer[8], (float *)rx_buffer[9], (float *)rx_buffer[10]);
        for(i = 0; i < 4; i++)
        {
            tca9548_set_channels(dev, 4 + i);
            rx_buffer[11 + i] = vl53l1x_readSingle(vl53l1x[i], 1);
        }
    }
}


static void slave_task(void *arg)
{
    int data;
    while(1)
    {
        // Check if master sent a request
        int len = i2c_slave_read_buffer(I2C_SLAVE_NUM, &tx_buffer, sizeof(tx_buffer), 10 / portTICK_PERIOD_MS);

        if(len > 0)
        {
            data = rx_buffer[tx_buffer];
            i2c_slave_write_buffer(I2C_SLAVE_NUM, (uint8_t *)data, sizeof(rx_buffer), 10 / portTICK_PERIOD_MS);
        }
    }

}
void app_main(void)
{
    i2c_master_init();
    i2c_slave_init();
    sensor_init();
    // Start background tasks
    xTaskCreate(sensor_task,  "sensor_task",  4096, NULL, 5, NULL);
    xTaskCreate(slave_task,  "slave_task",  4096, NULL, 5, NULL);
}
