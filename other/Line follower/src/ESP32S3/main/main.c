#include <stdio.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include "freertos/projdefs.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "common.h"
#define TXD_PIN (13)
#define RXD_PIN (14)

static const int RX_BUF_SIZE = 1024;


void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, RX_BUF_SIZE * 2, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void handle_command(char* cmd) {
    char response[64];

    if (strncmp(cmd, "ANG", 3) == 0) {
        snprintf(response, sizeof(response), "ANG:%d\n", angle);
    } else {
        snprintf(response, sizeof(response), "ERROR:UNKNOWN_CMD\n");
    }

    vTaskDelay(pdMS_TO_TICKS(20));
    uart_write_bytes(UART_NUM_1, response, strlen(response));
}

void uart_task(void *arg)
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

void app_main(void)
{
    uart_init();
    adc_init();
    servo_init();
    gpio_set_direction(S0, GPIO_MODE_OUTPUT);
    gpio_set_direction(S1, GPIO_MODE_OUTPUT);
    gpio_set_direction(S2, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(-20))); //-20
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(20))); //20
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));
    xTaskCreatePinnedToCore(uart_task, "uart_task", 4096, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(PID, "PID", 8192, NULL, 5, NULL, 1);
}
