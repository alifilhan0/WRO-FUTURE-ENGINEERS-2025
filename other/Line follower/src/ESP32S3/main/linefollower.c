#include <stdio.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include "freertos/projdefs.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "common.h"
//sensor.ino
int s[8], k[8], mid[8] = {2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000}, sensor_weight[8] = {-4, -3, -2, -1, 1, 2, 3, 4}, E[4];
int active_lines;
int sum;
int speed = 255;
char flag = 'r';
char side = 'r';

int angle = 0;
float kp = -5, previous_error;
int adj = 0;
int error = 0;
int write(int channel)
{    
    gpio_set_level(S0, channel & 0x01);
    gpio_set_level(S1, (channel >> 1) & 0x01);
    gpio_set_level(S2, (channel >> 2) & 0x01);
    return 0;
}


void work(){
    write(0);
    s[7] = analogRead(SIG);
    write(1);
    s[6] = analogRead(SIG);
    write(2);
    s[5] = analogRead(SIG);
    write(3);
    s[4] = analogRead(SIG);
    write(4);
    s[3] = analogRead(SIG);
    write(5);
    s[2] = analogRead(SIG);
    write(6);
    s[1] = analogRead(SIG);
    write(7);
    s[0] = analogRead(SIG);
}


void error_calculation(){
  for(int i = 0;i<4;i++) E[i] = 0;

  active_lines = 0;
  int i = 0;
  int indi = 0;

  for(int j = 0;j<4;j++){
    while(i<8 && k[i]==0) i++;
    if(i>7) break;
    while(k[i] == 1 && i<8) {
      E[active_lines]+=sensor_weight[i];
      indi++;
      i++;
    }
    E[active_lines] /= indi;
    if(E[active_lines]!=0) active_lines++;
    indi = 0;
  }

  if(active_lines == 0) error = 0;
  else if(active_lines == 1) error = E[0];
  else if(active_lines == 2) error = (side == 'r') ? E[1] : E[0];
  else if(active_lines%2==1) error = E[active_lines/2];
  else error = (E[active_lines/2]+E[active_lines/2-1])/2;
}

void reading(){
    work();
    sum = 0;
  
    for (int i = 0; i < 8; i++) {
    k[i] = (s[i] < mid[i]) ? 0 : 1;
    sum += k[i];
}

    if(k[0]==1 && k[7]==0) flag = 'l';
    else if(k[0]==0 && k[7]==1) flag = 'r'; 

    error_calculation();
}

// sensor.ino end
//PID.ino start

void PID(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(20); // 50Hz update
    TickType_t last_wake = xTaskGetTickCount();
    esp_err_t err;

    for (;;) {
        // periodic timing
        vTaskDelayUntil(&last_wake, period);

        // read sensors and compute
        reading();

        float servo = 1.2f;
        if (sum == 0 && flag != 's') {
            adj = (flag == 'r') ? -16 : 16; // -20 : 20;
            servo *= (20 + adj);
            angle = map(servo, 0, 40, -16, 16); // -20: 20

            err = mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle));
            if (err != ESP_OK) {
                printf("mcpwm set compare failed: %d\n", err);
            }

            // if stuck in lost-line state, poll with small blocking delay but still yield
            int stuck_count = 0;
            while (sum == 0 && stuck_count++ < 200) { // limit to 200 iterations (~4s)
                reading();
                angle = map(servo, 0, 40, -16, 16); // -20, 20
                err = mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle));
                if (err != ESP_OK) {
                    printf("mcpwm set compare failed: %d\n", err);
                }
                vTaskDelay(pdMS_TO_TICKS(5)); // yield
            }
        } else {
            adj = (int)(kp * error);
            servo *= (20 + adj);
            angle = map(servo, 0, 40, -16, 16); // -20, 20
            err = mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle));
            if (err != ESP_OK) {
                printf("mcpwm set compare failed: %d\n", err);
            }
        }

        // Small housekeeping: feed WDT if you're using it manually
        // esp_task_wdt_reset();

        // optional debug print (rate-limited to once per second)
        static int tick = 0;
        if (++tick >= (1000 / 20)) {
            tick = 0;
            //printf("PID: sum=%d flag=%c error=%d adj=%d angle=%d\n", sum, flag, error, adj, angle);
        }

        vTaskDelay(pdMS_TO_TICKS(15)); // 200 Hz sensor update
    }
}

//PID.ino end
