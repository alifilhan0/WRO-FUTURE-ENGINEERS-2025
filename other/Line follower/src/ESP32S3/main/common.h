#ifndef COMMON_H
#define COMMON_H
#include <math.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/mcpwm_prelude.h"


#define S0 10  
#define S1 8   
#define S2 7   
#define SIG 18 
#define CALIBRATION -23 // Compensate for mechanical errors
extern int angle;
extern adc_oneshot_unit_handle_t adc2_handle;
extern adc_cali_handle_t adc2_cali_handle;
extern mcpwm_cmpr_handle_t comparator;
extern bool do_calibration2;

extern int adj;
extern int error;
uint32_t example_angle_to_compare(int angle);
int analogRead(int channel);
void adc_init(void);
void servo_init(void);
void uart_init(void);
void PID(void *arg);
int write(int channel);
long map(long x, long in_min, long in_max, long out_min, long out_max);
#endif // COMMON_H
