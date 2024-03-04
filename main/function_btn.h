#ifndef FUNC_BTN
#define FUNC_BTN

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define FUNC_BTN_PIN            12
#define FUNC_BTN_PRESSED        0
#define FUNC_BTN_RELEASED       1

esp_err_t init_function_btn();
void get_func_btn_state(int* state, int* state_last_time_ms);

#endif // end of FUNC_BTN