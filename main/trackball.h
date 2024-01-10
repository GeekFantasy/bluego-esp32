#ifndef TRACKBALL
#define TRACKBALL

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include "esp_log.h"
#include "driver/gpio.h"

#define LED_YELLOW_PIN          2      // actually cannot be used in bluego V2.1
#define LED_BLUE_PIN            20
#define TRACK_BALL_TOUCH_PIN    15
#define TRACK_BALL_UP_PIN       9
#define TRACK_BALL_DOWN_PIN     4
#define TRACK_BALL_LEFT_PIN     10
#define TRACK_BALL_RIGHT_PIN    13

#define TRACK_BALL_DIRECTION_NONE    0
#define TRACK_BALL_DIRECTION_UP      1
#define TRACK_BALL_DIRECTION_DOWN    2
#define TRACK_BALL_DIRECTION_LEFT    3
#define TRACK_BALL_DIRECTION_RIGHT   4

#define TRACK_BALL_TOUCH_DOWN   0
#define TRACK_BALL_TOUCH_UP     0

#define TRACK_BALL_TURN_ON_LED(pin)          gpio_set_level(pin, 0)
#define TRACK_BALL_TURN_OFF_LED(pin)         gpio_set_level(pin, 1)

#define TRACK_BALL_MOVEMENT_DIVIDER 6   // used to divide the track ball moved steps, if larger than 1, the actual steps will be divided into small steps.



typedef struct 
{
    uint16_t up;
    uint16_t down;
    uint16_t left;
    uint16_t right;

} track_ball_movement;


esp_err_t init_track_ball_led();
esp_err_t init_track_ball();
track_ball_movement get_track_ball_movement();
int get_track_ball_touch_state();
void clear_track_ball_step_counters();
void get_track_ball_main_movement(int* direction, int* steps);

#endif // end of TRACKBALL