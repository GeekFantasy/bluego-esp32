#include "trackball.h"

#define TRACK_BALL_INFO_TAG         "TRACK_BALL"

// store the changing number of 4 directions
int tkb_move_up_steps = 0, tkb_move_down_steps = 0, tkb_move_left_steps = 0, tkb_move_right_steps = 0;

esp_err_t init_track_ball_led()
{
    esp_err_t err = 0;
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1ULL << LED_YELLOW_PIN) | (1ULL << LED_BLUE_PIN));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    err = gpio_config(&io_conf);

    if (err != ESP_OK)
    {
        ESP_LOGI(TRACK_BALL_INFO_TAG, "Failed to init_track_ball_led gpio config, error: %d.", err);
    }
    else
    {
        TRACK_BALL_TURN_OFF_LED(LED_BLUE_PIN);
        ESP_LOGI(TRACK_BALL_INFO_TAG, "Success to init_track_ball_led gpio config.");
    }
    return err;
}


static void track_ball_event_handler(void *arg)
{
    (*(int*)arg)++;
}


esp_err_t init_track_ball()
{
    esp_err_t err = 0;
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << TRACK_BALL_TOUCH_PIN) | (1ULL << TRACK_BALL_UP_PIN) | (1ULL << TRACK_BALL_DOWN_PIN | (1ULL << TRACK_BALL_LEFT_PIN) | (1ULL << TRACK_BALL_RIGHT_PIN));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;

    err = gpio_config(&io_conf);
    if (err != ESP_OK)
    {
        ESP_LOGI(TRACK_BALL_INFO_TAG, "Failed to init_track_ball gpio config, error: %d.", err);
    }
    else
    {
        ESP_LOGI(TRACK_BALL_INFO_TAG, "Success to init_track_ball gpio config");
    }
    // Ensure that this is only invoked once in the whole app
    // err = gpio_install_isr_service(0);
    // if (err != ESP_OK)
    // {
    //     ESP_LOGI(HID_DEMO_TAG, "Failed to install isr service, error: %d.", err);
    //     return err;
    // }

    // hook isr handler for specific gpio pin
    err = gpio_isr_handler_add(TRACK_BALL_UP_PIN, track_ball_event_handler, (void*)&tkb_move_up_steps);
    if (err != ESP_OK)
    {
        ESP_LOGI(TRACK_BALL_INFO_TAG, "Failed to add isr hanlder for track ball up, error: %d.", err);
        return err;
    }

    err = gpio_isr_handler_add(TRACK_BALL_DOWN_PIN, track_ball_event_handler, (void*)&tkb_move_down_steps);
    if (err != ESP_OK)
    {
        ESP_LOGI(TRACK_BALL_INFO_TAG, "Failed to add isr hanlder for track ball down, error: %d.", err);
        return err;
    }

    err = gpio_isr_handler_add(TRACK_BALL_LEFT_PIN, track_ball_event_handler, (void*)&tkb_move_left_steps);
    if (err != ESP_OK)
    {
        ESP_LOGI(TRACK_BALL_INFO_TAG, "Failed to add isr hanlder for track ball left, error: %d.", err);
        return err;
    }

    err = gpio_isr_handler_add(TRACK_BALL_RIGHT_PIN, track_ball_event_handler, (void*)&tkb_move_right_steps);
    if (err != ESP_OK)
    {
        ESP_LOGI(TRACK_BALL_INFO_TAG, "Failed to add isr hanlder for track ball right, error: %d.", err);
        return err;
    }

    err = init_track_ball_led();

    return err;
}

track_ball_movement get_track_ball_movement()
{
    track_ball_movement tkb_mv ={};
    uint16_t move_steps = 0;
   
    move_steps = tkb_move_up_steps / TRACK_BALL_MOVEMENT_DIVIDER;
    tkb_move_up_steps -= (move_steps * TRACK_BALL_MOVEMENT_DIVIDER);
    tkb_mv.up = move_steps;

    move_steps = tkb_move_down_steps / TRACK_BALL_MOVEMENT_DIVIDER;
    tkb_move_down_steps -= (move_steps * TRACK_BALL_MOVEMENT_DIVIDER);
    tkb_mv.down = move_steps;

    move_steps = tkb_move_left_steps / TRACK_BALL_MOVEMENT_DIVIDER;
    tkb_move_left_steps -= (move_steps * TRACK_BALL_MOVEMENT_DIVIDER);
    tkb_mv.left = move_steps;

    move_steps = tkb_move_right_steps / TRACK_BALL_MOVEMENT_DIVIDER;
    tkb_move_right_steps -= (move_steps * TRACK_BALL_MOVEMENT_DIVIDER);
    tkb_mv.right = move_steps;

    return tkb_mv;
}

int get_track_ball_touch_state()
{
    int touch_level = gpio_get_level(TRACK_BALL_TOUCH_PIN);
    return touch_level;
}
