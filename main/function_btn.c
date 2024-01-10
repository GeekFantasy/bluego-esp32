#include "function_btn.h"

#define  FUNC_BTN_INFO_TAG   "FUNCTION_BUTTON_LOG"

static TickType_t state_change_time;
static int button_state = 1;

static void func_btn_event_handler(void *arg)
{
    state_change_time = xTaskGetTickCount();
    button_state = gpio_get_level(FUNC_BTN_PIN);
}

esp_err_t init_function_btn()
{
    esp_err_t err = 0;
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << FUNC_BTN_PIN;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;

    err = gpio_config(&io_conf);
    if (err != ESP_OK)
    {
        ESP_LOGI(FUNC_BTN_INFO_TAG, "Failed to function button gpio config, error: %d.", err);
    }
    else
    {
        ESP_LOGI(FUNC_BTN_INFO_TAG, "Success to function button gpio config");
    }

    // Ensure that this is only invoked once in the whole app
    // err = gpio_install_isr_service(0);
    // if (err != ESP_OK)
    // {
    //     ESP_LOGI(HID_DEMO_TAG, "Failed to install isr service, error: %d.", err);
    //     return err;
    // }

    // hook isr handler for specific gpio pin
    err = gpio_isr_handler_add(FUNC_BTN_PIN, func_btn_event_handler, NULL);
    if (err != ESP_OK)
    {
        ESP_LOGI(FUNC_BTN_INFO_TAG, "Failed to add isr hanlder for function button, error: %d.", err);
    }

    state_change_time = xTaskGetTickCount();

    return err;
}

void get_func_btn_state(int* state, int* state_last_time_ms)
{
    if(state != NULL)
        *state = button_state;

    if(state_last_time_ms != NULL)
        *state_last_time_ms = xTaskGetTickCount() - state_change_time;
}