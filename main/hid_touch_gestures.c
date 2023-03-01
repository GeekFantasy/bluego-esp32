#include <stdio.h>
#include "hid_touch_gestures.h"
#include "hidd_le_prf_int.h"
#include "paj7620.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define delay(t) vTaskDelay(t / portTICK_PERIOD_MS)

void send_touch_gesture(uint16_t hid_conn_id, int gesture)
{
    switch (gesture)
    {
        case GES_UP_FLAG:
            send_slide_up(hid_conn_id);
            break;
        case GES_DOWN_FLAG:
            send_slide_down(hid_conn_id);
            break;
        case GES_LEFT_FLAG:
            send_slide_left(hid_conn_id);
            break;
        case GES_RIGHT_FLAG:
            send_slide_right(hid_conn_id);
            Break;
        case GES_CLOCKWISE_FLAG:
            send_double_tap(hid_conn_id);
            break;  
        case GES_COUNT_CLOCKWISE_FLAG:
            send_back(hid_conn_id);
            break;
        case GES_FORWARD_FLAG:
            send_tap(hid_conn_id);
            break;            
        default:
            break;
    }
}

void send_slide_up(uint16_t hid_conn_id)
{
    esp_hidd_send_touch_value(hid_conn_id, 1, 1, 1, 0, 90, 300, 1, 1);
    delay(TOUCH_DELAY);

    for (int j = 1; j <= 10; j++)
    {
        esp_hidd_send_touch_value(hid_conn_id, 1, 1, 1, 0, 90, 300 - 20 * j, 1, 1);
        delay(TOUCH_INTERVAL);
    }

    esp_hidd_send_touch_value(hid_conn_id, 0, 1, 1, 0, 90, 100, 0, 0);
    delay(TOUCH_DELAY);
}

void send_slide_down(uint16_t hid_conn_id)
{
    esp_hidd_send_touch_value(hid_conn_id, 1, 1, 1, 0, 90, 100, 1, 1);
    delay(TOUCH_DELAY);

    for (int j = 1; j <= 10; j++)
    {
        esp_hidd_send_touch_value(hid_conn_id, 1, 1, 1, 0, 90, 100 + 20 * j, 1, 1);
        delay(TOUCH_INTERVAL);
    }

    esp_hidd_send_touch_value(hid_conn_id, 0, 1, 1, 0, 90, 300, 0, 0);
    delay(TOUCH_DELAY);
}

void send_slide_left(uint16_t hid_conn_id)
{
    esp_hidd_send_touch_value(hid_conn_id, 1, 1, 1, 0, 150, 200, 1, 1);
    delay(TOUCH_DELAY);

    for (int j = 1; j <= 10; j++)
    {
        esp_hidd_send_touch_value(hid_conn_id, 1, 1, 1, 0, 150 - 10 * j, 200, 1, 1);
        delay(TOUCH_INTERVAL);
    }

    esp_hidd_send_touch_value(hid_conn_id, 0, 1, 1, 0, 50, 200, 0, 0);
    delay(TOUCH_DELAY);
}

void send_slide_right(uint16_t hid_conn_id)
{
    esp_hidd_send_touch_value(hid_conn_id, 1, 1, 1, 0, 50, 200, 1, 1);
    delay(TOUCH_DELAY);

    for (int j = 1; j <= 10; j++)
    {
        esp_hidd_send_touch_value(hid_conn_id, 1, 1, 1, 0, 50 + 10 * j, 200, 1, 1);
        delay(TOUCH_INTERVAL);
    }

    esp_hidd_send_touch_value(hid_conn_id, 0, 1, 1, 0, 150, 200, 0, 0);
    delay(TOUCH_DELAY);
}

// send back gesture 
void send_back(uint16_t hid_conn_id)
{
    esp_hidd_send_touch_value(hid_conn_id, 1, 1, 1, 0, 179, 200, 1, 1);
    delay(TOUCH_DELAY);

    for (int j = 1; j <= 10; j++)
    {
        esp_hidd_send_touch_value(hid_conn_id, 1, 1, 1, 0, 179 - 4 * j, 200, 1, 1);
        delay(TOUCH_INTERVAL);
    }

    esp_hidd_send_touch_value(hid_conn_id, 0, 1, 1, 0, 139, 200, 0, 0);
    delay(TOUCH_DELAY);
}

void send_tap(uint16_t hid_conn_id)
{
    esp_hidd_send_touch_value(hid_conn_id, 1, 1, 1, 0, 90, 200, 1, 1);
    delay(TAP_DELAY);
    esp_hidd_send_touch_value(hid_conn_id, 0, 1, 1, 0, 90, 200, 0, 0);
    delay(TAP_DELAY);
}

void send_press_down(uint16_t hid_conn_id)
{
    esp_hidd_send_touch_value(hid_conn_id, 1, 1, 1, 0, 90, 200, 1, 1);
    delay(TAP_DELAY);
}

void send_press_up(uint16_t hid_conn_id)
{
    esp_hidd_send_touch_value(hid_conn_id, 0, 1, 1, 0, 90, 200, 0, 0);
    delay(TAP_DELAY);
}

void send_double_tap(uint16_t hid_conn_id)
{
    esp_hidd_send_touch_value(hid_conn_id, 1, 1, 1, 0, 90, 200, 1, 1);
    delay(TAP_DELAY);

    esp_hidd_send_touch_value(hid_conn_id, 0, 1, 1, 0, 90, 200, 0, 0);
    delay(TAP_DELAY);

    esp_hidd_send_touch_value(hid_conn_id, 1, 1, 1, 0, 90, 200, 1, 1);
    delay(TOUCH_DELAY);

    esp_hidd_send_touch_value(hid_conn_id, 0, 1, 1, 0, 90, 200, 0, 0);
    delay(TOUCH_DELAY);
}

