/* This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this software is
   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "hid_dev.h"
#include "paj7620.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "mpu6500.h"
#include "hid_touch_gestures.h"
#include "operations.h"

#define HID_DEMO_TAG "BLUEGO"
#define IMU_LOG_TAG "IMU DATA"
#define HIDD_DEVICE_NAME "Bluego"
#define delay(t) vTaskDelay(t / portTICK_PERIOD_MS)

#define SWITCH_KEY_UP_LEVEL      135
#define SWITCH_KEY_DOWN_LEVEL    588
#define SWITCH_KEY_LEFT_LEVEL    780
#define SWITCH_KEY_RIGHT_LEVEL   429
#define SWITCH_KEY_MIDDLE_LEVEL  280
#define SWITCH_KEY_RANGE         35

const TickType_t time_delay_for_mfs = 50;
const TickType_t time_delay_for_ges = 50;
const TickType_t time_delay_for_gyro = 10;
const TickType_t tick_delay_msg_send = 10;

static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
int isr = 0;
uint8_t curr_mode;
gesture_state generic_gs;

QueueHandle_t oper_queue = NULL;

typedef struct
{
    uint16_t oper_key;
    oper_param oper_param;
} oper_message;

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0x00,
    0xef,
    0x00,
    0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,   // HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x30,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch (event)
    {
    case ESP_HIDD_EVENT_REG_FINISH:
    {
        if (param->init_finish.state == ESP_HIDD_INIT_OK)
        {
            // esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
            esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
            esp_ble_gap_config_adv_data(&hidd_adv_data);
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_REG_FINISH");
        }
        break;
    }
    case ESP_BAT_EVENT_REG:
    {
        ESP_LOGI(HID_DEMO_TAG, "ESP_BAT_EVENT_REG");
        break;
    }
    case ESP_HIDD_EVENT_DEINIT_FINISH:
        ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_DEINIT_FINISH");
        break;
    case ESP_HIDD_EVENT_BLE_CONNECT:
    {
        ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
        hid_conn_id = param->connect.conn_id;
        break;
    }
    case ESP_HIDD_EVENT_BLE_DISCONNECT:
    {
        sec_conn = false;
        ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    }
    case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT:
    {
        ESP_LOGI(HID_DEMO_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
        ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->vendor_write.data, param->vendor_write.length);
        break;
    }
    case ESP_MODE_SETTING_UPDATED:
    {
        oper_message op_msg;
        op_msg.oper_key = OPER_KEY_ESP_RESTART;
        ESP_LOGI(HID_DEMO_TAG, "%s, ESP_MODE_SETTING_UPDATED", __func__); 
        xQueueSend(oper_queue, &op_msg, tick_delay_msg_send / portTICK_PERIOD_MS);
        break;
    }
    default:
        break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        ESP_LOGI(HID_DEMO_TAG, "GAP advertizing started...");
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        for (int i = 0; i < ESP_BD_ADDR_LEN; i++)
        {
            ESP_LOGD(HID_DEMO_TAG, "%x:", param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        ESP_LOGI(HID_DEMO_TAG, "GAP Security responded...");
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(HID_DEMO_TAG, "remote BD_ADDR: %08x%04x",
                 (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                 (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(HID_DEMO_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(HID_DEMO_TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
        if (!param->ble_security.auth_cmpl.success)
        {
            ESP_LOGE(HID_DEMO_TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

void readpaj7620()
{
    uint8_t data = 0, error;

    // ESP_LOGI(HID_DEMO_TAG, "Gesture Int value: %d", isr);

    if (isr)
    {
        error = paj7620ReadReg(0x43, 1, &data); // Read Bank_0_Reg_0x43/0x44 for gesture result.
        // ESP_LOGI(HID_DEMO_TAG, "READ Gesture, error：%d", error);

        isr = 0;

        if (!error)
        {
            switch (data) // When different gestures be detected, the variable 'data' will be set to different values by paj7620ReadReg(0x43, 1, &data).
            {
            case GES_RIGHT_FLAG:
                delay(GES_ENTRY_TIME);
                paj7620ReadReg(0x43, 1, &data);
                if (data == GES_FORWARD_FLAG)
                {
                    set_gesture(&generic_gs, GES_FORWARD_FLAG);
                    ESP_LOGI(HID_DEMO_TAG, "Forward");
                    delay(GES_QUIT_TIME);
                }
                else if (data == GES_BACKWARD_FLAG)
                {
                    set_gesture(&generic_gs, GES_BACKWARD_FLAG);
                    ESP_LOGI(HID_DEMO_TAG, "Backward");
                    delay(GES_QUIT_TIME);
                }
                else
                {
                    set_gesture(&generic_gs, GES_RIGHT_FLAG);
                    ESP_LOGI(HID_DEMO_TAG, "Right");
                }
                break;
            case GES_LEFT_FLAG:
                delay(GES_ENTRY_TIME);
                paj7620ReadReg(0x43, 1, &data);
                if (data == GES_FORWARD_FLAG)
                {
                    set_gesture(&generic_gs, GES_FORWARD_FLAG);
                    ESP_LOGI(HID_DEMO_TAG, "Forward");
                    delay(GES_QUIT_TIME);
                }
                else if (data == GES_BACKWARD_FLAG)
                {
                    set_gesture(&generic_gs, GES_BACKWARD_FLAG);
                    ESP_LOGI(HID_DEMO_TAG, "Backward");
                    delay(GES_QUIT_TIME);
                }
                else
                {
                    set_gesture(&generic_gs, GES_LEFT_FLAG);
                    ESP_LOGI(HID_DEMO_TAG, "Left");
                }
                break;
            case GES_UP_FLAG:
                delay(GES_ENTRY_TIME);
                paj7620ReadReg(0x43, 1, &data);
                if (data == GES_FORWARD_FLAG)
                {
                    set_gesture(&generic_gs, GES_FORWARD_FLAG);
                    ESP_LOGI(HID_DEMO_TAG, "Forward");
                    delay(GES_QUIT_TIME);
                }
                else if (data == GES_BACKWARD_FLAG)
                {
                    set_gesture(&generic_gs, GES_BACKWARD_FLAG);
                    ESP_LOGI(HID_DEMO_TAG, "Backward");
                    delay(GES_QUIT_TIME);
                }
                else
                {
                    set_gesture(&generic_gs, GES_UP_FLAG);
                    ESP_LOGI(HID_DEMO_TAG, "Up");
                }
                break;
            case GES_DOWN_FLAG:
                delay(GES_ENTRY_TIME);
                paj7620ReadReg(0x43, 1, &data);
                if (data == GES_FORWARD_FLAG)
                {
                    set_gesture(&generic_gs, GES_FORWARD_FLAG);
                    ESP_LOGI(HID_DEMO_TAG, "Forward");
                    delay(GES_QUIT_TIME);
                }
                else if (data == GES_BACKWARD_FLAG)
                {
                    set_gesture(&generic_gs, GES_BACKWARD_FLAG);
                    ESP_LOGI(HID_DEMO_TAG, "Backward");
                    delay(GES_QUIT_TIME);
                }
                else
                {
                    set_gesture(&generic_gs, GES_DOWN_FLAG);
                    ESP_LOGI(HID_DEMO_TAG, "Down");
                }
                break;
            case GES_FORWARD_FLAG:
                set_gesture(&generic_gs, GES_FORWARD_FLAG);
                ESP_LOGI(HID_DEMO_TAG, "Forward");
                delay(GES_QUIT_TIME);
                break;
            case GES_BACKWARD_FLAG:
                set_gesture(&generic_gs, GES_BACKWARD_FLAG);
                ESP_LOGI(HID_DEMO_TAG, "Backward");
                delay(GES_QUIT_TIME);
                break;
            case GES_CLOCKWISE_FLAG:
                set_gesture(&generic_gs, GES_CLOCKWISE_FLAG);
                ESP_LOGI(HID_DEMO_TAG, "Clockwise");
                break;
            case GES_COUNT_CLOCKWISE_FLAG:
                set_gesture(&generic_gs, GES_COUNT_CLOCKWISE_FLAG);
                ESP_LOGI(HID_DEMO_TAG, "anti-clockwise");
                break;
            default:
                paj7620ReadReg(0x44, 1, &data);
                if (data == GES_WAVE_FLAG)
                {
                    set_gesture(&generic_gs, GES_WAVE_FLAG);
                    ESP_LOGI(HID_DEMO_TAG, "wave");
                }
                break;
            }
        }
    }

    delay(GES_REACTION_TIME);
}

int read_ges_from_paj7620()
{
    uint8_t data = 0, error;
    uint16_t ges_key = 0;

    if (isr)
    {
        error = paj7620ReadReg(0x43, 1, &data); // Read Bank_0_Reg_0x43/0x44 for gesture result.
        isr = 0;

        if (!error)
        {
            switch (data) // When different gestures be detected, the variable 'data' will be set to different values by paj7620ReadReg(0x43, 1, &data).
            {
            case GES_RIGHT_FLAG:
                ges_key = OPER_KEY_GES_RIGHT;
                break;
            case GES_LEFT_FLAG:
                ges_key = OPER_KEY_GES_LEFT;
                break;
            case GES_UP_FLAG:
                ges_key = OPER_KEY_GES_UP;
                break;
            case GES_DOWN_FLAG:
                ges_key = OPER_KEY_GES_DOWN;
                break;
            case GES_FORWARD_FLAG: // combine 2 operations as one
            case GES_BACKWARD_FLAG:
                ges_key = OPER_KEY_GES_FORWOARD;
                break;
            case GES_CLOCKWISE_FLAG:
                ges_key = OPER_KEY_GES_CLK;
                break;
            case GES_COUNT_CLOCKWISE_FLAG:
                ges_key = OPER_KEY_GES_ACLK;
                break;
            default:
                paj7620ReadReg(0x44, 1, &data);
                if (data == GES_WAVE_FLAG)
                {
                    // gesture = GES_WAVE_FLAG;
                    ges_key = 0;
                }
                break;
            }
        }
    }

    // delay(GES_REACTION_TIME);
    return ges_key;
}

static void paj7620_event_handler(void *arg)
{
    isr = 1;
    // ESP_LOGI(HID_DEMO_TAG, "Paj7620 interrup triggered.");
}

esp_err_t initPaj7620Interrupt()
{
    esp_err_t err = 0;
    gpio_config_t io_conf = {};
    // interrupt of failing edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    // set as input mode
    io_conf.mode = GPIO_MODE_DEF_INPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << INTERRUPT_PIN);
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // enable pull-up mode
    io_conf.pull_up_en = 1;

    err = gpio_config(&io_conf);
    if (err != ESP_OK)
    {
        ESP_LOGI(HID_DEMO_TAG, "Failed to gpio config, error: %d.", err);
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK)
    {
        ESP_LOGI(HID_DEMO_TAG, "Failed to install isr service, error: %d.", err);
        return err;
    }
    // hook isr handler for specific gpio pin
    err = gpio_isr_handler_add(INTERRUPT_PIN, paj7620_event_handler, NULL);
    if (err != ESP_OK)
    {
        ESP_LOGI(HID_DEMO_TAG, "Failed to add isr hanlder, error: %d.", err);
        return err;
    }

    return err;
}

void init_adc()
{
    esp_err_t ret;
    ret = adc2_config_channel_atten(ADC2_CHANNEL_7, ADC_ATTEN_DB_11);
    if (ret)
    {
        ESP_LOGI(HID_DEMO_TAG, "ADC intilese failed. \n");
    }
    else
    {
        ESP_LOGI(HID_DEMO_TAG, "ADC intilese successfully\n");
    }
}

/// @brief Task for checking the multiple function switch
/// @param pvParameters
void multi_fun_switch_task(void *pvParameters)
{
    int read_raw;
    oper_message op_msg = {0};
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        if (sec_conn && get_oper_code(OPER_KEY_MFS) == 1 && oper_queue != NULL)
        {
            adc2_get_raw(ADC2_CHANNEL_7, ADC_WIDTH_10Bit, &read_raw);
            //ESP_LOGE(HID_DEMO_TAG, "The data from adc is: %d", read_raw);

            // Handle key down event for key-up
            if (read_raw > (SWITCH_KEY_UP_LEVEL - SWITCH_KEY_RANGE) 
                && read_raw < (SWITCH_KEY_UP_LEVEL + SWITCH_KEY_RANGE))
            {
                //Make sure only send once for a key down event
                if(op_msg.oper_param.key_state.pressed == 1 && op_msg.oper_key == OPER_KEY_MFS_UP)
                {
                    vTaskDelayUntil(&xLastWakeTime, time_delay_for_mfs);
                    continue;
                }
                op_msg.oper_param.key_state.pressed = 1;
                op_msg.oper_key = OPER_KEY_MFS_UP;
            }
            else if (read_raw > (SWITCH_KEY_DOWN_LEVEL - SWITCH_KEY_RANGE) 
                && read_raw < (SWITCH_KEY_DOWN_LEVEL + SWITCH_KEY_RANGE))
            {
                if(op_msg.oper_param.key_state.pressed == 1 && op_msg.oper_key == OPER_KEY_MFS_DOWN)
                {
                    vTaskDelayUntil(&xLastWakeTime, time_delay_for_mfs);
                    continue;
                }
                op_msg.oper_param.key_state.pressed = 1;
                op_msg.oper_key = OPER_KEY_MFS_DOWN;
            }
            else if (read_raw > (SWITCH_KEY_LEFT_LEVEL - SWITCH_KEY_RANGE) 
                && read_raw < (SWITCH_KEY_LEFT_LEVEL + SWITCH_KEY_RANGE))
            {
                if(op_msg.oper_param.key_state.pressed == 1 && op_msg.oper_key == OPER_KEY_MFS_LEFT)
                {
                    vTaskDelayUntil(&xLastWakeTime, time_delay_for_mfs);
                    continue;
                }
                op_msg.oper_param.key_state.pressed = 1;
                op_msg.oper_key = OPER_KEY_MFS_LEFT;
            }
            else if (read_raw > (SWITCH_KEY_RIGHT_LEVEL - SWITCH_KEY_RANGE) 
                && read_raw < (SWITCH_KEY_RIGHT_LEVEL + SWITCH_KEY_RANGE))
            {
                if(op_msg.oper_param.key_state.pressed == 1 && op_msg.oper_key == OPER_KEY_MFS_RIGHT)
                {
                    vTaskDelayUntil(&xLastWakeTime, time_delay_for_mfs);
                    continue;
                }
                op_msg.oper_param.key_state.pressed = 1;
                op_msg.oper_key = OPER_KEY_MFS_RIGHT;
            }
            else if (read_raw > (SWITCH_KEY_MIDDLE_LEVEL - SWITCH_KEY_RANGE) 
                && read_raw < (SWITCH_KEY_MIDDLE_LEVEL + SWITCH_KEY_RANGE))
            {
                if(op_msg.oper_param.key_state.pressed == 1 && op_msg.oper_key == OPER_KEY_MFS_MIDDLE)
                {
                    vTaskDelayUntil(&xLastWakeTime, time_delay_for_mfs);
                    continue;
                }
                op_msg.oper_param.key_state.pressed = 1;
                op_msg.oper_key = OPER_KEY_MFS_MIDDLE;
            }
            else
            {   // handle key up event 
                if(op_msg.oper_param.key_state.pressed == 0) // make sure only set  once 
                {
                    vTaskDelayUntil(&xLastWakeTime, time_delay_for_mfs);
                    continue;
                }
                op_msg.oper_param.key_state.pressed = 0;
            }

            xQueueSend(oper_queue, &op_msg, tick_delay_msg_send / portTICK_PERIOD_MS);
        }
        else
        {
            delay(200);
        }
    }
}

/// @brief Task for checking the gesture detector pay7620
/// @param pvParameters
void gesture_detect_task(void *pvParameters)
{
    uint16_t ges_key;
    oper_message op_msg;
    TickType_t last_wake_time = xTaskGetTickCount();
    ESP_LOGI(HID_DEMO_TAG, "Entering gesture detect task");

    while (1)
    {
        if (sec_conn && get_oper_code(OPER_KEY_GES) == 1)
        {
            ges_key = read_ges_from_paj7620();

            if (ges_key != 0 && oper_queue != NULL)
            {
                op_msg.oper_key = ges_key;
                op_msg.oper_param.key_state.pressed = 1;
                xQueueSend(oper_queue, &op_msg, tick_delay_msg_send / portTICK_PERIOD_MS);
                ESP_LOGI(HID_DEMO_TAG, "Message send with key: %d.", op_msg.oper_key);
            }

            vTaskDelayUntil(&last_wake_time, time_delay_for_ges);
        }
        else
        {
            delay(200);
        }
    }
}

/// @brief Task for checking the gyro from the imu MPU6500
/// @param pvParameters
void imu_gyro_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    angle angle_diff = {0};
    gyro gyro;

    struct timeval tv_now;
    int64_t time_us_old = 0;
    int64_t time_us_now = 0;
    int64_t time_us_diff = 0;

    oper_message op_msg;
    ESP_LOGI(HID_DEMO_TAG, "Entering gyro detect task");

    while (1)
    {
        if (sec_conn && (get_oper_code(OPER_KEY_IMU) == 1))
        {
            mpu6500_GYR_read(&gyro);
            gettimeofday(&tv_now, NULL);
            time_us_now = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
            time_us_diff = time_us_now - time_us_old;
            time_us_old = time_us_now;
            if (time_us_diff > 100 * 1000) // if the time exceed 100ms, shorten it to 100ms to avoid big movement
            {
                time_us_diff = 100 * 1000;
            }

            angle_diff.x = (float)time_us_diff / 1000000 * gyro.x;
            angle_diff.y = (float)time_us_diff / 1000000 * gyro.y;
            angle_diff.z = (float)time_us_diff / 1000000 * gyro.z;

            // pay attention to the abs, it only apply to int, and the float will be convert to int when passing in
            // So the below means the angle change exceed 0.02 degree will be recognized mouse movement
            if ((abs(100 * angle_diff.x) >= 2) || (abs(100 * angle_diff.z) >= 2) || (abs(100 * angle_diff.y) >= 360))
            {
                int x, y, z;
                x = angle_diff.x / 0.02; // Every 0.02 degree movement are counted as 1 pixel movement on screen
                z = angle_diff.z / 0.02;
                y = angle_diff.y / 3.6;
                op_msg.oper_key = OPER_KEY_IMU_GYRO;
                op_msg.oper_param.mouse.point_x = -z; // gyro z axis is used as x on screen
                op_msg.oper_param.mouse.point_y = x; // gyro x axis is used as y on screen
                op_msg.oper_param.mouse.wheel = y; // gyro x axis is used as y on screen
                xQueueSend(oper_queue, &op_msg, tick_delay_msg_send / portTICK_PERIOD_MS);
                //ESP_LOGI(IMU_LOG_TAG, "M:%d,%d,%d,%lld", op_msg.oper_param.mouse.point_x, op_msg.oper_param.mouse.point_y,op_msg.oper_param.mouse.wheel, time_us_diff);
            }

            vTaskDelayUntil(&xLastWakeTime, time_delay_for_gyro);
        }
        else
        {
            delay(200);
        }
    }
}

void hid_main_task(void *pvParameters)
{
    delay(100);

    oper_message op_msg;
    uint16_t oper_code;

    while (1)
    {
        if (sec_conn)
        {
            // uint8_t key_vaule = {HID_KEY_A};
            // esp_hidd_send_keyboard_value(hid_conn_id, 0, &key_vaule, 1);
            // esp_hidd_send_consumer_value()
            // esp_hidd_send_mouse_value(hid_conn_id, 0, 0, 0, 1);
            // delay(200);

            if (xQueueReceive(oper_queue, &op_msg, tick_delay_msg_send / portTICK_PERIOD_MS))
            {
                ESP_LOGI(HID_DEMO_TAG, "msg key:%d", op_msg.oper_key);
                if(op_msg.oper_key != OPER_KEY_ESP_RESTART)
                {
                    oper_code = get_oper_code(op_msg.oper_key);
                    send_operation(hid_conn_id, oper_code, op_msg.oper_param);
                }
                else
                {
                    //delay(100);
                    //esp_restart();  // Restart is not necessary 
                }
            }
        }
        else
        {
            delay(200);
        }
    }
}

void app_main(void)
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(HID_DEMO_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(HID_DEMO_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    if ((ret = esp_hidd_profile_init()) != ESP_OK)
    {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
    }

    /// register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND; // bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;       // set the IO capability to No output No input
    uint8_t key_size = 16;                          // the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    // Initialize PAJ7620
    paj7620Init();
    // Initialize PAJ7620 interrupt
    initPaj7620Interrupt();

    // init MPU6500
    mpu6500_init();
    mpu6500_who_am_i();

    init_adc();

    if(read_curr_mode_from_nvs(&curr_mode)) //if failed to get the current mode write the defualt operations to nvs
    {
        curr_mode = 1;
        write_curr_mode_to_nvs(curr_mode);
        write_all_operations_to_nvs();
        ESP_LOGI(HID_DEMO_TAG, "Initialize the operations table to NVS for the first time.");
    }

    // 读取NVS_Record
    read_all_operations();

    // test string parse
    // if(update_operations_tab(data_buff, data_len))
    // {
    //     ESP_LOGI(HID_DEMO_TAG, "Error in updating the operation tables.");
    // }


    oper_queue = xQueueCreate(10, sizeof(oper_message));

    ESP_LOGI(HID_DEMO_TAG, "imu_gyro_check task initialed.");
    xTaskCreate(&imu_gyro_task, "imu_gyro_check", 2048, NULL, 1, NULL);

    ESP_LOGI(HID_DEMO_TAG, "multi_fun_switch task initialed.");
    xTaskCreate(&multi_fun_switch_task, "multi_fun_switch", 2048, NULL, 1, NULL);

    ESP_LOGI(HID_DEMO_TAG, "ges_check task initialed.");
    xTaskCreate(&gesture_detect_task, "ges_check", 2048, NULL, 1, NULL);

    xTaskCreate(&hid_main_task, "hid_task", 2048 * 2, NULL, 5, NULL);
}
