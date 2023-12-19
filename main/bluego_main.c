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
#include "esp_system.h"
#include "driver/spi_master.h"

#define HID_DEMO_TAG "BLUEGO"
#define EPD_TAG      "E-Paper"
#define IMU_LOG_TAG "IMU DATA"
#define HIDD_DEVICE_NAME "Bluego"
#define Delay(t) vTaskDelay(t / portTICK_PERIOD_MS)

#define SWITCH_KEY_UP_LEVEL      135
#define SWITCH_KEY_DOWN_LEVEL    588
#define SWITCH_KEY_LEFT_LEVEL    780
#define SWITCH_KEY_RIGHT_LEVEL   429
#define SWITCH_KEY_MIDDLE_LEVEL  280
#define SWITCH_KEY_RANGE         45

const TickType_t time_delay_for_mfs = 50;
const TickType_t time_delay_for_ges = 100;
const TickType_t time_delay_for_gyro = 10;
const TickType_t time_delay_when_idle = 500;
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
    uint8_t oper_type;
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
    0x12,
    0x18,
    0x00,
    0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c1,   // ？
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

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(HID_DEMO_TAG, "ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT is triggered");
        printf("Updated Connection Parameters: \n");
        printf("\t Status: %d \n", param->update_conn_params.status);
        printf("\t Interval min: %d \n", param->update_conn_params.min_int); 
        printf("\t Interval max: %d \n", param->update_conn_params.max_int);
        printf("\t Latency: %d \n", param->update_conn_params.latency);
        printf("\t Timeout: %d \n", param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

int read_ges_from_paj7620()
{
    uint8_t data = 0, error;
    uint16_t ges_key = 0;

    if (isr)
    {
        error = paj7620_read_reg(0x43, 1, &data); // Read Bank_0_Reg_0x43/0x44 for gesture result.
        isr = 0;

        if (!error)
        {
            switch (data) // When different gestures be detected, the variable 'data' will be set to different values by paj7620_read_reg(0x43, 1, &data).
            {
            case GES_RIGHT_FLAG: // the gestures mapping need to change according to direction of the sensor mounted
                ges_key = OPER_KEY_GES_LEFT;
                break;
            case GES_LEFT_FLAG:
                ges_key = OPER_KEY_GES_RIGHT;
                break;
            case GES_UP_FLAG:
                ges_key = OPER_KEY_GES_DOWN;
                break;
            case GES_DOWN_FLAG:
                ges_key = OPER_KEY_GES_UP;
                break;
            case GES_FORWARD_FLAG: 
                ges_key = OPER_KEY_GES_FORWOARD;
                break;
            case GES_BACKWARD_FLAG:  
                ges_key = 0; // disable backward gesture as it always follows forward
                break;
            case GES_CLOCKWISE_FLAG:
                ges_key = OPER_KEY_GES_CLK;
                break;
            case GES_COUNT_CLOCKWISE_FLAG:
                ges_key = OPER_KEY_GES_ACLK;
                break;
            default:
                paj7620_read_reg(0x44, 1, &data);
                if (data == GES_WAVE_FLAG)
                {
                    // gesture = GES_WAVE_FLAG;
                    ges_key = 0;
                }
                break;
            }
        }
    }

    // Delay(GES_REACTION_TIME);
    return ges_key;
}

static void paj7620_event_handler(void *arg)
{
    isr = 1;
    // ESP_LOGI(HID_DEMO_TAG, "Paj7620 interrup triggered.");
}

esp_err_t init_paj7620_interrupt()
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
        ESP_LOGI(HID_DEMO_TAG, "ADC initialization failed. \n");
    }
    else
    {
        ESP_LOGI(HID_DEMO_TAG, "ADC initialized successfully\n");
    }
}

/// @brief Task for checking the multiple function switch
/// @param pvParameters
void multi_fun_switch_task(void *pvParameters)
{
    int read_raw;
    oper_message op_msg = {0};
    op_msg.oper_type = OPER_TYPE_TRIGGER_CANCEL;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    ESP_LOGI(HID_DEMO_TAG, "Entering switch detect task");

    while (1)
    {
        if (sec_conn && get_action_code(OPER_KEY_MFS) == 1 && oper_queue != NULL)
        {
            adc2_get_raw(ADC2_CHANNEL_7, ADC_WIDTH_10Bit, &read_raw);
            //ESP_LOGE(HID_DEMO_TAG, "The data from adc is: %d", read_raw);

            // Handle op_key down event for op_key-up
            if (read_raw > (SWITCH_KEY_UP_LEVEL - SWITCH_KEY_RANGE) 
                && read_raw < (SWITCH_KEY_UP_LEVEL + SWITCH_KEY_RANGE))
            {
                //Make sure only send once for a op_key down event
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
            {   // handle op_key up event 
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
            Delay(200);
        }
    }
}

/// @brief Task for checking the gesture detector pay7620
/// @param pvParameters
void gesture_detect_task(void *pvParameters)
{
    uint16_t ges_key;
    oper_message op_msg;
    op_msg.oper_type = OPER_TYPE_TRIGGER_ONLY;
    TickType_t last_wake_time = xTaskGetTickCount();
    ESP_LOGI(HID_DEMO_TAG, "Entering gesture detect task");

    while (1)
    {
        if (sec_conn && get_action_code(OPER_KEY_GES) == 1)
        {
            ges_key = read_ges_from_paj7620();

            if (ges_key != 0 && oper_queue != NULL)
            {
                op_msg.oper_key = ges_key;
                op_msg.oper_param.key_state.pressed = 1;
                xQueueSend(oper_queue, &op_msg, tick_delay_msg_send / portTICK_PERIOD_MS);
                ESP_LOGI(HID_DEMO_TAG, "Message send with op_key: %d.", op_msg.oper_key);
            }

            vTaskDelayUntil(&last_wake_time, time_delay_for_ges);
        }
        else
        {
            Delay(time_delay_when_idle);
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
    op_msg.oper_type = OPER_TYPE_TRIGGER_ONLY; 
    ESP_LOGI(HID_DEMO_TAG, "Entering gyro detect task");

    while (1)
    {
        if (sec_conn && (get_action_code(OPER_KEY_IMU) == 1))
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
            Delay(time_delay_when_idle);
        }
    }
}

void hid_main_task(void *pvParameters)
{
    Delay(100);

    oper_message op_msg;
    uint16_t action_code;

    while (1)
    {
        if (sec_conn)
        {
            // uint8_t key_vaule = {HID_KEY_A};
            // esp_hidd_send_keyboard_value(hid_conn_id, 0, &key_vaule, 1);
            // esp_hidd_send_consumer_value()
            // esp_hidd_send_mouse_value(hid_conn_id, 0, 0, 0, 1);
            // Delay(200);

            if (xQueueReceive(oper_queue, &op_msg, tick_delay_msg_send / portTICK_PERIOD_MS))
            {
                ESP_LOGI(HID_DEMO_TAG, "msg op_key:%d", op_msg.oper_key);
                if(op_msg.oper_key != OPER_KEY_ESP_RESTART)
                {
                    action_code = get_action_code(op_msg.oper_key);
                    send_operation_action(hid_conn_id, action_code, op_msg.oper_param, op_msg.oper_type);
                }
                else
                {
                    //Delay(100);
                    //esp_restart();  // Restart is not necessary 

                    // Commented out the service indication part, seems not work on Mi11.
                    // ESP_LOGI(HID_DEMO_TAG, "Send service changed indication");
                    // uint8_t serv_version = hidd_get_service_changed_version();
                    // hidd_set_service_changed_version(serv_version + 1);
                    // esp_hidd_send_service_changed_value(hid_conn_id, hidd_get_service_changed_version());
                }
            }
        }
        else
        {
            Delay(time_delay_when_idle);
        }
    }
}


#define EPD_WIDTH       80
#define EPD_HEIGHT      128
#define EPD_CS_PIN      15
#define EPD_RST_PIN     33
#define EPD_DC_PIN      12
#define EPD_BUSY_PIN    32

#define EPD_MOSI_PIN    13
#define EPD_MISo_PIN    -1
#define EPD_CLK_PIN     14

#define SET_PIN_HIGH(pin)   gpio_set_level(pin, 1)
#define SET_PIN_LOW(pin)    gpio_set_level(pin, 0)

/**
 * full screen update LUT
**/
const unsigned char lut_w1[] =
{
0x60	,0x5A	,0x5A	,0x00	,0x00	,0x01	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
 	
};	
const unsigned char lut_b1[] =
{
0x90	,0x5A	,0x5A	,0x00	,0x00	,0x01	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,


};
/**
 * partial screen update LUT
**/
const unsigned char lut_w[] =
{
0x60	,0x01	,0x01	,0x00	,0x00	,0x01	,
0x80	,0x0f	,0x00	,0x00	,0x00	,0x01	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,

};	
const unsigned char lut_b[] =
{
0x90	,0x01	,0x01	,0x00	,0x00	,0x01	,
0x40	,0x0f	,0x00	,0x00	,0x00	,0x01	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,

};

uint8_t edp_buff[1280] = {0xFF};


void epd_send_command(spi_device_handle_t spi, const uint8_t cmd)
{
    ESP_LOGI(EPD_TAG, "Entering epd_send_command() with cmd: 0x %hx", cmd);
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    
    int dc = (int)t.user;
    ESP_LOGD(EPD_TAG, "epd_send_command(), dc is : %x", dc);

    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
    ESP_LOGI(EPD_TAG, "Exiting epd_send_command().");
}

void epd_send_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1

    // int dc = (int)t.user;
    // ESP_LOGD(EPD_TAG, "epd_send_data(), dc is : %x", dc);

    ret = spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret == ESP_OK);            //Should have had no issues.
}

void epd_send_byte_data(spi_device_handle_t spi, const uint8_t data)
{
    epd_send_data(spi, &data, 1);
}

void epd_set_full_reg(spi_device_handle_t spi)
{
    ESP_LOGI(EPD_TAG, "Entering epd_set_full_reg().");

    epd_send_command(spi, 0x23);
    epd_send_data(spi, lut_w1, sizeof(lut_w1));
  
    epd_send_command(spi, 0x24);
    epd_send_data(spi, lut_w1, sizeof(lut_b1));

    ESP_LOGI(EPD_TAG, "Exiting epd_set_full_reg().");
}

void epd_reset(void)
{
    ESP_LOGI(EPD_TAG, "Entering epd_reset().");
    SET_PIN_LOW(EPD_RST_PIN);
    Delay(20);
    SET_PIN_HIGH(EPD_RST_PIN);
    Delay(20);
    ESP_LOGI(EPD_TAG, "Exiting epd_reset().");
}

void epd_wait_until_ilde(spi_device_handle_t spi)
{
    ESP_LOGI(EPD_TAG, "Entering epd_wait_until_ilde().");
    int busy;
    do{
        busy = gpio_get_level(EPD_BUSY_PIN);
        Delay(1);
    }while(!busy);
    Delay(10);
    ESP_LOGI(EPD_TAG, "Exiting epd_wait_until_ilde().");
}

void epd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(EPD_DC_PIN, dc);
}

void epd_init(spi_device_handle_t spi)
{
    ESP_LOGI(EPD_TAG, "Entering epd_init().");
    epd_reset();

    epd_send_command(spi, 0x00);
    epd_send_byte_data(spi, 0x5F);  // changed from 0x6F to 0x5F, and it's working now.

    epd_send_command(spi, 0x2A);
    epd_send_byte_data(spi, 0x00);
    epd_send_byte_data(spi, 0x00);

    epd_send_command(spi, 0x04);

    epd_wait_until_ilde(spi);
    ESP_LOGI(EPD_TAG, "Exiting epd_init().");
}

void epd_turn_on_display(spi_device_handle_t spi)
{
    ESP_LOGI(EPD_TAG, "Entering epd_turn_on_display().");
    epd_send_command(spi, 0x04); //power on
    epd_wait_until_ilde(spi);

    epd_send_command(spi, 0x12); //start refreshing the screen
    epd_wait_until_ilde(spi);

    epd_send_command(spi, 0x02); //power off
    epd_wait_until_ilde(spi);
    ESP_LOGI(EPD_TAG, "Exiting epd_turn_on_display().");
}

void epd_send_full_black(spi_device_handle_t spi)
{
    ESP_LOGI(EPD_TAG, "Entering epd_send_full_black().");

    int Width = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1);
    epd_send_command(spi, 0x10);
    for (size_t i = 0; i < EPD_HEIGHT; i++)
    {
        for (size_t j = 0; j < Width; j++)
        {
            epd_send_byte_data(spi, 0xFF);
        }
    }

    epd_send_command(spi, 0x13);
    for (size_t i = 0; i < EPD_HEIGHT; i++)
    {
        for (size_t j = 0; j < Width; j++)
        {
            epd_send_byte_data(spi, 0x0F);
        }
    }

    epd_turn_on_display(spi);

    ESP_LOGI(EPD_TAG, "Exiting epd_send_full_black().");
}

void epd_send_full_black_v2(spi_device_handle_t spi)
{
    ESP_LOGI(EPD_TAG, "Entering epd_send_full_black().");

    epd_send_command(spi, 0x10);
    epd_send_data(spi, edp_buff, sizeof(edp_buff));
    
    for (size_t i = 0; i < 1280; i++)
    {
       edp_buff[i] = 0x0F;
    }

    epd_send_command(spi, 0x13);
    epd_send_data(spi, edp_buff, sizeof(edp_buff));    

    epd_turn_on_display(spi);

    ESP_LOGI(EPD_TAG, "Exiting epd_send_full_black().");
}


uint8_t epd_get_byte(spi_device_handle_t spi, uint8_t cmd)
{
    ESP_LOGI(EPD_TAG, "Entering epd_get_byte().");
    epd_send_command(spi, cmd);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)1;
    t.rx_data[0] = 0, t.rx_data[1] = 0, t.rx_data[2] = 0, t.rx_data[3] = 0;

    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    assert( ret == ESP_OK );

    ESP_LOGI(EPD_TAG, "Exiting epd_get_byte().");

    return t.rx_data[0];
}

uint8_t epd_get_ic_status(spi_device_handle_t spi)
{
    uint8_t rx = 0;
    ESP_LOGI(EPD_TAG, "Entering epd_get_ic_status().");
    epd_send_command(spi, 0x71);

    rx = epd_get_byte(spi, 0x71);

    ESP_LOGI(EPD_TAG, "Exiting epd_get_ic_status().");

    return rx;
}


void init_e_paper_display()
{
    ESP_LOGI(EPD_TAG, "Entering init_e_paper_display().");

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = ((1ULL << EPD_CS_PIN) | (1ULL << EPD_RST_PIN) | (1ULL << EPD_DC_PIN));
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = 1ULL << EPD_BUSY_PIN;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);


    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num = -1,
        .mosi_io_num = EPD_MOSI_PIN,
        .sclk_io_num = EPD_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 128*80*2+8,
        .flags = SPICOMMON_BUSFLAG_MASTER ,
        .intr_flags = 0
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz = 4*1000*1000,           //Clock out at 4 MHz
        .mode = 0,                                //SPI mode 3, 0 and 3 seems both working well
        .spics_io_num = EPD_CS_PIN,              //CS pin
        .queue_size = 1,                          //We want to be able to queue 7 transactions at a time
        .flags = SPI_DEVICE_3WIRE ,
        .pre_cb = epd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    epd_init(spi);

    uint8_t rx = 0;
    rx = epd_get_byte(spi, 0x11);
    ESP_LOGI(EPD_TAG, "The data read from 0x11 before send data is: %x.", rx);

    //epd_turn_on_display(spi);
    //epd_send_full_black(spi);
    epd_send_full_black_v2(spi);

    rx = epd_get_byte(spi, 0x11);
    ESP_LOGI(EPD_TAG, "The data read from 0x11 after send data is: %x.", rx);

    Delay(6000);

    rx = epd_get_ic_status(spi);
    ESP_LOGI(EPD_TAG, "after send full balck, epd_get_ic_status() get data: %x.", rx);

    ESP_LOGI(EPD_TAG, "Exiting init_e_paper_display().");
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

    // Check the current mode of the device.
    if(read_curr_mode_from_nvs(&curr_mode)) //if failed to get the current mode write the defualt operations to nvs
    {
        curr_mode = 1;
        write_curr_mode_to_nvs(curr_mode);
        write_all_operations_to_nvs();
        ESP_LOGI(HID_DEMO_TAG, "Initialize the operations table to NVS for the first time.");
    }
    // Read the operation matrix to memory.
    read_all_operations();

    // If the gesture is eneabled, use the report map with stylus and consumer control
    // Or use the one with mouse, keyborad and consumer control.
    if(check_stylus_enableed())
    {
        hidd_set_report_map(HIDD_REPORT_MAP_STYLUS_CC);
        ESP_LOGI(HID_DEMO_TAG, "***Stylus is used***");
    }


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

    /* set the security iocap & auth_req & op_key size & init op_key response op_key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND; // bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;       // set the IO capability to No output No input
    uint8_t key_size = 16;                          // the op_key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of op_key of the master should distribute to you,
    and the response op_key means which op_key you can distribute to the Master;
    If your BLE device act as a master, the response op_key means you hope which types of op_key of the slave should distribute to you,
    and the init op_key means which op_key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    // Initialize PAJ7620
    paj7620_init();
    // Initialize PAJ7620 interrupt
    init_paj7620_interrupt();

    // init MPU6500
    mpu6500_init();
    mpu6500_who_am_i();

    // init 5-direcgtion-button
    init_adc();

    // init e-paper-display
    init_e_paper_display();

    // Create queue for processing operations.
    oper_queue = xQueueCreate(10, sizeof(oper_message));

    ESP_LOGI(HID_DEMO_TAG, "imu_gyro_check task initialed.");
    xTaskCreate(&imu_gyro_task, "imu_gyro_check", 2048, NULL, 1, NULL);

    ESP_LOGI(HID_DEMO_TAG, "multi_fun_switch task initialed.");
    xTaskCreate(&multi_fun_switch_task, "multi_fun_switch", 2048, NULL, 1, NULL);

    ESP_LOGI(HID_DEMO_TAG, "ges_check task initialed.");
    xTaskCreate(&gesture_detect_task, "ges_check", 2048, NULL, 1, NULL);

    xTaskCreate(&hid_main_task, "hid_task", 2048 * 2, NULL, 5, NULL);
}
