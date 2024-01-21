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
#include "trackball.h"
#include "function_btn.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "image_data.h"
#include "epaper_display.h"
#include "image_display.h"
#include "lvgl.h"
#include "driver/timer.h"

#define HID_DEMO_TAG "BLUEGO"
#define IMU_LOG_TAG "IMU DATA"
#define HIDD_DEVICE_NAME "Bluego"
#define Delay(t) vTaskDelay(t / portTICK_PERIOD_MS)

#define SWITCH_KEY_UP_LEVEL      135
#define SWITCH_KEY_DOWN_LEVEL    588
#define SWITCH_KEY_LEFT_LEVEL    780
#define SWITCH_KEY_RIGHT_LEVEL   429
#define SWITCH_KEY_MIDDLE_LEVEL  280
#define SWITCH_KEY_RANGE         45

#define MODE_MAX_NUM             5
#define HOLD_TIME_MS_TO_SLEEP    2 * 1000
#define ILDE_TIME_TO_POWER_OFF   5 * 60 * 1000          // If there is no operation for 5 minutes, it will power off

#define POWER_ADC_CHANNEL       ADC1_CHANNEL_7

const TickType_t time_delay_for_mfs = 50;
const TickType_t time_delay_for_ges = 200;
const TickType_t time_delay_for_tkb = 200;
const TickType_t time_delay_for_gyro = 10;
const TickType_t time_delay_when_idle = 500;
const TickType_t tick_delay_msg_send = 10;

static uint16_t hid_conn_id = 0;
static bool ble_connected = false;
int8_t curr_mode;
gesture_state generic_gs;
int in_mode_setting = 0;
spi_device_handle_t epd_spi;
TickType_t last_oper_time;

// Action processing queue
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
        ble_connected = false;
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
        ble_connected = true;
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

    if (paj7620_gesture_triggered())
    {
        error = paj7620_read_reg(0x43, 1, &data); // Read Bank_0_Reg_0x43/0x44 for gesture result.

        if (!error)
        {
            switch (data) // When different gestures be detected, the variable 'data' will be set to different values by paj7620_read_reg(0x43, 1, &data).
            {
            case GES_RIGHT_FLAG: // the gestures mapping need to change according to direction of the sensor mounted
                ges_key = OPER_KEY_GES_UP;
                break;
            case GES_LEFT_FLAG:
                ges_key = OPER_KEY_GES_DOWN;
                break;
            case GES_UP_FLAG:
                ges_key = OPER_KEY_GES_RIGHT;
                break;
            case GES_DOWN_FLAG:
                ges_key = OPER_KEY_GES_LEFT;
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

void init_power_voltage_adc()
{
    esp_err_t ret;
    ret = adc1_config_channel_atten(POWER_ADC_CHANNEL, ADC_ATTEN_DB_11);
    ret = adc1_config_width(ADC_WIDTH_BIT_12);
    if (ret)
    {
        ESP_LOGE(HID_DEMO_TAG, "Power ADC initialization failed. \n");
    }
    else
    {
        ESP_LOGI(HID_DEMO_TAG, "Power ADC initialized successfully\n");
    }
}

int get_track_ball_movement_key(track_ball_movement tkb_mv)
{
    int oper_key = 0;

    if(tkb_mv.up > 0)
    {
        if(tkb_mv.up >= tkb_mv.left && tkb_mv.up >= tkb_mv.right)
        {
            oper_key = OPER_KEY_TKB_UP;
        }
    }
    else if(tkb_mv.down > 0)
    {
        if(tkb_mv.down >= tkb_mv.left && tkb_mv.down >= tkb_mv.right)
        {
            oper_key = OPER_KEY_TKB_DOWN;
        }
    }

    if(tkb_mv.left > 0)
    {
        if(tkb_mv.left > tkb_mv.up && tkb_mv.left > tkb_mv.down)
        {
            oper_key = OPER_KEY_TKB_LEFT;
        }
    }
    else if(tkb_mv.right > 0)
    {
        if(tkb_mv.right > tkb_mv.up && tkb_mv.right > tkb_mv.down)
        {
            oper_key = OPER_KEY_TKB_RIGHT;
        }
    }
    
    return oper_key;
}

/// @brief Reset all gpio before go to deep sleep mode to save power
void reset_all_gpio()
{
    gpio_reset_pin(EPD_CS_PIN);
    gpio_reset_pin(EPD_RST_PIN);
    gpio_reset_pin(EPD_DC_PIN);
    gpio_reset_pin(EPD_BUSY_PIN);
    gpio_reset_pin(EPD_MOSI_PIN);
    gpio_reset_pin(EPD_CLK_PIN);

    gpio_reset_pin(LED_YELLOW_PIN);
    gpio_reset_pin(LED_BLUE_PIN);
    gpio_reset_pin(TRACK_BALL_TOUCH_PIN);
    gpio_reset_pin(TRACK_BALL_UP_PIN);
    gpio_reset_pin(TRACK_BALL_DOWN_PIN);
    gpio_reset_pin(TRACK_BALL_LEFT_PIN);
    gpio_reset_pin(TRACK_BALL_RIGHT_PIN);

    // F*** this! Reset pin hurted a lot, all the weird behavor after suspend is caused by this
    // F***! Comment out this cannot fix the issue completely, but sometimes it works well. Weird, really weird!
    //gpio_reset_pin(PAJ7620_I2C_SCL);             
    //gpio_reset_pin(PAJ7620_I2C_SDA);
    //gpio_reset_pin(PAJ7620_INTERRUPT_PIN);

    // Comment out this to avoid potential issue like paj7620.
    //gpio_reset_pin(MPU6500_I2C_SCL);
    //gpio_reset_pin(MPU6500_I2C_SDA);
}

void suspend_imu_and_ges_detector()
{
    esp_err_t err_code = ESP_OK;
    err_code = mpu6500_set_sleep(true); 
    if(ESP_OK != err_code)
    {
        ESP_LOGE(HID_DEMO_TAG, "Failed to set mpu6500 to sleeep mode before powering off. Error: %d", err_code);
    }
    
    // Able to suspend, but not working well after power on again.
    // Found that paj7620 sometimes works well, sometimes not after power off/on. 
    err_code = paj7620_suspend(); 
    if(ESP_OK != err_code)
    {
        ESP_LOGE(HID_DEMO_TAG, "Failed to suspend PAJ7620 before powering off.");
    }
}

int is_valid_operation(oper_message op_msg)
{
    int valid_op = 0;
    switch (op_msg.oper_key)
    {
    case OPER_KEY_IMU_GYRO:
        if(op_msg.oper_param.mouse.point_x > 2 || op_msg.oper_param.mouse.point_y > 2 || op_msg.oper_param.mouse.wheel > 0)  // Filter small movement
            valid_op = 1;
        break;
    case OPER_KEY_MFS_UP:
    case OPER_KEY_MFS_DOWN:
    case OPER_KEY_MFS_LEFT:
    case OPER_KEY_MFS_RIGHT:
    case OPER_KEY_MFS_MIDDLE:
    case OPER_KEY_GES_UP:
    case OPER_KEY_GES_DOWN:
    case OPER_KEY_GES_LEFT:
    case OPER_KEY_GES_RIGHT:
    case OPER_KEY_GES_FORWOARD:
    case OPER_KEY_GES_CLK:
    case OPER_KEY_GES_ACLK:
    case OPER_KEY_TKB_UP:
    case OPER_KEY_TKB_DOWN:
    case OPER_KEY_TKB_LEFT:
    case OPER_KEY_TKB_RIGHT:
    case OPER_KEY_TKB_TOUCH:
        if(op_msg.oper_param.key_state.pressed)
            valid_op = 1;
        break;    
    default:
        valid_op = 1;
        break;
    }
    
    return valid_op;
}

void track_ball_task(void *pvParameters)
{
    ESP_LOGI(HID_DEMO_TAG, "Entering track_ball_task task");

    uint16_t tkb_key;
    int tb_touch_state = 1, tb_touch_state_old = 1;
    oper_message op_msg;
    op_msg.oper_type = OPER_TYPE_TRIGGER_ONLY;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        if (ble_connected && check_module_enabled(OPER_KEY_TKB) && (!in_mode_setting))
        {
            tb_touch_state = get_track_ball_touch_state();
            if((TRACK_BALL_TOUCH_DOWN == tb_touch_state) && (TRACK_BALL_TOUCH_DOWN != tb_touch_state_old))
            {
                tkb_key = OPER_KEY_TKB_TOUCH;
            }
            else
            {
                tkb_key = get_track_ball_movement_key(get_track_ball_movement());
            }
            tb_touch_state_old = tb_touch_state;

            if(tkb_key != 0 && oper_queue != NULL)
            {
                op_msg.oper_key = tkb_key;
                op_msg.oper_param.key_state.pressed = 1;
                xQueueSend(oper_queue, &op_msg, tick_delay_msg_send / portTICK_PERIOD_MS);
                ESP_LOGI(HID_DEMO_TAG, "Message send with op_key: %d.", op_msg.oper_key);
            }

            vTaskDelayUntil(&last_wake_time, time_delay_for_tkb);
        }
        else
        {
            Delay(200);
        }        
    } 
}

/// @brief The task for deal with mode settings 
/// @param pvParameters 
void mode_setting_task(void *pvParameters)
{
    ESP_LOGI(HID_DEMO_TAG, "Entering mode_setting_task task");
    int func_btn_state = 1, func_btn_state_old = 1;
    int tb_touch_state = 1, tb_touch_state_old = 1;
    int display_updated = 0;
    int mv_direction = TRACK_BALL_DIRECTION_NONE;
    int mv_steps = 0;
    
    while (1)
    {
        if(in_mode_setting)
        {
            get_track_ball_main_movement(&mv_direction, &mv_steps);
            tb_touch_state = get_track_ball_touch_state();
            
            switch (mv_direction)
            {
            case TRACK_BALL_DIRECTION_UP:
            case TRACK_BALL_DIRECTION_LEFT:
                curr_mode++;
                curr_mode %= MODE_MAX_NUM;
                partial_display_work_mode(epd_spi, curr_mode);
                clear_track_ball_step_counters();
                display_updated = 1;
                /* code */
                break;
            case TRACK_BALL_DIRECTION_DOWN:
            case TRACK_BALL_DIRECTION_RIGHT:
                curr_mode--;
                curr_mode = (curr_mode + MODE_MAX_NUM) % (MODE_MAX_NUM);
                partial_display_work_mode(epd_spi, curr_mode);
                clear_track_ball_step_counters();
                display_updated = 1;
                /* code */
                break;
            default:
                break;
            }

            if(display_updated)    // No need to delay if e-paper is updated
            {
                display_updated = 0;
                last_oper_time = xTaskGetTickCount();
            }
            else
            {            
                Delay(200);
            }
        }
        else
        {
            Delay(200);
        }

        // Dealing with entering and exiting mode setting
        get_func_btn_state(&func_btn_state, NULL);
        if((func_btn_state != func_btn_state_old && func_btn_state == FUNC_BTN_PRESSED) 
            || (tb_touch_state != tb_touch_state_old && tb_touch_state == TRACK_BALL_TOUCH_DOWN ))
        {
            if(in_mode_setting)
            {
                ESP_LOGI(HID_DEMO_TAG, "Exiting mode setting ...");
                TRACK_BALL_TURN_OFF_LED(LED_BLUE_PIN);
                in_mode_setting = 0;
                epd_deep_sleep(epd_spi);
                clear_track_ball_step_counters();
                write_mode_num_to_nvs(curr_mode);
                read_mode_to_matrix(curr_mode);
            }
            else
            {
                ESP_LOGI(HID_DEMO_TAG, "Entering mode setting ...");
                TRACK_BALL_TURN_ON_LED(LED_BLUE_PIN);
                in_mode_setting = 1;
                epd_power_on_to_partial_display(epd_spi);
                clear_track_ball_step_counters();
            }

            last_oper_time = xTaskGetTickCount();
        }

        tb_touch_state_old = tb_touch_state;
        func_btn_state_old = func_btn_state;
    }
}

/// @brief Task for checking power voltage ADC
/// @param pvParameters
void power_measure_task(void *pvParameters)
{
    int average,  read_raw = 0, min, max;
    ESP_LOGI(HID_DEMO_TAG, "Entering power_measure_task  task");

    while(1)
    {
        average = 0;
        min = 65536;
        max = 0;

        for (size_t i = 0; i < 10; i++)
        {
            read_raw = adc1_get_raw(POWER_ADC_CHANNEL);
            if(read_raw < min) min = read_raw;
            if(read_raw > max) max = read_raw;
            average += read_raw;
            ESP_LOGI(HID_DEMO_TAG, "******Power voltage ADC %d: %d",i,  read_raw);
            Delay(6 * 1000);
        }
        
        ESP_LOGI(HID_DEMO_TAG, "******Power voltage ADC min:%d, max: %d, average: %d", min, max, average/10);
    }
}


/// @brief Task for checking the multiple function switch
/// @param pvParameters
///  mfs hardware is remove from bluego v2.1, so this piece of code is kept but not used
void multi_fun_switch_task(void *pvParameters)
{
    int read_raw;
    oper_message op_msg = {0};
    op_msg.oper_type = OPER_TYPE_TRIGGER_CANCEL;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    ESP_LOGI(HID_DEMO_TAG, "Entering switch detect task");

    while (1)
    {
        if (ble_connected && get_action_code(OPER_KEY_MFS) == 1 && oper_queue != NULL)
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
                if(op_msg.oper_param.key_state.pressed == 0) // make sure only set once 
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

    int state_switch = 0,  tb_touch_state = 1, tb_touch_state_old = 1;

    while (1)
    {
        if (ble_connected && check_module_enabled(OPER_KEY_GES) && (!in_mode_setting))
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

        tb_touch_state = get_track_ball_touch_state();
        if((TRACK_BALL_TOUCH_DOWN == tb_touch_state) && (TRACK_BALL_TOUCH_DOWN != tb_touch_state_old))
        {
            // After suspend and wake up, sometimes paj7620 would be work well again.
            if(state_switch == 0)
            {
                ESP_LOGI(HID_DEMO_TAG, "paj7620_suspend");
                paj7620_suspend();
            }
            
            if(state_switch == 1)
            {
                ESP_LOGI(HID_DEMO_TAG, "paj7620_wake_up.");
                paj7620_wake_up();
            }
            state_switch++; 
            state_switch = state_switch % 2;
        }
        tb_touch_state_old = tb_touch_state;
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
        if (ble_connected && check_module_enabled(OPER_KEY_IMU)  && (!in_mode_setting))
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
                y = angle_diff.y / 0.02; // Every 0.02 degree movement are counted as 1 pixel movement on screen
                z = angle_diff.z / 0.02;
                x = angle_diff.x / 3.6;
                op_msg.oper_key = OPER_KEY_IMU_GYRO;
                op_msg.oper_param.mouse.point_x = -z; // gyro z axis is used as x on screen
                op_msg.oper_param.mouse.point_y = y; // gyro x axis is used as y on screen
                op_msg.oper_param.mouse.wheel = x; // gyro x axis is used as y on screen
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
    int func_btn_state = 1;
    TickType_t current_tick = 0;
    int state_last_time_ms = xTaskGetTickCount();

    while (1)
    {
        if (ble_connected && (!in_mode_setting))
        {
            if (xQueueReceive(oper_queue, &op_msg, tick_delay_msg_send / portTICK_PERIOD_MS))
            {
                ESP_LOGI(HID_DEMO_TAG, "msg op_key:%d", op_msg.oper_key);
                if(op_msg.oper_key != OPER_KEY_ESP_RESTART)
                {
                    action_code = get_action_code(op_msg.oper_key);
                    send_operation_action(hid_conn_id, action_code, op_msg.oper_param, op_msg.oper_type);
                    if(is_valid_operation(op_msg))
                        last_oper_time = xTaskGetTickCount(); 
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

        // Go to deep sleep (power off) mode
        get_func_btn_state(&func_btn_state, &state_last_time_ms);
        current_tick = xTaskGetTickCount();
        //ESP_LOGI(HID_DEMO_TAG, "******FUNC KEY STATE: %d and state last time: %d.", func_btn_state, state_last_time_ms);
        if((current_tick - last_oper_time > ILDE_TIME_TO_POWER_OFF) 
            || (FUNC_BTN_PRESSED == func_btn_state && state_last_time_ms >= HOLD_TIME_MS_TO_SLEEP))
        {
            // Show power off screen
            epd_power_on_to_partial_display(epd_spi);
            partial_display_work_mode(epd_spi, 5);
            Delay(1000);
            // Show white screen
            epd_init_full_display(epd_spi);
            epd_full_display_white(epd_spi);
            epd_deep_sleep(epd_spi);
            // Wait untill func btn is released          
            do
            {
                Delay(100);
                get_func_btn_state(&func_btn_state, &state_last_time_ms);
            } while(FUNC_BTN_PRESSED == func_btn_state );
            
            ESP_LOGI(HID_DEMO_TAG, "Will make ESP32 go into deep sleep mode.");
            // Go to deep sleep mode
            esp_sleep_enable_ext0_wakeup(FUNC_BTN_PIN, FUNC_BTN_PRESSED);
            rtc_gpio_pullup_en(FUNC_BTN_PIN);
            rtc_gpio_pulldown_dis(FUNC_BTN_PIN);
            suspend_imu_and_ges_detector(); 
            reset_all_gpio();
            esp_deep_sleep_start();

            // Energy saving test results:
            // Working without ble connection but broadcasting: ~50 ma
            // Working with ble connection: ~42 ma, air mouse sending： 62 ma
            // Sleeping with reset gpio and suspend sensors: 0.732 ma 
            // Sleeping Without reset gpio: 0.8 ma
            // Sleeping without reset gpio and suspend mpu & gesture: 4.38 ma, when gesture detected: 6 ma 
            // Burning code : 20 ma
        }
    }
}

#define EPD_HOR_RES         80
#define EPD_VER_RES         128
#define TIMER_DIVIDER         16  // 硬件定时器时钟分频器
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // 将定时器计数器值转换为秒
#define TIMER_INTERVAL0_SEC   (0.01) // 定时器间隔为10毫秒

static int timer_test_cnt = 0;
lv_indev_t * encoder_indev = NULL;

uint8_t flush_buff[1280] = {};

/*Flush the content of the internal buffer the specific area on the display
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_disp_flush_ready()' has to be called when finished.*/
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    ESP_LOGI(HID_DEMO_TAG, "***Display flush is called.***");
    ESP_LOGI(HID_DEMO_TAG, "Area: (%d, %d), (%d, %d)", area->x1, area->y1, area->x2, area->y2);

    for(int y = area->y1; y <= area->y2; y++) {
        for(int x = area->x1; x <= area->x2; x++) {
            uint8_t color = color_p->full; 
            bool pixel = (color == 1); 
            // Convert lvgl pixel to the buff of e paper
            if (pixel) {
                flush_buff[y * (EPD_HOR_RES / 8) + x / 8] |= (1 << (7 - x % 8));
            } else {
                flush_buff[y * (EPD_HOR_RES / 8) + x / 8] &= ~(1 << (7 - x % 8));
            }
            color_p++;
        }
    }
    epd_partial_display_full_image(epd_spi, flush_buff, sizeof(flush_buff));

    /*IMPORTANT!!!
     *Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
}

// 定时器回调函数
void IRAM_ATTR timer_group0_isr(void *para) {
    // 定时器的状态将在这里处理
    int timer_idx = (int) para;

    // 清除中断并获取中断状态
    TIMERG0.int_clr_timers.t0 = 1;

    timer_test_cnt++;
    lv_tick_inc(10);
    // 重新启用闹钟
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
}

void btn_event_cb(lv_event_t * e) {
    lv_obj_t * obj = lv_event_get_target(e);
    lv_event_code_t code = lv_event_get_code(e);
    int data = lv_event_get_user_data(e);
    
    if(code == LV_EVENT_CLICKED) {
        // 处理按钮点击事件
        ESP_LOGI(HID_DEMO_TAG, "*Button %d is clicked*.", data);
    }
}

void ui_demo()
 {
     lv_obj_t *label1, *label2, *label3 ;

     static lv_style_t my_style;
     lv_style_init(&my_style);

    // 设置样式的文本颜色
    lv_style_set_text_color(&my_style, lv_color_make(0x00, 0x00, 0x00));

    lv_obj_t * btn1 = lv_btn_create(lv_scr_act());
   // lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -40);
    //lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0); // 将标签居中

    lv_obj_t * btn2 = lv_btn_create(lv_scr_act());
   // lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t * btn3 = lv_btn_create(lv_scr_act());
   // lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn3, LV_ALIGN_CENTER, 0, 40);

    label1 = lv_label_create(btn1);
    lv_label_set_text(label1, "Button 1");
    lv_obj_add_style(label1,  &my_style, LV_PART_MAIN );
    lv_obj_center(label1);

     label2 = lv_label_create(btn2);
    lv_label_set_text(label2, "Button 2");
    lv_obj_add_style(label2,  &my_style, LV_PART_MAIN );
    lv_obj_center(label2);

     label3 = lv_label_create(btn3);
    lv_label_set_text(label3, "Button 3");
    lv_obj_add_style(label3,  &my_style, LV_PART_MAIN );
    lv_obj_center(label3);

    static lv_style_t style_btn;
    lv_style_init(&style_btn);
    // lv_style_set_bg_color(&style_btn, LV_COLOR_WHITE);
    // lv_style_set_border_color(&style_btn, LV_COLOR_BLACK);
    // lv_style_set_border_width(&style_btn,  2);
    // 设置背景颜色为白色
    lv_style_set_bg_opa(&style_btn, LV_OPA_COVER);
    lv_style_set_bg_color(&style_btn, lv_color_white());

    // 设置边框颜色为黑色和边框宽度
    lv_style_set_border_color(&style_btn, lv_color_black());
    lv_style_set_border_width(&style_btn, 2);

    lv_obj_add_style(btn1,  &style_btn, 0);
    lv_obj_add_style(btn2,  &style_btn, 0);
    lv_obj_add_style(btn3,  &style_btn, 0);

    lv_group_t * g = lv_group_create();
    lv_group_add_obj(g, btn1); // 将按钮添加到组
    lv_group_add_obj(g, btn2); // 将按钮添加到组
    lv_group_add_obj(g, btn3); // 将按钮添加到组

    lv_obj_add_event_cb(btn1, btn_event_cb, LV_EVENT_CLICKED, (void*)1);
    lv_obj_add_event_cb(btn2, btn_event_cb, LV_EVENT_CLICKED, (void*)2);
    lv_obj_add_event_cb(btn3, btn_event_cb, LV_EVENT_CLICKED, (void*)3);

    // 添加其他UI元素到组
    lv_indev_set_group(encoder_indev, g); // 将编码器和组关联
}

int16_t enc_get_new_moves()
{
    int16_t moves = 0;
    int mv_direction = TRACK_BALL_DIRECTION_NONE;
    int mv_steps = 0;
    get_track_ball_main_movement(&mv_direction, &mv_steps);

    if(mv_direction == TRACK_BALL_DIRECTION_UP || mv_direction == TRACK_BALL_DIRECTION_LEFT)
    {
        moves = -mv_steps;
    }
    else if(mv_direction == TRACK_BALL_DIRECTION_DOWN || mv_direction == TRACK_BALL_DIRECTION_RIGHT)
    {
        moves = mv_steps;
    }

    return moves;
}

int enc_pressed()
{
    return (get_track_ball_touch_state() == 0);
}

void encoder_read(lv_indev_drv_t * drv, lv_indev_data_t*data)
{
    data->enc_diff = enc_get_new_moves();

    if(enc_pressed()) data->state = LV_INDEV_STATE_PRESSED;
    else data->state = LV_INDEV_STATE_RELEASED;
    if(data->enc_diff != 0 || data->state == LV_INDEV_STATE_PRESSED )
        ESP_LOGI(HID_DEMO_TAG, "*Diff: %d, Press: %d *", data->enc_diff, data->state);
}

void lv_indev_init()
{
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);      /*Basic initialization*/
    indev_drv.type = LV_INDEV_TYPE_ENCODER;                 /*See below.*/
    indev_drv.read_cb = encoder_read;              /*See below.*/
    /*Register the driver in LVGL and save the created input device object*/
    encoder_indev = lv_indev_drv_register(&indev_drv);
}

void lv_disp_init()
{
    //Create a buffer for drawing
    static lv_disp_draw_buf_t draw_buf_dsc_3;
    static lv_color_t buf_3_1[EPD_HOR_RES * EPD_VER_RES];            /*A screen sized buffer*/
    static lv_color_t buf_3_2[EPD_HOR_RES * EPD_VER_RES];            /*Another screen sized buffer*/
    lv_disp_draw_buf_init(&draw_buf_dsc_3, buf_3_1, buf_3_2, EPD_HOR_RES * EPD_VER_RES);   /*Initialize the display buffer*/

    //Register the display in LVGL
    static lv_disp_drv_t disp_drv;                         /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = EPD_HOR_RES;
    disp_drv.ver_res = EPD_VER_RES;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = disp_flush;

    /*Set a display buffer*/
    disp_drv.draw_buf = &draw_buf_dsc_3;

    /*Required for Example 3)*/
    disp_drv.full_refresh = 1;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);

    // 定时器组0，定时器0的配置
    timer_config_t config = {
    .divider = TIMER_DIVIDER,
    .counter_dir = TIMER_COUNT_UP,
    .counter_en = TIMER_PAUSE,
    .alarm_en = TIMER_ALARM_EN,
    .auto_reload = true,
    };

    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // 定时器的中断服务和中断优先级配置
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr, (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    // 设置定时器间隔
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL0_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_start(TIMER_GROUP_0, TIMER_0);
}

void lv_task(void *pvParameters)
{
    while(1){
        lv_timer_handler();
        Delay(200);
        //ESP_LOGI(HID_DEMO_TAG, "*Timer CNT: %d *.", timer_test_cnt);
    }
}

void app_main(void)
{
    esp_err_t ret;

    // if(ESP_OK == nvs_flash_erase())
    // {
    //     ESP_LOGI(HID_DEMO_TAG, "NVS defualt partition is erased.");
    // }

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Check the current mode of the device.
    if(ESP_OK != read_working_mode_num_from_nvs(&curr_mode)) //if failed to get the current mode write the defualt operations to nvs
    {
        curr_mode = 0;
        write_mode_num_to_nvs(curr_mode);
        write_all_modes_to_nvs();
        ESP_LOGI(HID_DEMO_TAG, "Initialize the operations table to NVS for the first time.");
    }
    // Read the operation matrix to memory.
    read_mode_to_matrix(curr_mode);
    ESP_LOGI(HID_DEMO_TAG, "Initialized the mode matrix.");

    // init e-paper-display
    ret = edp_init_spi_device(&epd_spi);
    epd_power_on_to_partial_display(epd_spi);
    epd_partial_display_full_image(epd_spi, gImage_poweringon, EPD_DIS_ARRAY);
    
    // If the gesture is eneabled, use the report map with stylus and consumer control
    // Or use the one with mouse, keyborad and consumer control.
    if(check_stylus_enableed())
    {
        hidd_set_report_map(HIDD_REPORT_MAP_STYLUS_CC);
        ESP_LOGI(HID_DEMO_TAG, "Stylus is used in current mode.");
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
    if(esp_ble_gap_register_callback(gap_event_handler) != ESP_OK)
        ESP_LOGE(HID_DEMO_TAG, "esp_ble_gap_register_callback failed\n");

    if(esp_hidd_register_callbacks(hidd_event_callback)!= ESP_OK)
        ESP_LOGE(HID_DEMO_TAG, "esp_hidd_register_callbacks failed\n");;

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
 
    ESP_LOGI(HID_DEMO_TAG, "The cause of wakeup is %d", esp_sleep_get_wakeup_cause());
    
    // Initialize PAJ7620
    if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED)
    {
        if(init_paj7620_i2c() == ESP_OK)
            ESP_LOGI(HID_DEMO_TAG,"PAJ7620 I2C configed sucessfully!");
        if(init_paj7620_registers() != ESP_OK)
            ESP_LOGI(HID_DEMO_TAG,"PAJ7620 registers configed sucessfully!");
        init_paj7620_interrupt();
    }
    else
    {
        if(init_paj7620_i2c() == ESP_OK)
            ESP_LOGI(HID_DEMO_TAG,"PAJ7620 I2C configed sucessfully!");
        init_paj7620_interrupt();
        paj7620_wake_up();
        if(init_paj7620_registers() != ESP_OK)
            ESP_LOGI(HID_DEMO_TAG,"PAJ7620 registers configed sucessfully!");
    }    
    
    // init MPU6500
    mpu6500_init();
    mpu6500_who_am_i();  // check if initialised successfully

    // init power voltage adc
    init_power_voltage_adc();

    //Init led indicator and touch ball input
    init_track_ball();

    // Init function button
    init_function_btn();

    // Create queue for processing operations.
    oper_queue = xQueueCreate(10, sizeof(oper_message));

    // Record the last operation time which is used for power off
    last_oper_time = xTaskGetTickCount(); 

    //ESP_LOGI(HID_DEMO_TAG, "multi_fun_switch task initialed.");
    //xTaskCreate(&multi_fun_switch_task, "multi_fun_switch", 2048, NULL, 1, NULL);

    ESP_LOGI(HID_DEMO_TAG, "imu_gyro_check task initialised.");
    xTaskCreate(&imu_gyro_task, "imu_gyro_check", 2048, NULL, 1, NULL);

    ESP_LOGI(HID_DEMO_TAG, "track_ball_task task initiinitialisedaled.");
    xTaskCreate(&track_ball_task, "track_ball_task", 2048, NULL, 1, NULL);
    
    ESP_LOGI(HID_DEMO_TAG, "ges_check task initialised.");
    xTaskCreate(&gesture_detect_task, "ges_check", 2048, NULL, 1, NULL);

    ESP_LOGI(HID_DEMO_TAG, "mode_setting_task task initiinitialisedaled.");
    xTaskCreate(&mode_setting_task, "mode_setting_task", 4096, NULL, 1, NULL);

    ESP_LOGI(HID_DEMO_TAG, "power_measure_task task initialised.");
    xTaskCreate(&power_measure_task, "power_measure_task", 2048, NULL, 1, NULL);

    ESP_LOGI(HID_DEMO_TAG, "hid_task task initialised.");
    xTaskCreate(&hid_main_task, "hid_task", 2048 * 2, NULL, 5, NULL);

    lv_init();
    lv_disp_init();
    lv_indev_init();
    ui_demo();

    ESP_LOGI(HID_DEMO_TAG, "lv_task task initialised.");
    xTaskCreate(&lv_task, "lv_task", 2048 * 5, NULL, 5, NULL);

    // Show current working mode after initialization done.
    //partial_display_work_mode(epd_spi, curr_mode);
    //epd_deep_sleep(epd_spi);
}
