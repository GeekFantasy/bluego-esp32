#include "operations.h"
#include "esp_log.h"
#include "paj7620.h"
#include "esp_hidd_prf_api.h"
#include "hid_touch_gestures.h"
#include "hid_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define OPERATIONS_TAG "BG_OPER"
#define Delay(t) vTaskDelay(t / portTICK_PERIOD_MS)
#define ACTION_DURATION 100

operation_action operation_action_matrix[OPER_KEY_MAX_NUM] = {
    {"imu", 1},
    {"imu_gyro", 201},
    {"mfs", 1},
    {"mfs_up", 204},
    {"mfs_down", 203},
    {"mfs_left", 0},
    {"mfs_right", 0},
    {"mfs_middle", 202},
    {"ges", 0},
    {"ges_up", 101},
    {"ges_down", 102},
    {"ges_left", 103},
    {"ges_right", 104},
    {"ges_forward", 105},
    {"ges_clk", 106},
    {"ges_aclk", 107}};

uint8_t data_buff[] = {'i', 'm', 'u', ':', 0x00, 0x00, ',', 'i', 'm', 'u', '_', 'g', 'y', 'r', 'o', ':', 0x3, 0x3, ',', 'm', 'f', 's', ':', 0x02, 0x0};
int data_len = sizeof(data_buff);
uint8_t mouse_key_state = 0; // store the state of 3 mouse op_key

// 初始化NVS
esp_err_t nvs_init()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

// 写入NVS_Record
void write_oper_to_nvs(nvs_handle_t handle, operation_action record)
{
    if (nvs_set_u16(handle, record.op_key, record.action_code))
    {
        ESP_LOGE(OPERATIONS_TAG, "Failed to write op_key %s,action_code = %d", record.op_key, record.action_code);
    }
    else
    {
        ESP_LOGE(OPERATIONS_TAG, "Write op_key %s,action_code = %d", record.op_key, record.action_code);
    }

    nvs_commit(handle);
}

// 读取NVS_Record
void read_oper_from_nvs(nvs_handle_t handle, operation_action *record)
{
    if (record != NULL)
    {
        if (nvs_get_u16(handle, record->op_key, &(record->action_code)))
        {
            ESP_LOGE(OPERATIONS_TAG, "Failed to read op_key %s, original action_code = %d", record->op_key, record->action_code);
        }
        else
        {
            ESP_LOGI(OPERATIONS_TAG, "Read op_key %s, action_code = %d", record->op_key, record->action_code);
        }
    }
}

esp_err_t write_curr_mode_to_nvs(uint8_t curr_mode)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(OPER_STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        return err;
    }

    err = nvs_set_u8(handle, CURRENT_MODE_KEY, curr_mode);
    err = nvs_commit(handle);
    nvs_close(handle);

    return err;
}

esp_err_t read_curr_mode_from_nvs(uint8_t *curr_mode)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(OPER_STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        return err;
    }

    err = nvs_get_u8(handle, CURRENT_MODE_KEY, curr_mode);
    nvs_close(handle);

    return err;
}

// 把所有的operation写入 nvs
void write_all_operations_to_nvs()
{
    // 写入operations record
    nvs_handle_t handle;
    esp_err_t err = nvs_open(OPER_STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        return;
    }

    for (int i = 0; i < OPER_KEY_MAX_NUM; i++)
    {
        write_oper_to_nvs(handle, operation_action_matrix[i]);
    }

    nvs_close(handle);
}

// 读取所有NVS_Record
void read_all_operations()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(OPER_STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        return;
    }

    for (int i = 0; i < OPER_KEY_MAX_NUM; i++)
    {
        read_oper_from_nvs(handle, &operation_action_matrix[i]);
    }

    nvs_close(handle);
}

uint16_t get_action_code(int oper_key)
{
    if (oper_key >= 0 && oper_key < OPER_KEY_MAX_NUM)
    {
        return operation_action_matrix[oper_key].action_code;
    }

    return INVALID_OPER_CODE;
}

esp_err_t update_operation(operation_action *op)
{
    esp_err_t err = 1;
    for (int i = 0; i < OPER_KEY_MAX_NUM; i++)
    {
        if (strcmp(operation_action_matrix[i].op_key, op->op_key) == 0)
        {
            err = 0;
            operation_action_matrix[i].action_code = op->action_code;
            ESP_LOGE(OPERATIONS_TAG, "Updated op_key %s, action_code = %x", operation_action_matrix[i].op_key, operation_action_matrix[i].action_code);
            break;
        }
    }
    return err;
}

esp_err_t update_operations_tab(const uint8_t *data, int data_len)
{
    esp_err_t err = 0;
    operation_action op = {0};

    // As only parts of the settings from mobile will be sent. The other part of the settings need to be cleared 
    // to avoid mistakes.
    clear_operations_tab_action_code();

    for (int i = 0, j = 0; i < data_len; i++)
    {
        if (data[i] == ',')
            continue;

        if (data[i] != ':')
        {
            op.op_key[j] = data[i];
            j++;
            continue;
        }

        i++;
        op.op_key[j] = '\0';
        op.action_code = data[i] << 8 | data[i + 1];

        j = 0, i++;
        ESP_LOGE(OPERATIONS_TAG, "Parsed op_key %s, action_code = %x", op.op_key, op.action_code);
        if (update_operation(&op))
        {
            err = 1;
        }
    }

    return err;
}

void send_keyboard_key(uint16_t hid_conn_id, uint8_t op_key, oper_param op_param)
{
    uint8_t key_vaule = {0};
    if (op_param.key_state.pressed)
    {
        key_vaule = op_key;
    }
    esp_hidd_send_keyboard_value(hid_conn_id, 0, &key_vaule, 1);
}

void send_keyboard_key_with_mask(uint16_t hid_conn_id, uint8_t mask, uint8_t op_key, oper_param op_param)
{
    uint8_t key_vaule = {0};
    uint8_t key_mask = {0};
    if (op_param.key_state.pressed)
    {
        key_vaule = op_key;
        key_mask = mask;
    }
    esp_hidd_send_keyboard_value(hid_conn_id, key_mask, &key_vaule, 1);
}

#define SEND_KEY_CANCEL_IF_NECESSARY if(oper_type == OPER_TYPE_TRIGGER_ONLY) { Delay(ACTION_DURATION); op_param.key_state.pressed = false; send_keyboard_key(hid_conn_id, HID_KEY_RESERVED, op_param);}
#define SEND_CONSUMER_CANCEL_IF_NECESSARY if(oper_type == OPER_TYPE_TRIGGER_ONLY) {Delay(ACTION_DURATION); esp_hidd_send_consumer_value(hid_conn_id, 0, false);}

void send_operation_action(uint16_t hid_conn_id, uint16_t action_code, oper_param op_param, uint8_t oper_type)
{
    ESP_LOGI(OPERATIONS_TAG, "send op code: %d", action_code);

    switch (action_code)
    {
    case ACTION_CODE_MOUSE_POINTOR:
        esp_hidd_send_mouse_value(hid_conn_id, mouse_key_state, op_param.mouse.point_x,
                                  op_param.mouse.point_y, op_param.mouse.wheel);
        break;
    case ACTION_CODE_MOUSE_LEFT_CLICK:
        if (oper_type == OPER_TYPE_TRIGGER_CANCEL)
        {
            if (op_param.key_state.pressed)
            {
                mouse_key_state |= MOUSE_LEFT_KEY_SET_MASK;
            }
            else
            {
                mouse_key_state &= MOUSE_LEFT_KEY_CLEAR_MASK;
            }
            esp_hidd_send_mouse_value(hid_conn_id, mouse_key_state, 0, 0, 0);
        }
        break;
    case ACTION_CODE_MOUSE_RIGHT_CLICK:
        if (oper_type == OPER_TYPE_TRIGGER_CANCEL)
        {
            if (op_param.key_state.pressed)
            {
                mouse_key_state |= MOUSE_RIGHT_KEY_SET_MASK;
            }
            else
            {
                mouse_key_state &= MOUSE_RIGHT_KEY_CLEAR_MASK;
            }
            esp_hidd_send_mouse_value(hid_conn_id, mouse_key_state, 0, 0, 0);
        }

        break;
    case ACTION_CODE_MOUSE_MIDDLE_CLICK:
        if (oper_type == OPER_TYPE_TRIGGER_CANCEL)
        {
            if (op_param.key_state.pressed)
            {
                mouse_key_state |= MOUSE_MIDDLE_KEY_SET_MASK;
            }
            else
            {
                mouse_key_state &= MOUSE_MIDDLE_KEY_CLEAR_MASK;
            }
            esp_hidd_send_mouse_value(hid_conn_id, mouse_key_state, 0, 0, 0);
        }
        break;
    case ACTION_CODE_PHONE_SLIDE_UP:
        if (op_param.key_state.pressed)
            send_slide_up(hid_conn_id);
        break;
    case ACTION_CODE_PHONE_SLIDE_DOWN:
        if (op_param.key_state.pressed)
            send_slide_down(hid_conn_id);
        break;
    case ACTION_CODE_PHONE_SLIDE_LEFT:
        if (op_param.key_state.pressed)
            send_slide_left(hid_conn_id);
        break;
    case ACTION_CODE_PHONE_SLIDE_RIGHT:
        if (op_param.key_state.pressed)
            send_slide_right(hid_conn_id);
        break;
    case ACTION_CODE_PHONE_TAP:
        if (op_param.key_state.pressed)
            send_tap(hid_conn_id);
        break;
    case ACTION_CODE_PHONE_DOUBLE_TAP:
        if (op_param.key_state.pressed)
            send_double_tap(hid_conn_id);
        break;
    case ACTION_CODE_PHONE_BACK:
        if (op_param.key_state.pressed)
            send_back(hid_conn_id);
        break;
    case ACTION_CODE_KEYBOARD_KEY_UP:
        send_keyboard_key(hid_conn_id, HID_KEY_UP_ARROW, op_param);
        SEND_KEY_CANCEL_IF_NECESSARY
        break;
    case ACTION_CODE_KEYBOARD_KEY_DOWN:
        send_keyboard_key(hid_conn_id, HID_KEY_DOWN_ARROW, op_param);
        SEND_KEY_CANCEL_IF_NECESSARY
        break;
    case ACTION_CODE_KEYBOARD_KEY_LEFT:
        send_keyboard_key(hid_conn_id, HID_KEY_LEFT_ARROW, op_param);
        SEND_KEY_CANCEL_IF_NECESSARY
        break;
    case ACTION_CODE_KEYBOARD_KEY_RIGHT:
        send_keyboard_key(hid_conn_id, HID_KEY_RIGHT_ARROW, op_param);
        SEND_KEY_CANCEL_IF_NECESSARY
        break;
    case ACTION_CODE_KEYBOARD_KEY_SPACE:
        send_keyboard_key(hid_conn_id, HID_KEY_SPACEBAR, op_param);
        SEND_KEY_CANCEL_IF_NECESSARY
        break;
    case ACTION_CODE_KEYBOARD_KEY_ENTER:
        send_keyboard_key(hid_conn_id, HID_KEY_RETURN, op_param);
        SEND_KEY_CANCEL_IF_NECESSARY
        break;
    case ACTION_CODE_KEYBOARD_SWITCH_WINDOW:
        send_keyboard_key_with_mask(hid_conn_id, RIGHT_ALT_KEY_MASK, HID_KEY_TAB, op_param);
        SEND_KEY_CANCEL_IF_NECESSARY
        break;
    case ACTION_CODE_KEYBOARD_DESKTOP_NEXT:
        send_keyboard_key_with_mask(hid_conn_id, LEFT_GUI_KEY_MASK | LEFT_CONTROL_KEY_MASK, HID_KEY_RIGHT_ARROW, op_param);
        SEND_KEY_CANCEL_IF_NECESSARY
        break;
    case ACTION_CODE_KEYBOARD_DESKTOP_PREV:
        send_keyboard_key_with_mask(hid_conn_id, LEFT_GUI_KEY_MASK | LEFT_CONTROL_KEY_MASK, HID_KEY_LEFT_ARROW, op_param);
        SEND_KEY_CANCEL_IF_NECESSARY
        break;
    case ACTION_CODE_KEYBOARD_MINIMIZE_ALL:
        send_keyboard_key_with_mask(hid_conn_id, LEFT_GUI_KEY_MASK, HID_KEY_M, op_param);
        SEND_KEY_CANCEL_IF_NECESSARY
        break;
    case ACTION_CODE_CONSUMER_VOLUME_UP:
        esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, op_param.key_state.pressed);
        SEND_CONSUMER_CANCEL_IF_NECESSARY
        break;
    case ACTION_CODE_CONSUMER_VOLUME_DOWN:
        esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, op_param.key_state.pressed);
        SEND_CONSUMER_CANCEL_IF_NECESSARY
        break;
    case ACTION_CODE_CONSUMER_MUTE:
        esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_MUTE, op_param.key_state.pressed);
        SEND_CONSUMER_CANCEL_IF_NECESSARY
        break;
    case ACTION_CODE_CONSUMER_POWER:
        esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_POWER, op_param.key_state.pressed);
        SEND_CONSUMER_CANCEL_IF_NECESSARY
        break;
    case ACTION_CODE_CONSUMER_RESET:
        esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_RESET, op_param.key_state.pressed);
        SEND_CONSUMER_CANCEL_IF_NECESSARY
        break;
    case ACTION_CODE_CONSUMER_SLEEP:
        esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_SLEEP, op_param.key_state.pressed);
        SEND_CONSUMER_CANCEL_IF_NECESSARY
        break;
    default:
        break;
    }
}

// Check if stulus is enabled
uint16_t check_stylus_enableed()
{
    uint16_t enabled_flag = 0;

    if(get_action_code(OPER_KEY_MFS))
    {
        for (int i = OPER_KEY_MFS_UP; i <= OPER_KEY_MFS_MIDDLE; i++)
        {
            enabled_flag += ((operation_action_matrix[i].action_code >= ACTION_CODE_PHONE_SLIDE_UP) 
                && (operation_action_matrix[i].action_code <= ACTION_CODE_PHONE_BACK));
        }  
    }
    
    if(get_action_code(OPER_KEY_GES))
    {
        for (int i = OPER_KEY_GES_UP; i <= OPER_KEY_GES_ACLK; i++)
        {
            enabled_flag += ((operation_action_matrix[i].action_code >= ACTION_CODE_PHONE_SLIDE_UP) 
                && (operation_action_matrix[i].action_code <= ACTION_CODE_PHONE_BACK));
        }
    }

    return enabled_flag;
}

void clear_operations_tab_action_code()
{
    for (size_t i = 0; i < OPER_KEY_MAX_NUM; i++)
    {
        operation_action_matrix[i].action_code = 0;
    }
}