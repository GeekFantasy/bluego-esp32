#include "operations.h"
#include "esp_log.h"
#include "paj7620.h"
#include "esp_hidd_prf_api.h"
#include "hid_touch_gestures.h"
#include "hid_dev.h"

#define OPERATIONS_TAG "BG_OPER"

operation device_operations[OPER_KEY_MAX_NUM] = {
    {"imu", 0},
    {"imu_gyro", 201},
    {"mfs", 1},
    {"mfs_up", 301},
    {"mfs_down", 302},
    {"mfs_left", 303},
    {"mfs_right", 304},
    {"mfs_middle", 305},
    {"ges", 0},
    {"ges_up", 101},
    {"ges_down", 102},
    {"ges_left", 103},
    {"ges_right", 104},
    {"ges_forward", 105},
    {"ges_clk", 106},
    {"ges_aclk", 107}};

uint8_t data_buff[] = {'i', 'm', 'u', ':', 0x00, 0x00, ',', 'i','m','u','_','g','y','r','o',':', 0x3, 0x3, ',','m','f','s',':',0x02, 0x0};
int data_len = sizeof(data_buff);
uint8_t mouse_key_state = 0; // store the state of 3 mouse key

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
void write_oper_to_nvs(nvs_handle_t handle, operation record)
{
    if(nvs_set_u16(handle, record.key, record.value))
    {
        ESP_LOGE(OPERATIONS_TAG, "Failed to write key %s,value = %d", record.key, record.value);
    }
    else
    {
        ESP_LOGE(OPERATIONS_TAG, "Write key %s,value = %d", record.key, record.value);
    }
    
    nvs_commit(handle);
}

// 读取NVS_Record
void read_oper_from_nvs(nvs_handle_t handle, operation *record)
{
    if (record != NULL)
    {
        if (nvs_get_u16(handle, record->key, &(record->value)))
        {
            ESP_LOGE(OPERATIONS_TAG, "Failed to read key %s, original value = %d", record->key, record->value);
        }
        else
        {
            ESP_LOGI(OPERATIONS_TAG, "Read key %s, value = %d", record->key, record->value);
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

esp_err_t read_curr_mode_from_nvs(uint8_t* curr_mode)
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
        write_oper_to_nvs(handle, device_operations[i]);
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
        read_oper_from_nvs(handle, &device_operations[i]);
    }

    nvs_close(handle);
}

uint16_t get_oper_code(int oper_key)
{
    if (oper_key >= 0 && oper_key < OPER_KEY_MAX_NUM)
    {
        return device_operations[oper_key].value;
    }

    return INVALID_OPER_CODE;
}

esp_err_t update_operation(operation* op)
{ 
    esp_err_t err = 1;
    for(int i = 0; i < OPER_KEY_MAX_NUM; i++)  
    {
        if(strcmp(device_operations[i].key, op->key)==0)
        {
            err = 0;
            device_operations[i].value = op->value;
            ESP_LOGE(OPERATIONS_TAG, "Updated key %s, value = %x", device_operations[i].key, device_operations[i].value);
            break;
        }
    }
    return err;
}

esp_err_t update_operations_tab(const uint8_t* data, int data_len)
{   
    esp_err_t err = 0;
    operation op = {0};

    for(int i = 0, j = 0; i < data_len; i++)
    {  
        if(data[i] == ',') 
            continue;

        if(data[i] != ':')
        {
            op.key[j] = data[i];
            j++;
            continue;
        }

        i++;
        op.key[j] = '\0';
        op.value = data[i] << 8 | data[i+1];

        j = 0, i++;
        ESP_LOGE(OPERATIONS_TAG, "Parsed key %s, value = %x", op.key, op.value);
        if(update_operation(&op))
        {
            err = 1;
        }
    }

    return err;
}

void send_keyboard_key(uint16_t hid_conn_id, uint8_t key, oper_param op_param)
{
    uint8_t key_vaule = {0};
    if (op_param.key_state.pressed)
    {
        key_vaule = key;
    }
    esp_hidd_send_keyboard_value(hid_conn_id, 0, &key_vaule, 1);
}


void send_operation(uint16_t hid_conn_id, uint16_t oper_code, oper_param op_param)
{
    ESP_LOGI(OPERATIONS_TAG, "send op code: %d", oper_code);
    switch (oper_code)
    {    
    case OP_CODE_MOUSE_POINTOR:
        esp_hidd_send_mouse_value(hid_conn_id, mouse_key_state, op_param.mouse.point_x, 
            op_param.mouse.point_y, op_param.mouse.wheel);
        break;
    case OP_CODE_MOUSE_LEFT_CLICK:
        if(op_param.key_state.pressed){
            mouse_key_state |= MOUSE_LEFT_KEY_SET_MASK;
        }
        else{
            mouse_key_state &= MOUSE_LEFT_KEY_CLEAR_MASK;
        }
        esp_hidd_send_mouse_value(hid_conn_id, mouse_key_state, 0, 0, 0);
        break;
    case OP_CODE_MOUSE_RIGHT_CLICK:
        if(op_param.key_state.pressed){
            mouse_key_state |= MOUSE_RIGHT_KEY_SET_MASK;
        }            
        else{
            mouse_key_state &= MOUSE_RIGHT_KEY_CLEAR_MASK;
        }
        esp_hidd_send_mouse_value(hid_conn_id, mouse_key_state, 0, 0, 0);
        break;
    case OP_CODE_MOUSE_MIDDLE_CLICK:
        if(op_param.key_state.pressed){
            mouse_key_state |= MOUSE_MIDDLE_KEY_SET_MASK;
        }            
        else{
            mouse_key_state &= MOUSE_MIDDLE_KEY_CLEAR_MASK;
        }          
        esp_hidd_send_mouse_value(hid_conn_id, mouse_key_state, 0, 0, 0);
        break;      
    case OP_CODE_PHONE_SLIDE_UP:
        if(op_param.key_state.pressed)
            send_slide_up(hid_conn_id);
        break;
    case OP_CODE_PHONE_SLIDE_DOWN:
        if(op_param.key_state.pressed)
            send_slide_down(hid_conn_id);
        break;
    case OP_CODE_PHONE_SLIDE_LEFT:
        if(op_param.key_state.pressed)
            send_slide_left(hid_conn_id);
        break;
    case OP_CODE_PHONE_SLIDE_RIGHT:
        if(op_param.key_state.pressed)
            send_slide_right(hid_conn_id);
        break;
    case OP_CODE_PHONE_TAP:
        if(op_param.key_state.pressed)
            send_tap(hid_conn_id);
        break;
    case OP_CODE_PHONE_DOUBLE_TAP:
        if(op_param.key_state.pressed)
            send_double_tap(hid_conn_id);
        break;
    case OP_CODE_PHONE_BACK:
        if(op_param.key_state.pressed)
            send_back(hid_conn_id);
        break;
    case OP_CODE_KEYBOARD_KEY_UP:
        send_keyboard_key(hid_conn_id, HID_KEY_UP_ARROW, op_param);
        break;
    case OP_CODE_KEYBOARD_KEY_DOWN:
        send_keyboard_key(hid_conn_id, HID_KEY_DOWN_ARROW, op_param);
        break;
    case OP_CODE_KEYBOARD_KEY_LEFT:
        send_keyboard_key(hid_conn_id, HID_KEY_LEFT_ARROW, op_param);
        break;
    case OP_CODE_KEYBOARD_KEY_RIGHT:
        send_keyboard_key(hid_conn_id, HID_KEY_RIGHT_ARROW, op_param);
        break;
    case OP_CODE_KEYBOARD_KEY_SPACE:
        send_keyboard_key(hid_conn_id, HID_KEY_SPACEBAR, op_param);
        break;
    case OP_CODE_KEYBOARD_KEY_ENTER:
        send_keyboard_key(hid_conn_id, HID_KEY_RETURN, op_param);
        break;        
    default:
        break;
    }
}