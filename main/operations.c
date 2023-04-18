#include "operations.h"
#include "esp_log.h"
#include "paj7620.h"
#include "esp_hidd_prf_api.h"
#include "hid_touch_gestures.h"

#define OPERATIONS_TAG "BLUEGO_OPERATIONS"

operation device_operations[MAX_OPER_NUM] = {
    {"imu", 1},
    {"imu_gyro", 0x0201},
    {"mfs", 0},
    {"mfs_up", 0x0101},
    {"mfs_down", 0x0102},
    {"mfs_left", 0x0103},
    {"mfs_right", 0x0104},
    {"mfs_middle", 0x0105},
    {"ges", 1},
    {"ges_up", 0x0101},
    {"ges_down", 0x0102},
    {"ges_left", 0x0103},
    {"ges_right", 0x0104},
    {"ges_forward", 0x0105},
    {"ges_clk", 0x0106},
    {"ges_aclk", 0x0107}};

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
    nvs_set_u16(handle, record.key, record.value);
    ESP_LOGE(OPERATIONS_TAG, "write key %s,value = %d", record.key, record.value);
    nvs_commit(handle);
}

// 读取NVS_Record
void read_oper_from_nvs(nvs_handle_t handle, operation *record)
{
    if (record != NULL)
    {
        nvs_get_u16(handle, record->key, &(record->value));
        ESP_LOGE(OPERATIONS_TAG, "Read key %s, value = %d", record->key, record->value);
    }
}

// 读取所有NVS_Record
void read_all_operations()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        return;
    }

    for (int i = 0; i < MAX_OPER_NUM; i++)
    {
        read_oper_from_nvs(handle, &device_operations[i]);
    }

    nvs_close(handle);
}

uint16_t get_oper_code(int oper_key)
{
    if (oper_key >= 0 && oper_key < MAX_OPER_NUM)
    {
        return device_operations[oper_key].value;
    }

    return INVALID_OPER_CODE;
}

void send_operation(uint16_t hid_conn_id, uint16_t oper_code, uint8_t point_x, uint8_t point_y)
{
    //ESP_LOGI(OPERATIONS_TAG, "Send OP with oper_code %d", oper_code);
    switch (oper_code)
    {
    case OP_CODE_MOUSE_POINTOR:
        esp_hidd_send_mouse_value(hid_conn_id, 0, point_x, point_y);
        break;
    case OP_CODE_PHONE_SLIDE_UP:
        send_slide_up(hid_conn_id);
        break;
    case OP_CODE_PHONE_SLIDE_DOWN:
        send_slide_down(hid_conn_id);
        break;
    case OP_CODE_PHONE_SLIDE_LEFT:
        send_slide_left(hid_conn_id);
        break;
    case OP_CODE_PHONE_SLIDE_RIGHT:
        send_slide_right(hid_conn_id);
        break;
    case OP_CODE_PHONE_TAP:
        send_double_tap(hid_conn_id);
        break;
    case OP_CODE_PHONE_DOUBLE_TAP:
        send_back(hid_conn_id);
        break;
    default:
        send_tap(hid_conn_id);
        break;
    }
}