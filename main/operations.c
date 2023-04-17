#include "operations.h"
#include "esp_log.h"

#define OPERATIONS_TAG "BLUEGO_OPERATIONS"

Operation device_operations[MAX_OPER_NUM] = {
    {"imu",         1},
    {"mfs",         1},
    {"mfs_up",      0x0101},
    {"mfs_down",    0x0102},
    {"mfs_left",    0x0103},
    {"mfs_right",   0x0104},
    {"mfs_middle",  0x0105},
    {"ges",         1},
    {"ges_up",      0x0301},
    {"ges_down",    0x0302},
    {"ges_left",    0x0303},
    {"ges_right",   0x0304},
    {"ges_forward", 0x0305},
    {"ges_clk",     0x0306},
    {"ges_aclk",    0x0307}};

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
void write_oper_to_nvs(nvs_handle_t handle, Operation record)
{
    nvs_set_u16(handle, record.key, record.value);
    ESP_LOGE(OPERATIONS_TAG, "write key %s,value = %d", record.key, record.value);
    nvs_commit(handle);
}

// 读取NVS_Record
void read_oper_from_nvs(nvs_handle_t handle, Operation *record)
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
