#ifndef OPERATIONS
#define OPERATIONS

#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

#define STORAGE_NAMESPACE "operations"
#define MAX_OPER_NUM 15

typedef struct
{
    char key[15];
    uint16_t value;
} Operation;

// 定义一个数组，用于存储NVS_Record
extern Operation device_operations[];

esp_err_t nvs_init();
void write_oper_to_nvs(nvs_handle_t handle, Operation record);
void read_oper_from_nvs(nvs_handle_t handle, Operation *record);
void read_all_operations();

#endif