#ifndef OPERATIONS
#define OPERATIONS

#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

#define OPER_KEY_IMU          0

#define OPER_KEY_MFS          1
#define OPER_KEY_MFS_UP       2 
#define OPER_KEY_MFS_DOWN     3
#define OPER_KEY_MFS_LEFT     4
#define OPER_KEY_MFS_RIGHT    5
#define OPER_KEY_MFS_MIDDLE   6   

#define OPER_KEY_GES          7
#define OPER_KEY_GES_UP       8
#define OPER_KEY_GES_DOWN     9
#define OPER_KEY_GES_LEFT     10
#define OPER_KEY_GES_RIGHT    11
#define OPER_KEY_GES_FORWOARD 12
#define OPER_KEY_GES_CLK      13
#define OPER_KEY_GES_ACLK     14

#define INVALID_OPER_CODE     0xFF

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