#ifndef OPERATIONS
#define OPERATIONS

#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

#define OP_CODE_PHONE_SLIDE_UP 0x0101
#define OP_CODE_PHONE_SLIDE_DOWN 0x0102
#define OP_CODE_PHONE_SLIDE_LEFT 0x0103
#define OP_CODE_PHONE_SLIDE_RIGHT 0x0104
#define OP_CODE_PHONE_TAP 0x0105
#define OP_CODE_PHONE_DOUBLE_TAP 0x0106
#define OP_CODE_PHONE_BACK 0x0107

#define OP_CODE_MOUSE_POINTOR 0x0201
#define OP_CODE_MOUSE_LEFT_CLICK 0x0202
#define OP_CODE_MOUSE_RIGHT_CLICK 0x0203

#define OP_CODE_KEYBOARD_KEY_UP 0x0301
#define OP_CODE_KEYBOARD_KEY_DOWN 0x0302
#define OP_CODE_KEYBOARD_KEY_LEFT 0x0303
#define OP_CODE_KEYBOARD_KEY_RIGHT 0x0304
#define OP_CODE_KEYBOARD_KEY_SPACE 0x0305
#define OP_CODE_KEYBOARD_KEY_ENTER 0x0306

#define INVALID_OPER_CODE 0xFF

enum
{
    OPER_KEY_IMU,
    OPER_KEY_IMU_GYRO,

    OPER_KEY_MFS,
    OPER_KEY_MFS_UP,
    OPER_KEY_MFS_DOWN,
    OPER_KEY_MFS_LEFT,
    OPER_KEY_MFS_RIGHT,
    OPER_KEY_MFS_MIDDLE,

    OPER_KEY_GES,
    OPER_KEY_GES_UP,
    OPER_KEY_GES_DOWN,
    OPER_KEY_GES_LEFT,
    OPER_KEY_GES_RIGHT,
    OPER_KEY_GES_FORWOARD,
    OPER_KEY_GES_CLK,
    OPER_KEY_GES_ACLK
};

#define STORAGE_NAMESPACE "operations"
#define MAX_OPER_NUM 16

typedef struct
{
    char key[15];
    uint16_t value;
} operation;

// 定义一个数组，用于存储NVS_Record
extern operation device_operations[];

esp_err_t nvs_init();
void write_oper_to_nvs(nvs_handle_t handle, operation record);
void read_oper_from_nvs(nvs_handle_t handle, operation *record);
void read_all_operations();
uint16_t get_oper_code(int oper_key);
void send_operation(uint16_t hid_conn_id, uint16_t oper_code, uint8_t point_x, uint8_t point_y);

#endif