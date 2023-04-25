#ifndef OPERATIONS
#define OPERATIONS

#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

#define OP_CODE_RESTART_DEVICE      000

#define OP_CODE_PHONE_SLIDE_UP      101
#define OP_CODE_PHONE_SLIDE_DOWN    102
#define OP_CODE_PHONE_SLIDE_LEFT    103
#define OP_CODE_PHONE_SLIDE_RIGHT   104
#define OP_CODE_PHONE_TAP           105
#define OP_CODE_PHONE_DOUBLE_TAP    106
#define OP_CODE_PHONE_BACK          107

#define OP_CODE_MOUSE_POINTOR       201
#define OP_CODE_MOUSE_LEFT_CLICK    202
#define OP_CODE_MOUSE_RIGHT_CLICK   203

#define OP_CODE_KEYBOARD_KEY_UP     301
#define OP_CODE_KEYBOARD_KEY_DOWN   302
#define OP_CODE_KEYBOARD_KEY_LEFT   303
#define OP_CODE_KEYBOARD_KEY_RIGHT  304
#define OP_CODE_KEYBOARD_KEY_SPACE  305

#define OP_CODE_KEYBOARD_KEY_ENTER  306

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
    OPER_KEY_GES_ACLK,

    OPER_KEY_MAX_NUM
};

#define OPER_KEY_ESP_RESTART     0XFFFF

#define OPER_STORAGE_NAMESPACE "operations"
#define CURRENT_MODE_KEY "curr_mode"
#define MAX_OPER_NUM 16

typedef struct
{
    char key[15];
    uint16_t value;
} operation;

typedef union 
{
    struct {
        uint8_t point_x;
        uint8_t point_y;
        uint8_t wheel;
    }mouse_pointer;
    struct {
        uint8_t left_key_down;
        uint8_t right_key_down;
    }mouse_key;
}oper_param;

// 定义一个数组，用于存储NVS_Record
extern operation device_operations[];
extern uint8_t data_buff[];
extern int data_len;

esp_err_t nvs_init();
void write_oper_to_nvs(nvs_handle_t handle, operation record);
void read_oper_from_nvs(nvs_handle_t handle, operation *record);
void write_all_operations_to_nvs();
void read_all_operations();
uint16_t get_oper_code(int oper_key);
void send_operation(uint16_t hid_conn_id, uint16_t oper_code, oper_param op_parm);
esp_err_t write_curr_mode_to_nvs(uint8_t curr_mode);
esp_err_t read_curr_mode_from_nvs(uint8_t* curr_mode);
esp_err_t update_operations_tab(const uint8_t* data, int data_len);

#endif