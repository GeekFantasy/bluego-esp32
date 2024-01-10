#ifndef OPERATIONS
#define OPERATIONS

#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

#define ACTION_CODE_RESTART_DEVICE          000

#define ACTION_CODE_SCREEN_SLIDE_UP          101
#define ACTION_CODE_SCREEN_SLIDE_DOWN        102
#define ACTION_CODE_SCREEN_SLIDE_LEFT        103
#define ACTION_CODE_SCREEN_SLIDE_RIGHT       104
#define ACTION_CODE_SCREEN_TAP               105
#define ACTION_CODE_SCREEN_DOUBLE_TAP        106
#define ACTION_CODE_SCREEN_BACKWARD          107

#define ACTION_CODE_MOUSE_POINTOR           201
#define ACTION_CODE_MOUSE_LEFT_CLICK        202
#define ACTION_CODE_MOUSE_RIGHT_CLICK       203
#define ACTION_CODE_MOUSE_MIDDLE_CLICK      204

#define ACTION_CODE_KEYBOARD_KEY_UP         301
#define ACTION_CODE_KEYBOARD_KEY_DOWN       302
#define ACTION_CODE_KEYBOARD_KEY_LEFT       303
#define ACTION_CODE_KEYBOARD_KEY_RIGHT      304
#define ACTION_CODE_KEYBOARD_KEY_SPACE      305
#define ACTION_CODE_KEYBOARD_KEY_ENTER      306
#define ACTION_CODE_KEYBOARD_SWITCH_WINDOW  307
#define ACTION_CODE_KEYBOARD_DESKTOP_NEXT   308
#define ACTION_CODE_KEYBOARD_DESKTOP_PREV   309
#define ACTION_CODE_KEYBOARD_MINIMIZE_ALL   310

#define ACTION_CODE_CONSUMER_VOLUME_UP      401
#define ACTION_CODE_CONSUMER_VOLUME_DOWN    402
#define ACTION_CODE_CONSUMER_MUTE           403
#define ACTION_CODE_CONSUMER_POWER          404
#define ACTION_CODE_CONSUMER_RESET          405
#define ACTION_CODE_CONSUMER_SLEEP          406

#define INVALID_OPER_CODE 0xFF

#define MOUSE_LEFT_KEY_SET_MASK         0X01;
#define MOUSE_LEFT_KEY_CLEAR_MASK       0XFE; 
#define MOUSE_RIGHT_KEY_SET_MASK        0X02;
#define MOUSE_RIGHT_KEY_CLEAR_MASK      0XFD;
#define MOUSE_MIDDLE_KEY_SET_MASK       0X04;
#define MOUSE_MIDDLE_KEY_CLEAR_MASK     0XFB;

enum
{
    OPER_KEY_IMU,
    OPER_KEY_IMU_GYRO,

    OPER_KEY_MFS,               // mfs hardware is remove from bluego v2.1
    OPER_KEY_MFS_UP,            // so this piece of code is kept but not used
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

    OPER_KEY_TKB,
    OPER_KEY_TKB_UP,
    OPER_KEY_TKB_DOWN,
    OPER_KEY_TKB_LEFT,
    OPER_KEY_TKB_RIGHT,
    OPER_KEY_TKB_TOUCH,

    OPER_KEY_MAX_NUM
};

enum
{
    OPER_TYPE_NONE,
    OPER_TYPE_TRIGGER_ONLY,  //operation with just a trigger and no cancellation followed, eg. gesture detector operations and gyro operation
    OPER_TYPE_TRIGGER_CANCEL // operation with a trigger and cancel opeation, eg. multiple fun switch, a pressing operation is followed by a cancel (or release) operation.
};

#define OPER_KEY_ESP_RESTART     0XFFFF

#define OPER_STORAGE_NAMESPACE "operations"
#define CURRENT_MODE_KEY "curr_mode"
#define MAX_OPER_NUM 16

typedef struct
{
    char op_key[15];
    uint16_t action_code;
} operation_action;

typedef union 
{
    struct {
        uint8_t point_x;
        uint8_t point_y;
        uint8_t wheel;
    }mouse;
    struct {
        uint8_t pressed;
    }key_state;
}oper_param;


// 定义一个数组，用于存储NVS_Record
extern operation_action operation_action_matrix[];
extern uint8_t data_buff[];
extern int data_len;

esp_err_t nvs_init();
void write_oper_to_nvs(nvs_handle_t handle, operation_action record);
void read_oper_from_nvs(nvs_handle_t handle, operation_action *record);
void write_all_operations_to_nvs();
void read_all_operations();
uint16_t get_action_code(int oper_key);
void send_operation_action(uint16_t hid_conn_id, uint16_t action_code, oper_param op_param, uint8_t oper_type);
esp_err_t write_mode_num_to_nvs(uint8_t curr_mode);
esp_err_t read_working_mode_num_from_nvs(uint8_t* curr_mode);
esp_err_t update_operations_tab(const uint8_t* data, int data_len);
void clear_operations_tab_action_code();
// Check if gesture is enabled
uint16_t check_stylus_enableed();
void write_mode_operations_to_nvs(uint8_t mode_num);
void read_mode_to_matrix(uint8_t mode_num);
void write_all_modes_to_nvs();
int  check_module_enabled(int oper_key);

#endif