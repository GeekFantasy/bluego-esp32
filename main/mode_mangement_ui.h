#ifndef MODE_SETTING_UI_H
#define MODE_SETTING_UI_H

#include "lvgl.h"
#include "esp_log.h"

typedef void (*mode_switch_callback_t)(int mode_num);

void init_mode_management(lv_indev_t * enc_indev, mode_switch_callback_t md_sw_cb);
void update_volt_and_ble_status(uint32_t voltage, bool ble_status);
void create_setting_ui();
void ui_demo();
void mode_management_start(int mode_num);
void create_mode_switch_scr();

#endif // End of MODE_SETTING_UI_H