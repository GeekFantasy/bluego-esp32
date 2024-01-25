#ifndef MODE_SETTING_UI_H
#define MODE_SETTING_UI_H

#include "lvgl.h"
#include "esp_log.h"

void init_mode_setting_ui(lv_indev_t * enc_indev);
void create_setting_ui();
void ui_demo();

#endif // End of MODE_SETTING_UI_H