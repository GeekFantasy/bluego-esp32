// Copyright 2017-2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "esp_hidd_prf_api.h"
#include "hidd_le_prf_int.h"
#include "hid_dev.h"
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"

// HID keyboard input report length
#define HID_KEYBOARD_IN_RPT_LEN     8

// HID LED output report length
#define HID_LED_OUT_RPT_LEN         1

// HID mouse input report length
#define HID_MOUSE_IN_RPT_LEN        5

// HID stylus input report length
#define HID_STYLUS_IN_RPT_LEN       7

// HID touch screen report length
#define HID_TOUCH_SCREEN_IN_RPT_LEN        13

// HID consumer control input report length
#define HID_CC_IN_RPT_LEN           2

// HID service changed indication length
#define HID_SERVICE_CHANGED_IN_LEN           4

esp_err_t esp_hidd_register_callbacks(esp_hidd_event_cb_t callbacks) 
{
    esp_err_t hidd_status;

    if(callbacks != NULL) {
   	    hidd_le_env.hidd_cb = callbacks;
    } else {
        return ESP_FAIL;
    }

    if((hidd_status = hidd_register_cb()) != ESP_OK) {
        return hidd_status;
    }
    
    // Finally, this is the key making stylus works
    if(esp_ble_gatts_app_register(DEVICE_INFO_APP_ID))
    {
        ESP_LOGE(HID_LE_PRF_TAG, "APP with id %x register failed.", DEVICE_INFO_APP_ID);
    }

    if(esp_ble_gatts_app_register(MODE_APP_ID))
    {
        ESP_LOGE(HID_LE_PRF_TAG, "APP with id %x register failed.", MODE_APP_ID);
    }

    esp_ble_gatts_app_register(BATTRAY_APP_ID);

    if((hidd_status = esp_ble_gatts_app_register(HIDD_APP_ID)) != ESP_OK) {
        return hidd_status;
    }

    return hidd_status;
}

esp_err_t esp_hidd_profile_init(void)
{
     if (hidd_le_env.enabled) {
        ESP_LOGE(HID_LE_PRF_TAG, "HID device profile already initialized");
        return ESP_FAIL;
    }
    // Reset the hid device target environment
    memset(&hidd_le_env, 0, sizeof(hidd_le_env_t));
    hidd_le_env.enabled = true;
    return ESP_OK;
}

esp_err_t esp_hidd_profile_deinit(void)
{
    uint16_t hidd_svc_hdl = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_SVC];
    if (!hidd_le_env.enabled) {
        ESP_LOGE(HID_LE_PRF_TAG, "HID device profile already initialized");
        return ESP_OK;
    }

    if(hidd_svc_hdl != 0) {
	esp_ble_gatts_stop_service(hidd_svc_hdl);
	esp_ble_gatts_delete_service(hidd_svc_hdl);
    } else {
	return ESP_FAIL;
   }

    /* register the HID device profile to the BTA_GATTS module*/
    esp_ble_gatts_app_unregister(hidd_le_env.gatt_if);

    return ESP_OK;
}

uint16_t esp_hidd_get_version(void)
{
	return HIDD_VERSION;
}

// Send HID service changed indication
void esp_hidd_send_service_changed_value(uint16_t conn_id, uint8_t serv_version)
{
    uint8_t buffer[HID_SERVICE_CHANGED_IN_LEN] = {0};
    buffer[0] = serv_version;

    ESP_LOGI(HID_LE_PRF_TAG, "buffer[0] = %x", buffer[0]);
    hid_dev_send_indication(hidd_le_env.gatt_if, conn_id,
                        HID_SERVICE_CHANGED_IN, HID_REPORT_TYPE_INPUT, HID_SERVICE_CHANGED_IN_LEN, buffer);
    return;
}

void esp_hidd_send_consumer_value(uint16_t conn_id, uint8_t key_cmd, bool key_pressed)
{
    uint8_t buffer[HID_CC_IN_RPT_LEN] = {0, 0};
    if (key_pressed) {
        ESP_LOGI(HID_LE_PRF_TAG, "hid_consumer_build_report");
        hid_consumer_build_report(buffer, key_cmd);
    }
    ESP_LOGD(HID_LE_PRF_TAG, "buffer[0] = %x, buffer[1] = %x", buffer[0], buffer[1]);
    hid_dev_send_report(hidd_le_env.gatt_if, conn_id,
                        HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT, HID_CC_IN_RPT_LEN, buffer);
    return;
}

void esp_hidd_send_keyboard_value(uint16_t conn_id, key_mask_t special_key_mask, uint8_t *keyboard_cmd, uint8_t num_key)
{
    if (num_key > HID_KEYBOARD_IN_RPT_LEN - 2) {
        ESP_LOGE(HID_LE_PRF_TAG, "%s(), the number key should not be more than %d", __func__, HID_KEYBOARD_IN_RPT_LEN);
        return;
    }

    uint8_t buffer[HID_KEYBOARD_IN_RPT_LEN] = {0};

    buffer[0] = special_key_mask;

    for (int i = 0; i < num_key; i++) {
        buffer[i+2] = keyboard_cmd[i];
    }

    ESP_LOGD(HID_LE_PRF_TAG, "the key vaule = %d,%d,%d, %d, %d, %d,%d, %d", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
    hid_dev_send_report(hidd_le_env.gatt_if, conn_id,
                        HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT, HID_KEYBOARD_IN_RPT_LEN, buffer);
    return;
}

void esp_hidd_send_mouse_value(uint16_t conn_id, uint8_t mouse_button, int8_t mickeys_x, int8_t mickeys_y, int8_t wheel)
{
    ESP_LOGD(HID_LE_PRF_TAG, "Send Mouse,B:%x X:%d,Y:%d,W:%d",mouse_button, mickeys_x, mickeys_y, wheel);
    uint8_t buffer[HID_MOUSE_IN_RPT_LEN];

    buffer[0] = mouse_button;   // Buttons
    buffer[1] = mickeys_x;           // X
    buffer[2] = mickeys_y;           // Y
    buffer[3] = wheel;           // Wheel
    buffer[4] = 0;           // AC Pan

    hid_dev_send_report(hidd_le_env.gatt_if, conn_id,
                        HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT, HID_MOUSE_IN_RPT_LEN, buffer);
    return;
}

void esp_hidd_send_touch_value(uint16_t conn_id, uint8_t touch_down, uint8_t contact_count,  uint8_t contact_id, uint16_t scan_time, uint16_t touch_x, uint16_t touch_y, uint16_t touch_width, uint16_t touch_height)
{
    ESP_LOGD("HID_LE_PRF_TAG","ID:%d,x:%d,y:%d,count:%d,down:%d,time:%d", contact_id, touch_x, touch_y, contact_count,touch_down, scan_time);
    uint8_t buffer[HID_STYLUS_IN_RPT_LEN] = {0};

    buffer[0] = touch_down & 0x01;         // Buttons
    buffer[0] |= (touch_down << 1) & 0x02; // set in range
    buffer[1] = contact_id ;         // Contact Identifier
    buffer[2] = touch_x;            // X low byte
    buffer[3] = touch_x >> 8;       // X High byte
    buffer[4] = touch_y;            // y low byte
    buffer[5] = touch_y >> 8;       // y High byte
    buffer[6] = contact_count;       // Contact count

    hid_dev_send_report(hidd_le_env.gatt_if, conn_id,
                        HID_RPT_ID_STYLUS, HID_REPORT_TYPE_INPUT,HID_STYLUS_IN_RPT_LEN, buffer);
    return;
}