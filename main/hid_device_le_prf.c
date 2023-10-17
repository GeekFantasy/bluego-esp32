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

#include "hidd_le_prf_int.h"
#include <string.h>
#include "esp_log.h"
#include "operations.h"

#define PREPARE_BUF_MAX_SIZE 1024

/// characteristic presentation information
struct prf_char_pres_fmt
{
    /// Unit (The Unit is a UUID)
    uint16_t unit;
    /// Description
    uint16_t description;
    /// Format
    uint8_t format;
    /// Exponent
    uint8_t exponent;
    /// Name space
    uint8_t name_space;
};

typedef struct
{
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

// HID report mapping table
static hid_report_map_t hid_rpt_map[HID_NUM_REPORTS];

// HID Report Map characteristic value for mouse , keyborad and consumer control
static const uint8_t hid_report_map_mouse_kb_CC[] = {
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x02, // Usage (Mouse)
    0xA1, 0x01, // Collection (Application)
    0x85, 0x01, // Report Id (1)
    0x09, 0x01, //   Usage (Pointer)
    0xA1, 0x00, //   Collection (Physical)
    0x05, 0x09, //     Usage Page (Buttons)
    0x19, 0x01, //     Usage Minimum (01) - Button 1
    0x29, 0x03, //     Usage Maximum (03) - Button 3
    0x15, 0x00, //     Logical Minimum (0)
    0x25, 0x01, //     Logical Maximum (1)
    0x75, 0x01, //     Report Size (1)
    0x95, 0x03, //     Report Count (3)
    0x81, 0x02, //     Input (Data, Variable, Absolute) - Button states
    0x75, 0x05, //     Report Size (5)
    0x95, 0x01, //     Report Count (1)
    0x81, 0x01, //     Input (Constant) - Padding or Reserved bits
    0x05, 0x01, //     Usage Page (Generic Desktop)
    0x09, 0x30, //     Usage (X)
    0x09, 0x31, //     Usage (Y)
    0x09, 0x38, //     Usage (Wheel)
    0x15, 0x81, //     Logical Minimum (-127)
    0x25, 0x7F, //     Logical Maximum (127)
    0x75, 0x08, //     Report Size (8)
    0x95, 0x03, //     Report Count (3)
    0x81, 0x06, //     Input (Data, Variable, Relative) - X & Y coordinate
    0xC0,       //   End Collection
    0xC0,       // End Collection

    0x05, 0x01, // Usage Pg (Generic Desktop)
    0x09, 0x06, // Usage (Keyboard)
    0xA1, 0x01, // Collection: (Application)
    0x85, 0x02, // Report Id (2)
    0x05, 0x07, //   Usage Pg (Key Codes)
    0x19, 0xE0, //   Usage Min (224)
    0x29, 0xE7, //   Usage Max (231)
    0x15, 0x00, //   Log Min (0)
    0x25, 0x01, //   Log Max (1)

    //   Modifier byte
    0x75, 0x01, //   Report Size (1)
    0x95, 0x08, //   Report Count (8)
    0x81, 0x02, //   Input: (Data, Variable, Absolute)

    //   Reserved byte
    0x95, 0x01, //   Report Count (1)
    0x75, 0x08, //   Report Size (8)
    0x81, 0x01, //   Input: (Constant)

    //   LED report
    0x95, 0x05, //   Report Count (5)
    0x75, 0x01, //   Report Size (1)
    0x05, 0x08, //   Usage Pg (LEDs)
    0x19, 0x01, //   Usage Min (1)
    0x29, 0x05, //   Usage Max (5)
    0x91, 0x02, //   Output: (Data, Variable, Absolute)

    //   LED report padding
    0x95, 0x01, //   Report Count (1)
    0x75, 0x03, //   Report Size (3)
    0x91, 0x01, //   Output: (Constant)

    //   Key arrays (6 bytes)
    0x95, 0x06, //   Report Count (6)
    0x75, 0x08, //   Report Size (8)
    0x15, 0x00, //   Log Min (0)
    0x25, 0x65, //   Log Max (101)
    0x05, 0x07, //   Usage Pg (Key Codes)
    0x19, 0x00, //   Usage Min (0)
    0x29, 0x65, //   Usage Max (101)
    0x81, 0x00, //   Input: (Data, Array)
    0xC0, // End Collection
    
    0x05, 0x0C, // Usage Pg (Consumer Devices)
    0x09, 0x01, // Usage (Consumer Control)
    0xA1, 0x01, // Collection (Application)
    0x85, 0x03, // Report Id (3)
    0x09, 0x02, //   Usage (Numeric Key Pad)
    0xA1, 0x02, //   Collection (Logical)
    0x05, 0x09, //     Usage Pg (Button)
    0x19, 0x01, //     Usage Min (Button 1)
    0x29, 0x0A, //     Usage Max (Button 10)
    0x15, 0x01, //     Logical Min (1)
    0x25, 0x0A, //     Logical Max (10)
    0x75, 0x04, //     Report Size (4)
    0x95, 0x01, //     Report Count (1)
    0x81, 0x00, //     Input (Data, Ary, Abs)
    0xC0,       //   End Collection
    0x05, 0x0C, //   Usage Pg (Consumer Devices)
    0x09, 0x86, //   Usage (Channel)
    0x15, 0xFF, //   Logical Min (-1)
    0x25, 0x01, //   Logical Max (1)
    0x75, 0x02, //   Report Size (2)
    0x95, 0x01, //   Report Count (1)
    0x81, 0x46, //   Input (Data, Var, Rel, Null)
    0x09, 0xE9, //   Usage (Volume Up)
    0x09, 0xEA, //   Usage (Volume Down)
    0x15, 0x00, //   Logical Min (0)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x02, //   Report Count (2)
    0x81, 0x02, //   Input (Data, Var, Abs)
    0x09, 0xE2, //   Usage (Mute)
    0x09, 0x30, //   Usage (Power)
    0x09, 0x83, //   Usage (Recall Last)
    0x09, 0x81, //   Usage (Assign Selection)
    0x09, 0xB0, //   Usage (Play)
    0x09, 0xB1, //   Usage (Pause)
    0x09, 0xB2, //   Usage (Record)
    0x09, 0xB3, //   Usage (Fast Forward)
    0x09, 0xB4, //   Usage (Rewind)
    0x09, 0xB5, //   Usage (Scan Next)
    0x09, 0xB6, //   Usage (Scan Prev)
    0x09, 0xB7, //   Usage (Stop)
    0x15, 0x01, //   Logical Min (1)
    0x25, 0x0C, //   Logical Max (12)
    0x75, 0x04, //   Report Size (4)
    0x95, 0x01, //   Report Count (1)
    0x81, 0x00, //   Input (Data, Ary, Abs)
    0x09, 0x80, //   Usage (Selection)
    0xA1, 0x02, //   Collection (Logical)
    0x05, 0x09, //     Usage Pg (Button)
    0x19, 0x01, //     Usage Min (Button 1)
    0x29, 0x03, //     Usage Max (Button 3)
    0x15, 0x01, //     Logical Min (1)
    0x25, 0x03, //     Logical Max (3)
    0x75, 0x02, //     Report Size (2)
    0x81, 0x00, //     Input (Data, Ary, Abs)
    0xC0,       //   End Collection
    0x81, 0x03, //   Input (Const, Var, Abs)
    0xC0,       // End Collectionq
};

// HID Report Map characteristic value for stylus and consumer control
static const uint8_t hid_report_map_CC_stylus[] = {      
    0x05, 0x0C, // Usage Pg (Consumer Devices)
    0x09, 0x01, // Usage (Consumer Control)
    0xA1, 0x01, // Collection (Application)
    0x85, 0x03, // Report Id (3)
    0x09, 0x02, //   Usage (Numeric Key Pad)
    0xA1, 0x02, //   Collection (Logical)
    0x05, 0x09, //     Usage Pg (Button)
    0x19, 0x01, //     Usage Min (Button 1)
    0x29, 0x0A, //     Usage Max (Button 10)
    0x15, 0x01, //     Logical Min (1)
    0x25, 0x0A, //     Logical Max (10)
    0x75, 0x04, //     Report Size (4)
    0x95, 0x01, //     Report Count (1)
    0x81, 0x00, //     Input (Data, Ary, Abs)
    0xC0,       //   End Collection
    0x05, 0x0C, //   Usage Pg (Consumer Devices)
    0x09, 0x86, //   Usage (Channel)
    0x15, 0xFF, //   Logical Min (-1)
    0x25, 0x01, //   Logical Max (1)
    0x75, 0x02, //   Report Size (2)
    0x95, 0x01, //   Report Count (1)
    0x81, 0x46, //   Input (Data, Var, Rel, Null)
    0x09, 0xE9, //   Usage (Volume Up)
    0x09, 0xEA, //   Usage (Volume Down)
    0x15, 0x00, //   Logical Min (0)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x02, //   Report Count (2)
    0x81, 0x02, //   Input (Data, Var, Abs)
    0x09, 0xE2, //   Usage (Mute)
    0x09, 0x30, //   Usage (Power)
    0x09, 0x83, //   Usage (Recall Last)
    0x09, 0x81, //   Usage (Assign Selection)
    0x09, 0xB0, //   Usage (Play)
    0x09, 0xB1, //   Usage (Pause)
    0x09, 0xB2, //   Usage (Record)
    0x09, 0xB3, //   Usage (Fast Forward)
    0x09, 0xB4, //   Usage (Rewind)
    0x09, 0xB5, //   Usage (Scan Next)
    0x09, 0xB6, //   Usage (Scan Prev)
    0x09, 0xB7, //   Usage (Stop)
    0x15, 0x01, //   Logical Min (1)
    0x25, 0x0C, //   Logical Max (12)
    0x75, 0x04, //   Report Size (4)
    0x95, 0x01, //   Report Count (1)
    0x81, 0x00, //   Input (Data, Ary, Abs)
    0x09, 0x80, //   Usage (Selection)
    0xA1, 0x02, //   Collection (Logical)
    0x05, 0x09, //     Usage Pg (Button)
    0x19, 0x01, //     Usage Min (Button 1)
    0x29, 0x03, //     Usage Max (Button 3)
    0x15, 0x01, //     Logical Min (1)
    0x25, 0x03, //     Logical Max (3)
    0x75, 0x02, //     Report Size (2)
    0x81, 0x00, //     Input (Data, Ary, Abs)
    0xC0,       //   End Collection
    0x81, 0x03, //   Input (Const, Var, Abs)
    0xC0,       // End Collectionq

    // The stylus combined with consumer device can work on most devices.
    0x05, 0x0D,        // Usage Page (Digitizer)
    0x09, 0x01,        // Usage (Digitizer)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x05,        //   Report ID (5)
    0x09, 0x20,        //   Usage (Stylus)
    0xA1, 0x02,        //   Collection (Logical)
    0x09, 0x42,        //     Usage (Tip Switch)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x75, 0x01,        //     Report Size (1)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x09, 0x32,        //     Usage (In Range)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x06,        //     Report Count (6)
    0x81, 0x03,        //     Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x75, 0x08,        //     Report Size (8)
    0x09, 0x51,        //     Usage (0x51)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x26, 0xFF, 0x0F,  //     Logical Maximum (4095)
    0x75, 0x10,        //     Report Size (16)
    0x55, 0x0E,        //     Unit Exponent (-2)
    0x65, 0x33,        //     Unit (System: English Linear, Length: Inch)
    0x09, 0x30,        //     Usage (X)
    0x35, 0x00,        //     Physical Minimum (0)
    0x46, 0xB5, 0x04,  //     Physical Maximum (1205)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x46, 0x8A, 0x03,  //     Physical Maximum (906)
    0x09, 0x31,        //     Usage (Y)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              //   End Collection
    0x05, 0x0D,        //   Usage Page (Digitizer)
    0x09, 0x54,        //   Usage (0x54)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              // End Collection
};


/// Battery Service Attributes Indexes
enum
{
    BAS_IDX_SVC,

    BAS_IDX_BATT_LVL_CHAR,
    BAS_IDX_BATT_LVL_VAL,
    BAS_IDX_BATT_LVL_NTF_CFG,
    BAS_IDX_BATT_LVL_PRES_FMT,

    BAS_IDX_NB,
};

/// Device Information Service Attributes Indexes
enum
{
    DEV_INFO_IDX_SVC,

    PNP_ID_IDX_PNP_ID_CHAR,
    PNP_ID_IDX_PNP_ID_VAL,

    DEV_INFO_IDX_NB,
};

// Mode setting service Attributes Indexes
enum
{
    MSS_IDX_SVC,
    MSS_IDX_CHAR_CURR_MODE,
    MSS_IDX_CHAR_VAL_CURR_MODE,
    MSS_IDX_CHAR_CFG_CURR_MODE,

    MSS_IDX_CHAR_MODE_SETTING,
    MSS_IDX_CHAR_VAL_MODE_SETTING,
    MSS_IDX_CHAR_CFG_MODE_SETTING,

    MSS_IDX_NB,
};

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a)&0xFF)
#define PROFILE_NUM 3
#define HID_PROFILE_APP_IDX 0
#define MODE_PROFILE_APP_IDX 1
#define DEVICE_INFO_PROFILE_APP_IDX 2

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
};

hidd_le_env_t hidd_le_env;

uint16_t mode_svc_hdl_tab[MSS_IDX_NB];
uint16_t device_info_svc_hdl_tab[DEV_INFO_IDX_NB];

// HID report map length
uint8_t hidProtocolMode = HID_PROTOCOL_MODE_REPORT;

// HID report mapping table

// HID Information characteristic value
static const uint8_t hidInfo[HID_INFORMATION_LEN] = {
    LO_UINT16(0x0101), HI_UINT16(0x0101), // bcdHID (USB HID version) // Original:0x0111
    0x00,                                 // bCountryCode
    HID_KBD_FLAGS                         // Flags
};

// HID External Report Reference Descriptor
static uint16_t hidExtReportRefDesc = ESP_GATT_UUID_BATTERY_LEVEL;

// HID Report Reference characteristic descriptor, mouse input
static uint8_t hidReportRefMouseIn[HID_REPORT_REF_LEN] =
    {HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT};


// HID Report Reference characteristic descriptor, key input
static uint8_t hidReportRefKeyIn[HID_REPORT_REF_LEN] =
    {HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT};

// HID Report Reference characteristic descriptor, LED output
static uint8_t hidReportRefLedOut[HID_REPORT_REF_LEN] =
    {HID_RPT_ID_LED_OUT, HID_REPORT_TYPE_OUTPUT};

#if (SUPPORT_REPORT_VENDOR == true)

static uint8_t hidReportRefVendorOut[HID_REPORT_REF_LEN] =
    {HID_RPT_ID_VENDOR_OUT, HID_REPORT_TYPE_OUTPUT};
#endif

// HID Report Reference characteristic descriptor, stylus input
static uint8_t hidReportRefStylusIn[HID_REPORT_REF_LEN] =
    {HID_RPT_ID_STYLUS, HID_REPORT_TYPE_INPUT};

// HID Report Reference characteristic descriptor, Feature
static uint8_t hidReportRefFeature[HID_REPORT_REF_LEN] =
    {HID_RPT_ID_FEATURE, HID_REPORT_TYPE_FEATURE};

// HID Report Reference characteristic descriptor, consumer control input
static uint8_t hidReportRefCCIn[HID_REPORT_REF_LEN] =
    {HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT};

// HID service changed descriptor, service changed indication
static uint8_t hidServiceChangedIn[HID_REPORT_REF_LEN] =
    {HID_SERVICE_CHANGED_IN, HID_REPORT_TYPE_INPUT};

/*
 *  Heart Rate PROFILE ATTRIBUTES
 ****************************************************************************************
 */

/// hid Service uuid
static uint16_t hid_le_svc = ATT_SVC_HID;
uint16_t hid_count = 0;
esp_gatts_incl_svc_desc_t incl_svc = {0};

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
/// the uuid definition
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t include_service_uuid = ESP_GATT_UUID_INCLUDE_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t character_service_changed_uuid= ESP_GATT_UUID_GATT_SRV_CHGD;
static const uint16_t hid_info_char_uuid = ESP_GATT_UUID_HID_INFORMATION;
static const uint16_t hid_report_map_uuid = ESP_GATT_UUID_HID_REPORT_MAP;
static const uint16_t hid_control_point_uuid = ESP_GATT_UUID_HID_CONTROL_POINT;
static const uint16_t hid_report_uuid = ESP_GATT_UUID_HID_REPORT;
static const uint16_t hid_proto_mode_uuid = ESP_GATT_UUID_HID_PROTO_MODE;
static const uint16_t hid_kb_input_uuid = ESP_GATT_UUID_HID_BT_KB_INPUT;
static const uint16_t hid_kb_output_uuid = ESP_GATT_UUID_HID_BT_KB_OUTPUT;
static const uint16_t hid_mouse_input_uuid = ESP_GATT_UUID_HID_BT_MOUSE_INPUT;
static const uint16_t hid_repot_map_ext_desc_uuid = ESP_GATT_UUID_EXT_RPT_REF_DESCR;
static const uint16_t hid_report_ref_descr_uuid = ESP_GATT_UUID_RPT_REF_DESCR;
static const uint16_t descriptor_service_changed_uuid = ESP_GATT_UUID_CHAR_PRESENT_FORMAT;

/// the propoty definition
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write_nr = ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_indicate = ESP_GATT_CHAR_PROP_BIT_INDICATE;
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;


/// battary Service
static const uint16_t battary_svc = ESP_GATT_UUID_BATTERY_SERVICE_SVC;

static const uint16_t bat_lev_uuid = ESP_GATT_UUID_BATTERY_LEVEL;
static const uint8_t bat_lev_ccc[2] = {0x00, 0x00};
static const uint16_t char_format_uuid = ESP_GATT_UUID_CHAR_PRESENT_FORMAT;

static uint8_t battary_lev = 80;
/// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t bas_att_db[BAS_IDX_NB] =
    {
        // Battary Service Declaration
        [BAS_IDX_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(battary_svc), (uint8_t *)&battary_svc}},

        // Battary level Characteristic Declaration
        [BAS_IDX_BATT_LVL_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

        // Battary level Characteristic Value
        [BAS_IDX_BATT_LVL_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&bat_lev_uuid, ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), &battary_lev}},

        // Battary level Characteristic - Client Characteristic Configuration Descriptor
        [BAS_IDX_BATT_LVL_NTF_CFG] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(bat_lev_ccc), (uint8_t *)bat_lev_ccc}},

        // Battary level report Characteristic Declaration
        [BAS_IDX_BATT_LVL_PRES_FMT] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_format_uuid, ESP_GATT_PERM_READ, sizeof(struct prf_char_pres_fmt), 0, NULL}},
};

/// Device infomation Service
static const uint16_t device_info_svc = ESP_GATT_UUID_DEVICE_INFO_SVC;
static const uint16_t pnp_id_uuid = ESP_GATT_UUID_PNP_ID;
static uint8_t pnp_id[] = {0x02, 0xAC, 0x05, 0x2C, 0x02, 0x1B, 0x01};

static const esp_gatts_attr_db_t pnp_id_att_db[DEV_INFO_IDX_NB] =
{
    // Device infomation Service Declaration
    [DEV_INFO_IDX_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(device_info_svc), (uint8_t *)&device_info_svc}},

    // PnP-ID Characteristic Declaration
    [PNP_ID_IDX_PNP_ID_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    // PnP-ID Characteristic Value
    [PNP_ID_IDX_PNP_ID_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&pnp_id_uuid, ESP_GATT_PERM_READ, sizeof(pnp_id), sizeof(pnp_id), (uint8_t *)pnp_id}},
};

/*Mode setting Service */
static const uint16_t GATTS_SERVICE_UUID = 0xEF00;
static const uint16_t GATTS_CHAR_UUID_CURR_MODE = 0xEF01;
static const uint16_t GATTS_CHAR_UUID_MODE_SETTING = 0xEF02;
static const uint8_t current_mode_ccc[2] = {0x00, 0x00};
static const uint8_t mode_setting_ccc[2] = {0x00, 0x00};
static const uint8_t curr_mode_char_value[4] = {0x11, 0x22, 0x33, 0x44};
static const uint8_t char_value_1[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};

static const esp_gatts_attr_db_t mode_setting_att_db[MSS_IDX_NB] =
    {
        // Service Declaration
        [MSS_IDX_SVC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID), (uint8_t *)&GATTS_SERVICE_UUID}},

        /* Characteristic Declaration */
        [MSS_IDX_CHAR_CURR_MODE] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

        /* Characteristic Value */
        [MSS_IDX_CHAR_VAL_CURR_MODE] =
            {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_CURR_MODE, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HIDD_LE_CHAR_VAL_MAX_LEN, sizeof(curr_mode_char_value), (uint8_t *)curr_mode_char_value}},

        /* Client Characteristic Configuration Descriptor */
        [MSS_IDX_CHAR_CFG_CURR_MODE] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(current_mode_ccc), (uint8_t *)current_mode_ccc}},

        /* Characteristic Declaration */
        [MSS_IDX_CHAR_MODE_SETTING] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        /* Characteristic Value */
        [MSS_IDX_CHAR_VAL_MODE_SETTING] =
            {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_MODE_SETTING, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HIDD_LE_CHAR_VAL_MAX_LEN, sizeof(char_value_1), (uint8_t *)char_value_1}},

        /* Client Characteristic Configuration Descriptor */
        [MSS_IDX_CHAR_CFG_MODE_SETTING] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(mode_setting_ccc), (uint8_t *)mode_setting_ccc}},
};

/// Full Hid device Database Description - Used to add attributes into the database
static uint8_t service_changed_indication[4] = {0x00, 0x00, 0x00, 0x00};
static esp_gatts_attr_db_t hidd_le_gatt_db[HIDD_LE_IDX_NB] =
    {
        // HID Service Declaration
        [HIDD_LE_IDX_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ_ENCRYPTED, sizeof(uint16_t), sizeof(hid_le_svc), (uint8_t *)&hid_le_svc}},

        // HID Service Declaration
        [HIDD_LE_IDX_INCL_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&include_service_uuid, ESP_GATT_PERM_READ, sizeof(esp_gatts_incl_svc_desc_t), sizeof(esp_gatts_incl_svc_desc_t), (uint8_t *)&incl_svc}},

        // HID Information Characteristic Declaration
        [HIDD_LE_IDX_HID_INFO_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},
        // HID Information Characteristic Value
        [HIDD_LE_IDX_HID_INFO_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_info_char_uuid, ESP_GATT_PERM_READ, sizeof(hids_hid_info_t), sizeof(hidInfo), (uint8_t *)&hidInfo}},

        // HID Control Point Characteristic Declaration
        [HIDD_LE_IDX_HID_CTNL_PT_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write_nr}},
        // HID Control Point Characteristic Value
        [HIDD_LE_IDX_HID_CTNL_PT_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_control_point_uuid, ESP_GATT_PERM_WRITE, sizeof(uint8_t), 0, NULL}},

        // Protocol Mode Characteristic Declaration
        [HIDD_LE_IDX_PROTO_MODE_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},
        // Protocol Mode Characteristic Value
        [HIDD_LE_IDX_PROTO_MODE_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_proto_mode_uuid, (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint8_t), sizeof(hidProtocolMode), (uint8_t *)&hidProtocolMode}},

        // Report Characteristic Declaration for MOUSE IN
        [HIDD_LE_IDX_REPORT_MOUSE_IN_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
        [HIDD_LE_IDX_REPORT_MOUSE_IN_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid, ESP_GATT_PERM_READ, HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
        [HIDD_LE_IDX_REPORT_MOUSE_IN_CCC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint16_t), 0, NULL}},
        [HIDD_LE_IDX_REPORT_MOUSE_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ, sizeof(hidReportRefMouseIn), sizeof(hidReportRefMouseIn), hidReportRefMouseIn}},

        // Report Characteristic Declaration for KEY IN
        [HIDD_LE_IDX_REPORT_KEY_IN_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
        // Report Characteristic Value
        [HIDD_LE_IDX_REPORT_KEY_IN_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid, ESP_GATT_PERM_READ, HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
        // Report KEY INPUT Characteristic - Client Characteristic Configuration Descriptor
        [HIDD_LE_IDX_REPORT_KEY_IN_CCC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint16_t), 0, NULL}},
        // Report Characteristic - Report Reference Descriptor
        [HIDD_LE_IDX_REPORT_KEY_IN_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ, sizeof(hidReportRefKeyIn), sizeof(hidReportRefKeyIn), hidReportRefKeyIn}},

        // Report Characteristic Declaration for LED OUT
        [HIDD_LE_IDX_REPORT_LED_OUT_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},
        [HIDD_LE_IDX_REPORT_LED_OUT_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
        [HIDD_LE_IDX_REPORT_LED_OUT_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ, sizeof(hidReportRefLedOut), sizeof(hidReportRefLedOut), hidReportRefLedOut}},

#if (SUPPORT_REPORT_VENDOR == true)
        // Report Characteristic Declaration
        [HIDD_LE_IDX_REPORT_VENDOR_OUT_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
        [HIDD_LE_IDX_REPORT_VENDOR_OUT_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
        [HIDD_LE_IDX_REPORT_VENDOR_OUT_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ, sizeof(hidReportRefVendorOut), sizeof(hidReportRefVendorOut), hidReportRefVendorOut}},
#endif
        // Report Characteristic Declaration for consumer control
        [HIDD_LE_IDX_REPORT_CC_IN_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},
        // Report Characteristic Value
        [HIDD_LE_IDX_REPORT_CC_IN_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid, ESP_GATT_PERM_READ, HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
        // Report Characteristic - Client Characteristic Configuration Descriptor
        [HIDD_LE_IDX_REPORT_CC_IN_CCC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED), sizeof(uint16_t), 0, NULL}},
        // Report Characteristic - Report Reference Descriptor
        [HIDD_LE_IDX_REPORT_CC_IN_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ, sizeof(hidReportRefCCIn), sizeof(hidReportRefCCIn), hidReportRefCCIn}},

        // Report Characteristic Declaration for stylus IN
        [HIDD_LE_IDX_REPORT_STYLUS_IN_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},
        [HIDD_LE_IDX_REPORT_STYLUS_IN_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid, ESP_GATT_PERM_READ, HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
        [HIDD_LE_IDX_REPORT_STYLUS_IN_CCC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint16_t), 0, NULL}},
        [HIDD_LE_IDX_REPORT_STYLUS_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ, sizeof(hidReportRefStylusIn), sizeof(hidReportRefStylusIn), hidReportRefStylusIn}},

        // Report Characteristic Declaration for Feature
        [HIDD_LE_IDX_REPORT_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},
        // Report Characteristic Value
        [HIDD_LE_IDX_REPORT_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid, ESP_GATT_PERM_READ, HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
        // Report Characteristic - Report Reference Descriptor
        [HIDD_LE_IDX_REPORT_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ, sizeof(hidReportRefFeature), sizeof(hidReportRefFeature), hidReportRefFeature}},

        // Report Map Characteristic Declaration
        [HIDD_LE_IDX_REPORT_MAP_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},
        // Report Map Characteristic Value
        [HIDD_LE_IDX_REPORT_MAP_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_map_uuid, ESP_GATT_PERM_READ, HIDD_LE_REPORT_MAP_MAX_LEN, sizeof(hid_report_map_mouse_kb_CC), (uint8_t *)&hid_report_map_mouse_kb_CC}},
        // Report Map Characteristic - External Report Reference Descriptor
        [HIDD_LE_IDX_REPORT_MAP_EXT_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_repot_map_ext_desc_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&hidExtReportRefDesc}},

        // Service Changed Characteristic Declaration
        [HIDD_LE_IDX_SERVICE_CHANGED_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_indicate}},
        // Service Changed Characteristic Value
        [HIDD_LE_IDX_SERVICE_CHANGED_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_service_changed_uuid, ESP_GATT_PERM_READ, sizeof(service_changed_indication), sizeof(service_changed_indication), (uint8_t *)service_changed_indication}},
        //Service Changed Characteristic Descriptor - Client Characteristic Configuration Descriptor
        [HIDD_LE_IDX_SERVICE_CHANGED_CCC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint16_t), 0, NULL}},
};

static struct gatts_profile_inst gatt_profile_tab[];
static prepare_type_env_t mode_setting_prep_write_env;
static void hid_add_id_tbl(void);

void write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_status_t status = ESP_GATT_OK;

    if (param->write.need_rsp)
    {
        if (prepare_write_env->prepare_buf == NULL)
        {
            prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
            prepare_write_env->prepare_len = 0;
            if (prepare_write_env->prepare_buf == NULL)
            {
                ESP_LOGI(HID_LE_PRF_TAG, "Gatt_server prep no mem");
                status = ESP_GATT_NO_RESOURCES;
            }
        }
        else
        {
            if (param->write.offset > PREPARE_BUF_MAX_SIZE)
            {
                status = ESP_GATT_INVALID_OFFSET;
            }
            else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
            {
                status = ESP_GATT_INVALID_ATTR_LEN;
            }
        }

        ESP_LOGI(HID_LE_PRF_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
        esp_log_buffer_hex(HID_LE_PRF_TAG, param->write.value, param->write.len);

        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        gatt_rsp->attr_value.len = param->write.len;
        gatt_rsp->attr_value.handle = param->write.handle;
        gatt_rsp->attr_value.offset = param->write.offset;
        gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
        memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
        esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
        if (response_err != ESP_OK)
        {
            ESP_LOGI(HID_LE_PRF_TAG, "Send response error\n");
        }
        free(gatt_rsp);
        if (status != ESP_GATT_OK)
        {
            return;
        }
        memcpy(prepare_write_env->prepare_buf + param->write.offset,
               param->write.value,
               param->write.len);
        prepare_write_env->prepare_len += param->write.len;
    }
}

void esp_hidd_prf_cb_hd(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                        esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
    {
        ESP_LOGI(HID_LE_PRF_TAG, "esp_hidd_prf_cb_hd is called on case: ESP_GATTS_REG_EVT.");
        esp_ble_gap_config_local_icon(ESP_BLE_APPEARANCE_GENERIC_HID);
        esp_hidd_cb_param_t hidd_param;
        hidd_param.init_finish.state = param->reg.status;
        if (param->reg.app_id == HIDD_APP_ID)
        {
            hidd_le_env.gatt_if = gatts_if;
            if (hidd_le_env.hidd_cb != NULL)
            {
                (hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_REG_FINISH, &hidd_param);
                hidd_le_create_service(hidd_le_env.gatt_if);
            }
        }
        if (param->reg.app_id == BATTRAY_APP_ID)
        {
            hidd_param.init_finish.gatts_if = gatts_if;
            if (hidd_le_env.hidd_cb != NULL)
            {
                (hidd_le_env.hidd_cb)(ESP_BAT_EVENT_REG, &hidd_param);
            }
        }

        break;
    }
    case ESP_GATTS_CONF_EVT:
    {
        break;
    }
    case ESP_GATTS_CREATE_EVT:
        break;

    case ESP_GATTS_START_EVT:
        ESP_LOGI(HID_LE_PRF_TAG, "ESP_GATTS_START_EVT is triggerred");
        // esp_bd_addr_t remote_device;
        // memset(remote_device, 0, sizeof(esp_bd_addr_t));
        // if(esp_ble_gap_set_prefer_conn_params(remote_device, 0x06, 0x09 , 100, 600))
        // {
        //     ESP_LOGD(HID_LE_PRF_TAG, "Failed to update peripheral conn param!");
        // }

        // ESP_LOGD(HID_LE_PRF_TAG, "Successfully updated peripheral conn param!");

        break;
    case ESP_GATTS_CONNECT_EVT:
    {
        esp_hidd_cb_param_t cb_param = {0};
        ESP_LOGI(HID_LE_PRF_TAG, "HID connection establish, conn_id = %x", param->connect.conn_id);
        memcpy(cb_param.connect.remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        cb_param.connect.conn_id = param->connect.conn_id;
        hidd_clcb_alloc(param->connect.conn_id, param->connect.remote_bda);
        esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_NO_MITM);
        if (hidd_le_env.hidd_cb != NULL)
        {
            (hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_BLE_CONNECT, &cb_param);
        }

        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the iOS system, please refer the official Apple documents about BLE connection parameters restrictions. */
        conn_params.latency = 100;
        conn_params.max_int = 0x09;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x06;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 600;    // timeout = 400*10ms = 4000ms
        if(esp_ble_gap_update_conn_params(&conn_params))
        {
            ESP_LOGD(HID_LE_PRF_TAG, "Failed to set peripheral conn param!");
        }

        esp_gap_conn_params_t curr_param = {0};

        if(esp_ble_get_current_conn_params(param->connect.remote_bda, &curr_param))
        {
            ESP_LOGD(HID_LE_PRF_TAG, "Failed to get peripheral conn param!");
        }
        ESP_LOGD(HID_LE_PRF_TAG, "Connection param AFTER setting, L: %d, I: %d, T: %d.", curr_param.latency, curr_param.interval, curr_param.timeout);

        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
    {
        if (hidd_le_env.hidd_cb != NULL)
        {
            (hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_BLE_DISCONNECT, NULL);
        }
        hidd_clcb_dealloc(param->disconnect.conn_id);
        break;
    }
    case ESP_GATTS_CLOSE_EVT:
        break;
    case ESP_GATTS_WRITE_EVT:
    {
#if (SUPPORT_REPORT_VENDOR == true)
        ESP_LOGI(HID_LE_PRF_TAG, "esp_hidd_prf_cb_hd, case: ESP_GATTS_WRITE_EVT");
        esp_hidd_cb_param_t cb_param = {0};
        if (param->write.handle == hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_VENDOR_OUT_VAL] &&
            hidd_le_env.hidd_cb != NULL)
        {
            cb_param.vendor_write.conn_id = param->write.conn_id;
            cb_param.vendor_write.report_id = HID_RPT_ID_VENDOR_OUT;
            cb_param.vendor_write.length = param->write.len;
            cb_param.vendor_write.data = param->write.value;
            (hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT, &cb_param);
        }
#endif
        break;
    }
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        if (param->add_attr_tab.num_handle == BAS_IDX_NB &&
            param->add_attr_tab.svc_uuid.uuid.uuid16 == ESP_GATT_UUID_BATTERY_SERVICE_SVC &&
            param->add_attr_tab.status == ESP_GATT_OK)
        {
            incl_svc.start_hdl = param->add_attr_tab.handles[BAS_IDX_SVC];
            incl_svc.end_hdl = incl_svc.start_hdl + BAS_IDX_NB - 1;
            ESP_LOGI(HID_LE_PRF_TAG, "%s(), start added the hid service to the stack database. incl_handle = %d",
                     __func__, incl_svc.start_hdl);
            esp_ble_gatts_create_attr_tab(hidd_le_gatt_db, gatts_if, HIDD_LE_IDX_NB, 0);
        }
        if (param->add_attr_tab.num_handle == HIDD_LE_IDX_NB &&
            param->add_attr_tab.status == ESP_GATT_OK)
        {
            memcpy(hidd_le_env.hidd_inst.att_tbl, param->add_attr_tab.handles,
                   HIDD_LE_IDX_NB * sizeof(uint16_t));
            ESP_LOGI(HID_LE_PRF_TAG, "hid svc handle = %x", hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_SVC]);
            hid_add_id_tbl();
            esp_ble_gatts_start_service(hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_SVC]);
        }
        else
        {
            esp_ble_gatts_start_service(param->add_attr_tab.handles[0]);
        }
        break;
    }

    default:
        break;
    }
}

void device_info_prf_cb_hd(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                        esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
    {
        ESP_LOGI(HID_LE_PRF_TAG, "device_info_prf_cb_hd is called on case: ESP_GATTS_REG_EVT.");
        esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(pnp_id_att_db, gatts_if, DEV_INFO_IDX_NB, 0);
        if (create_attr_ret)
        {
            ESP_LOGI(HID_LE_PRF_TAG, "create attr table failed, error code = %x", create_attr_ret);
        }
        break;
    }
    case ESP_GATTS_CONF_EVT:
    {
        break;
    }
    case ESP_GATTS_CREATE_EVT:
        break;

    case ESP_GATTS_START_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
    {
        ESP_LOGI(HID_LE_PRF_TAG, "device_info_prf_cb_hd: ESP_GATTS_CONNECT_EVT, conn_id = %x", param->connect.conn_id);
        gatt_profile_tab[DEVICE_INFO_PROFILE_APP_IDX].conn_id = param->connect.conn_id;
        ESP_LOGD(HID_LE_PRF_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
    {
        ESP_LOGI(HID_LE_PRF_TAG, "device_info_prf_cb_hd: ESP_GATTS_DISCONNECT_EVT");
        break;
    }
    case ESP_GATTS_CLOSE_EVT:
        break;
    case ESP_GATTS_WRITE_EVT:
    {
        break;
    }
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(HID_LE_PRF_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        else if (param->add_attr_tab.num_handle != DEV_INFO_IDX_NB)
        {
            ESP_LOGE(HID_LE_PRF_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to MSS_IDX_NB(%d)",
                     param->add_attr_tab.num_handle, MSS_IDX_NB);
        }
        else
        {
            ESP_LOGI(HID_LE_PRF_TAG, "create attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
            memcpy(device_info_svc_hdl_tab, param->add_attr_tab.handles, sizeof(device_info_svc_hdl_tab));
            if(esp_ble_gatts_start_service(device_info_svc_hdl_tab[DEV_INFO_IDX_SVC]) == ESP_FAIL)
            {
                ESP_LOGI(HID_LE_PRF_TAG, "Failed to create attribute table for device_info_svc_hdl_tab.");
            }
        }
        break;
    }
    default:
        break;
    }
}

void esp_mode_prf_cb_hd(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                        esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
    {
        ESP_LOGI(HID_LE_PRF_TAG, "esp_mode_prf_cb_hd is called on case: ESP_GATTS_REG_EVT.");
        esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(mode_setting_att_db, gatts_if, MSS_IDX_NB, 0);
        if (create_attr_ret)
        {
            ESP_LOGI(HID_LE_PRF_TAG, "create attr table failed, error code = %x", create_attr_ret);
        }
        break;
    }
    case ESP_GATTS_READ_EVT:
    { // F**k, Windows is capricious, so please make sure to return something for a characteristic, or even other services would get block
      // Android is good.
        ESP_LOGI(HID_LE_PRF_TAG, "esp_mode_prf_cb_hd is called on case: ESP_GATTS_READ_EVT. ");
        ESP_LOGI(HID_LE_PRF_TAG, "ESP_GATTS_READ_EVT, conn_id: %d, trans_id: %d, read handle: %d.", param->read.conn_id, param->read.trans_id, param->read.handle);
        if (param->read.handle == mode_svc_hdl_tab[MSS_IDX_CHAR_VAL_CURR_MODE]) // handle read for current mode char
        {
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            uint8_t curr_mode = 0;
            read_curr_mode_from_nvs(&curr_mode);
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = 1;
            rsp.attr_value.value[0] = curr_mode;
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                        ESP_GATT_OK, &rsp);
        }
        else if (param->read.handle == mode_svc_hdl_tab[MSS_IDX_CHAR_VAL_MODE_SETTING]) // handle read for mode setting char
        {
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            uint8_t curr_mode = 0;
            read_curr_mode_from_nvs(&curr_mode);
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = 1;
            rsp.attr_value.value[0] = curr_mode;
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                        ESP_GATT_OK, &rsp);
        }
        break;
    }
    case ESP_GATTS_WRITE_EVT:
    {
        ESP_LOGI(HID_LE_PRF_TAG, "esp_mode_prf_cb_hd is called on case: ESP_GATTS_WRITE_EVT. ");
        ESP_LOGI(HID_LE_PRF_TAG, "ESP_GATTS_WRITE_EVT, conn_id: %d, trans_id: %d, write handle: %d.", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (param->write.handle == mode_svc_hdl_tab[MSS_IDX_CHAR_VAL_CURR_MODE]) // handle write for current mode char
        {
            if (!param->write.is_prep)
            {
                ESP_LOGI(HID_LE_PRF_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
                esp_log_buffer_hex(HID_LE_PRF_TAG, param->write.value, param->write.len);
                if (param->write.len >= 1)
                {
                    write_curr_mode_to_nvs(param->write.value[0]);
                }

                if (param->write.need_rsp)
                {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
        }
        else if (param->write.handle == mode_svc_hdl_tab[MSS_IDX_CHAR_VAL_MODE_SETTING]) // handle write for mode setting char
        {
            if (!param->write.is_prep)
            {
                ESP_LOGI(HID_LE_PRF_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
                esp_log_buffer_hex(HID_LE_PRF_TAG, param->write.value, param->write.len);
                if (param->write.need_rsp)
                {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            else
            {
                write_event_env(gatts_if, &mode_setting_prep_write_env, param);
            }
        }
        else if (param->write.handle == mode_svc_hdl_tab[MSS_IDX_CHAR_CFG_CURR_MODE])
        {
            ESP_LOGI(HID_LE_PRF_TAG, "GATT_WRITE_EVT->CHAR_CFG_CURR_MODE");
        }
        else if (param->write.handle == mode_svc_hdl_tab[MSS_IDX_CHAR_CFG_MODE_SETTING])
        {
            ESP_LOGI(HID_LE_PRF_TAG, "GATT_WRITE_EVT->CHAR_CFG_MODE_SETTING");
        }

        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
    {
        ESP_LOGI(HID_LE_PRF_TAG, "esp_mode_prf_cb_hd is called on case: ESP_GATTS_EXEC_WRITE_EVT. ");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        ESP_LOGI(HID_LE_PRF_TAG, "ESP_GATTS_EXEC_WRITE_EVT, conn_id: %d, trans_id: %d, write handle: %d.", param->write.conn_id, param->write.trans_id, param->write.handle);

        bool need_reboot = false;

        if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC)
        {
            esp_log_buffer_hex(HID_LE_PRF_TAG, mode_setting_prep_write_env.prepare_buf, mode_setting_prep_write_env.prepare_len);
            // TODO need to save data here
            ESP_LOGI(HID_LE_PRF_TAG, "ESP_GATTS_EXEC_WRITE_EVT: Begin to update the operation_action table.");
            update_operations_tab(mode_setting_prep_write_env.prepare_buf, mode_setting_prep_write_env.prepare_len);
            write_all_operations_to_nvs();
            need_reboot = true;
        }
        else
        {
            ESP_LOGI(HID_LE_PRF_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
        }

        if (mode_setting_prep_write_env.prepare_buf)
        {
            free(mode_setting_prep_write_env.prepare_buf);
            mode_setting_prep_write_env.prepare_buf = NULL;
        }
        mode_setting_prep_write_env.prepare_len = 0;

        if (need_reboot == true && hidd_le_env.hidd_cb != NULL)
        {
            (hidd_le_env.hidd_cb)(ESP_MODE_SETTING_UPDATED, NULL);
        }

        break;
    }
    case ESP_GATTS_CONNECT_EVT:
    {
        ESP_LOGI(HID_LE_PRF_TAG, "Mode setting handler evnt: ESP_GATTS_CONNECT_EVT, conn_id = %x", param->connect.conn_id);
        gatt_profile_tab[MODE_PROFILE_APP_IDX].conn_id = param->connect.conn_id;
        ESP_LOGD(HID_LE_PRF_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
    {
        ESP_LOGI(HID_LE_PRF_TAG, "Mode setting handler event: ESP_GATTS_DISCONNECT_EVT");
        break;
    }
    case ESP_GATTS_CLOSE_EVT:
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(HID_LE_PRF_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        else if (param->add_attr_tab.num_handle != MSS_IDX_NB)
        {
            ESP_LOGE(HID_LE_PRF_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to MSS_IDX_NB(%d)",
                     param->add_attr_tab.num_handle, MSS_IDX_NB);
        }
        else
        {
            ESP_LOGI(HID_LE_PRF_TAG, "create attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
            memcpy(mode_svc_hdl_tab, param->add_attr_tab.handles, sizeof(mode_svc_hdl_tab));
            esp_ble_gatts_start_service(mode_svc_hdl_tab[MSS_IDX_SVC]);
        }
        break;
    }
    default:
        break;
    }
}

void hidd_le_create_service(esp_gatt_if_t gatts_if)
{
    /* Here should added the battery service first, because the hid service should include the battery service.
       After finish to added the battery service then can added the hid service. */
    esp_ble_gatts_create_attr_tab(bas_att_db, gatts_if, BAS_IDX_NB, 0);
}

void hidd_le_init(void)
{

    // Reset the hid device target environment
    memset(&hidd_le_env, 0, sizeof(hidd_le_env_t));
}

void hidd_clcb_alloc(uint16_t conn_id, esp_bd_addr_t bda)
{
    uint8_t i_clcb = 0;
    hidd_clcb_t *p_clcb = NULL;

    for (i_clcb = 0, p_clcb = hidd_le_env.hidd_clcb; i_clcb < HID_MAX_APPS; i_clcb++, p_clcb++)
    {
        if (!p_clcb->in_use)
        {
            p_clcb->in_use = true;
            p_clcb->conn_id = conn_id;
            p_clcb->connected = true;
            memcpy(p_clcb->remote_bda, bda, ESP_BD_ADDR_LEN);
            break;
        }
    }
    return;
}

bool hidd_clcb_dealloc(uint16_t conn_id)
{
    uint8_t i_clcb = 0;
    hidd_clcb_t *p_clcb = NULL;

    for (i_clcb = 0, p_clcb = hidd_le_env.hidd_clcb; i_clcb < HID_MAX_APPS; i_clcb++, p_clcb++)
    {
        memset(p_clcb, 0, sizeof(hidd_clcb_t));
        return true;
    }

    return false;
}
static struct gatts_profile_inst gatt_profile_tab[PROFILE_NUM] = {
    [HID_PROFILE_APP_IDX] = {
        .gatts_cb = esp_hidd_prf_cb_hd,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    [MODE_PROFILE_APP_IDX] = {
        .gatts_cb = esp_mode_prf_cb_hd, .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    [DEVICE_INFO_PROFILE_APP_IDX] = {
        .gatts_cb = device_info_prf_cb_hd, .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        ESP_LOGI(HID_LE_PRF_TAG, "gatts_event_handle, case: ESP_GATTS_REG_EVT");
        if (param->reg.status == ESP_GATT_OK)
        {
            if (param->reg.app_id == HIDD_APP_ID || param->reg.app_id == BATTRAY_APP_ID)
            {
                gatt_profile_tab[HID_PROFILE_APP_IDX].gatts_if = gatts_if;
            }
            else if (param->reg.app_id == MODE_APP_ID)
            {
                gatt_profile_tab[MODE_PROFILE_APP_IDX].gatts_if = gatts_if;
            }
            else if (param->reg.app_id == DEVICE_INFO_APP_ID)
            {
                gatt_profile_tab[DEVICE_INFO_PROFILE_APP_IDX].gatts_if = gatts_if;
            }

            ESP_LOGI(HID_LE_PRF_TAG, "Assign gatt_if:%d to app id: %x", gatts_if, param->reg.app_id);
        }
        else
        {
            ESP_LOGI(HID_LE_PRF_TAG, "Reg app failed, app_id %04x, status %d\n",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == gatt_profile_tab[idx].gatts_if)
            {
                if (gatt_profile_tab[idx].gatts_cb)
                {
                    gatt_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

esp_err_t hidd_register_cb(void)
{
    esp_err_t status;
    status = esp_ble_gatts_register_callback(gatts_event_handler);
    return status;
}

void hidd_set_attr_value(uint16_t handle, uint16_t val_len, const uint8_t *value)
{
    hidd_inst_t *hidd_inst = &hidd_le_env.hidd_inst;
    if (hidd_inst->att_tbl[HIDD_LE_IDX_HID_INFO_VAL] <= handle &&
        hidd_inst->att_tbl[HIDD_LE_IDX_REPORT_REP_REF] >= handle)
    {
        esp_ble_gatts_set_attr_value(handle, val_len, value);
    }
    else
    {
        ESP_LOGE(HID_LE_PRF_TAG, "%s error:Invalid handle value.", __func__);
    }
    return;
}

void hidd_get_attr_value(uint16_t handle, uint16_t *length, uint8_t **value)
{
    hidd_inst_t *hidd_inst = &hidd_le_env.hidd_inst;
    if (hidd_inst->att_tbl[HIDD_LE_IDX_HID_INFO_VAL] <= handle &&
        hidd_inst->att_tbl[HIDD_LE_IDX_REPORT_REP_REF] >= handle)
    {
        esp_ble_gatts_get_attr_value(handle, length, (const uint8_t **)value);
    }
    else
    {
        ESP_LOGE(HID_LE_PRF_TAG, "%s error:Invalid handle value.", __func__);
    }

    return;
}

void hidd_set_service_changed_version(uint8_t serv_version)
{
    service_changed_indication[0] = serv_version;
}

uint8_t hidd_get_service_changed_version()
{
    return service_changed_indication[0];
}

static void hid_add_id_tbl(void)
{
    // Mouse input report
    hid_rpt_map[0].id = hidReportRefMouseIn[0];
    hid_rpt_map[0].type = hidReportRefMouseIn[1];
    hid_rpt_map[0].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_MOUSE_IN_VAL];
    hid_rpt_map[0].cccdHandle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_MOUSE_IN_CCC];
    hid_rpt_map[0].mode = HID_PROTOCOL_MODE_REPORT;

    // Key input report
    hid_rpt_map[1].id = hidReportRefKeyIn[0];
    hid_rpt_map[1].type = hidReportRefKeyIn[1];
    hid_rpt_map[1].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_KEY_IN_VAL];
    hid_rpt_map[1].cccdHandle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_KEY_IN_CCC];
    hid_rpt_map[1].mode = HID_PROTOCOL_MODE_REPORT;

    // Consumer Control input report
    hid_rpt_map[2].id = hidReportRefCCIn[0];
    hid_rpt_map[2].type = hidReportRefCCIn[1];
    hid_rpt_map[2].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_CC_IN_VAL];
    hid_rpt_map[2].cccdHandle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_CC_IN_CCC];
    hid_rpt_map[2].mode = HID_PROTOCOL_MODE_REPORT;

    // LED output report
    hid_rpt_map[3].id = hidReportRefLedOut[0];
    hid_rpt_map[3].type = hidReportRefLedOut[1];
    hid_rpt_map[3].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_LED_OUT_VAL];
    hid_rpt_map[3].cccdHandle = 0;
    hid_rpt_map[3].mode = HID_PROTOCOL_MODE_REPORT;

    // Feature report
    hid_rpt_map[4].id = hidReportRefFeature[0];
    hid_rpt_map[4].type = hidReportRefFeature[1];
    hid_rpt_map[4].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_VAL];
    hid_rpt_map[4].cccdHandle = 0;
    hid_rpt_map[4].mode = HID_PROTOCOL_MODE_REPORT;

    // stylus input report
    hid_rpt_map[5].id = hidReportRefStylusIn[0];
    hid_rpt_map[5].type = hidReportRefStylusIn[1];
    hid_rpt_map[5].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_STYLUS_IN_VAL];
    hid_rpt_map[5].cccdHandle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_STYLUS_IN_CCC];
    hid_rpt_map[5].mode = HID_PROTOCOL_MODE_REPORT;

    // service change indication
    hid_rpt_map[6].id = hidServiceChangedIn[0];
    hid_rpt_map[6].type = hidServiceChangedIn[1];
    hid_rpt_map[6].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_SERVICE_CHANGED_VAL];
    hid_rpt_map[6].cccdHandle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_SERVICE_CHANGED_CCC];
    hid_rpt_map[6].mode = HID_PROTOCOL_MODE_REPORT;

    // Setup report ID map
    hid_dev_register_reports(HID_NUM_REPORTS, hid_rpt_map);
}

void hidd_set_report_map(int report_map_type)
{
    if(report_map_type == HIDD_REPORT_MAP_MS_KB_CC)
    {
        // Nothing to do, as the defualt report map is mouse ,kb and cc 
    }
    else if(report_map_type == HIDD_REPORT_MAP_STYLUS_CC)
    {
        hidd_le_gatt_db[HIDD_LE_IDX_REPORT_MAP_VAL].att_desc.length = sizeof(hid_report_map_CC_stylus);
        hidd_le_gatt_db[HIDD_LE_IDX_REPORT_MAP_VAL].att_desc.value = (uint8_t *)&hid_report_map_CC_stylus;
    }
}
