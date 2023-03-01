#ifndef HID_TOUCH_GESTURES
#define HID_TOUCH_GESTURES

#define TOUCH_INTERVAL 10
#define TOUCH_DELAY    20
#define TAP_DELAY      80

void send_slide_up(uint16_t hid_conn_id);
void send_slide_down(uint16_t hid_conn_id);
void send_slide_left(uint16_t hid_conn_id);
void send_slide_right(uint16_t hid_conn_id);
void send_back(uint16_t hid_conn_id);
void send_tap(uint16_t hid_conn_id);
void send_press_down(uint16_t hid_conn_id);
void send_press_up(uint16_t hid_conn_id);
void send_double_tap(uint16_t hid_conn_id);
void send_touch_gesture(uint16_t hid_conn_id, int gesture);

#endif