#ifndef HID_TOUCH_GESTURES
#define HID_TOUCH_GESTURES

#define TOUCH_INTERVAL 15
#define TOUCH_DELAY    20
#define TAP_DELAY      100

typedef struct
{
    int available;
    int gesture;
} gesture_state;


int gesture_available(gesture_state gs);
int get_gesture(gesture_state *gs);
void set_gesture(gesture_state *gs, int gesture);


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