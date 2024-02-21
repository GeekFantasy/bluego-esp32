#include "mode_setting_ui.h"
#include "operations.h"

#define MODE_SETTING_UI_TAG     "MODE SETTING"


static lv_indev_t * encoder_indev = NULL;
lv_obj_t * scr_setting = NULL;
lv_obj_t * scr_actions = NULL;
lv_obj_t * scr_acts_slide = NULL;


void init_mode_setting_ui(lv_indev_t * enc_indev)
{
    encoder_indev = enc_indev;
}

static void sw_event_cb(lv_event_t * event)
{
    lv_obj_t * sw = lv_event_get_target(event); 
    lv_obj_t * container = lv_event_get_user_data(event); 

    if(lv_obj_has_state(sw, LV_STATE_CHECKED)) {
        lv_obj_clear_flag(container, LV_OBJ_FLAG_HIDDEN); 
    } else {
        lv_obj_add_flag(container, LV_OBJ_FLAG_HIDDEN); 
    }
}

lv_obj_t *list1;

static void event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        ESP_LOGI(MODE_SETTING_UI_TAG, "Button is clicked: %s.", lv_list_get_btn_text(list1, obj));
    }
}

static void radio_event_cb(lv_event_t * e) {
    ESP_LOGI(MODE_SETTING_UI_TAG, "Radio button event is triggered");
    if(lv_event_get_code(e) == LV_EVENT_KEY) 
    {
        const uint32_t * key = lv_event_get_param(e);
        lv_obj_t * obj = lv_event_get_target(e);
        lv_group_t * g = lv_obj_get_group(obj);
        if(*key == LV_KEY_RIGHT) 
        {
            lv_group_focus_next(g);
        } 
        else if(*key == LV_KEY_LEFT) 
        {
            lv_group_focus_prev(g);
        }
    }
}

void create_actions_slide_ui()
{
    scr_acts_slide = lv_obj_create(NULL);

    lv_group_t * g = lv_group_create();
    lv_indev_set_group(encoder_indev, g);

    lv_obj_t *label = lv_label_create(scr_acts_slide);
    lv_label_set_text(label, "Select: ");
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 2 , 2);

    lv_obj_t *cont1 = lv_list_create(scr_acts_slide);
    lv_obj_set_size(cont1, LV_PCT(100), 110 );
    lv_obj_set_scrollbar_mode(cont1, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_flex_flow(cont1, LV_FLEX_FLOW_COLUMN);
    //lv_cont_set_layout(cont1, LV_LAYOUT_COLUMN_LEFT); // 设置为垂直布局
    lv_obj_align_to(cont1, label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

    /*Add buttons to the list*/

    static lv_style_t style_radio;
    static lv_style_t style_radio_chk;
    static lv_style_t style_main;
    static lv_style_t style_main_focused;

    lv_style_init(&style_main);
    lv_style_set_pad_all(&style_main, 1);
    lv_style_set_border_color(&style_main, lv_color_white());
    lv_style_set_border_width(&style_main, 1);
    lv_style_set_text_font(&style_main, &lv_font_montserrat_10);

    lv_style_init(&style_main_focused);
    lv_style_set_pad_all(&style_main_focused, 1);
    lv_style_set_border_color(&style_main_focused, lv_color_black());
    lv_style_set_border_width(&style_main_focused, 1);
    lv_style_set_text_font(&style_main_focused, &lv_font_montserrat_10);

    lv_style_init(&style_radio);
    lv_style_set_radius(&style_radio, LV_RADIUS_CIRCLE);
    lv_style_set_pad_all(&style_radio, 2);
    lv_style_set_border_color(&style_radio, lv_color_black());
    lv_style_set_border_width(&style_radio, 1);
    lv_style_set_bg_opa(&style_radio, LV_OPA_COVER );
    lv_style_set_bg_color(&style_radio, lv_color_white());
    
    lv_style_init(&style_radio_chk);
    lv_style_set_radius(&style_radio_chk, LV_RADIUS_CIRCLE);
    lv_style_set_pad_all(&style_radio_chk, 2);
    lv_style_set_border_color(&style_radio_chk, lv_color_black());
    lv_style_set_border_width(&style_radio_chk, 2);
    lv_style_set_bg_opa(&style_radio_chk, LV_OPA_COVER );
    lv_style_set_bg_color(&style_radio_chk, lv_color_black());

    //lv_style_set_bg_img_src(&style_radio_chk, NULL);


    lv_obj_t * chx = lv_checkbox_create(cont1);
    lv_group_add_obj(g, chx);
    lv_obj_add_event_cb(chx, radio_event_cb, LV_EVENT_KEY, NULL);
    lv_checkbox_set_text(chx, " Slide Up");
    //lv_obj_add_flag(chx, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_style(chx, &style_main, LV_PART_MAIN);
    lv_obj_add_style(chx, &style_main_focused, LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_add_style(chx, &style_radio, LV_PART_INDICATOR);
    lv_obj_add_style(chx, &style_radio_chk, LV_PART_INDICATOR | LV_STATE_CHECKED);

    chx = lv_checkbox_create(cont1);
    lv_group_add_obj(g, chx);
    lv_obj_add_event_cb(chx, radio_event_cb, LV_EVENT_KEY, NULL);
    lv_checkbox_set_text(chx, " Slide Down");
    //lv_obj_add_flag(chx, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_style(chx, &style_main, LV_PART_MAIN);
    lv_obj_add_style(chx, &style_main_focused, LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_add_style(chx, &style_radio, LV_PART_INDICATOR);
    lv_obj_add_style(chx, &style_radio_chk, LV_PART_INDICATOR | LV_STATE_CHECKED);

    chx = lv_checkbox_create(cont1);
    lv_group_add_obj(g, chx);
    lv_obj_add_event_cb(chx, radio_event_cb, LV_EVENT_KEY, NULL);
    lv_checkbox_set_text(chx, " Slide Left");
    //lv_obj_add_flag(chx, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_style(chx, &style_main, LV_PART_MAIN);
    lv_obj_add_style(chx, &style_main_focused, LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_add_style(chx, &style_radio, LV_PART_INDICATOR);
    lv_obj_add_style(chx, &style_radio_chk, LV_PART_INDICATOR | LV_STATE_CHECKED);

    chx = lv_checkbox_create(cont1);
    lv_group_add_obj(g, chx);
    lv_obj_add_event_cb(chx, radio_event_cb, LV_EVENT_KEY, NULL);
    lv_checkbox_set_text(chx, " Slide Right");
    //lv_obj_add_flag(chx, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_style(chx, &style_main, LV_PART_MAIN);
    lv_obj_add_style(chx, &style_main_focused, LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_add_style(chx, &style_radio, LV_PART_INDICATOR);
    lv_obj_add_style(chx, &style_radio_chk, LV_PART_INDICATOR | LV_STATE_CHECKED);

    chx = lv_checkbox_create(cont1);
    lv_group_add_obj(g, chx);
    lv_obj_add_event_cb(chx, radio_event_cb, LV_EVENT_KEY, NULL);
    lv_checkbox_set_text(chx, " Tap");
    //lv_obj_add_flag(chx, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_style(chx, &style_main, LV_PART_MAIN);
    lv_obj_add_style(chx, &style_main_focused, LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_add_style(chx, &style_radio, LV_PART_INDICATOR);
    lv_obj_add_style(chx, &style_radio_chk, LV_PART_INDICATOR | LV_STATE_CHECKED);

    chx = lv_checkbox_create(cont1);
    lv_group_add_obj(g, chx);
    lv_obj_add_event_cb(chx, radio_event_cb, LV_EVENT_KEY, NULL);
    lv_checkbox_set_text(chx, " Dbl Tap");
    //lv_obj_add_flag(chx, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_style(chx, &style_main, LV_PART_MAIN);
    lv_obj_add_style(chx, &style_main_focused, LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_add_style(chx, &style_radio, LV_PART_INDICATOR);
    lv_obj_add_style(chx, &style_radio_chk, LV_PART_INDICATOR | LV_STATE_CHECKED);

    chx = lv_checkbox_create(cont1);
    lv_group_add_obj(g, chx);
    lv_obj_add_event_cb(chx, radio_event_cb, LV_EVENT_KEY, NULL);
    lv_checkbox_set_text(chx, " Backward");
    //lv_obj_add_flag(chx, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_style(chx, &style_main, LV_PART_MAIN);
    lv_obj_add_style(chx, &style_main_focused, LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_add_style(chx, &style_radio, LV_PART_INDICATOR);
    lv_obj_add_style(chx, &style_radio_chk, LV_PART_INDICATOR | LV_STATE_CHECKED);

    //lv_obj_add_state(lv_obj_get_child(cont1, 0), LV_STATE_CHECKED);
}

void create_actions_ui()
{
    uint16_t  action_key = 204;
    char action_string[20] = {};

    scr_actions = lv_obj_create(NULL);

    lv_group_t * g = lv_group_create();
    lv_indev_set_group(encoder_indev, g);

    lv_obj_t *label = lv_label_create(scr_actions);
    lv_label_set_text(label, "Current Action:");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_10, 0);
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 0);

    if(get_action_str(action_key, action_string) == 0)
    {
        ESP_LOGI(MODE_SETTING_UI_TAG, "Succeed to get action string: %s.", action_string);
    }
    else
    {
        ESP_LOGI(MODE_SETTING_UI_TAG, "Failed to get action string: %s.", action_string);
    }

    lv_obj_t *label_action = lv_label_create(scr_actions);
    lv_label_set_text(label_action, action_string);
    lv_obj_set_style_text_font(label_action, &lv_font_montserrat_10, 0);
    lv_obj_align_to(label_action, label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

    static lv_point_t line_points[] = {{1, 0}, {78, 0}};
    static lv_style_t style_line;
    lv_style_init(&style_line);
    lv_style_set_line_width(&style_line, 2);
    lv_style_set_line_color(&style_line, lv_color_black());

    /*Create a line and apply the new style*/
    lv_obj_t * line;
    line = lv_line_create(scr_actions);
    lv_line_set_points(line, line_points, 2);     /*Set the points*/
    lv_obj_add_style(line, &style_line, 0);
    lv_obj_align_to(line, label_action, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 2);


    list1 = lv_list_create(scr_actions);
    lv_obj_set_size(list1, LV_PCT(100), LV_SIZE_CONTENT);
    //lv_obj_align(list1, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_align_to(list1, line, LV_ALIGN_BOTTOM_LEFT, 0 , 0);

    /*Add buttons to the list*/
    lv_obj_t * btn;

    static lv_style_t list_label;
    static lv_style_t list_main;
    static lv_style_t list_button;
    static lv_style_t list_button_foc;

    lv_style_init(&list_label);
    lv_style_set_pad_top(&list_label, 2);
    lv_style_set_pad_bottom(&list_label, 2);

    lv_style_init(&list_main);
    lv_style_set_pad_left(&list_main, 5);
    lv_obj_add_style(list1, &list_main, 0);

    lv_style_init(&list_button);
    lv_style_set_pad_left(&list_button, 2);
    lv_style_set_bg_opa(&list_button, LV_OPA_COVER);
    lv_style_set_bg_color(&list_button, lv_color_white());
    lv_style_set_text_color(&list_button, lv_color_black());

    lv_style_init(&list_button_foc);
    lv_style_set_pad_left(&list_button_foc, 2);
    lv_style_set_bg_opa(&list_button_foc, LV_OPA_COVER);
    lv_style_set_bg_color(&list_button_foc, lv_color_black());
    lv_style_set_text_color(&list_button_foc, lv_color_white());
    
    lv_list_add_text(list1, "Actions");

    btn = lv_list_add_btn(list1, LV_SYMBOL_FILE, "Slide");
    lv_obj_add_style(btn, &list_button, LV_STATE_DEFAULT);
    lv_obj_add_style(btn, &list_button_foc, LV_STATE_FOCUSED);
    lv_group_add_obj(g, btn);
    lv_obj_add_event_cb(btn, event_handler, LV_EVENT_CLICKED, NULL);

    btn = lv_list_add_btn(list1, LV_SYMBOL_DIRECTORY, "Mouse");
    lv_obj_add_style(btn, &list_button, LV_STATE_DEFAULT);
    lv_obj_add_style(btn, &list_button_foc, LV_STATE_FOCUSED);
    lv_group_add_obj(g, btn);
    lv_obj_add_event_cb(btn, event_handler, LV_EVENT_CLICKED, NULL);

    btn = lv_list_add_btn(list1, LV_SYMBOL_KEYBOARD, "Keyboard");
    lv_obj_add_style(btn, &list_button, LV_STATE_DEFAULT);
    lv_obj_add_event_cb(btn, event_handler, LV_EVENT_CLICKED, NULL);
    lv_group_add_obj(g, btn);
    lv_obj_add_style(btn, &list_button_foc, LV_STATE_FOCUSED);

    btn = lv_list_add_btn(list1, LV_SYMBOL_CLOSE, "Dev Ctrl");
    lv_obj_add_style(btn, &list_button, LV_STATE_DEFAULT);
    lv_obj_add_style(btn, &list_button_foc, LV_STATE_FOCUSED);
    lv_group_add_obj(g, btn);
    lv_obj_add_event_cb(btn, event_handler, LV_EVENT_CLICKED, NULL);
}

void create_setting_ui()
{
    lv_obj_t * label;
    lv_obj_t * sw;
    lv_obj_t * btn;
    lv_obj_t * btn_label;

    uint16_t  imu_flag = get_action_code_from_tmp_matrix(OPER_KEY_IMU);
    uint16_t  ges_flag = get_action_code_from_tmp_matrix(OPER_KEY_GES);
    uint16_t  tbk_flag = get_action_code_from_tmp_matrix(OPER_KEY_TKB);

    lv_group_t * g = lv_group_create();
    lv_indev_set_group(encoder_indev, g);

    scr_setting = lv_obj_create(NULL);

    lv_obj_t * container = lv_obj_create(scr_setting);
    lv_obj_set_size(container, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_align(container, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);

    static lv_style_t style_sw_main;
    static lv_style_t style_sw_indic;
    static lv_style_t style_sw_knob;
    static lv_style_t style_sw_knob_foc;
    static lv_style_t styel_btn_focused;
    static lv_style_t styel_btn;

    lv_style_init(&style_sw_main);
    lv_style_set_width(&style_sw_main, 24);
    lv_style_set_height(&style_sw_main, 12);
    lv_style_set_bg_opa(&style_sw_main, LV_OPA_COVER);
    lv_style_set_bg_color(&style_sw_main, lv_color_white());
    lv_style_set_radius(&style_sw_main, LV_RADIUS_CIRCLE); 
    lv_style_set_border_color(&style_sw_main, lv_color_black());
    lv_style_set_border_width(&style_sw_main, 1);
    //lv_style_set_text_font(&style_sw_main, &lv_font_montserrat_10);

    lv_style_init(&style_sw_indic);
    lv_style_set_bg_opa(&style_sw_indic, LV_OPA_COVER);
    lv_style_set_bg_color(&style_sw_indic, lv_color_black());
    lv_style_set_radius(&style_sw_indic, LV_RADIUS_CIRCLE);
    lv_style_set_border_color(&style_sw_indic, lv_color_black());
    lv_style_set_border_width(&style_sw_indic, 1);

    lv_style_init(&style_sw_knob);
    lv_style_set_bg_opa(&style_sw_knob, LV_OPA_COVER);
    lv_style_set_bg_color(&style_sw_knob, lv_color_white()); 
    lv_style_set_border_color(&style_sw_knob, lv_color_black()); 
    lv_style_set_border_width(&style_sw_knob, 2); 
    lv_style_set_radius(&style_sw_knob, LV_RADIUS_CIRCLE);

    lv_style_init(&style_sw_knob_foc);
    lv_style_set_bg_opa(&style_sw_knob_foc, LV_OPA_COVER);
    lv_style_set_bg_color(&style_sw_knob_foc, lv_color_black()); 
    lv_style_set_border_color(&style_sw_knob_foc, lv_color_black()); 
    lv_style_set_border_width(&style_sw_knob_foc, 2); 
    lv_style_set_radius(&style_sw_knob_foc, LV_RADIUS_CIRCLE);
    
    lv_style_init(&styel_btn_focused);
    lv_style_set_bg_opa(&styel_btn_focused, LV_OPA_COVER);
    lv_style_set_bg_color(&styel_btn_focused, lv_color_black()); 
    lv_style_set_border_color(&styel_btn_focused, lv_color_black()); 
    lv_style_set_border_width(&styel_btn_focused, 1);
    lv_style_set_text_color(&styel_btn_focused, lv_color_white());
    lv_style_set_text_font(&styel_btn_focused, &lv_font_montserrat_10);

    lv_style_init(&styel_btn);
    lv_style_set_bg_opa(&styel_btn, LV_OPA_COVER);
    lv_style_set_bg_color(&styel_btn, lv_color_white()); 
    lv_style_set_border_color(&styel_btn, lv_color_white()); 
    lv_style_set_border_width(&styel_btn, 1);
    lv_style_set_text_color(&styel_btn, lv_color_black());
    lv_style_set_text_font(&styel_btn, &lv_font_montserrat_10);

    // Setting section for IMU
    lv_obj_t * cont_title = lv_obj_create(container);
    lv_obj_set_size(cont_title, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(cont_title, LV_FLEX_FLOW_ROW);

    label = lv_label_create(cont_title);
    lv_label_set_text(label, "IMU ");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);

    sw = lv_switch_create(cont_title);
    lv_group_add_obj(g, sw);
    lv_obj_add_style(sw, &style_sw_knob, LV_PART_KNOB);
    lv_obj_add_style(sw, &style_sw_knob_foc, LV_PART_KNOB | LV_STATE_FOCUSED);
    lv_obj_add_style(sw, &style_sw_main, LV_PART_MAIN);
    lv_obj_add_style(sw, &style_sw_indic, LV_PART_INDICATOR | LV_STATE_CHECKED);
    
    //lv_obj_add_state(sw, LV_STATE_CHECKED);
    //lv_obj_align_to(sw, label, LV_ALIGN_OUT_RIGHT_MID, 5, 0); 

    static lv_point_t line_points[] = {{1, 0}, {78, 0}};
    static lv_style_t style_line;
    lv_style_init(&style_line);
    lv_style_set_line_width(&style_line, 2);
    lv_style_set_line_color(&style_line, lv_color_black());

    /*Create a line and apply the new style*/
    lv_obj_t * line;
    line = lv_line_create(container);
    lv_line_set_points(line, line_points, 2);     /*Set the points*/
    lv_obj_add_style(line, &style_line, 0);
    lv_obj_align_to(line, label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 2);

    /*Create a container with COLUMN flex direction*/
    lv_obj_t * cont_imu = lv_obj_create(container);
    lv_obj_add_event_cb(sw, sw_event_cb, LV_EVENT_VALUE_CHANGED, cont_imu);

    // Set the state of IMU switch and container
    if(imu_flag == 1)
    {
        lv_obj_add_state(sw, LV_STATE_CHECKED);
        lv_obj_clear_flag(cont_imu, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_clear_state(sw, LV_STATE_CHECKED);
        lv_obj_add_flag(cont_imu, LV_OBJ_FLAG_HIDDEN);
    }

    //lv_obj_set_width(cont_ges, LV_PCT(100));
    lv_obj_set_size(cont_imu, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_align_to(cont_imu, line, LV_ALIGN_OUT_BOTTOM_MID, 0, 2);
    lv_obj_set_flex_flow(cont_imu, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_left(cont_imu, 5, 0);
    //lv_obj_set_flex_align(cont_ges, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    btn = lv_btn_create(cont_imu);
    //lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);
    lv_obj_add_style(btn, &styel_btn, LV_STATE_DEFAULT);
    lv_group_add_obj(g, btn);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Gyroscope" LV_SYMBOL_RIGHT); 
    lv_obj_align(btn_label, LV_ALIGN_LEFT_MID, 0, 0);
     
     // Setting section for Gesture
    cont_title = lv_obj_create(container);
    lv_obj_set_size(cont_title, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(cont_title, LV_FLEX_FLOW_ROW);

    label = lv_label_create(cont_title);
    lv_label_set_text(label, "Gesture ");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
    //lv_obj_align_to(label, cont_ges, LV_ALIGN_OUT_BOTTOM_LEFT, 0,5);

    sw = lv_switch_create(cont_title);
    lv_group_add_obj(g, sw);
    lv_obj_add_style(sw, &style_sw_knob, LV_PART_KNOB);
    lv_obj_add_style(sw, &style_sw_knob_foc, LV_PART_KNOB | LV_STATE_FOCUSED);
    lv_obj_add_style(sw, &style_sw_main, LV_PART_MAIN);
    lv_obj_add_style(sw, &style_sw_indic, LV_PART_INDICATOR | LV_STATE_CHECKED);
    
    //lv_obj_add_state(sw, LV_STATE_CHECKED);
    //lv_obj_align_to(sw, label, LV_ALIGN_OUT_RIGHT_MID, 0, 0); 

    /*Create a line and apply the new style*/
    line = lv_line_create(container);
    lv_line_set_points(line, line_points, 2);     /*Set the points*/
    lv_obj_add_style(line, &style_line, 0);
    lv_obj_align_to(line, label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

    /*Create a container with COLUMN flex direction*/
    lv_obj_t * cont_ges = lv_obj_create(container);
    lv_obj_add_event_cb(sw, sw_event_cb, LV_EVENT_VALUE_CHANGED, cont_ges);

    if(ges_flag == 1)
    {
        lv_obj_add_state(sw, LV_STATE_CHECKED);
        lv_obj_clear_flag(cont_ges, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_clear_state(sw, LV_STATE_CHECKED);
        lv_obj_add_flag(cont_ges, LV_OBJ_FLAG_HIDDEN);
    }

    //lv_obj_set_width(cont_ges, LV_PCT(100));
    lv_obj_set_size(cont_ges, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_align_to(cont_ges, line, LV_ALIGN_OUT_BOTTOM_MID, 0, 2);
    lv_obj_set_flex_flow(cont_ges, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_left(cont_ges, 5, 0);
    //lv_obj_set_flex_align(cont_ges, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    btn = lv_btn_create(cont_ges);
    lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_group_add_obj(g, btn);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);
    lv_obj_add_style(btn, &styel_btn, LV_STATE_DEFAULT);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Wave Up " LV_SYMBOL_RIGHT); 
    //lv_obj_align(btn_label, LV_ALIGN_LEFT_MID, 0, 0);

    btn = lv_btn_create(cont_ges);
    lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_group_add_obj(g, btn);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);
    lv_obj_add_style(btn, &styel_btn, LV_STATE_DEFAULT);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Wave Down " LV_SYMBOL_RIGHT); 

    btn = lv_btn_create(cont_ges);
    lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_group_add_obj(g, btn);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);
    lv_obj_add_style(btn, &styel_btn, LV_STATE_DEFAULT);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Wave Left " LV_SYMBOL_RIGHT); 

    btn = lv_btn_create(cont_ges);
    lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_group_add_obj(g, btn);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);
    lv_obj_add_style(btn, &styel_btn, LV_STATE_DEFAULT);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Wave Right " LV_SYMBOL_RIGHT); 

    btn = lv_btn_create(cont_ges);
    lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_group_add_obj(g, btn);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);
    lv_obj_add_style(btn, &styel_btn, LV_STATE_DEFAULT);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Push Forward " LV_SYMBOL_RIGHT); 

    btn = lv_btn_create(cont_ges);
    lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_group_add_obj(g, btn);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);
    lv_obj_add_style(btn, &styel_btn, LV_STATE_DEFAULT);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Clockwise " LV_SYMBOL_RIGHT);

    btn = lv_btn_create(cont_ges);
    lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_group_add_obj(g, btn);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);
    lv_obj_add_style(btn, &styel_btn, LV_STATE_DEFAULT);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Counter-clock " LV_SYMBOL_RIGHT);

    // Setting section for TrackBall
    cont_title = lv_obj_create(container);
    lv_obj_set_size(cont_title, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(cont_title, LV_FLEX_FLOW_ROW);

    label = lv_label_create(cont_title);
    lv_label_set_text(label, "Trackball ");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
    //lv_obj_align_to(label, cont_ges, LV_ALIGN_OUT_BOTTOM_LEFT, 0,5);

    sw = lv_switch_create(cont_title);
    lv_group_add_obj(g, sw);
    lv_obj_add_style(sw, &style_sw_knob, LV_PART_KNOB);
    lv_obj_add_style(sw, &style_sw_knob_foc, LV_PART_KNOB | LV_STATE_FOCUSED);
    lv_obj_add_style(sw, &style_sw_main, LV_PART_MAIN);
    lv_obj_add_style(sw, &style_sw_indic, LV_PART_INDICATOR | LV_STATE_CHECKED);
    
    //lv_obj_add_state(sw, LV_STATE_CHECKED);
    //lv_obj_align_to(sw, label, LV_ALIGN_OUT_RIGHT_MID, 0, 0); 

    /*Create a line and apply the new style*/
    line = lv_line_create(container);
    lv_line_set_points(line, line_points, 2);     /*Set the points*/
    lv_obj_add_style(line, &style_line, 0);
    lv_obj_align_to(line, label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

    /*Create a container with COLUMN flex direction*/
    lv_obj_t * cont_track = lv_obj_create(container);
    lv_obj_add_event_cb(sw, sw_event_cb, LV_EVENT_VALUE_CHANGED, cont_track);

    if(tbk_flag == 1)
    {
        lv_obj_add_state(sw, LV_STATE_CHECKED);
        lv_obj_clear_flag(cont_track, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_clear_state(sw, LV_STATE_CHECKED);
        lv_obj_add_flag(cont_track, LV_OBJ_FLAG_HIDDEN);
    }

    //lv_obj_set_width(cont_track, LV_PCT(100));
    lv_obj_set_size(cont_track, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_align_to(cont_track, line, LV_ALIGN_OUT_BOTTOM_MID, 0, 2);
    lv_obj_set_flex_flow(cont_track, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_left(cont_track, 5, 0);
    //lv_obj_set_flex_align(cont_ges, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    btn = lv_btn_create(cont_track);
    lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_group_add_obj(g, btn);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);
    lv_obj_add_style(btn, &styel_btn, LV_STATE_DEFAULT);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Slide Up " LV_SYMBOL_RIGHT); 

    btn = lv_btn_create(cont_track);
    lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);
    lv_obj_add_style(btn, &styel_btn, LV_STATE_DEFAULT);
    lv_group_add_obj(g, btn);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Slide Down " LV_SYMBOL_RIGHT);

    btn = lv_btn_create(cont_track);
    lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);
    lv_obj_add_style(btn, &styel_btn, LV_STATE_DEFAULT);
    lv_group_add_obj(g, btn);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Slide Left " LV_SYMBOL_RIGHT);

    btn = lv_btn_create(cont_track);
    lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);
    lv_obj_add_style(btn, &styel_btn, LV_STATE_DEFAULT);
    lv_group_add_obj(g, btn);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Slide Right " LV_SYMBOL_RIGHT);

    btn = lv_btn_create(cont_track);
    lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);
    lv_obj_add_style(btn, &styel_btn, LV_STATE_DEFAULT);
    lv_group_add_obj(g, btn);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Press " LV_SYMBOL_RIGHT);
}

void btn_event_cb(lv_event_t * e) {
    lv_obj_t * obj = lv_event_get_target(e);
    lv_event_code_t code = lv_event_get_code(e);
    int data = lv_event_get_user_data(e);
    
    if(code == LV_EVENT_CLICKED) {
        // 处理按钮点击事件
        ESP_LOGI(MODE_SETTING_UI_TAG, "*Button %d is clicked*.", data);
    }
}

void ui_demo()
 {
     lv_obj_t *label_title,  *label1, *label2, *label3 ;

    //  static lv_style_t my_style;
    //  lv_style_init(&my_style);

    // // 设置样式的文本颜色
    // lv_style_set_text_color(&my_style, lv_color_white());

    label_title = lv_label_create(lv_scr_act());
    lv_label_set_text(label_title, " Air Mouse ");


    /*Create an array for the points of the line*/
    static lv_point_t line_points[] = {{0, 15}, {79, 15}};

    /*Create style*/
    static lv_style_t style_line;
    lv_style_init(&style_line);
    lv_style_set_line_width(&style_line, 2);
    lv_style_set_line_color(&style_line, lv_color_black());

    /*Create a line and apply the new style*/
    lv_obj_t * line1;
    line1 = lv_line_create(lv_scr_act());
    lv_line_set_points(line1, line_points, 2);     /*Set the points*/
    lv_obj_add_style(line1, &style_line, 0);
    //lv_obj_center(line1);

    lv_obj_t * btn1 = lv_btn_create(lv_scr_act());
   // lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -20);
    //lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0); // 将标签居中

    lv_obj_t * btn2 = lv_btn_create(lv_scr_act());
   // lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 10);

    lv_obj_t * btn3 = lv_btn_create(lv_scr_act());
   // lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn3, LV_ALIGN_CENTER, 0, 40);

    
    label1 = lv_label_create(btn1);
    lv_label_set_text(label1, "IMU " LV_SYMBOL_RIGHT );
    //lv_obj_add_style(label1,  &my_style, LV_PART_MAIN );
    lv_obj_center(label1);

    label2 = lv_label_create(btn2);
    lv_label_set_text(label2, "Gesture " LV_SYMBOL_RIGHT );
    //lv_obj_add_style(label2,  &my_style, LV_PART_MAIN );
    lv_obj_center(label2);

    label3 = lv_label_create(btn3);
    lv_label_set_text(label3, "Track Ball " LV_SYMBOL_RIGHT );
    //lv_obj_add_style(label3,  &my_style, LV_PART_MAIN );
    lv_obj_center(label3);

    // static lv_style_t style_label_focused;
    // lv_style_init(&style_label_focused);
    // lv_style_set_text_color(&style_label_focused, lv_color_white()); // 设置文本颜色为白色

    // // 应用新样式到按钮内的标签
    // lv_obj_add_style(label1, &style_label_focused, LV_STATE_FOCUSED);
    // lv_obj_add_style(label2, &style_label_focused, LV_STATE_FOCUSED);
    // lv_obj_add_style(label3, &style_label_focused, LV_STATE_FOCUSED);

    static lv_style_t style_btn;
    lv_style_init(&style_btn);

    // 设置背景颜色为白色
    lv_style_set_bg_opa(&style_btn, LV_OPA_COVER);
    lv_style_set_bg_color(&style_btn, lv_color_white());
    lv_style_set_text_color(&style_btn, lv_color_black()); // 文本为白色

    // 设置边框颜色为黑色和边框宽度
    lv_style_set_border_color(&style_btn, lv_color_black());
    lv_style_set_border_width(&style_btn, 1);

    lv_style_set_width(&style_btn, 78);
    lv_style_set_height(&style_btn, 20);

    lv_obj_add_style(btn1,  &style_btn, 0);
    lv_obj_add_style(btn2,  &style_btn, 0);
    lv_obj_add_style(btn3,  &style_btn, 0);

    static lv_style_t style_focused;
    lv_style_init(&style_focused);

    // 为选中状态设置反向颜色
    lv_style_set_bg_color(&style_focused, lv_color_black()); // 背景为黑色
    lv_style_set_text_color(&style_focused, lv_color_white()); // 文本为白色
    //lv_style_set_border_color(&style_focused, lv_color_white()); // 边框为白色

    // 将新样式应用于按钮的选中状态
    lv_obj_add_style(btn1, &style_focused, LV_STATE_FOCUSED);
    lv_obj_add_style(btn2, &style_focused, LV_STATE_FOCUSED);
    lv_obj_add_style(btn3, &style_focused, LV_STATE_FOCUSED);

    lv_group_t * g = lv_group_create();
    lv_group_add_obj(g, btn1); // 将按钮添加到组
    lv_group_add_obj(g, btn2); // 将按钮添加到组
    lv_group_add_obj(g, btn3); // 将按钮添加到组

    lv_obj_add_event_cb(btn1, btn_event_cb, LV_EVENT_CLICKED, (void*)1);
    lv_obj_add_event_cb(btn2, btn_event_cb, LV_EVENT_CLICKED, (void*)2);
    lv_obj_add_event_cb(btn3, btn_event_cb, LV_EVENT_CLICKED, (void*)3);

    lv_indev_set_group(encoder_indev, g); // 将编码器和组关联

    //create_setting_ui();
    //lv_scr_load(scr_setting);

    create_actions_ui();
    lv_scr_load(scr_actions);

    //create_actions_slide_ui();
    //lv_scr_load(scr_acts_slide);
}