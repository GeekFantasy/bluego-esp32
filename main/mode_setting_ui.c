#include "mode_setting_ui.h"

#define MODE_SETTING_UI_TAG     "MODE SETTING"


static lv_indev_t * encoder_indev = NULL;
lv_obj_t * scr_setting = NULL;


void init_mode_setting_ui(lv_indev_t * enc_indev)
{
    encoder_indev = enc_indev;
}

static void sw_event_cb(lv_event_t * event)
{
    lv_obj_t * sw = lv_event_get_target(event); // 获取触发事件的开关对象
    lv_obj_t * layout = lv_event_get_user_data(event); // 获取传递给事件处理函数的用户数据，即 layout 对象

    if(lv_obj_has_state(sw, LV_STATE_CHECKED)) {
        lv_obj_clear_flag(layout, LV_OBJ_FLAG_HIDDEN); // 显示 layout
    } else {
        lv_obj_add_flag(layout, LV_OBJ_FLAG_HIDDEN); // 隐藏 layout
    }
}

void create_setting_ui()
{
    lv_obj_t * label;
    lv_obj_t * sw;
    lv_obj_t * btn;
    lv_obj_t * btn_label;
    lv_group_t * g = lv_group_create();
    lv_indev_set_group(encoder_indev, g);

    scr_setting = lv_obj_create(NULL);

    lv_obj_t * container = lv_obj_create(scr_setting);
    lv_obj_set_size(container, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_align(container, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);

    // Setting section for IMU
    lv_obj_t * cont_title = lv_obj_create(container);
    lv_obj_set_size(cont_title, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(cont_title, LV_FLEX_FLOW_ROW);

    label = lv_label_create(cont_title);
    lv_label_set_text(label, "IMU ");

    static lv_style_t style_sw_main;
    static lv_style_t style_sw_indic;
    static lv_style_t style_sw_knob;
    static lv_style_t style_sw_knob_foc;
    static lv_style_t styel_btn_focused;

    lv_style_init(&style_sw_main);
    lv_style_set_width(&style_sw_main, 25);
    lv_style_set_height(&style_sw_main, 13);
    lv_style_set_bg_opa(&style_sw_main, LV_OPA_COVER);
    lv_style_set_bg_color(&style_sw_main, lv_color_white());
    lv_style_set_radius(&style_sw_main, LV_RADIUS_CIRCLE); 
    lv_style_set_border_color(&style_sw_main, lv_color_black());
    lv_style_set_border_width(&style_sw_main, 1);

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
    lv_style_set_border_width(&styel_btn_focused, 2);
    lv_style_set_text_color(&styel_btn_focused, lv_color_white());
    
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

    //lv_obj_set_width(cont_ges, LV_PCT(100));
    lv_obj_set_size(cont_imu, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_align_to(cont_imu, line, LV_ALIGN_OUT_BOTTOM_MID, 0, 2);
    lv_obj_set_flex_flow(cont_imu, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_left(cont_imu, 5, 0);
    //lv_obj_set_flex_align(cont_ges, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    btn = lv_btn_create(cont_imu);
    //lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);
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

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Wave Up " LV_SYMBOL_RIGHT); 
    //lv_obj_align(btn_label, LV_ALIGN_LEFT_MID, 0, 0);

    btn = lv_btn_create(cont_ges);
    lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_group_add_obj(g, btn);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Wave Down " LV_SYMBOL_RIGHT); 

    btn = lv_btn_create(cont_ges);
    lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_group_add_obj(g, btn);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Wave Left " LV_SYMBOL_RIGHT); 

    btn = lv_btn_create(cont_ges);
    lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_group_add_obj(g, btn);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Wave Right " LV_SYMBOL_RIGHT); 

    // Setting section for TrackBall
    cont_title = lv_obj_create(container);
    lv_obj_set_size(cont_title, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(cont_title, LV_FLEX_FLOW_ROW);

    label = lv_label_create(cont_title);
    lv_label_set_text(label, "TrackBall ");
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

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Scroll Up " LV_SYMBOL_RIGHT); 

    btn = lv_btn_create(cont_track);
    lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);
    lv_group_add_obj(g, btn);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Scroll Down " LV_SYMBOL_RIGHT);

    btn = lv_btn_create(cont_track);
    lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);
    lv_group_add_obj(g, btn);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Scroll Left " LV_SYMBOL_RIGHT);

    btn = lv_btn_create(cont_track);
    lv_obj_set_size(btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_add_style(btn, &styel_btn_focused, LV_STATE_FOCUSED);
    lv_group_add_obj(g, btn);

    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Scroll Right " LV_SYMBOL_RIGHT);
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

    create_setting_ui();
    lv_scr_load(scr_setting);
}