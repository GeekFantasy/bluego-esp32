#ifdef __has_include
    #if __has_include("lvgl.h")
        #ifndef LV_LVGL_H_INCLUDE_SIMPLE
            #define LV_LVGL_H_INCLUDE_SIMPLE
        #endif
    #endif
#endif

#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif


#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_IMG_IMG_TRACKBALL
#define LV_ATTRIBUTE_IMG_IMG_TRACKBALL
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMG_IMG_TRACKBALL uint8_t img_trackball_map[] = {
  0xfb, 0xfb, 0xfb, 0xff, 	/*Color of index 0*/
  0x20, 0x1f, 0x1e, 0xff, 	/*Color of index 1*/

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x80, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xc0, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x3f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x01, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x07, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x1f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x3f, 0xc0, 0x01, 0xff, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x00, 0x00, 
  0x00, 0x01, 0xfc, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x0c, 0x00, 
  0x00, 0x03, 0xf0, 0x01, 0xc0, 0x07, 0xe0, 0x00, 0x33, 0x00, 
  0x00, 0x07, 0xe0, 0x03, 0xe0, 0x01, 0xf0, 0x00, 0x60, 0x80, 
  0x00, 0x0f, 0xc0, 0x03, 0xf0, 0x00, 0xf8, 0x00, 0x40, 0x80, 
  0x00, 0x1f, 0x80, 0x07, 0xf8, 0x00, 0x7c, 0x00, 0x40, 0x00, 
  0x00, 0x1f, 0x00, 0x0f, 0xf8, 0x00, 0x3e, 0x00, 0x40, 0x80, 
  0x00, 0x3e, 0x00, 0x1f, 0x7c, 0x00, 0x3e, 0x00, 0x40, 0x80, 
  0x00, 0x3c, 0x00, 0x1f, 0x3e, 0x00, 0x1f, 0x00, 0x33, 0x00, 
  0x00, 0x7c, 0x00, 0x3f, 0xfe, 0x00, 0x0f, 0x00, 0x0c, 0x00, 
  0x00, 0x78, 0x00, 0x3f, 0xff, 0x00, 0x0f, 0x80, 0x00, 0x00, 
  0x00, 0xf0, 0x00, 0x3f, 0xff, 0x00, 0x07, 0x80, 0x00, 0x00, 
  0x00, 0xf0, 0x00, 0x15, 0x54, 0x00, 0x07, 0x80, 0x00, 0x00, 
  0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 
  0x01, 0xe0, 0x07, 0x03, 0xe0, 0x38, 0x03, 0xc0, 0x00, 0x00, 
  0x01, 0xe0, 0x1f, 0x0f, 0xf8, 0x7c, 0x03, 0xc0, 0x00, 0x00, 
  0x01, 0xe0, 0x3f, 0x1f, 0xfc, 0x7e, 0x01, 0xe0, 0x00, 0x00, 
  0x01, 0xe0, 0x7f, 0x3f, 0xfe, 0x7f, 0x81, 0xe0, 0x00, 0x00, 
  0x03, 0xc1, 0xff, 0x3e, 0x1e, 0x7f, 0xc1, 0xe0, 0x00, 0x00, 
  0x03, 0xc3, 0xff, 0x3c, 0x0f, 0x7f, 0xe1, 0xe0, 0x0f, 0x00, 
  0x03, 0xc3, 0xef, 0x78, 0x0f, 0x7b, 0xf1, 0xe0, 0x10, 0x80, 
  0x03, 0xc7, 0xef, 0x3c, 0x0f, 0x7b, 0xf1, 0xe0, 0x20, 0x40, 
  0x03, 0xc3, 0xff, 0x3c, 0x0f, 0x7f, 0xe1, 0xe0, 0x20, 0x40, 
  0x01, 0xe1, 0xff, 0x3e, 0x1e, 0x7f, 0xc1, 0xe0, 0x20, 0x40, 
  0x03, 0xc0, 0x7f, 0x3f, 0xfe, 0x7f, 0x81, 0xe0, 0x10, 0x40, 
  0x01, 0xe0, 0x3f, 0x1f, 0xfc, 0x7e, 0x01, 0xe0, 0x19, 0x80, 
  0x01, 0xe0, 0x1f, 0x0f, 0xf8, 0x7c, 0x03, 0xc0, 0x06, 0x00, 
  0x01, 0xe0, 0x07, 0x03, 0xe0, 0x38, 0x03, 0xc0, 0x00, 0x00, 
  0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 
  0x00, 0xf0, 0x00, 0x0a, 0xa8, 0x00, 0x07, 0xc0, 0x00, 0x00, 
  0x00, 0xf0, 0x00, 0x3f, 0xfe, 0x00, 0x07, 0x80, 0x00, 0x00, 
  0x00, 0x78, 0x00, 0x3f, 0xff, 0x00, 0x0f, 0x80, 0x00, 0x00, 
  0x00, 0x7c, 0x00, 0x3f, 0xfe, 0x00, 0x0f, 0x00, 0x00, 0x00, 
  0x00, 0x3c, 0x00, 0x3f, 0x5e, 0x00, 0x1f, 0x00, 0x00, 0x00, 
  0x00, 0x3e, 0x00, 0x1f, 0x7c, 0x00, 0x3e, 0x00, 0x00, 0x00, 
  0x00, 0x1f, 0x00, 0x0f, 0xf8, 0x00, 0x3e, 0x00, 0x04, 0x00, 
  0x00, 0x1f, 0x80, 0x07, 0xf8, 0x00, 0x7c, 0x00, 0x1f, 0x00, 
  0x00, 0x0f, 0xc0, 0x03, 0xf0, 0x00, 0xf8, 0x00, 0x3f, 0x80, 
  0x00, 0x07, 0xe0, 0x03, 0xe0, 0x03, 0xf0, 0x00, 0x3f, 0x80, 
  0x00, 0x03, 0xf0, 0x01, 0xc0, 0x07, 0xe0, 0x00, 0x7f, 0xc0, 
  0x00, 0x01, 0xfc, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x3f, 0x80, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x3f, 0x80, 
  0x00, 0x00, 0x7f, 0xc0, 0x01, 0xff, 0x00, 0x00, 0x1f, 0x00, 
  0x00, 0x00, 0x1f, 0xff, 0x7f, 0xfc, 0x00, 0x00, 0x04, 0x00, 
  0x00, 0x00, 0x07, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x01, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x1f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x80, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x40, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x40, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x40, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x80, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x80, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 
  0x00, 0x1f, 0x80, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x06, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x04, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x06, 0x3c, 0xe1, 0xe8, 0xc0, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x04, 0x11, 0x13, 0x25, 0x80, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x06, 0x30, 0x32, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x04, 0x11, 0xda, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x06, 0x21, 0x12, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x04, 0x13, 0x33, 0x25, 0x80, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x06, 0x21, 0xd1, 0xc8, 0xc0, 0x00, 0x00, 0x0f, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x80, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x40, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x40, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x40, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x40, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x80, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x07, 0x80, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0c, 0x40, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x04, 0x40, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0c, 0xc7, 0x93, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x07, 0x80, 0x99, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0c, 0x40, 0xd3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x04, 0x67, 0x99, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0c, 0x28, 0xd3, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xc0, 
  0x00, 0x04, 0x48, 0x99, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xc0, 
  0x00, 0x0f, 0x8f, 0xd3, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

const lv_img_dsc_t img_trackball = {
  .header.cf = LV_IMG_CF_INDEXED_1BIT,
  .header.always_zero = 0,
  .header.reserved = 0,
  .header.w = 80,
  .header.h = 114,
  .data_size = 1148,
  .data = img_trackball_map,
};
