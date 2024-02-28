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

#ifndef LV_ATTRIBUTE_IMG_GESTURE
#define LV_ATTRIBUTE_IMG_GESTURE
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMG_GESTURE uint8_t gesture_map[] = {
  0xfc, 0xfc, 0xfb, 0xff, 	/*Color of index 0*/
  0x39, 0x38, 0x37, 0xff, 	/*Color of index 1*/

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x03, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 
  0x00, 0x00, 0x07, 0xff, 0x80, 0x00, 0x00, 0x00, 0x1c, 0x00, 
  0x00, 0x00, 0x0f, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x3f, 0x00, 
  0x00, 0x00, 0x3f, 0x03, 0xe0, 0x00, 0x00, 0x00, 0x7f, 0x80, 
  0x00, 0x00, 0x3c, 0x00, 0xf0, 0x00, 0x00, 0x00, 0xff, 0xc0, 
  0x00, 0x00, 0x78, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x70, 0x7c, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0xe1, 0xfe, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0xe3, 0xff, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x01, 0xc3, 0xff, 0x8e, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x01, 0xc7, 0xff, 0x8e, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x01, 0xcf, 0xc7, 0xce, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x01, 0x8f, 0x87, 0xc6, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x03, 0xcf, 0x03, 0xce, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x01, 0x8f, 0x03, 0xc6, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x8f, 0x03, 0xc2, 0x00, 0x00, 0x00, 0x1e, 0x00, 
  0x00, 0x00, 0x0f, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x31, 0x00, 
  0x00, 0x00, 0x0f, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x40, 0x80, 
  0x00, 0x00, 0x0f, 0x03, 0xfe, 0x00, 0x00, 0x00, 0x40, 0x80, 
  0x00, 0x00, 0x0f, 0x03, 0xff, 0x00, 0x00, 0x00, 0x40, 0x80, 
  0x00, 0x00, 0x0f, 0x03, 0xff, 0xf0, 0x00, 0x00, 0x40, 0x80, 
  0x00, 0x00, 0x0f, 0x03, 0xff, 0xfc, 0x00, 0x00, 0x33, 0x00, 
  0x00, 0x00, 0x0f, 0x03, 0xcf, 0xfe, 0x00, 0x00, 0x1e, 0x00, 
  0x00, 0x00, 0x0f, 0x03, 0xc7, 0xff, 0xe0, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x0f, 0x03, 0xc7, 0xff, 0xf8, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x0f, 0x03, 0xc7, 0x9f, 0xfc, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x0f, 0x03, 0xc7, 0x8f, 0xfc, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x0f, 0x03, 0xc7, 0x8f, 0xfe, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x0f, 0x03, 0xc7, 0x8f, 0x3e, 0x00, 0x00, 0x00, 
  0x00, 0x07, 0xcf, 0x03, 0xc7, 0x8f, 0x1e, 0x00, 0x00, 0x00, 
  0x00, 0x0f, 0xff, 0x03, 0xc7, 0x8f, 0x1e, 0x00, 0x00, 0x00, 
  0x00, 0x1f, 0xff, 0x03, 0xc7, 0x8f, 0x1e, 0x00, 0x04, 0x00, 
  0x00, 0x3f, 0xff, 0x03, 0xc7, 0x8f, 0x1e, 0x00, 0x1f, 0x00, 
  0x00, 0x7f, 0xff, 0x03, 0xc7, 0x8f, 0x1e, 0x00, 0x3f, 0x80, 
  0x00, 0x7c, 0x7f, 0x03, 0xc7, 0x8f, 0x1e, 0x00, 0x3f, 0xc0, 
  0x00, 0x78, 0x3f, 0x03, 0xc7, 0x8f, 0x1e, 0x00, 0x7f, 0x80, 
  0x00, 0x78, 0x1f, 0x01, 0x83, 0x06, 0x1e, 0x00, 0x3f, 0xc0, 
  0x00, 0x78, 0x0f, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x3f, 0x80, 
  0x00, 0x78, 0x06, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x1f, 0x00, 
  0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x0a, 0x00, 
  0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 
  0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 
  0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 
  0x00, 0x1f, 0x80, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 
  0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 
  0x00, 0x0f, 0xc0, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 
  0x00, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 
  0x00, 0x03, 0xf0, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 
  0x00, 0x01, 0xf8, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x0e, 0x00, 
  0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x35, 0x00, 
  0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x61, 0x80, 
  0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x40, 0x80, 
  0x00, 0x00, 0x1f, 0x80, 0x00, 0x00, 0x3c, 0x00, 0x40, 0x00, 
  0x00, 0x00, 0x0f, 0xc0, 0x00, 0x00, 0x3c, 0x00, 0x40, 0x80, 
  0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x7c, 0x00, 0x60, 0x80, 
  0x00, 0x00, 0x03, 0xf0, 0x00, 0x00, 0xf8, 0x00, 0x33, 0x80, 
  0x00, 0x00, 0x03, 0xf8, 0x00, 0x01, 0xf8, 0x00, 0x1e, 0x00, 
  0x00, 0x00, 0x01, 0xfe, 0x00, 0x03, 0xf0, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0xff, 0x80, 0x1f, 0xf0, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x03, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xbf, 0xe0, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x80, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x80, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x80, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x80, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0c, 0x10, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x04, 0x10, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0c, 0x30, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x04, 0x11, 0xc7, 0xc3, 0xe0, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0f, 0xf2, 0x66, 0x6c, 0x60, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x04, 0x10, 0x64, 0x6c, 0x60, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0c, 0x33, 0xe4, 0x28, 0x20, 0x00, 0x00, 0x1e, 0x00, 
  0x00, 0x04, 0x16, 0x24, 0x6c, 0x60, 0x00, 0x00, 0x21, 0x00, 
  0x00, 0x0c, 0x32, 0x64, 0x2c, 0x60, 0x00, 0x00, 0x60, 0x80, 
  0x00, 0x04, 0x13, 0xe4, 0x67, 0xe0, 0x00, 0x00, 0x40, 0x80, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x80, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61, 0x80, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x06, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x04, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0c, 0x01, 0xc7, 0x7d, 0x89, 0xe7, 0x00, 0x00, 0x00, 
  0x00, 0x08, 0x06, 0x64, 0x91, 0x09, 0x99, 0x80, 0x00, 0x00, 
  0x00, 0x08, 0xf6, 0x34, 0x19, 0x89, 0x18, 0x80, 0x7f, 0xc0, 
  0x00, 0x0c, 0x37, 0xe7, 0x11, 0x09, 0x1f, 0xc0, 0x7f, 0x80, 
  0x00, 0x0c, 0x16, 0x01, 0x99, 0x89, 0x18, 0x00, 0x1f, 0x00, 
  0x00, 0x06, 0x32, 0x28, 0x91, 0x99, 0x08, 0x80, 0x1e, 0x00, 
  0x00, 0x03, 0xe3, 0xc7, 0x9c, 0xf9, 0x0f, 0x00, 0x0c, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

const lv_img_dsc_t gesture = {
  .header.cf = LV_IMG_CF_INDEXED_1BIT,
  .header.always_zero = 0,
  .header.reserved = 0,
  .header.w = 80,
  .header.h = 128,
  .data_size = 1288,
  .data = gesture_map,
};
