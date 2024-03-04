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
  0x00, 0x00, 0x00, 0x7f, 0xfc, 0x00, 0x00, 0x00, 0x7f, 0xc0, 
  0x00, 0x00, 0x03, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x0f, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x3f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x7f, 0x80, 0x03, 0xfe, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x01, 0xfe, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x03, 0xf8, 0x00, 0x00, 0x1f, 0x80, 0x00, 0x00, 0x00, 
  0x00, 0x07, 0xe0, 0x03, 0x80, 0x0f, 0xc0, 0x00, 0x00, 0x00, 
  0x00, 0x0f, 0xc0, 0x07, 0xc0, 0x03, 0xe0, 0x00, 0x00, 0x00, 
  0x00, 0x1f, 0x80, 0x07, 0xe0, 0x01, 0xf0, 0x00, 0x00, 0x00, 
  0x00, 0x3f, 0x00, 0x0f, 0xf0, 0x00, 0xf8, 0x00, 0x0c, 0x00, 
  0x00, 0x3e, 0x00, 0x1f, 0xf0, 0x00, 0x7c, 0x00, 0x33, 0x00, 
  0x00, 0x7c, 0x00, 0x3e, 0xf8, 0x00, 0x7c, 0x00, 0x60, 0x80, 
  0x00, 0x78, 0x00, 0x3e, 0x7c, 0x00, 0x3e, 0x00, 0x40, 0x80, 
  0x00, 0xf8, 0x00, 0x7f, 0xfc, 0x00, 0x1e, 0x00, 0x40, 0x00, 
  0x00, 0xf0, 0x00, 0x7f, 0xfe, 0x00, 0x1f, 0x00, 0x40, 0x80, 
  0x01, 0xe0, 0x00, 0x7f, 0xfe, 0x00, 0x0f, 0x00, 0x40, 0x80, 
  0x01, 0xe0, 0x00, 0x2a, 0xa8, 0x00, 0x0f, 0x00, 0x33, 0x00, 
  0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x0c, 0x00, 
  0x03, 0xc0, 0x0e, 0x07, 0xc0, 0x70, 0x07, 0x80, 0x00, 0x00, 
  0x03, 0xc0, 0x3e, 0x1f, 0xf0, 0xf8, 0x07, 0x80, 0x00, 0x00, 
  0x03, 0xc0, 0x7e, 0x3f, 0xf8, 0xfc, 0x03, 0xc0, 0x00, 0x00, 
  0x07, 0xc0, 0xfe, 0x7f, 0xfc, 0xff, 0x03, 0xc0, 0x00, 0x00, 
  0x03, 0x83, 0xfe, 0x7c, 0x3c, 0xff, 0x83, 0xc0, 0x00, 0x00, 
  0x07, 0x87, 0xfe, 0x78, 0x1e, 0xff, 0xc3, 0xc0, 0x00, 0x00, 
  0x07, 0x87, 0xde, 0xf0, 0x1e, 0xf7, 0xe3, 0xc0, 0x00, 0x00, 
  0x07, 0x8f, 0xde, 0x78, 0x1e, 0xf7, 0xe3, 0xc0, 0x00, 0x00, 
  0x07, 0x87, 0xfe, 0x78, 0x1e, 0xff, 0xc3, 0xc0, 0x00, 0x00, 
  0x03, 0xc3, 0xfe, 0x7c, 0x3c, 0xff, 0x83, 0xc0, 0x0f, 0x00, 
  0x07, 0x80, 0xfe, 0x7f, 0xfc, 0xff, 0x03, 0xc0, 0x10, 0x80, 
  0x03, 0xc0, 0x7e, 0x3f, 0xf8, 0xfc, 0x03, 0xc0, 0x20, 0x40, 
  0x03, 0xc0, 0x3e, 0x1f, 0xf0, 0xf8, 0x07, 0x80, 0x20, 0x40, 
  0x03, 0xc0, 0x0e, 0x07, 0xc0, 0x70, 0x07, 0x80, 0x20, 0x40, 
  0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x10, 0x40, 
  0x01, 0xe0, 0x00, 0x15, 0x50, 0x00, 0x0f, 0x80, 0x19, 0x80, 
  0x01, 0xe0, 0x00, 0x7f, 0xfc, 0x00, 0x0f, 0x00, 0x06, 0x00, 
  0x00, 0xf0, 0x00, 0x7f, 0xfe, 0x00, 0x1f, 0x00, 0x00, 0x00, 
  0x00, 0xf8, 0x00, 0x7f, 0xfc, 0x00, 0x1e, 0x00, 0x00, 0x00, 
  0x00, 0x78, 0x00, 0x3e, 0xbc, 0x00, 0x3e, 0x00, 0x00, 0x00, 
  0x00, 0x7c, 0x00, 0x3e, 0xf8, 0x00, 0x7c, 0x00, 0x00, 0x00, 
  0x00, 0x3e, 0x00, 0x1f, 0xf0, 0x00, 0x7c, 0x00, 0x00, 0x00, 
  0x00, 0x3f, 0x00, 0x0f, 0xf0, 0x00, 0xf8, 0x00, 0x00, 0x00, 
  0x00, 0x1f, 0x80, 0x07, 0xe0, 0x01, 0xf0, 0x00, 0x00, 0x00, 
  0x00, 0x0f, 0xc0, 0x07, 0xc0, 0x07, 0xe0, 0x00, 0x00, 0x00, 
  0x00, 0x07, 0xe0, 0x03, 0x80, 0x0f, 0xc0, 0x00, 0x04, 0x00, 
  0x00, 0x03, 0xf8, 0x00, 0x00, 0x1f, 0x80, 0x00, 0x1f, 0x00, 
  0x00, 0x01, 0xfe, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x3f, 0x80, 
  0x00, 0x00, 0xff, 0x80, 0x03, 0xfe, 0x00, 0x00, 0x3f, 0x80, 
  0x00, 0x00, 0x3f, 0xfe, 0xff, 0xf8, 0x00, 0x00, 0x7f, 0xc0, 
  0x00, 0x00, 0x0f, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x3f, 0x80, 
  0x00, 0x00, 0x03, 0xff, 0xff, 0x80, 0x00, 0x00, 0x3f, 0x80, 
  0x00, 0x00, 0x00, 0x3f, 0xfc, 0x00, 0x00, 0x00, 0x1f, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
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
  0x00, 0x7f, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0c, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x08, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0c, 0x79, 0xc3, 0xd1, 0x80, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x08, 0x22, 0x26, 0x4b, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0c, 0x60, 0x64, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x08, 0x23, 0xb4, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0c, 0x42, 0x24, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x08, 0x26, 0x66, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0c, 0x43, 0xa3, 0x99, 0x80, 0x00, 0x00, 0x0f, 0x00, 
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
