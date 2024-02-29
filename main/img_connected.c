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

#ifndef LV_ATTRIBUTE_IMG_IMG_CONNECTED
#define LV_ATTRIBUTE_IMG_IMG_CONNECTED
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMG_IMG_CONNECTED uint8_t img_connected_map[] = {
  0xfe, 0xfe, 0xfe, 0xff, 	/*Color of index 0*/
  0x0c, 0x0c, 0x0c, 0xff, 	/*Color of index 1*/

  0x00, 0x00, 0x00, 
  0x1f, 0xdf, 0xc0, 
  0x1f, 0xdf, 0xc0, 
  0x18, 0x00, 0xc0, 
  0x19, 0xfc, 0xc0, 
  0x19, 0xfc, 0xc0, 
  0x18, 0x00, 0xc0, 
  0x1f, 0xdf, 0xc0, 
  0x1f, 0xdf, 0xc0, 
  0x00, 0x00, 0x00, 
};

const lv_img_dsc_t img_connected = {
  .header.cf = LV_IMG_CF_INDEXED_1BIT,
  .header.always_zero = 0,
  .header.reserved = 0,
  .header.w = 20,
  .header.h = 10,
  .data_size = 38,
  .data = img_connected_map,
};
