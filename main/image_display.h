#ifndef IMAGE_DISPLAY
#define IMAGE_DISPLAY

#include "epaper_display.h"

void partial_display_work_mode(spi_device_handle_t spi, int8_t mode);
void full_display_work_mode(spi_device_handle_t spi, int8_t mode_num);

#endif  //End of IMAGE_DISPLAY