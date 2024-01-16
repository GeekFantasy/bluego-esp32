#ifndef IMAGE_DISPLAY
#define IMAGE_DISPLAY

#include "epaper_display.h"

void epd_partial_display_mode(spi_device_handle_t spi, int8_t mode);
void epd_full_display_mode(spi_device_handle_t spi, int8_t mode_num);

#endif  //End of IMAGE_DISPLAY