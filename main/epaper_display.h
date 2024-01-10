#ifndef EPAPER_DISPLAY
#define EPAPER_DISPLAY

#include "esp_system.h"
#include "driver/spi_master.h"

#define EPD_TAG      "E-Paper"
#define EPD_DIS_ARRAY     1280

esp_err_t edp_init_spi_device(spi_device_handle_t *spi);
void epd_update_display(spi_device_handle_t spi);
void epd_init_full_display(spi_device_handle_t spi);
void epd_init_partial_display(spi_device_handle_t spi);
void epd_display_mode(spi_device_handle_t spi, uint8_t mode_num);

#endif // end of EPAPER_DISPLAY