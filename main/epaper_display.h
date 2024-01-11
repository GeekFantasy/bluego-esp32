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
void epd_full_display_mode(spi_device_handle_t spi, int8_t mode_num);
void epd_power_on_to_partial_display(spi_device_handle_t spi);
void epd_partial_display_mode(spi_device_handle_t spi, int8_t mode);
void epd_deep_sleep(spi_device_handle_t spi);
void epd_partial_display_full_image(spi_device_handle_t spi, const uint8_t* data, int len);
void epd_partial_display_full_white(spi_device_handle_t spi);
void epd_full_display_full_white(spi_device_handle_t spi);

#endif // end of EPAPER_DISPLAY