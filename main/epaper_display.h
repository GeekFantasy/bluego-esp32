#ifndef EPAPER_DISPLAY
#define EPAPER_DISPLAY

#include "esp_system.h"
#include "driver/spi_master.h"

#define EPD_TAG      "E-Paper"
#define EPD_DIS_ARRAY     1280

#define EPD_WIDTH       80
#define EPD_HEIGHT      128

#define EPD_CS_PIN      25
#define EPD_RST_PIN     27
#define EPD_DC_PIN      26
#define EPD_BUSY_PIN    14 

#define EPD_MOSI_PIN    32
#define EPD_MISO_PIN    -1
#define EPD_CLK_PIN     33

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