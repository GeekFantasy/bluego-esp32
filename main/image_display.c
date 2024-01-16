#include "image_display.h"
#include "image_data.h"

const unsigned char *mode_image_array[] = 
{
    gImage_airmouse,
    gImage_gesture,
    gImage_trackball,
    gImage_custom1,
    gImage_custom2,
    gImage_poweringoff,
    gImage_poweringon,
    gImage_config
};


/// @brief Test display partial functions
/// @param spi 
/// Status: this is now working
void full_display_work_mode(spi_device_handle_t spi, int8_t mode)
{
    epd_init_full_display(spi);

    if(mode >=0 && mode <= 5)
        epd_full_display_image(spi, mode_image_array[mode], EPD_DIS_ARRAY);
    else
        epd_full_display_image(spi, mode_image_array[6], EPD_DIS_ARRAY);
}

/// @brief Need to init as partial display mode first before calling this funciton
/// @param spi 
/// @param mode 
void partial_display_work_mode(spi_device_handle_t spi, int8_t mode)
{
    if(mode >=0 && mode <= 5)
        epd_partial_display_full_image(spi, mode_image_array[mode], EPD_DIS_ARRAY);
    else
        epd_partial_display_full_image(spi, mode_image_array[6], EPD_DIS_ARRAY);
}


/// @brief Test display full functions
/// @param spi 
void test_full_display_image(spi_device_handle_t spi)
{
    epd_init_full_display(spi);
    //Delay(6000);

    epd_full_display_white(spi);
    //Delay(6000);

    // uint8_t rx = 0;
    // rx = epd_get_byte(spi, 0x11);
    // ESP_LOGI(EPD_TAG, "The data read from 0x11 before send data is: %x.", rx);
    epd_full_display_image(spi, g_image_naruto, sizeof(g_image_naruto));


    epd_full_display_image(spi, gImage_airmouse, sizeof(gImage_airmouse));


    epd_full_display_image(spi, gImage_gesture, sizeof(gImage_airmouse));


    epd_full_display_image(spi, gImage_trackball, sizeof(gImage_airmouse));


    epd_full_display_image(spi, gImage_custom1, sizeof(gImage_airmouse));


    epd_full_display_image(spi, gImage_custom2, sizeof(gImage_airmouse));

}