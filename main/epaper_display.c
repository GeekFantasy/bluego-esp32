
#include "epaper_display.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
 
#define SET_PIN_HIGH(pin)   gpio_set_level(pin, 1)
#define SET_PIN_LOW(pin)    gpio_set_level(pin, 0)

#define Delay(t) vTaskDelay(t / portTICK_PERIOD_MS)

uint8_t old_data[EPD_DIS_ARRAY] = {0};
uint8_t epd_buff[1280] = {0xFF};
int old_data_init_flag = 0;
uint8_t part_flag = 1;

/**
 * full screen update LUT
**/
const unsigned char lut_w1[] =
{
0x60	,0x5A	,0x5A	,0x00	,0x00	,0x01	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
 	
};	
const unsigned char lut_b1[] =
{
0x90	,0x5A	,0x5A	,0x00	,0x00	,0x01	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
};
/**
 * partial screen update LUT
**/
const uint8_t lut_w[] =
{
0x60  ,0x01 ,0x01 ,0x00 ,0x00 ,0x01 ,
0x80  ,0x0f ,0x00 ,0x00 ,0x00 ,0x01 ,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,

};  
const uint8_t lut_b[] =
{
0x90  ,0x01 ,0x01 ,0x00 ,0x00 ,0x01 ,
0x40  ,0x0f ,0x00 ,0x00 ,0x00 ,0x01 ,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,

};

void epd_send_command(spi_device_handle_t spi, const uint8_t cmd)
{
    ESP_LOGD(EPD_TAG, "Entering epd_send_command() with cmd: 0x %hx", cmd);
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));           //Zero out the transaction
    t.length = 8;                       //Command is 8 bits
    t.tx_buffer = &cmd;                 //The data is the cmd itself
    t.user = (void*)0;                  //D/C needs to be set to 0
    
    int dc = (int)t.user;
    ESP_LOGD(EPD_TAG, "epd_send_command(), dc is : %x", dc);

    ret = spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret == ESP_OK);            //Should have had no issues.
}

void epd_send_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = len * 8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;               //Data
    t.user = (void*)1;                //D/C needs to be set to 1

    // int dc = (int)t.user;
    // ESP_LOGD(EPD_TAG, "epd_send_data(), dc is : %x", dc);

    ret = spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret == ESP_OK);            //Should have had no issues.
}

void epd_send_byte_data(spi_device_handle_t spi, const uint8_t data)
{
    epd_send_data(spi, &data, 1);
}

void epd_set_lut_reg(spi_device_handle_t spi)
{
    ESP_LOGD(EPD_TAG, "Entering epd_set_lut_reg().");

    epd_send_command(spi, 0x23);
    epd_send_data(spi, lut_w1, sizeof(lut_w1));
  
    epd_send_command(spi, 0x24);
    epd_send_data(spi, lut_w1, sizeof(lut_b1));
}

void epd_reset_device(void)
{
    ESP_LOGD(EPD_TAG, "Entering epd_reset_device().");
    SET_PIN_LOW(EPD_RST_PIN);
    Delay(10);
    SET_PIN_HIGH(EPD_RST_PIN);
    Delay(10);
}

void epd_wait_until_ilde(spi_device_handle_t spi)
{
    ESP_LOGD(EPD_TAG, "Entering epd_wait_until_ilde().");
    int busy;
    do{
        busy = gpio_get_level(EPD_BUSY_PIN);
        Delay(1);
    }while(!busy);
    Delay(5);
}

void epd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(EPD_DC_PIN, dc);
}

void epd_lut_bw(spi_device_handle_t spi)
{
    epd_send_command(spi, 0x23);
    for (size_t i = 0; i < 42; i++)
    {
        epd_send_byte_data(spi, lut_w[i]);
    }
    
    epd_send_command(spi, 0x24);
    for (size_t i = 0; i < 42; i++)
    {
        epd_send_byte_data(spi, lut_b[i]);
    }
}

void epd_set_raw_value_base_map(spi_device_handle_t spi, const uint8_t* data )
{
    epd_send_command(spi, 0x10);

    memset(epd_buff, 0xFF, sizeof(epd_buff));
    epd_send_data(spi, epd_buff, sizeof(epd_buff));

    epd_send_command(spi, 0x13);
    epd_send_data(spi, data, EPD_DIS_ARRAY);
    memcpy(old_data, data, EPD_DIS_ARRAY);

    epd_update_display(spi);
}

void epd_init_full_display(spi_device_handle_t spi)
{
    ESP_LOGD(EPD_TAG, "Entering epd_init_full_display().");
    epd_reset_device();
    epd_wait_until_ilde(spi);

    epd_send_command(spi, 0x00);
    epd_send_byte_data(spi, 0x5F);      //changed from 0x6F to 0x5F, and it's working now.

    epd_send_command(spi, 0x2A);
    epd_send_byte_data(spi, 0x00);
    epd_send_byte_data(spi, 0x00);

    epd_send_command(spi, 0x04);
    epd_wait_until_ilde(spi);

    epd_send_command(spi, 0x50);        //VCOM AND DATA INTERVAL SETTING
    epd_send_byte_data(spi, 0x97);      //WBmode:VBDF 17|D7 VBDW 97 VBDB 57   WBRmode:VBDF F7 VBDW 77 VBDB 37  VBDR B7

    if(!old_data_init_flag)
    {
        memset(old_data, 0xFF, sizeof(old_data));
        old_data_init_flag = 1;
    }
}

void epd_init_partial_display(spi_device_handle_t spi)
{
    ESP_LOGD(EPD_TAG, "Entering epd_init_partial_display().");
    epd_reset_device();
    epd_wait_until_ilde(spi);

    epd_send_command(spi, 0xD2);
    epd_send_byte_data(spi, 0x3F);

    epd_send_command(spi, 0x00);
    epd_send_byte_data(spi, 0x6F); 

    epd_send_command(spi, 0x01);
    epd_send_byte_data(spi, 0x03);
    epd_send_byte_data(spi, 0x00);
    epd_send_byte_data(spi, 0x26);
    epd_send_byte_data(spi, 0x26);

    epd_send_command(spi, 0x06);
    epd_send_byte_data(spi, 0x3F);

    epd_send_command(spi, 0x2A);
    epd_send_byte_data(spi, 0x00);
    epd_send_byte_data(spi, 0x00);

    epd_send_command(spi, 0x30);
    epd_send_byte_data(spi, 0x13);  //50Hz

    epd_send_command(spi, 0x50);
    epd_send_byte_data(spi, 0xF2);

    epd_send_command(spi, 0x60);
    epd_send_byte_data(spi, 0x22);

    epd_send_command(spi, 0x82);
    epd_send_byte_data(spi, 0x12);

    epd_send_command(spi, 0xE3);
    epd_send_byte_data(spi, 0x33);

    epd_lut_bw(spi);

    epd_send_command(spi, 0x04);
    epd_wait_until_ilde(spi);

    if(!old_data_init_flag)
    {
        memset(old_data, 0xFF, sizeof(old_data));
        old_data_init_flag = 1;
    }
}

//Enter the sleep mode and please do not delete it, otherwise it will reduce the lifespan of the screen.
void epd_deep_sleep(spi_device_handle_t spi)
{
    ESP_LOGI(EPD_TAG, "Entering epd_deep_sleep().");
    epd_wait_until_ilde(spi);
    epd_send_command(spi, 0x50);         //VCOM AND DATA INTERVAL SETTING 
    epd_send_byte_data(spi, 0x7F);       //WBmode:VBDF 17|D7 VBDW 97 VBDB 57    WBRmode:VBDF F7 VBDW 77 VBDB 37  VBDR B7

    epd_send_command(spi,0x02);         //power off
    epd_wait_until_ilde(spi);

    Delay(100);

    epd_send_command(spi, 0x07);         //deep sleep
    epd_send_byte_data(spi, 0xA5);
}

void epd_update_display(spi_device_handle_t spi)
{
    epd_send_command(spi, 0x12); //start refreshing the screen
    //epd_wait_until_ilde(spi);
}

void epd_full_display_black(spi_device_handle_t spi)
{
    ESP_LOGD(EPD_TAG, "Entering epd_full_display_black().");

    int Width = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1);
    epd_send_command(spi, 0x10);
    for (size_t i = 0; i < EPD_HEIGHT; i++)
    {
        for (size_t j = 0; j < Width; j++)
        {
            epd_send_byte_data(spi, 0xFF);
        }
    }

    epd_send_command(spi, 0x13);
    for (size_t i = 0; i < EPD_HEIGHT; i++)
    {
        for (size_t j = 0; j < Width; j++)
        {
            epd_send_byte_data(spi, 0x00);
        }
    }

    epd_update_display(spi);
}

/// @brief Display full white on screen 
/// @param spi 
/// Status: Working well excepting the first black line emerging later after a while
void epd_full_display_white_v2(spi_device_handle_t spi)
{
    ESP_LOGD(EPD_TAG, "Entering epd_full_display_white_v2().");

    int Width = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1);
    epd_send_command(spi, 0x10);
    for (size_t i = 0; i < EPD_HEIGHT; i++)
    {
        for (size_t j = 0; j < Width; j++)
        {
            epd_send_byte_data(spi, 0xFF);
        }
    }

    epd_send_command(spi, 0x13);
    for (size_t i = 0; i < EPD_HEIGHT; i++)
    {
        for (size_t j = 0; j < Width; j++)
        {
            epd_send_byte_data(spi, 0xFF);
        }
    }

    epd_update_display(spi);
}

void epd_full_display_black_v2(spi_device_handle_t spi)
{
    ESP_LOGD(EPD_TAG, "Entering epd_full_display_black_v2().");

    epd_send_command(spi, 0x10);
    epd_send_data(spi, epd_buff, sizeof(epd_buff));
    
    for (size_t i = 0; i < EPD_DIS_ARRAY; i++)
    {
       epd_buff[i] = 0x00;
    }

    epd_send_command(spi, 0x13);
    epd_send_data(spi, epd_buff, sizeof(epd_buff));    

    epd_update_display(spi);
}

/// @brief 
/// @param spi 
/// Status: same issue with epd_full_display_white_v2
void epd_full_display_white(spi_device_handle_t spi)
{
    ESP_LOGD(EPD_TAG, "Entering epd_full_display_white().");
    
    memset(epd_buff, 0xFF, sizeof(epd_buff));
    epd_send_command(spi, 0x10);
    epd_send_data(spi, epd_buff, sizeof(epd_buff));

    epd_send_command(spi, 0x13);
    epd_send_data(spi, epd_buff, sizeof(epd_buff));  

    epd_update_display(spi);
}

void epd_partial_display_white(spi_device_handle_t spi)
{
    ESP_LOGD(EPD_TAG, "Entering epd_partial_display_white.");
    memset(epd_buff, 0xFF, sizeof(epd_buff));
    epd_partial_display_full_image(spi, epd_buff, sizeof(epd_buff));
}

void epd_full_display_image(spi_device_handle_t spi, const uint8_t* data, int len)
{
    ESP_LOGD(EPD_TAG, "Entering epd_full_display_image().");
    
    memset(epd_buff, 0xFF, sizeof(epd_buff));
    epd_send_command(spi, 0x10);
    epd_send_data(spi, epd_buff, sizeof(epd_buff));

    epd_send_command(spi, 0x13);
    epd_send_data(spi, data, len);
    memcpy(old_data, data, len);

    epd_update_display(spi);
}

/// @brief Show full image in the partial display 
/// @param spi 
/// @param data 
/// @param len 
void epd_partial_display_full_image(spi_device_handle_t spi, const uint8_t* data, int len)
{
    ESP_LOGD(EPD_TAG, "Entering epd_full_display_image().");
    
    epd_wait_until_ilde(spi);

    epd_send_command(spi, 0x10);
    epd_send_data(spi, old_data, sizeof(old_data));

    epd_send_command(spi, 0x13);
    epd_send_data(spi, data, len);
    memcpy(old_data, data, len);
    
    epd_update_display(spi);
}

void epd_enter_partial_display(spi_device_handle_t spi)
{
    epd_wait_until_ilde(spi);
      
    epd_send_command(spi, 0x91);     //This command makes the display enter partial display
    epd_send_command(spi, 0x90);     //resolution setting

    epd_send_byte_data(spi, 0);
    epd_send_byte_data(spi, 79);
    epd_send_byte_data(spi, 0);
    epd_send_byte_data(spi, 127);
    epd_send_byte_data(spi, 0x00);
}

void epd_exit_partial_to_full_display(spi_device_handle_t spi)
{
    epd_send_command(spi, 0x92); 
}

/// @brief Power on the e-paer and entering partial display mode with full scale window 80*128
/// @param spi 
void epd_power_on_to_partial_display(spi_device_handle_t spi)
{
    epd_init_partial_display(spi);
    epd_enter_partial_display(spi);
}

uint8_t epd_get_byte(spi_device_handle_t spi, uint8_t cmd)
{
    ESP_LOGD(EPD_TAG, "Entering epd_get_byte().");
    epd_send_command(spi, cmd);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)1;
    t.rx_data[0] = 0, t.rx_data[1] = 0, t.rx_data[2] = 0, t.rx_data[3] = 0;

    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    assert( ret == ESP_OK );

    return t.rx_data[0];
}

uint8_t epd_get_ic_status(spi_device_handle_t spi)
{
    uint8_t rx = 0;
    ESP_LOGD(EPD_TAG, "Entering epd_get_ic_status().");
    epd_send_command(spi, 0x71);

    rx = epd_get_byte(spi, 0x71);
    return rx;
}

// /// @brief Test display partial functions
// /// @param spi 
// /// Status: this is now working
// void epd_test_partial_display_image_with_full_scale(spi_device_handle_t spi)
// {
//     epd_init_full_display(spi);

//     for (size_t i = 0; i < EPD_DIS_ARRAY; i++)
//     {
//         epd_buff[i] = 0xFF;
//     }
    
//     epd_set_raw_value_base_map(spi, epd_buff);
//     //Delay(2000);

//     epd_init_partial_display(spi);
//     epd_send_command(spi, 0x91);     //This command makes the display enter partial mode
//     epd_send_command(spi, 0x90);     //resolution setting

//     epd_send_byte_data(spi, 0);
//     epd_send_byte_data(spi, 79);
//     epd_send_byte_data(spi, 0);
//     epd_send_byte_data(spi, 127);
//     epd_send_byte_data(spi, 0x00);

//     // uint8_t rx = 0;
//     // rx = epd_get_byte(spi, 0x11);
//     // ESP_LOGI(EPD_TAG, "The data read from 0x11 before send data is: %x.", rx);

//     epd_partial_display_full_image(spi, gImage_airmouse, sizeof(gImage_airmouse));
//     //Delay(3000);

//     epd_partial_display_full_image(spi, gImage_gesture, sizeof(g_image_naruto));
//     //Delay(3000);

//     epd_partial_display_full_image(spi, gImage_trackball, sizeof(g_image_naruto));
//     //Delay(3000);

//     epd_partial_display_full_image(spi, gImage_custom1, sizeof(g_image_naruto));
//     //Delay(3000);

//     epd_partial_display_full_image(spi, gImage_custom2, sizeof(g_image_naruto));
//     //Delay(3000);
// }


/// @brief Test display partial functions
/// @param spi 
/// Status: this is not working
// void epd_test_partial_display_image_v2(spi_device_handle_t spi)
// {
//     epd_init_full_display(spi);

//     memset(epd_buff, 0xFF, sizeof(epd_buff)); 
//     epd_set_raw_value_base_map(spi, epd_buff);
//     Delay(2000);

//     for (size_t i = 0; i < 10; i++)
//     {
//         epd_dis_part_time(spi, 24, 4, number[i], number[0], g_image_numdot, number[0], number[1], 5, 24, 32);
//     }
    
//     Delay(2000);
// }

/// @brief Test display partial functions
/// @param spi 
/// Status: this is not working
// void epd_test_partial_display_image_v3(spi_device_handle_t spi)
// {
//     epd_init_full_display(spi);

//     memset(epd_buff, 0xFF, sizeof(epd_buff)); 
//     epd_set_raw_value_base_map(spi, epd_buff);

//     epd_init_partial_display(spi);  

//     for (size_t i = 0; i < 10; i++)
//     {
//         epd_dis_part_time_v2(spi, 24, 4, number[i], number[0], g_image_numdot, number[0], number[1], 5, 24, 32);
//     }
    
//     Delay(2000);
// }

esp_err_t  edp_init_spi_device(spi_device_handle_t *spi)
{
    ESP_LOGD(EPD_TAG, "Entering edp_init_spi_device().");

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = ((1ULL << EPD_CS_PIN) | (1ULL << EPD_RST_PIN) | (1ULL << EPD_DC_PIN));
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = 1ULL << EPD_BUSY_PIN;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    esp_err_t ret;
    //spi_device_handle_t spi;

    spi_bus_config_t buscfg={
        .miso_io_num = EPD_MISO_PIN,
        .mosi_io_num = EPD_MOSI_PIN,
        .sclk_io_num = EPD_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 128*80*2+8,
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .intr_flags = 0
    };

    spi_device_interface_config_t devcfg={
        .clock_speed_hz = 4 * 1000 * 1000,          //Clock out at 4 MHz
        .mode = 0,                                  //SPI mode 3, 0 and 3 seems both working well
        .spics_io_num = EPD_CS_PIN,                 //CS pin
        .queue_size = 7,                            //We want to be able to queue 7 transactions at a time
        .flags = SPI_DEVICE_3WIRE ,
        .pre_cb = epd_spi_pre_transfer_callback,    //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    //ESP_ERROR_CHECK(ret);
    
    //Attach the LCD to the SPI bus
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, spi);
    //ESP_ERROR_CHECK(ret);

    return ret;
}

