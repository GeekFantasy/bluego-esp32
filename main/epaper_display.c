
#include "epaper_display.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "image_data.h"


#define EPD_WIDTH       80
#define EPD_HEIGHT      128
#define EPD_CS_PIN      15
#define EPD_RST_PIN     33
#define EPD_DC_PIN      12
#define EPD_BUSY_PIN    32

#define EPD_MOSI_PIN    13
#define EPD_MISo_PIN    -1
#define EPD_CLK_PIN     14

#define SET_PIN_HIGH(pin)   gpio_set_level(pin, 1)
#define SET_PIN_LOW(pin)    gpio_set_level(pin, 0)

#define Delay(t) vTaskDelay(t / portTICK_PERIOD_MS)

uint8_t old_data[EPD_DIS_ARRAY] = {0};
uint8_t old_data_p[256];
uint8_t old_data_a[256];
uint8_t old_data_b[256];
uint8_t old_data_c[256];
uint8_t old_data_d[256];
uint8_t old_data_e[256];

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

uint8_t edp_buff[1280] = {0xFF};

void epd_send_command(spi_device_handle_t spi, const uint8_t cmd)
{
    ESP_LOGI(EPD_TAG, "Entering epd_send_command() with cmd: 0x %hx", cmd);
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    
    int dc = (int)t.user;
    ESP_LOGD(EPD_TAG, "epd_send_command(), dc is : %x", dc);

    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
    ESP_LOGI(EPD_TAG, "Exiting epd_send_command().");
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

void epd_set_full_reg(spi_device_handle_t spi)
{
    ESP_LOGI(EPD_TAG, "Entering epd_set_full_reg().");

    epd_send_command(spi, 0x23);
    epd_send_data(spi, lut_w1, sizeof(lut_w1));
  
    epd_send_command(spi, 0x24);
    epd_send_data(spi, lut_w1, sizeof(lut_b1));

    ESP_LOGI(EPD_TAG, "Exiting epd_set_full_reg().");
}

void epd_reset(void)
{
    ESP_LOGI(EPD_TAG, "Entering epd_reset().");
    SET_PIN_LOW(EPD_RST_PIN);
    Delay(10);
    SET_PIN_HIGH(EPD_RST_PIN);
    Delay(10);
    ESP_LOGI(EPD_TAG, "Exiting epd_reset().");
}

void epd_wait_until_ilde(spi_device_handle_t spi)
{
    ESP_LOGI(EPD_TAG, "Entering epd_wait_until_ilde().");
    int busy;
    do{
        busy = gpio_get_level(EPD_BUSY_PIN);
        Delay(1);
    }while(!busy);
    Delay(10);
    ESP_LOGI(EPD_TAG, "Exiting epd_wait_until_ilde().");
}

void epd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(EPD_DC_PIN, dc);
}

void edp_lut_bw(spi_device_handle_t spi)
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
    for (size_t i = 0; i < EPD_DIS_ARRAY; i++)
    {
        epd_send_byte_data(spi, 0xFF);
    }

    epd_send_command(spi, 0x13);
    for (size_t i = 0; i < EPD_DIS_ARRAY; i++)
    {
        epd_send_byte_data(spi, data[i]);
        old_data[i] = data[i];
    }
    
    epd_update_display(spi);
}

/// @brief Partial refresh write address and data
/// @param spi 
/// @param x_start 
/// @param y_start 
/// @param datas_A 
/// @param datas_B 
/// @param datas_C 
/// @param datas_D 
/// @param datas_E 
/// @param num 
/// @param PART_COLUMN 
/// @param PART_LINE 
void epd_dis_part_ram(spi_device_handle_t spi, unsigned int x_start,unsigned int y_start,
                        const uint8_t * data_A, const uint8_t * data_B,
                        const uint8_t * data_C, const uint8_t * data_D,
                        const uint8_t * data_E, unsigned char num,
                        unsigned int PART_COLUMN, unsigned int PART_LINE)
{
    ESP_LOGI(EPD_TAG, "Entering epd_dis_part_ram()");
    unsigned int i, x_end, y_end;
    x_start = x_start - x_start % 8;
    x_end = x_start + PART_LINE - 1; 
    y_end = y_start + PART_COLUMN * num - 1;

    epd_init_partial(spi);
    epd_send_command(spi, 0x91);     //This command makes the display enter partial mode
    epd_send_command(spi, 0x90);     //resolution setting

    epd_send_byte_data(spi, x_start);
    epd_send_byte_data(spi, x_end);
    epd_send_byte_data(spi, y_start);
    epd_send_byte_data(spi, y_end);
    epd_send_byte_data(spi, 0x00);

    epd_send_command(spi, 0x10);
    if(part_flag == 1)
    {
        part_flag = 0;
        for (size_t i = 0; i < PART_COLUMN * PART_LINE * num / 8; i++)
        {
            epd_send_byte_data(spi, 0xFF);
        } 
    }
    else
    {
        for(i = 0; i < PART_COLUMN * PART_LINE / 8; i++)       
            epd_send_byte_data(spi, old_data_a[i]); 
        for(i = 0; i < PART_COLUMN * PART_LINE / 8; i++)       
            epd_send_byte_data(spi, old_data_b[i]); 
        for(i = 0; i < PART_COLUMN * PART_LINE / 8; i++)       
            epd_send_byte_data(spi, old_data_c[i]); 
        for(i = 0; i < PART_COLUMN * PART_LINE / 8; i++)       
            epd_send_byte_data(spi, old_data_d[i]); 
        for(i = 0; i < PART_COLUMN * PART_LINE / 8; i++)       
            epd_send_byte_data(spi, old_data_e[i]); 
    }

    epd_send_command(spi, 0x13);

    for(i = 0; i < PART_COLUMN * PART_LINE  / 8; i++)    
    {   
       epd_send_byte_data(spi, data_A[i]); 
       old_data_a[i] = data_A[i];     
    } 

    for(i = 0; i < PART_COLUMN * PART_LINE  / 8; i++)    
    {   
       epd_send_byte_data(spi, data_B[i]); 
       old_data_b[i] = data_B[i];     
    } 

    for(i=0; i < PART_COLUMN * PART_LINE  / 8; i++)    
    {   
       epd_send_byte_data(spi, data_C[i]); 
       old_data_c[i] = data_C[i];     
    } 

    for(i=0; i < PART_COLUMN * PART_LINE  / 8; i++)    
    {   
       epd_send_byte_data(spi, data_D[i]); 
       old_data_d[i] = data_D[i];     
    } 

    for(i=0; i < PART_COLUMN * PART_LINE  / 8; i++)    
    {   
       epd_send_byte_data(spi, data_E[i]); 
       old_data_e[i] = data_E[i];     
    } 

    epd_update_display(spi);
}

void epd_dis_part_ram_v2(spi_device_handle_t spi, unsigned int x_start,unsigned int y_start,
                        const uint8_t * data_A, const uint8_t * data_B,
                        const uint8_t * data_C, const uint8_t * data_D,
                        const uint8_t * data_E, unsigned char num,
                        unsigned int PART_COLUMN, unsigned int PART_LINE)
{
    ESP_LOGI(EPD_TAG, "Entering epd_dis_part_ram()");
    unsigned int i, x_end, y_end;
    x_start = x_start - x_start % 8;
    x_end = x_start + PART_LINE - 1; 
    y_end = y_start + PART_COLUMN * num - 1;

    // epd_init_partial(spi);       // move inital code out 
    epd_send_command(spi, 0x91);     //This command makes the display enter partial mode
    epd_send_command(spi, 0x90);     //resolution setting

    epd_send_byte_data(spi, x_start);
    epd_send_byte_data(spi, x_end);
    epd_send_byte_data(spi, y_start);
    epd_send_byte_data(spi, y_end);
    epd_send_byte_data(spi, 0x00);

    epd_send_command(spi, 0x10);
    if(part_flag == 1)
    {
        part_flag = 0;
        for (size_t i = 0; i < PART_COLUMN * PART_LINE * num / 8; i++)
        {
            epd_send_byte_data(spi, 0xFF);
        } 
    }
    else
    {
        for(i = 0; i < PART_COLUMN * PART_LINE / 8; i++)       
            epd_send_byte_data(spi, old_data_a[i]); 
        for(i = 0; i < PART_COLUMN * PART_LINE / 8; i++)       
            epd_send_byte_data(spi, old_data_b[i]); 
        for(i = 0; i < PART_COLUMN * PART_LINE / 8; i++)       
            epd_send_byte_data(spi, old_data_c[i]); 
        for(i = 0; i < PART_COLUMN * PART_LINE / 8; i++)       
            epd_send_byte_data(spi, old_data_d[i]); 
        for(i = 0; i < PART_COLUMN * PART_LINE / 8; i++)       
            epd_send_byte_data(spi, old_data_e[i]); 
    }

    epd_send_command(spi, 0x13);

    for(i = 0; i < PART_COLUMN * PART_LINE  / 8; i++)    
    {   
       epd_send_byte_data(spi, data_A[i]); 
       old_data_a[i] = data_A[i];     
    } 

    for(i = 0; i < PART_COLUMN * PART_LINE  / 8; i++)    
    {   
       epd_send_byte_data(spi, data_B[i]); 
       old_data_b[i] = data_B[i];     
    } 

    for(i=0; i < PART_COLUMN * PART_LINE  / 8; i++)    
    {   
       epd_send_byte_data(spi, data_C[i]); 
       old_data_c[i] = data_C[i];     
    } 

    for(i=0; i < PART_COLUMN * PART_LINE  / 8; i++)    
    {   
       epd_send_byte_data(spi, data_D[i]); 
       old_data_d[i] = data_D[i];     
    } 

    for(i=0; i < PART_COLUMN * PART_LINE  / 8; i++)    
    {   
       epd_send_byte_data(spi, data_E[i]); 
       old_data_e[i] = data_E[i];     
    } 

    epd_update_display(spi);
}

void epd_dis_part_time(spi_device_handle_t spi, unsigned int x_start,unsigned int y_start,
                        const uint8_t * datas_A, const uint8_t * datas_B,
                        const uint8_t * datas_C, const uint8_t * datas_D,
                        const uint8_t * datas_E, unsigned char num,
                        unsigned int PART_COLUMN, unsigned int PART_LINE)
{
    epd_dis_part_ram(spi, x_start, y_start, datas_A, datas_B, datas_C, datas_D, datas_E, num, PART_COLUMN, PART_LINE);
}

void epd_dis_part_time_v2(spi_device_handle_t spi, unsigned int x_start,unsigned int y_start,
                        const uint8_t * datas_A, const uint8_t * datas_B,
                        const uint8_t * datas_C, const uint8_t * datas_D,
                        const uint8_t * datas_E, unsigned char num,
                        unsigned int PART_COLUMN, unsigned int PART_LINE)
{
    ESP_LOGI(EPD_TAG, "***************Display new image on partial mode*********************.");
    epd_dis_part_ram_v2(spi, x_start, y_start, datas_A, datas_B, datas_C, datas_D, datas_E, num, PART_COLUMN, PART_LINE);
}

void epd_init(spi_device_handle_t spi)
{
    ESP_LOGI(EPD_TAG, "Entering epd_init().");
    epd_reset();
    epd_wait_until_ilde(spi);

    epd_send_command(spi, 0x00);
    epd_send_byte_data(spi, 0x5F);  // changed from 0x6F to 0x5F, and it's working now.

    epd_send_command(spi, 0x2A);
    epd_send_byte_data(spi, 0x00);
    epd_send_byte_data(spi, 0x00);

    epd_send_command(spi, 0x04);
    epd_wait_until_ilde(spi);

    epd_send_command(spi, 0x50);        //VCOM AND DATA INTERVAL SETTING
    epd_send_byte_data(spi, 0x97);      //WBmode:VBDF 17|D7 VBDW 97 VBDB 57   WBRmode:VBDF F7 VBDW 77 VBDB 37  VBDR B7

    ESP_LOGI(EPD_TAG, "Exiting epd_init().");
}

void epd_init_partial(spi_device_handle_t spi)
{
    ESP_LOGI(EPD_TAG, "Entering epd_init_partial().");
    epd_reset();
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

    edp_lut_bw(spi);

    epd_send_command(spi, 0x04);
    epd_wait_until_ilde(spi);

    ESP_LOGI(EPD_TAG, "Exiting epd_init_partial().");
}

//Enter the sleep mode and please do not delete it, otherwise it will reduce the lifespan of the screen.
void edp_deep_sleep(spi_device_handle_t spi)
{
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
    epd_wait_until_ilde(spi);
}

void epd_display_full_black(spi_device_handle_t spi)
{
    ESP_LOGI(EPD_TAG, "Entering epd_display_full_black().");

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

    ESP_LOGI(EPD_TAG, "Exiting epd_display_full_black().");
}


/// @brief Display full white on screen 
/// @param spi 
/// Status: Working well excepting the first black line emerging later after a while
void epd_display_full_white_v2(spi_device_handle_t spi)
{
    ESP_LOGI(EPD_TAG, "Entering epd_display_full_white_v2().");

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

    ESP_LOGI(EPD_TAG, "Exiting epd_display_full_white_v2().");
}

void epd_display_full_black_v2(spi_device_handle_t spi)
{
    ESP_LOGI(EPD_TAG, "Entering epd_display_full_black().");

    epd_send_command(spi, 0x10);
    epd_send_data(spi, edp_buff, sizeof(edp_buff));
    
    for (size_t i = 0; i < EPD_DIS_ARRAY; i++)
    {
       edp_buff[i] = 0x00;
    }

    epd_send_command(spi, 0x13);
    epd_send_data(spi, edp_buff, sizeof(edp_buff));    

    epd_update_display(spi);

    ESP_LOGI(EPD_TAG, "Exiting epd_display_full_black().");
}

/// @brief 
/// @param spi 
/// Status: same issue with epd_display_full_white_v2
void epd_display_full_white(spi_device_handle_t spi)
{
    ESP_LOGI(EPD_TAG, "Entering epd_display_full_white().");
    for (size_t i = 0; i < EPD_DIS_ARRAY; i++)
    {
        edp_buff[i] = 0xFF;
    }

    epd_send_command(spi, 0x10);
    epd_send_data(spi, edp_buff, sizeof(edp_buff));

    epd_send_command(spi, 0x13);
    epd_send_data(spi, edp_buff, sizeof(edp_buff));  

    epd_update_display(spi);
    ESP_LOGI(EPD_TAG, "Exiting epd_display_full_white().");
}

void epd_display_full_image(spi_device_handle_t spi, const uint8_t* data, int len)
{
    ESP_LOGI(EPD_TAG, "Entering epd_display_full_image().");
    for (size_t i = 0; i < EPD_DIS_ARRAY; i++)
    {
        edp_buff[i] = 0xFF;
    }
    
    epd_send_command(spi, 0x10);
    epd_send_data(spi, edp_buff, sizeof(edp_buff));

    epd_send_command(spi, 0x13);
    epd_send_data(spi, data, len);

    epd_update_display(spi);
    ESP_LOGI(EPD_TAG, "Exiting epd_display_full_image().");
}

uint8_t epd_get_byte(spi_device_handle_t spi, uint8_t cmd)
{
    ESP_LOGI(EPD_TAG, "Entering epd_get_byte().");
    epd_send_command(spi, cmd);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)1;
    t.rx_data[0] = 0, t.rx_data[1] = 0, t.rx_data[2] = 0, t.rx_data[3] = 0;

    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    assert( ret == ESP_OK );

    ESP_LOGI(EPD_TAG, "Exiting epd_get_byte().");

    return t.rx_data[0];
}

uint8_t epd_get_ic_status(spi_device_handle_t spi)
{
    uint8_t rx = 0;
    ESP_LOGI(EPD_TAG, "Entering epd_get_ic_status().");
    epd_send_command(spi, 0x71);

    rx = epd_get_byte(spi, 0x71);

    ESP_LOGI(EPD_TAG, "Exiting epd_get_ic_status().");

    return rx;
}

/// @brief Test display full functions
/// @param spi 
void edp_test_display_full_image(spi_device_handle_t spi)
{
    epd_init(spi);
    //Delay(6000);

    epd_display_full_white(spi);
    Delay(6000);

    // uint8_t rx = 0;
    // rx = epd_get_byte(spi, 0x11);
    // ESP_LOGI(EPD_TAG, "The data read from 0x11 before send data is: %x.", rx);

    epd_display_full_image(spi, g_image_naruto, sizeof(g_image_naruto));

    Delay(6000);
}

/// @brief Test display partial functions
/// @param spi 
/// Status: this is not working
void edp_test_display_partial_image(spi_device_handle_t spi)
{
    epd_init_partial(spi);
    //Delay(6000);

    epd_display_full_white(spi);
    Delay(6000);

    // uint8_t rx = 0;
    // rx = epd_get_byte(spi, 0x11);
    // ESP_LOGI(EPD_TAG, "The data read from 0x11 before send data is: %x.", rx);

    epd_display_full_image(spi, g_image_naruto, sizeof(g_image_naruto));

    Delay(6000);
}

/// @brief Test display partial functions
/// @param spi 
/// Status: this is not working
void edp_test_display_partial_image_v2(spi_device_handle_t spi)
{
    epd_init(spi);

    for (size_t i = 0; i < EPD_DIS_ARRAY; i++)
    {
        edp_buff[i] = 0xFF;
    }
    
    epd_set_raw_value_base_map(spi, edp_buff);
    Delay(2000);

    for (size_t i = 0; i < 10; i++)
    {
        epd_dis_part_time(spi, 24, 4, number[i], number[0], g_image_numdot, number[0], number[1], 5, 24, 32);
    }
    
    Delay(2000);
}

/// @brief Test display partial functions
/// @param spi 
/// Status: this is not working
void edp_test_display_partial_image_v3(spi_device_handle_t spi)
{
    epd_init(spi);

    for (size_t i = 0; i < EPD_DIS_ARRAY; i++)
    {
        edp_buff[i] = 0xFF;
    }
    
    epd_set_raw_value_base_map(spi, edp_buff);

    epd_init_partial(spi);  

    for (size_t i = 0; i < 10; i++)
    {
        epd_dis_part_time_v2(spi, 24, 4, number[i], number[0], g_image_numdot, number[0], number[1], 5, 24, 32);
    }
    
    Delay(2000);
}


void init_e_paper_display()
{
    ESP_LOGI(EPD_TAG, "Entering init_e_paper_display().");

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
    spi_device_handle_t spi;

    spi_bus_config_t buscfg={
        .miso_io_num = -1,
        .mosi_io_num = EPD_MOSI_PIN,
        .sclk_io_num = EPD_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 128*80*2+8,
        .flags = SPICOMMON_BUSFLAG_MASTER ,
        .intr_flags = 0
    };

    spi_device_interface_config_t devcfg={
        .clock_speed_hz = 1*1000*1000,           //Clock out at 4 MHz
        .mode = 0,                                //SPI mode 3, 0 and 3 seems both working well
        .spics_io_num = EPD_CS_PIN,              //CS pin
        .queue_size = 1,                          //We want to be able to queue 7 transactions at a time
        .flags = SPI_DEVICE_3WIRE ,
        .pre_cb = epd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    edp_test_display_partial_image_v3(spi);
    // Test code

    edp_deep_sleep(spi);
    ESP_LOGI(EPD_TAG, "Exiting init_e_paper_display().");
}
