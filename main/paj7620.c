/*
 * paj7620.c
 * A library for Grove-Guesture 1.0
 *
 * Copyright (c) 2015 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : Wuruibin & Xiangnan
 * Modified Time: June 2015
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "paj7620.h"

typedef unsigned char uint8_t;

uint8_t rxBuffer[I2C_BUFFER_LENGTH];
size_t rxIndex = 0;
size_t rxLength = 0;

uint8_t txBuffer[I2C_BUFFER_LENGTH];
size_t txLength = 0;
uint16_t txAddress = 0;
int i2c_master_port = I2C_MASTER_NUM;

static int gesture_triggered = 0;

// PAJ7620U2_20140305.asc
/* Registers' initialization data */
unsigned char initRegisterArray[][2] = {	// Initial Gesture
    {0xEF,0x00},
	{0x32,0x29},
	{0x33,0x01},
	{0x34,0x00},
	{0x35,0x01},
	{0x36,0x00},
	{0x37,0x07},
	{0x38,0x17},
	{0x39,0x06},
	{0x3A,0x12},
	{0x3F,0x00},
	{0x40,0x02},
	{0x41,0xFF},
	{0x42,0x01},
	{0x46,0x2D},
	{0x47,0x0F},
	{0x48,0x3C},
	{0x49,0x00},
	{0x4A,0x1E},
	{0x4B,0x00},
	{0x4C,0x20},
	{0x4D,0x00},
	{0x4E,0x1A},
	{0x4F,0x14},
	{0x50,0x00},
	{0x51,0x10},
	{0x52,0x00},
	{0x5C,0x02},
	{0x5D,0x00},
	{0x5E,0x10},
	{0x5F,0x3F},
	{0x60,0x27},
	{0x61,0x28},
	{0x62,0x00},
	{0x63,0x03},
	{0x64,0xF7},
	{0x65,0x03},
	{0x66,0xD9},
	{0x67,0x03},
	{0x68,0x01},
	{0x69,0xC8},
	{0x6A,0x40},
	{0x6D,0x04},
	{0x6E,0x00},
	{0x6F,0x00},
	{0x70,0x80},
	{0x71,0x00},
	{0x72,0x00},
	{0x73,0x00},
	{0x74,0xF0},
	{0x75,0x00},
	{0x80,0x42},
	{0x81,0x44},
	{0x82,0x04},
	{0x83,0x20},
	{0x84,0x20},
	{0x85,0x00},
	{0x86,0x10},
	{0x87,0x00},
	{0x88,0x05},
	{0x89,0x18},
	{0x8A,0x10},
	{0x8B,0x01},
	{0x8C,0x37},
	{0x8D,0x00},
	{0x8E,0xF0},
	{0x8F,0x81},
	{0x90,0x06},
	{0x91,0x06},
	{0x92,0x1E},
	{0x93,0x0D},
	{0x94,0x0A},
	{0x95,0x0A},
	{0x96,0x0C},
	{0x97,0x05},
	{0x98,0x0A},
	{0x99,0x41},
	{0x9A,0x14},
	{0x9B,0x0A},
	{0x9C,0x3F},
	{0x9D,0x33},
	{0x9E,0xAE},
	{0x9F,0xF9},
	{0xA0,0x48},
	{0xA1,0x13},
	{0xA2,0x10},
	{0xA3,0x08},
	{0xA4,0x30},
	{0xA5,0x19},
	{0xA6,0x10},
	{0xA7,0x08},
	{0xA8,0x24},
	{0xA9,0x04},
	{0xAA,0x1E},
	{0xAB,0x1E},
	{0xCC,0x19},
	{0xCD,0x0B},
	{0xCE,0x13},
	{0xCF,0x64},
	{0xD0,0x21},
	{0xD1,0x0F},
	{0xD2,0x88},
	{0xE0,0x01},
	{0xE1,0x04},
	{0xE2,0x41},
	{0xE3,0xD6},
	{0xE4,0x00},
	{0xE5,0x0C},
	{0xE6,0x0A},
	{0xE7,0x00},
	{0xE8,0x00},
	{0xE9,0x00},
	{0xEE,0x07},
	{0xEF,0x01},
	{0x00,0x1E},
	{0x01,0x1E},
	{0x02,0x0F},
	{0x03,0x10},
	{0x04,0x02},
	{0x05,0x00},
	{0x06,0xB0},
	{0x07,0x04},
	{0x08,0x0D},
	{0x09,0x0E},
	{0x0A,0x9C},
	{0x0B,0x04},
	{0x0C,0x05},
	{0x0D,0x0F},
	{0x0E,0x02},
	{0x0F,0x12},
	{0x10,0x02},
	{0x11,0x02},
	{0x12,0x00},
	{0x13,0x01},
	{0x14,0x05},
	{0x15,0x07},
	{0x16,0x05},
	{0x17,0x07},
	{0x18,0x01},
	{0x19,0x04},
	{0x1A,0x05},
	{0x1B,0x0C},
	{0x1C,0x2A},
	{0x1D,0x01},
	{0x1E,0x00},
	{0x21,0x00},
	{0x22,0x00},
	{0x23,0x00},
	{0x25,0x01},
	{0x26,0x00},
	{0x27,0x39},
	{0x28,0x7F},
	{0x29,0x08},
	{0x30,0x03},
	{0x31,0x00},
	{0x32,0x1A},
	{0x33,0x1A},
	{0x34,0x07},
	{0x35,0x07},
	{0x36,0x01},
	{0x37,0xFF},
	{0x38,0x36},
	{0x39,0x07},
	{0x3A,0x00},
	{0x3E,0xFF},
	{0x3F,0x00},
	{0x40,0x77},
	{0x41,0x40},
	{0x42,0x00},
	{0x43,0x30},
	{0x44,0xA0},
	{0x45,0x5C},
	{0x46,0x00},
	{0x47,0x00},
	{0x48,0x58},
	{0x4A,0x1E},
	{0x4B,0x1E},
	{0x4C,0x00},
	{0x4D,0x00},
	{0x4E,0xA0},
	{0x4F,0x80},
	{0x50,0x00},
	{0x51,0x00},
	{0x52,0x00},
	{0x53,0x00},
	{0x54,0x00},
	{0x57,0x80},
	{0x59,0x10},
	{0x5A,0x08},
	{0x5B,0x94},
	{0x5C,0xE8},
	{0x5D,0x08},
	{0x5E,0x3D},
	{0x5F,0x99},
	{0x60,0x45},
	{0x61,0x40},
	{0x63,0x2D},
	{0x64,0x02},
	{0x65,0x96},
	{0x66,0x00},
	{0x67,0x97},
	{0x68,0x01},
	{0x69,0xCD},
	{0x6A,0x01},
	{0x6B,0xB0},
	{0x6C,0x04},
	{0x6D,0x2C},
	{0x6E,0x01},
	{0x6F,0x32},
	{0x71,0x00},
	{0x72,0x01},
	{0x73,0x35},
	{0x74,0x00},
	{0x75,0x33},
	{0x76,0x31},
	{0x77,0x01},
	{0x7C,0x84},
	{0x7D,0x03},
	{0x7E,0x01},
};

int init_paj7620_i2c(void)
{
   
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PAJ7620_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = PAJ7620_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };

    int err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        ESP_LOGI(PAJ7620_TAG, "Failed to config I2C, error:%d", err);
        return err;
    }
    ESP_LOGI(PAJ7620_TAG,"I2C configed sucessfully!");
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**************************************************************** 
 * Function Name: paj7620_write_reg
 * Description:  PAJ7620 Write reg cmd
 * Parameters: addr:reg address; cmd:function data
 * Return: error code; success: return 0
****************************************************************/ 
uint8_t paj7620_write_reg(uint8_t addr, uint8_t cmd)
{
	// char i = 1;
	// Wire.beginTransmission(PAJ7620_ID);		// start transmission to device 
	// //write cmd
	// Wire.write(addr);						// send register address
	// Wire.write(cmd);						// send value to write
    // i = Wire.endTransmission();  		    // end transmission
	// if(0 != i)
    // {
	// 	Serial.print("Transmission error!!!\n");
	// }

    int ret = 0;
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, PAJ7620_ID << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(handle, addr, ACK_CHECK_EN);
    i2c_master_write_byte(handle, cmd, ACK_CHECK_EN);
    i2c_master_stop(handle);
    ret = i2c_master_cmd_begin(i2c_master_port, handle, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(PAJ7620_TAG,"Failed to write pag7620 reg!");
        return ret;
    }
    ESP_LOGI(PAJ7620_TAG,"Succeeded to write addr: %x, data: %x", addr, cmd);
	return ret;
}

/**************************************************************** 
 * Function Name: paj7620_read_reg
 * Description:  PAJ7620 read reg data
 * Parameters: addr:reg address;
 *			   qty:number of data to read, addr continuously increase;
 *			   data[]:storage memory start address
 * Return: error code; success: return 0
****************************************************************/ 
uint8_t paj7620_read_reg(uint8_t addr, uint8_t qty, uint8_t data[])
{
	uint8_t error;

    *txBuffer = addr;
    txLength = 1;
    error = i2c_master_write_read_device(i2c_master_port, PAJ7620_ID, txBuffer, txLength, rxBuffer, qty, 50 / portTICK_RATE_MS);
    if(error)
    {
        ESP_LOGI(PAJ7620_TAG,"i2cWriteReadNonStop returned Error %d", error);
        return error;
    }

    rxIndex = 0;
    while(rxIndex < qty)
    {
        *data = rxBuffer[rxIndex++];
        ESP_LOGI(PAJ7620_TAG,"read addr: %d", addr);
        ESP_LOGI(PAJ7620_TAG,"read data: %d", *data);
        data++;
    }

	// clear trigger at the end of the gesture reading
	gesture_triggered = 0;

	return 0;
}

/**************************************************************** 
 * Function Name: paj7620_select_bank
 * Description:  PAJ7620 select register bank
 * Parameters: BANK0, BANK1
 * Return: none
****************************************************************/ 
void paj7620_select_bank(bank_e bank)
{
    switch(bank){
		case BANK0:
			paj7620_write_reg(PAJ7620_REGITER_BANK_SEL, PAJ7620_BANK0);
			break;
		case BANK1:
			paj7620_write_reg(PAJ7620_REGITER_BANK_SEL, PAJ7620_BANK1);
			break;
		default:
			break;
	}
}

/**************************************************************** 
 * Function Name: init_paj7620
 * Description:  PAJ7620 REG INIT
 * Parameters: none
 * Return: error code; success: return 0
****************************************************************/ 
uint8_t init_paj7620(void) 
{
	//Near_normal_mode_V5_6.15mm_121017 for 940nm
	int i = 0;
	uint8_t error;
	uint8_t data0 = 0, data1 = 0;
	//wakeup the sensor
	//Wait 700us for PAJ7620U2 to stabilize	
    //vTaskDelay(700 / portTICK_PERIOD_MS);
	
    init_paj7620_i2c();
	
    ESP_LOGI(PAJ7620_TAG, "INIT SENSOR...");

	paj7620_select_bank(BANK0);
	paj7620_select_bank(BANK0);

	error = paj7620_read_reg(0, 1, &data0);
	if (error)
	{
		return error;
	}
	error = paj7620_read_reg(1, 1, &data1);
	if (error)
	{
		return error;
	}
	
    ESP_LOGI(PAJ7620_TAG, "Addr0 = %d", data0);
    ESP_LOGI(PAJ7620_TAG, "Addr1 = %d", data1);

	if ( (data0 != 0x20 ) || (data1 != 0x76) )
	{
		return 0xff;
	}

	if ( data0 == 0x20 )
	{
        ESP_LOGI(PAJ7620_TAG, "wake-up finish.");
	}
	
	for (i = 0; i < INIT_REG_ARRAY_SIZE; i++) 
	{
		paj7620_write_reg(initRegisterArray[i][0], initRegisterArray[i][1]);
	}
	
	paj7620_select_bank(BANK0);  //gesture flage reg in Bank0
	
	// Serial.println("Paj7620 initialize register finished.");
    ESP_LOGI(PAJ7620_TAG, "Paj7620 initialize register finished.");

	return 0;
}

static void paj7620_event_handler(void *arg)
{
    gesture_triggered  = 1;
}

int paj7620_gesture_triggered()
{
	return gesture_triggered;
}

esp_err_t init_paj7620_interrupt()
{
    esp_err_t err = 0;
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_DEF_INPUT;
    io_conf.pin_bit_mask = (1ULL << PAJ7620_INTERRUPT_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;

    err = gpio_config(&io_conf);
    if (err != ESP_OK)
    {
        ESP_LOGI(PAJ7620_TAG, "Failed to gpio config, error: %d.", err);
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK)
    {
        ESP_LOGI(PAJ7620_TAG, "Failed to install isr service, error: %d.", err);
        return err;
    }
    // hook isr handler for specific gpio pin
    err = gpio_isr_handler_add(PAJ7620_INTERRUPT_PIN, paj7620_event_handler, NULL);
    if (err != ESP_OK)
    {
        ESP_LOGI(PAJ7620_TAG, "Failed to add isr hanlder, error: %d.", err);
        return err;
    }

    return err;
}

esp_err_t paj7620_suspend()
{
	esp_err_t err = ESP_OK;
	paj7620_select_bank(BANK1);
	err = paj7620_write_reg(0x72, 0x00); //Firstly disable paj7620

	if(ESP_OK != err)
	{
		ESP_LOGE(PAJ7620_TAG, "Failed to disable the sensor, error: %d.", err);
	}

	paj7620_select_bank(BANK0);
	err = paj7620_write_reg(0x03, 0x01); //Suspend the sensor

	if(ESP_OK != err)
	{
		ESP_LOGE(PAJ7620_TAG, "Failed to suspend the sensor, error: %d.", err);
	}

	return err;
}

esp_err_t paj7620_wake_up()
{
	esp_err_t err = ESP_OK;
	paj7620_select_bank(BANK1);  // Trigger the sensor to wake up.
	paj7620_select_bank(BANK1);

	vTaskDelay(1 / portTICK_PERIOD_MS); // wait for the sensor to stablelised
	err = paj7620_write_reg(0x72, 0x01);
	if(ESP_OK != err)
	{
		ESP_LOGE(PAJ7620_TAG, "Failed to enable the sensor, error: %d.", err);
	}
	
	return err;
}