#include <stdio.h>
#include <stddef.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6500.h"

int mpu_i2c_master_port = I2C_MASTER_NUM;

int mpu_i2c_master_init(void)
{
   
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };

    int err = i2c_param_config(mpu_i2c_master_port, &conf);
    if (err != ESP_OK) {
        ESP_LOGI(MPU6500_TAG, "Failed to config I2C, error:%d", err);
        return err;
    }
    ESP_LOGI(MPU6500_TAG,"I2C configed sucessfully!");
    return i2c_driver_install(mpu_i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void mpu6500_init(void)
{
    vTaskDelay(100 / portTICK_PERIOD_MS);
	
    ESP_ERROR_CHECK(mpu_i2c_master_init());
	
	// Serial.println("INIT SENSOR...");
    ESP_LOGI(MPU6500_TAG, "MPU6500 init successfully...");
}

uint8_t mpu6500_who_am_i(void)
{
    int ret;
    uint8_t write_addr = MPU6500_WHO_AM_I_REG_ADDR, data = 0x30;
    ret = i2c_master_write_read_device(mpu_i2c_master_port, MPU6500_I2C_ADDR, &write_addr, 1, &data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);   
    if(data == WHO_AM_I_VALUE)
    {
        ESP_LOGI(MPU6500_TAG,"Read Who am I successfully.");
    }
    else
    {
        ESP_LOGI(MPU6500_TAG,"Failed to read who am I!");
    }

    return ret;
}

uint8_t mpu6500_write_reg(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buff[2] = {reg_addr, data};
    ret = i2c_master_write_to_device(mpu_i2c_master_port, MPU6500_I2C_ADDR, write_buff, sizeof(write_buff), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);  
    return ret;
}

uint8_t mpu6500_read_reg(uint8_t reg_addr,uint8_t data[], uint8_t data_len)
{
    int ret;
    ret=i2c_master_write_read_device(mpu_i2c_master_port, MPU6500_I2C_ADDR, &reg_addr, 1, data, data_len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);   
    return ret;  
}


esp_err_t mpu6500_GYR_read(uint8_t GYR_DATA[])
{
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_GYRO_XOUT_H, &GYR_DATA[0], 1));
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_GYRO_XOUT_L, &GYR_DATA[1], 1));
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_GYRO_YOUT_H, &GYR_DATA[2], 1));
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_GYRO_YOUT_L, &GYR_DATA[3], 1));
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_GYRO_ZOUT_H, &GYR_DATA[4], 1));
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_GYRO_ZOUT_L, &GYR_DATA[5], 1));

    return ESP_OK;
}