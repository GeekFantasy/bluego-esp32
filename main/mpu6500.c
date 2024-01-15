#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6500.h"

#define MPU500_CALC_OFFSET

// Output value that exceeds jitter limt or 0
#define GYRO_JITTER_LIMIT    0.1
// define the full gyro scale to use
#define GYRO_FULL_SCALE  GYRO_SCALE_1000

// The average gyro offset tested
#define GYRO_OFFSET_X         85
#define GYRO_OFFSET_y         3
#define GYRO_OFFSET_z         -10

#if (GYRO_FULL_SCALE == GYRO_SCALE_250)
#define GYRO_FS_SETTING  GYRO_FS_250DPS
#elif (GYRO_FULL_SCALE == GYRO_SCALE_500)
#define GYRO_FS_SETTING  GYRO_FS_500DPS
#elif (GYRO_FULL_SCALE == GYRO_SCALE_1000)
#define GYRO_FS_SETTING  GYRO_FS_1000DPS
#else
#define GYRO_FS_SETTING  GYRO_FS_2000DPS
#endif

int mpu_i2c_master_port = 0;

int mpu_i2c_master_init(void)
{
   
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MPU6500_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = MPU6500_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
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
    //vTaskDelay(100 / portTICK_PERIOD_MS);
	
    ESP_ERROR_CHECK(mpu_i2c_master_init());
	// Serial.println("INIT SENSOR...");
    ESP_LOGI(MPU6500_TAG, "MPU6500 I2C init successfully...");

    if(mpu6500_reset()) 
    {
        ESP_LOGE(MPU6500_TAG, "MPU6500 reset with error");
    }
    else
    {
        ESP_LOGI(MPU6500_TAG, "MPU6500 reset successfully...");
    }

    if(mpu6500_set_sleep(false)) 
    {
        ESP_LOGE(MPU6500_TAG, "MPU6500 wake up modules with error.");
    }
    else
    {
        ESP_LOGI(MPU6500_TAG, "MPU6500 wake up moduleset successfully...");
    }

    gyro_raw gyro_r;

    // This piece of code used to calc the offset of the MPU.
    // For the first time please use below code to caculate the offset. Need to keep the board steady when caculating or it gets wrong offset
#ifdef MPU500_CALC_OFFSET
    mpu6500_compute_gyro_offset(&gyro_r);
    ESP_LOGI(MPU6500_TAG, "MPU6500 gyro offset: x = %d , y = %d , z = %d .", gyro_r.x, gyro_r.y, gyro_r.z);
    gyro_r.x = -gyro_r.x;
    gyro_r.y = -gyro_r.y;
    gyro_r.z = -gyro_r.z;

    gyro_raw gyro_original;
    gyro_original = mpu6500_get_gyro_offset();
    ESP_LOGI(MPU6500_TAG, "Original gyro offset is, x = %d, y = %d, z = %d.", gyro_original.x, gyro_original.y, gyro_original.z);

#else    
    // Use the code below to use predefined offset
    gyro_r.x = GYRO_OFFSET_X;
    gyro_r.y = GYRO_OFFSET_y;
    gyro_r.z = GYRO_OFFSET_z;
#endif // #endif MPU500_CALC_OFFSET

    if(mpu6500_set_gyro_offset(&gyro_r))
    {
        ESP_LOGE(MPU6500_TAG, "MPU6500 set gyro offset with error.");
    }
    else
    {
        ESP_LOGI(MPU6500_TAG, "MPU6500 set gyro offset successfully.");
    }

    gyro_r = mpu6500_get_gyro_offset();
    ESP_LOGI(MPU6500_TAG, "Set gyro offset is, x = %d, y = %d, z = %d.", gyro_r.x, gyro_r.y, gyro_r.z);

    if(mpu6500_set_clock_source(CLOCK_PLL)) 
    {
        ESP_LOGE(MPU6500_TAG, "MPU6500 set clock source with error.");
    }
    else
    {
        ESP_LOGI(MPU6500_TAG, "MPU6500 set clock source successfully...");
    }

    if(mpu6500_set_gyro_full_scale(GYRO_FS_SETTING)) 
    {
        ESP_LOGE(MPU6500_TAG, "MPU6500 set gyro scale with error.");
    }
    else
    {
        ESP_LOGI(MPU6500_TAG, "MPU6500 set gyro scale successfully...");
    }

    if(mpu6500_set_accel_full_scale(ACCEL_FS_16G)) 
    {
        ESP_LOGE(MPU6500_TAG, "MPU6500 set accel scale with error.");
    }
    else
    {
        ESP_LOGI(MPU6500_TAG, "MPU6500 set accel scale successfully...");
    }

    if(mpu6500_set_digital_low_pass_filter(DLPF_42HZ)) 
    {
        ESP_LOGE(MPU6500_TAG, "MPU6500 set low pass filter with error.");
    }
    else
    {
        ESP_LOGI(MPU6500_TAG, "MPU6500 set low pass filter successfully...");
    }

    if(mpu6500_set_sample_rate(100)) 
    {
        ESP_LOGE(MPU6500_TAG, "MPU6500 set sample rate with error.");
    }
    else
    {
        ESP_LOGI(MPU6500_TAG, "MPU6500 set sample rate successfully...");
    }

    ESP_LOGI(MPU6500_TAG, "MPU6500 Initialization complete...");
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
        ESP_LOGE(MPU6500_TAG,"Failed to read who am I!");
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

uint8_t mpu6500_write_bytes(uint8_t reg_addr, uint8_t* data, uint8_t data_len)
{
    int ret;
    uint8_t* buffer = (uint8_t*)malloc(data_len + 1);
    buffer[0] = reg_addr;
    for (size_t i = 0; i < data_len; i++)
    {
        buffer[i + 1] = data[i];
    }
    
    ret = i2c_master_write_to_device(mpu_i2c_master_port, MPU6500_I2C_ADDR, buffer, data_len + 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);  

    free(buffer);
    return ret;
}

uint8_t mpu6500_read_reg(uint8_t reg_addr,uint8_t data[], uint8_t data_len)
{
    int ret;
    ret = i2c_master_write_read_device(mpu_i2c_master_port, MPU6500_I2C_ADDR, &reg_addr, 1, data, data_len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);   
    return ret;  
}

/// @brief Write a single bit of the register
/// @param reg_addr 
/// @param bit_num 
/// @param data 
/// @return 
uint8_t mpu6500_write_bit(uint8_t reg_addr, uint8_t bit_num, uint8_t data)
{
    uint8_t ret = 0, buffer;

    ret = mpu6500_read_reg(reg_addr, &buffer, sizeof(buffer));
    if(ret) return ret;

    buffer = data? (buffer | (1 << bit_num)) : (buffer & ~(1 << bit_num));
    uint8_t write_buff[2] = {reg_addr, buffer};

    ret = i2c_master_write_to_device(mpu_i2c_master_port, MPU6500_I2C_ADDR, write_buff, sizeof(write_buff), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);  
    return ret;
}

/// @brief Write a single bit of the register
/// @param reg_addr 
/// @param bit_num 
/// @param data 
/// @return 
uint8_t mpu6500_write_bits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data)
{
    uint8_t ret = 0, buffer;

    ret = mpu6500_read_reg(reg_addr, &buffer, sizeof(buffer));
    if(ret) return ret;

    uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
    data <<= (bit_start - length + 1);
    data &= mask;
    buffer &= ~mask;
    buffer |= data;

    uint8_t write_buff[2] = {reg_addr, buffer};
    ret = i2c_master_write_to_device(mpu_i2c_master_port, MPU6500_I2C_ADDR, write_buff, sizeof(write_buff), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);  
    return ret;
}

/// @brief reset device (wait a little to clear all registers)
/// @return error code
esp_err_t mpu6500_reset()
{
    esp_err_t err_code = 0;
    err_code = mpu6500_write_bit(PWR_MGMT1, PWR1_DEVICE_RESET_BIT, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return err_code; 
}

/// @brief // wake-up or set asleep the device (power on-reset state is asleep for some models)
/// @param sleep, true to sleep and false to wake up all models 
/// @return error code
esp_err_t mpu6500_set_sleep(bool enable)
{
    esp_err_t err_code = 0;
    err_code = mpu6500_write_bit(PWR_MGMT1, PWR1_SLEEP_BIT, (uint8_t) enable);
    return err_code; 
}

/// @brief set clock source (to gyro PLL which is better than internal clock)
/// @param clockSrc 
/// @return 
esp_err_t mpu6500_set_clock_source(clock_src_t clockSrc)
{
    esp_err_t err_code = 0;
    mpu6500_write_bits(PWR_MGMT1, PWR1_CLK_SEL_BIT, PWR1_CLKSEL_LENGTH,clockSrc);
    return err_code; 
}

esp_err_t mpu6500_set_gyro_full_scale(gyro_fs_t fsr)
{
    esp_err_t err_code = 0;
    mpu6500_write_bits(GYRO_CONFIG, GCONFIG_FS_SEL_BIT, GCONFIG_FS_SEL_LENGTH, fsr);
    return err_code; 
}

esp_err_t mpu6500_set_accel_full_scale(accel_fs_t fsr)
{
    esp_err_t err_code = 0;
    mpu6500_write_bits(ACCEL_CONFIG, ACONFIG_FS_SEL_BIT, ACONFIG_FS_SEL_LENGTH, fsr);
    return err_code; 
}

esp_err_t mpu6500_set_digital_low_pass_filter(dlpf_t dlpf)
{
    esp_err_t err_code = 0;
    mpu6500_write_bits(CONFIG, CONFIG_DLPF_CFG_BIT, CONFIG_DLPF_CFG_LENGTH, dlpf);
    return err_code; 
}

esp_err_t mpu6500_set_sample_rate(uint16_t rate)
{
    esp_err_t err_code = 0;

    if (rate < 4) {
        rate = 4;
    }
    else if (rate > 1000) {
        rate = 1000;
    }
    
    uint16_t internalSampleRate = 1000;
    uint16_t divider = internalSampleRate / rate - 1;
    // Check for rate match
    uint16_t finalRate = (internalSampleRate / (1 + divider));
    if (finalRate != rate) {
        ESP_LOGI(MPU6500_TAG, "Sample rate constrained to %d Hz", finalRate);
    }
    else {
        ESP_LOGI(MPU6500_TAG, "Sample rate set to %d Hz", finalRate);
    }
    // Write divider to register
    err_code = mpu6500_write_reg(SMPLRT_DIV, (uint8_t)divider);

    return err_code; 
}

esp_err_t mpu6500_GYR_read_raw(uint8_t GYR_DATA[])
{
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_GYRO_XOUT_H, &GYR_DATA[0], 1));
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_GYRO_XOUT_L, &GYR_DATA[1], 1));
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_GYRO_YOUT_H, &GYR_DATA[2], 1));
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_GYRO_YOUT_L, &GYR_DATA[3], 1));
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_GYRO_ZOUT_H, &GYR_DATA[4], 1));
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_GYRO_ZOUT_L, &GYR_DATA[5], 1));

    return ESP_OK;
}

esp_err_t mpu6500_GYR_read(gyro* gyro)
{
    gyro_raw gyro_r;
    accel_raw acc_r;
    esp_err_t err = 0;

    err = mpu6500_motion_read_raw(&acc_r, &gyro_r);
        
    gyro->x = gyro_r.x / 32768.0 * GYRO_FULL_SCALE;
    gyro->y = gyro_r.y / 32768.0 * GYRO_FULL_SCALE;
    gyro->z = gyro_r.z / 32768.0 * GYRO_FULL_SCALE;

    if(abs(gyro->x) <= GYRO_JITTER_LIMIT) gyro->x = 0;
    if(abs(gyro->y) <= GYRO_JITTER_LIMIT) gyro->y = 0;
    if(abs(gyro->z) <= GYRO_JITTER_LIMIT) gyro->z = 0;

    return err;
}

esp_err_t mpu6500_ACC_read_raw(uint8_t ACC_DATA[])
{
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_ACCEL_XOUT_H, &ACC_DATA[0], 1));
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_ACCEL_XOUT_L, &ACC_DATA[1], 1));
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_ACCEL_YOUT_H, &ACC_DATA[2], 1));
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_ACCEL_YOUT_L, &ACC_DATA[3], 1));
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_ACCEL_ZOUT_H, &ACC_DATA[4], 1));
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_ACCEL_ZOUT_L, &ACC_DATA[5], 1));

    return ESP_OK;
}

esp_err_t mpu6500_motion_read_raw(accel_raw* accel, gyro_raw* gyro)
{
    uint8_t buffer[14];
    ESP_ERROR_CHECK(mpu6500_read_reg(MPU6500_ACCEL_XOUT_H, buffer, sizeof(buffer)));
    accel->x = buffer[0] << 8 | buffer[1];
    accel->y = buffer[2] << 8 | buffer[3];
    accel->z = buffer[4] << 8 | buffer[5];
    gyro->x  = buffer[8] << 8 | buffer[9];
    gyro->y  = buffer[10] << 8 | buffer[11];
    gyro->z  = buffer[12] << 8 | buffer[13]; 

    return ESP_OK;  
}

/// @brief Configure the sensors that will be written to the FIFO.
/// @param config 
/// @return 
esp_err_t mpu6500_set_fifo_config(fifo_config_t config)
{
    esp_err_t err = 0;
    err = mpu6500_write_reg(FIFO_EN, (uint8_t) config);
    return err;
}

/// @brief Enabled / disable FIFO module.
/// @param enable 
/// @return 
esp_err_t mpu6500_set_fifo_enable(bool enable)
{
    return mpu6500_write_bit(USER_CTRL, USERCTRL_FIFO_EN_BIT, (uint8_t) enable);
}

/// @brief Reset FIFO module.
/// Zero FIFO count, reset is asynchronous.
/// The bit auto clears after one clock cycle.
/// @return 
esp_err_t mpu6500_reset_fifo()
{
    return mpu6500_write_bit(USER_CTRL, USERCTRL_FIFO_RESET_BIT, 1);
}

/// @brief Return number of written bytes in the FIFO.
/// @note FIFO overflow generates an interrupt which can be check with getInterruptStatus().
/// @return 
uint16_t mpu6500_get_fifo_count()
{
    uint8_t buffer[2];
    mpu6500_read_reg(FIFO_COUNT_H, buffer, sizeof(buffer));
    uint16_t count = buffer[0] << 8 | buffer[1];
    return count;
}

/// @brief Read data contained in FIFO buffer.
/// @param data 
/// @param length 
/// @return 
esp_err_t mpu6500_read_fifo(uint8_t* data, size_t length)
{
   return mpu6500_read_reg(FIFO_R_W, data, length);
}

/// @brief  Compute the Biases in regular mode
/// @param gyroFS set the full scale when calculating the offset
/// @param gyro return offset
/// @attention This algorithm takes about ~400ms to compute offsets.
/// @return 
esp_err_t mpu6500_get_gyro_bias(gyro_fs_t gyroFS, gyro_raw* gyro_bias)
{
    esp_err_t err = 0;
    
    // configurations to compute biases
    uint16_t kSampleRate      = 1000;
    dlpf_t kDLPF              = DLPF_188HZ;
    fifo_config_t kFIFOConfig = FIFO_CFG_GYRO;
    size_t kPacketSize        = 6;

    // setup
    if ((err = mpu6500_set_sample_rate(kSampleRate))) return err;
    if ((err = mpu6500_set_digital_low_pass_filter(kDLPF))) return err;
    if ((err = mpu6500_set_gyro_full_scale(gyroFS))) return err;
    if ((err = mpu6500_set_fifo_config(kFIFOConfig))) return err;
    if ((err = mpu6500_set_fifo_enable(true))) return err;

    // wait for 200ms for sensors to stabilize
    vTaskDelay(200 / portTICK_PERIOD_MS);
    // fill FIFO for 100ms
    if ((err = mpu6500_reset_fifo())) return err;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if ((err = mpu6500_set_fifo_config(FIFO_CFG_NONE))) return err;
    // get FIFO count
    uint16_t fifoCount = mpu6500_get_fifo_count();
    int packetCount = fifoCount / kPacketSize;
    // read overrun bytes, if any
    int overrunCount = fifoCount - (packetCount * kPacketSize);
    uint8_t buffer[kPacketSize];
    if (overrunCount > 0) {
        if ((err = mpu6500_read_fifo(buffer, overrunCount))) return err;
    }

    // fetch data and add up
    int gyro_x = 0, gyro_y = 0, gyro_z = 0;
    for (int i = 0; i < packetCount; i++) {
        if ((err = mpu6500_read_fifo(buffer, kPacketSize))) return err;
        // retrieve data
        gyro_x  += (int16_t)((buffer[0] << 8) | buffer[1]);
        gyro_y  += (int16_t)((buffer[2] << 8) | buffer[3]);
        gyro_z  += (int16_t)((buffer[4] << 8) | buffer[5]);

        ESP_LOGI(MPU6500_TAG, "Package num: %d, x=%d, y=%d, z=%d", i, (int16_t)((buffer[0] << 8) | buffer[1]), (int16_t)((buffer[2] << 8) | buffer[3]), (int16_t)((buffer[4] << 8) | buffer[5]));
    }

    // calculate average
    gyro_x /= packetCount;
    gyro_y /= packetCount;
    gyro_z /= packetCount;

    // save biases
    gyro_bias->x  = (int16_t)gyro_x;
    gyro_bias->y  = (int16_t)gyro_y;
    gyro_bias->z  = (int16_t)gyro_z;    

    return err;
}

/// @brief Compute Gyroscope offsets.
/// This takes about ~400ms to compute offsets.
/// It is better to call computeOffsets() before any configuration is done (better right after
/// initialize()).
/// Note: Gyro offset output are LSB in 1000DPS format.
/// @return 
esp_err_t mpu6500_compute_gyro_offset(gyro_raw* gyro)
{
    esp_err_t err = 0;
    gyro_fs_t kGyroFS = GYRO_FS_250DPS;  // most sensitive

    if ((err = mpu6500_get_gyro_bias(kGyroFS, gyro))) return err;
    // convert offsets to 16G and 1000DPS format and invert values
    gyro->x =  (gyro->x >> (GYRO_FS_1000DPS - kGyroFS));
    gyro->y =  (gyro->y >> (GYRO_FS_1000DPS - kGyroFS));
    gyro->z =  (gyro->z >> (GYRO_FS_1000DPS - kGyroFS));

    return err;
}

esp_err_t mpu6500_set_gyro_offset(gyro_raw* bias)
{
    uint8_t buffer[6];
    buffer[0] = (uint8_t)(bias->x >> 8);
    buffer[1] = (uint8_t)(bias->x);
    buffer[2] = (uint8_t)(bias->y >> 8);
    buffer[3] = (uint8_t)(bias->y);
    buffer[4] = (uint8_t)(bias->z >> 8);
    buffer[5] = (uint8_t)(bias->z);

    for (size_t i = 0; i < sizeof(buffer); i++)
    {
        ESP_LOGI(MPU6500_TAG, "Data to write to gyro offset: num: %d, data: %X.", i, buffer[i]);
    }
    
    return mpu6500_write_bytes(XG_OFFSET_H, buffer, sizeof(buffer));
}

gyro_raw mpu6500_get_gyro_offset()
{
    uint8_t buffer[6];
    gyro_raw bias;
    mpu6500_read_reg(XG_OFFSET_H, buffer, sizeof(buffer));
    bias.x = (buffer[0] << 8) | buffer[1];
    bias.y = (buffer[2] << 8) | buffer[3];
    bias.z = (buffer[4] << 8) | buffer[5]; 
    return bias;
}
