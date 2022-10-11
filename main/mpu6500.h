#ifndef MPU6500
#define MPU6500

#include <stdio.h>
#include <stddef.h>
#include "driver/i2c.h"


#define MPU6500_TAG "MPU6500"

#define I2C_SCL 26               /*!< gpio number for I2C master clock */
#define I2C_SDA 25               /*!< gpio number for I2C master data  */

#define I2C_MASTER_NUM I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ    400000 
#define I2C_MASTER_TIMEOUT_MS 10
#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */

#define MPU6500_I2C_ADDR 0x68
#define WHO_AM_I_VALUE 0x70

// Register map
#define MPU6500_ACCEL_XOUT_H        (0x3B)
#define MPU6500_ACCEL_XOUT_L        (0x3C)
#define MPU6500_ACCEL_YOUT_H        (0x3D)
#define MPU6500_ACCEL_YOUT_L        (0x3E)
#define MPU6500_ACCEL_ZOUT_H        (0x3F)
#define MPU6500_ACCEL_ZOUT_L        (0x40)
#define MPU6500_TEMP_OUT_H          (0x41)
#define MPU6500_TEMP_OUT_L          (0x42)
#define MPU6500_GYRO_XOUT_H         (0x43)
#define MPU6500_GYRO_XOUT_L         (0x44)
#define MPU6500_GYRO_YOUT_H         (0x45)
#define MPU6500_GYRO_YOUT_L         (0x46)
#define MPU6500_GYRO_ZOUT_H         (0x47)
#define MPU6500_GYRO_ZOUT_L         (0x48)
#define MPU6500_WHO_AM_I_REG_ADDR   (0x75)


void mpu6500_init(void);
uint8_t mpu6500_who_am_i(void);
uint8_t mpu6500_write_reg(uint8_t reg_addr, uint8_t data);
uint8_t mpu6500_read_reg(uint8_t reg_addr,uint8_t data[], uint8_t data_len);
esp_err_t mpu6500_GYR_read(uint8_t GYR_DATA[]);

#endif