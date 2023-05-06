#ifndef MPU6500
#define MPU6500

#include <stdio.h>
#include <stddef.h>
#include "driver/i2c.h"


#define MPU6500_TAG "MPU6500"

#define I2C_SCL 26               /*!< gpio number for I2C master clock */
#define I2C_SDA 25               /*!< gpio number for I2C master data  */

// #define I2C_MASTER_NUM I2C_NUM_0   /*!< I2C port number for master dev */
// #define I2C_MASTER_FREQ_HZ    400000 
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

#define XG_OFFSET_H  (0x13)
#define XG_OFFSET_L  (0x14)
#define YG_OFFSET_H  (0x15)
#define YG_OFFSET_L  (0x16)
#define ZG_OFFSET_H  (0x17)
#define ZG_OFFSET_L  (0x18)
#define SMPLRT_DIV   (0x19)  
//------------------------------------------------------------------------------
#define PWR_MGMT1             (0x6B)
#define PWR1_DEVICE_RESET_BIT (7)
#define PWR1_SLEEP_BIT        (6)
#define PWR1_CYCLE_BIT        (5)
#define PWR1_GYRO_STANDBY_BIT (4)
#define PWR1_TEMP_DIS_BIT     (3)
#define PWR1_CLK_SEL_BIT      (2)
#define PWR1_CLKSEL_LENGTH    (3)
//------------------------------------------------------------------------------
#define CONFIG                     (0x1A)
#define CONFIG_FIFO_MODE_BIT       (6)
#define CONFIG_EXT_SYNC_SET_BIT    (5)
#define CONFIG_EXT_SYNC_SET_LENGTH (3)
#define CONFIG_DLPF_CFG_BIT        (2)
#define CONFIG_DLPF_CFG_LENGTH     (3)
//------------------------------------------------------------------------------
#define GYRO_CONFIG              (0x1B)
#define GCONFIG_XG_ST_BIT        (7)
#define GCONFIG_YG_ST_BIT        (6)
#define GCONFIG_ZG_ST_BIT        (5)
#define GCONFIG_FS_SEL_BIT       (4)
#define GCONFIG_FS_SEL_LENGTH    (2)
#define GCONFIG_FCHOICE_B        (1)
#define GCONFIG_FCHOICE_B_LENGTH (2)
//------------------------------------------------------------------------------
#define ACCEL_CONFIG          (0x1C)
#define ACONFIG_XA_ST_BIT     (7)
#define ACONFIG_YA_ST_BIT     (6)
#define ACONFIG_ZA_ST_BIT     (5)
#define ACONFIG_FS_SEL_BIT    (4)
#define ACONFIG_FS_SEL_LENGTH (2)
#define ACONFIG_HPF_BIT       (2)
#define ACONFIG_HPF_LENGTH    (3)

//------------------------------------------------------------------------------
#define FIFO_EN           (0x23)
#define FIFO_TEMP_EN_BIT  (7)
#define FIFO_XGYRO_EN_BIT (6)
#define FIFO_YGYRO_EN_BIT (5)
#define FIFO_ZGYRO_EN_BIT (4)
#define FIFO_ACCEL_EN_BIT (3)
#define FIFO_SLV_2_EN_BIT (2)
#define FIFO_SLV_1_EN_BIT (1)
#define FIFO_SLV_0_EN_BIT (0)

#define FIFO_CFG_GYRO     (1 << FIFO_XGYRO_EN_BIT | 1 << FIFO_YGYRO_EN_BIT | 1 << FIFO_ZGYRO_EN_BIT)
#define FIFO_CFG_NONE     (0x0)

//------------------------------------------------------------------------------
#define I2C_MST_CTRL                  (0x24)
#define I2CMST_CTRL_MULT_EN_BIT       (7)
#define I2CMST_CTRL_WAIT_FOR_ES_BIT   (6)
#define I2CMST_CTRL_SLV_3_FIFO_EN_BIT (5)
#define I2CMST_CTRL_P_NSR_BIT         (4)
#define I2CMST_CTRL_CLOCK_BIT         (3)
#define I2CMST_CTRL_CLOCK_LENGTH      (4)

//------------------------------------------------------------------------------
#define USER_CTRL                    (0x6A)
#define USERCTRL_DMP_EN_BIT          (7)
#define USERCTRL_FIFO_EN_BIT         (6)
#define USERCTRL_I2C_MST_EN_BIT      (5)
#define USERCTRL_I2C_IF_DIS_BIT      (4)
#define USERCTRL_DMP_RESET_BIT       (3)
#define USERCTRL_FIFO_RESET_BIT      (2)
#define USERCTRL_I2C_MST_RESET_BIT   (1)
#define USERCTRL_SIG_COND_RESET_BIT  (0)

//------------------------------------------------------------------------------
#define MEM_START_ADDR  (0x6E)
#define MEM_R_W         (0x6F)
#define PRGM_START_H    (0x70)
#define PRGM_START_L    (0x71)
#define FIFO_COUNT_H    (0x72)
#define FIFO_COUNT_L    (0x73)
#define FIFO_R_W        (0x74)
#define WHO_AM_I        (0x75)

//------------------------------------------------------------------------------
#define GYRO_SCALE_250    250
#define GYRO_SCALE_500    500
#define GYRO_SCALE_1000   1000
#define GYRO_SCALE_2000   2000

/*! Clock Source */
typedef enum {
    CLOCK_INTERNAL = 0,  //!< Internal oscillator: 20MHz for MPU6500 and 8MHz for MPU6050
    CLOCK_PLL      = 3,  //!< Selects automatically best pll source (recommended)
    CLOCK_KEEP_RESET = 7  //!< Stops the clock and keeps timing generator in reset
} clock_src_t;

/*! Gyroscope full-scale range */
typedef enum {
    GYRO_FS_250DPS  = 0,  //!< +/- 250 º/s  -> 131 LSB/(º/s)
    GYRO_FS_500DPS  = 1,  //!< +/- 500 º/s  -> 65.5 LSB/(º/s)
    GYRO_FS_1000DPS = 2,  //!< +/- 1000 º/s -> 32.8 LSB/(º/s)
    GYRO_FS_2000DPS = 3   //!< +/- 2000 º/s -> 16.4 LSB/(º/s)
} gyro_fs_t;

/*! Accel full-scale range */
typedef enum {
    ACCEL_FS_2G  = 0,  //!< +/- 2 g  -> 16.384 LSB/g
    ACCEL_FS_4G  = 1,  //!< +/- 4 g  -> 8.192 LSB/g
    ACCEL_FS_8G  = 2,  //!< +/- 8 g  -> 4.096 LSB/g
    ACCEL_FS_16G = 3   //!< +/- 16 g -> 2.048 LSB/g
} accel_fs_t;

/*! Digital low-pass filter (based on gyro bandwidth) */
typedef enum {
    DLPF_256HZ_NOLPF = 0,
    DLPF_188HZ       = 1,
    DLPF_98HZ        = 2,
    DLPF_42HZ        = 3,
    DLPF_20HZ        = 4,
    DLPF_10HZ        = 5,
    DLPF_5HZ         = 6,
    DLPF_3600HZ_NOLPF     = 7
} dlpf_t;

typedef struct 
{
    int16_t x, y, z;
} gyro_raw;

typedef struct 
{
    float x, y, z;
} gyro;

typedef struct 
{
    int16_t x, y, z;
} accel_raw;

typedef struct 
{
    float x, y, z;
} accel;

typedef struct
{
    float x;
    float y;
    float z;
    float temp;
} angle;

typedef uint16_t fifo_config_t;

void mpu6500_init(void);
uint8_t mpu6500_who_am_i(void);
uint8_t mpu6500_write_reg(uint8_t reg_addr, uint8_t data);
uint8_t mpu6500_read_reg(uint8_t reg_addr,uint8_t data[], uint8_t data_len);
uint8_t mpu6500_write_bit(uint8_t reg_addr, uint8_t bit_num, uint8_t data);
uint8_t mpu6500_write_bits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data);

esp_err_t mpu6500_GYR_read_raw(uint8_t GYR_DATA[]);
esp_err_t mpu6500_GYR_read(gyro* gyro);
esp_err_t mpu6500_ACC_read_raw(uint8_t ACC_DATA[]);
esp_err_t mpu6500_motion_read_raw(accel_raw* acc, gyro_raw* gyro);
esp_err_t mpu6500_reset();
esp_err_t mpu6500_set_sleep(bool enable);
esp_err_t mpu6500_set_clock_source(clock_src_t clockSrc);
esp_err_t mpu6500_set_gyro_full_scale(gyro_fs_t fsr);
esp_err_t mpu6500_set_accel_full_scale(accel_fs_t fsr);
esp_err_t mpu6500_set_digital_low_pass_filter(dlpf_t dlpf);
esp_err_t mpu6500_set_sample_rate(uint16_t rate);
esp_err_t mpu6500_set_fifo_config(fifo_config_t config);
esp_err_t mpu6500_set_fifo_enable(bool enable);
esp_err_t mpu6500_reset_fifo();
uint16_t mpu6500_get_fifo_count();
esp_err_t mpu6500_read_fifo(uint8_t* data, size_t length);
esp_err_t mpu6500_get_gyro_bias(gyro_fs_t gyroFS, gyro_raw* gyro_bias);
esp_err_t mpu6500_compute_gyro_offset(gyro_raw* gyro);
esp_err_t mpu6500_set_gyro_offset(gyro_raw* bias);
gyro_raw mpu6500_get_gyro_offset();

#endif