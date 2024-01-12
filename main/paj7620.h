/*
 * paj7620.h
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
 

#ifndef __PAJ7620_H__
#define __PAJ7620_H__

#include <stdlib.h>
#include "esp_log.h"
#include "hal/gpio_types.h"
#include "paj7620.h"
#include "driver/i2c.h"

#define PAJ7620_TAG "PAJ7620 LOG:"

// #define BIT(x)  1 << x

// REGISTER DESCRIPTION
#define PAJ7620_VAL(val, maskbit)		( val << maskbit )
#define PAJ7620_ADDR_BASE				0x00

// REGISTER BANK SELECT
#define PAJ7620_REGITER_BANK_SEL		(PAJ7620_ADDR_BASE + 0xEF)	//W

// DEVICE ID
#define PAJ7620_ID  0x73

// define I2C GPIO 
#define I2C_BUFFER_LENGTH  128
#define I2C_MASTER_FREQ_HZ    100000 
#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

#define PAJ7620_INTERRUPT_PIN 7			//Origianl: 37       /*!< I2C Interrupt pin */ 

#define PAJ7620_I2C_SCL 	8     //Origianl: 26          /*!< gpio number for I2C master clock */
#define PAJ7620_I2C_SDA 	5     //Origianl: 25          /*!< gpio number for I2C master data  */

#define I2C_MASTER_NUM I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */


// REGISTER BANK 0
#define PAJ7620_ADDR_SUSPEND_CMD		(PAJ7620_ADDR_BASE + 0x3)	//W
#define PAJ7620_ADDR_GES_PS_DET_MASK_0		(PAJ7620_ADDR_BASE + 0x41)	//RW
#define PAJ7620_ADDR_GES_PS_DET_MASK_1		(PAJ7620_ADDR_BASE + 0x42)	//RW
#define PAJ7620_ADDR_GES_PS_DET_FLAG_0		(PAJ7620_ADDR_BASE + 0x43)	//R
#define PAJ7620_ADDR_GES_PS_DET_FLAG_1		(PAJ7620_ADDR_BASE + 0x44)	//R
#define PAJ7620_ADDR_STATE_INDICATOR	(PAJ7620_ADDR_BASE + 0x45)	//R
#define PAJ7620_ADDR_PS_HIGH_THRESHOLD	(PAJ7620_ADDR_BASE + 0x69)	//RW
#define PAJ7620_ADDR_PS_LOW_THRESHOLD	(PAJ7620_ADDR_BASE + 0x6A)	//RW
#define PAJ7620_ADDR_PS_APPROACH_STATE	(PAJ7620_ADDR_BASE + 0x6B)	//R
#define PAJ7620_ADDR_PS_RAW_DATA		(PAJ7620_ADDR_BASE + 0x6C)	//R

// REGISTER BANK 1
#define PAJ7620_ADDR_PS_GAIN			(PAJ7620_ADDR_BASE + 0x44)	//RW
#define PAJ7620_ADDR_IDLE_S1_STEP_0		(PAJ7620_ADDR_BASE + 0x67)	//RW
#define PAJ7620_ADDR_IDLE_S1_STEP_1		(PAJ7620_ADDR_BASE + 0x68)	//RW
#define PAJ7620_ADDR_IDLE_S2_STEP_0		(PAJ7620_ADDR_BASE + 0x69)	//RW
#define PAJ7620_ADDR_IDLE_S2_STEP_1		(PAJ7620_ADDR_BASE + 0x6A)	//RW
#define PAJ7620_ADDR_OP_TO_S1_STEP_0	(PAJ7620_ADDR_BASE + 0x6B)	//RW
#define PAJ7620_ADDR_OP_TO_S1_STEP_1	(PAJ7620_ADDR_BASE + 0x6C)	//RW
#define PAJ7620_ADDR_OP_TO_S2_STEP_0	(PAJ7620_ADDR_BASE + 0x6D)	//RW
#define PAJ7620_ADDR_OP_TO_S2_STEP_1	(PAJ7620_ADDR_BASE + 0x6E)	//RW
#define PAJ7620_ADDR_OPERATION_ENABLE	(PAJ7620_ADDR_BASE + 0x72)	//RW

// PAJ7620_REGITER_BANK_SEL
#define PAJ7620_BANK0		PAJ7620_VAL(0,0)
#define PAJ7620_BANK1	PAJ7620_VAL(1,0)

// PAJ7620_ADDR_SUSPEND_CMD
#define PAJ7620_I2C_WAKEUP	PAJ7620_VAL(1,0)
#define PAJ7620_I2C_SUSPEND	PAJ7620_VAL(0,0)

// PAJ7620_ADDR_OPERATION_ENABLE
#define PAJ7620_ENABLE		PAJ7620_VAL(1,0)
#define PAJ7620_DISABLE		PAJ7620_VAL(0,0)

typedef enum {
	BANK0 = 0,
	BANK1,		
} bank_e;

#define GES_RIGHT_FLAG				PAJ7620_VAL(1,0)
#define GES_LEFT_FLAG				PAJ7620_VAL(1,1)
#define GES_UP_FLAG					PAJ7620_VAL(1,2)
#define GES_DOWN_FLAG				PAJ7620_VAL(1,3)
#define GES_FORWARD_FLAG			PAJ7620_VAL(1,4)
#define GES_BACKWARD_FLAG			PAJ7620_VAL(1,5)
#define GES_CLOCKWISE_FLAG			PAJ7620_VAL(1,6)
#define GES_COUNT_CLOCKWISE_FLAG	PAJ7620_VAL(1,7)
#define GES_WAVE_FLAG				PAJ7620_VAL(1,0)

#define GES_REACTION_TIME    50       // You can adjust the reaction time according to the actual circumstance.
#define GES_ENTRY_TIME      10      // When you want to recognize the Forward/Backward gestures, your gestures' reaction time must less than GES_ENTRY_TIME(0.8s). 
#define GES_QUIT_TIME     0


#define INIT_REG_ARRAY_SIZE (sizeof(initRegisterArray)/sizeof(initRegisterArray[0]))
//#define INIT_REG_ARRAY_SIZE (sizeof(initial_register_array)/sizeof(initial_register_array[0]))

int init_paj7620_i2c(void);
uint8_t init_paj7620_registers(void);
esp_err_t init_paj7620_interrupt();
int paj7620_gesture_triggered();
uint8_t paj7620_write_reg(uint8_t addr, uint8_t cmd);
uint8_t paj7620_read_reg(uint8_t addr, uint8_t qty, uint8_t data[]);
void paj7620_select_bank(bank_e bank);
esp_err_t paj7620_suspend();
esp_err_t paj7620_wake_up();
esp_err_t paj7620_reset();

#endif
