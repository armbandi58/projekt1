/*
 * mpu6050_i2c.h
 *
 *  Created on: 2020. okt. 21.
 *      Author: borsa
 */

#ifndef MPU6050_I2C_H_
#define MPU6050_I2C_H_

#include "main.h"

#define Who_am_i 0x75
#define Sensorworking 0x68
#define MPU6050_addr 0xD0

#define SMPLRT_DIV_reg 0x19
#define Gyro_config_reg 0x1B
#define Accel_config_reg 0x1C
#define Accel_xout_H_reg 0x3B
#define Gyro_xout_h_reg 0x43

#define Pwr_managment_1_reg 0x6B

void MPU6050_init(void);
void MPU_read_gyro(float * array);

#endif /* MPU6050_I2C_H_ */
