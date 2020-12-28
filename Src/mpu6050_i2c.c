/*
 * mpu6050_i2c.c
 *
 *  Created on: 2020. okt. 21.
 *      Author: borsa
 */

#include "mpu6050_i2c.h"

void MPU6050_init(){
	uint8_t check = 0, Data = 0x00;

	HAL_I2C_Mem_Read(&hi2c2,MPU6050_addr,Who_am_i,1,&check,1,1000);
	if(check == Sensorworking){
		HAL_I2C_Mem_Write(&hi2c2,MPU6050_addr,Pwr_managment_1_reg,1,&Data,1,1000);

		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c2,MPU6050_addr,SMPLRT_DIV_reg,1,&Data,1,1000);

		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2,MPU6050_addr,Gyro_config_reg,1,&Data,1,1000);
	}
}

void MPU_read_gyro(float *array){
	uint8_t gyrodata[6];
	int returndata[3];

	HAL_I2C_Mem_Read(&hi2c2,MPU6050_addr,Gyro_xout_h_reg,1,gyrodata,6,1000);

	returndata[0] = (int)(gyrodata[0] << 8 | gyrodata[1]);
	returndata[1] = (int)(gyrodata[2] << 8 | gyrodata[3]);
	returndata[2] = (int)(gyrodata[4] << 8 | gyrodata[5]);

	array[0] = returndata[0]/131.0;
	array[1] = returndata[1]/131.0;
	array[2] = returndata[2]/131.0;
}
