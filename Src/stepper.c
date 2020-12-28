/*
 * stepper.c
 *
 *  Created on: 2020. okt. 20.
 *      Author: borsa
 */

#include "stepper.h"

//				0-as motor
void allit_00(){
	  HAL_GPIO_WritePin(Stepper_00_GPIO_Port, Stepper_00_Pin,1);
	  HAL_GPIO_WritePin(Stepper_01_GPIO_Port, Stepper_01_Pin,0);
	  HAL_GPIO_WritePin(Stepper_02_GPIO_Port, Stepper_02_Pin,0);
	  HAL_GPIO_WritePin(Stepper_03_GPIO_Port, Stepper_03_Pin,0);
}

void allit_01(){
	  HAL_GPIO_WritePin(Stepper_00_GPIO_Port, Stepper_00_Pin,0);
	  HAL_GPIO_WritePin(Stepper_01_GPIO_Port, Stepper_01_Pin,1);
	  HAL_GPIO_WritePin(Stepper_02_GPIO_Port, Stepper_02_Pin,0);
	  HAL_GPIO_WritePin(Stepper_03_GPIO_Port, Stepper_03_Pin,0);
}

void allit_02(){
	  HAL_GPIO_WritePin(Stepper_00_GPIO_Port, Stepper_00_Pin,0);
	  HAL_GPIO_WritePin(Stepper_01_GPIO_Port, Stepper_01_Pin,0);
	  HAL_GPIO_WritePin(Stepper_02_GPIO_Port, Stepper_02_Pin,1);
	  HAL_GPIO_WritePin(Stepper_03_GPIO_Port, Stepper_03_Pin,0);
}

void allit_03(){
	  HAL_GPIO_WritePin(Stepper_00_GPIO_Port, Stepper_00_Pin,0);
	  HAL_GPIO_WritePin(Stepper_01_GPIO_Port, Stepper_01_Pin,0);
	  HAL_GPIO_WritePin(Stepper_02_GPIO_Port, Stepper_02_Pin,0);
	  HAL_GPIO_WritePin(Stepper_03_GPIO_Port, Stepper_03_Pin,1);
}

//				1-es motor

void allit_10(){
	  HAL_GPIO_WritePin(Stepper_10_GPIO_Port, Stepper_10_Pin,1);
	  HAL_GPIO_WritePin(Stepper_11_GPIO_Port, Stepper_11_Pin,0);
	  HAL_GPIO_WritePin(Stepper_12_GPIO_Port, Stepper_12_Pin,0);
	  HAL_GPIO_WritePin(Stepper_13_GPIO_Port, Stepper_13_Pin,0);
}

void allit_11(){
	  HAL_GPIO_WritePin(Stepper_10_GPIO_Port, Stepper_10_Pin,0);
	  HAL_GPIO_WritePin(Stepper_11_GPIO_Port, Stepper_11_Pin,1);
	  HAL_GPIO_WritePin(Stepper_12_GPIO_Port, Stepper_12_Pin,0);
	  HAL_GPIO_WritePin(Stepper_13_GPIO_Port, Stepper_13_Pin,0);
}

void allit_12(){
	  HAL_GPIO_WritePin(Stepper_10_GPIO_Port, Stepper_10_Pin,0);
	  HAL_GPIO_WritePin(Stepper_11_GPIO_Port, Stepper_11_Pin,0);
	  HAL_GPIO_WritePin(Stepper_12_GPIO_Port, Stepper_12_Pin,1);
	  HAL_GPIO_WritePin(Stepper_13_GPIO_Port, Stepper_13_Pin,0);
}

void allit_13(){
	  HAL_GPIO_WritePin(Stepper_10_GPIO_Port, Stepper_10_Pin,0);
	  HAL_GPIO_WritePin(Stepper_11_GPIO_Port, Stepper_11_Pin,0);
	  HAL_GPIO_WritePin(Stepper_12_GPIO_Port, Stepper_12_Pin,0);
	  HAL_GPIO_WritePin(Stepper_13_GPIO_Port, Stepper_13_Pin,1);
}

//				2-es motor
void allit_20(){
	  HAL_GPIO_WritePin(Stepper_20_GPIO_Port, Stepper_20_Pin,1);
	  HAL_GPIO_WritePin(Stepper_21_GPIO_Port, Stepper_21_Pin,0);
	  HAL_GPIO_WritePin(Stepper_22_GPIO_Port, Stepper_22_Pin,0);
	  HAL_GPIO_WritePin(Stepper_23_GPIO_Port, Stepper_23_Pin,0);
}

void allit_21(){
	  HAL_GPIO_WritePin(Stepper_20_GPIO_Port, Stepper_20_Pin,0);
	  HAL_GPIO_WritePin(Stepper_21_GPIO_Port, Stepper_21_Pin,1);
	  HAL_GPIO_WritePin(Stepper_22_GPIO_Port, Stepper_22_Pin,0);
	  HAL_GPIO_WritePin(Stepper_23_GPIO_Port, Stepper_23_Pin,0);
}

void allit_22(){
	  HAL_GPIO_WritePin(Stepper_20_GPIO_Port, Stepper_20_Pin,0);
	  HAL_GPIO_WritePin(Stepper_21_GPIO_Port, Stepper_21_Pin,0);
	  HAL_GPIO_WritePin(Stepper_22_GPIO_Port, Stepper_22_Pin,1);
	  HAL_GPIO_WritePin(Stepper_23_GPIO_Port, Stepper_23_Pin,0);
}

void allit_23(){
	  HAL_GPIO_WritePin(Stepper_20_GPIO_Port, Stepper_20_Pin,0);
	  HAL_GPIO_WritePin(Stepper_21_GPIO_Port, Stepper_21_Pin,0);
	  HAL_GPIO_WritePin(Stepper_22_GPIO_Port, Stepper_22_Pin,0);
	  HAL_GPIO_WritePin(Stepper_23_GPIO_Port, Stepper_23_Pin,1);
}


