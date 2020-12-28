/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <inttypes.h>
#include <stdbool.h>
#include <math.h>

#include "lcd.h"
#include "fuggvenyek.h"
#include "stepper.h"
#include "mpu6050_i2c.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c4;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart3;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD2_E_Pin GPIO_PIN_2
#define LCD2_E_GPIO_Port GPIOE
#define LCD2_DATA4_Pin GPIO_PIN_3
#define LCD2_DATA4_GPIO_Port GPIOE
#define LCD2_DATA5_Pin GPIO_PIN_4
#define LCD2_DATA5_GPIO_Port GPIOE
#define LCD2_DATA6_Pin GPIO_PIN_5
#define LCD2_DATA6_GPIO_Port GPIOE
#define LCD2_DATA7_Pin GPIO_PIN_6
#define LCD2_DATA7_GPIO_Port GPIOE
#define USERbutton_IT_Pin GPIO_PIN_13
#define USERbutton_IT_GPIO_Port GPIOC
#define USERbutton_IT_EXTI_IRQn EXTI15_10_IRQn
#define LCD2_Rs_Pin GPIO_PIN_8
#define LCD2_Rs_GPIO_Port GPIOF
#define Stepper_12_Pin GPIO_PIN_4
#define Stepper_12_GPIO_Port GPIOA
#define LED_zold_Pin GPIO_PIN_0
#define LED_zold_GPIO_Port GPIOB
#define test_Pin GPIO_PIN_12
#define test_GPIO_Port GPIOF
#define sensor_barrier_Pin GPIO_PIN_0
#define sensor_barrier_GPIO_Port GPIOG
#define LCD_Rs_Pin GPIO_PIN_7
#define LCD_Rs_GPIO_Port GPIOE
#define LCD_E_Pin GPIO_PIN_10
#define LCD_E_GPIO_Port GPIOE
#define HCS_trig_Pin GPIO_PIN_11
#define HCS_trig_GPIO_Port GPIOE
#define LCD_DATA4_Pin GPIO_PIN_12
#define LCD_DATA4_GPIO_Port GPIOE
#define LCD_DATA5_Pin GPIO_PIN_14
#define LCD_DATA5_GPIO_Port GPIOE
#define LCD_DATA6_Pin GPIO_PIN_15
#define LCD_DATA6_GPIO_Port GPIOE
#define LCD_DATA7_Pin GPIO_PIN_10
#define LCD_DATA7_GPIO_Port GPIOB
#define Stepper_33_Pin GPIO_PIN_12
#define Stepper_33_GPIO_Port GPIOB
#define Stepper_32_Pin GPIO_PIN_13
#define Stepper_32_GPIO_Port GPIOB
#define LED_piros_Pin GPIO_PIN_14
#define LED_piros_GPIO_Port GPIOB
#define Stepper_31_Pin GPIO_PIN_15
#define Stepper_31_GPIO_Port GPIOB
#define J1_in_Pin GPIO_PIN_2
#define J1_in_GPIO_Port GPIOG
#define J2_in_Pin GPIO_PIN_3
#define J2_in_GPIO_Port GPIOG
#define Stepper_30_Pin GPIO_PIN_6
#define Stepper_30_GPIO_Port GPIOC
#define Stepper_20_Pin GPIO_PIN_8
#define Stepper_20_GPIO_Port GPIOC
#define Stepper_21_Pin GPIO_PIN_9
#define Stepper_21_GPIO_Port GPIOC
#define Stepper_22_Pin GPIO_PIN_10
#define Stepper_22_GPIO_Port GPIOC
#define Stepper_23_Pin GPIO_PIN_11
#define Stepper_23_GPIO_Port GPIOC
#define Stepper_03_Pin GPIO_PIN_4
#define Stepper_03_GPIO_Port GPIOD
#define Stepper_02_Pin GPIO_PIN_5
#define Stepper_02_GPIO_Port GPIOD
#define Stepper_01_Pin GPIO_PIN_6
#define Stepper_01_GPIO_Port GPIOD
#define Stepper_00_Pin GPIO_PIN_7
#define Stepper_00_GPIO_Port GPIOD
#define Stepper_11_Pin GPIO_PIN_3
#define Stepper_11_GPIO_Port GPIOB
#define Stepper_13_Pin GPIO_PIN_4
#define Stepper_13_GPIO_Port GPIOB
#define Stepper_10_Pin GPIO_PIN_5
#define Stepper_10_GPIO_Port GPIOB
#define LED_kek_Pin GPIO_PIN_7
#define LED_kek_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
