/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//uint32_t adc_data[2] = {0,0};
bool flag_00 = 0, flag_01 = 0, flag_02 = 0, flag_03 = 0, flag_04 = 0;
bool flag_10 = 0, flag_11 = 0, flag_12 = 0, flag_13 = 0, flag_14 = 0;
bool flag_20 = 0, flag_21 = 0, flag_22 = 0, flag_23 = 0, flag_24 = 0;
uint32_t adcdata[4] = {0,0,0,0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM13_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C4_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/*
void irany0(void);
void irany1(void);
*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//uint8_t helo = 0;
	//uint8_t i = 20;
	uint32_t adc_mcutemp = 0;
	float mcutemp = 0;
	char buffer_mcutemp[10];
	float coordinate[3] = {0.0,0.0,0.0};

	char uart_buffer_tx[20] = "helo helo";
	char uartbuffer_x[10], uartbuffer_y[10], uartbuffer_z[10];
	//uint32_t adc_data_read[2] = {0,0};

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM13_Init();
  MX_I2C2_Init();
  MX_I2C4_Init();
  MX_ADC3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc3,adcdata,4);
  LCD_init_customcurzor(1,0);
  LCD2_init_customcurzor2(1,0);

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim10);

  HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);

  MPU6050_init();

  //HAL_Delay(500);

  //LCD_command(0x01);
 //LCD_string("na ez");

  LCD1_test();
  LCD2_test();

  LCD_goto(1,0);
  LCD_string("Joy1 ");
  LCD_goto(2,0);
  LCD_string("Joy2 ");

  HAL_UART_Transmit(&huart3,uart_buffer_tx,sizeof(uart_buffer_tx),1000);
  uart_newrow();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  __HAL_TIM_SetCompare(&htim13,TIM_CHANNEL_1,5);
	//irany1();
	// irany0();
	  /*
	  HAL_ADC_Start(&hadc1);
	 	  HAL_ADC_Start(&hadc3);
	 	  adc_data_read[0] = HAL_ADC_GetValue(&hadc1);
	 	  adc_data_read[1] = HAL_ADC_GetValue(&hadc3);
	  */
	  LCD_goto(1,6);
	  LCD_int(adcdata[0],sizeof(adcdata[0]));
	  LCD_goto(1,11);
	  LCD_int(adcdata[1],sizeof(adcdata[1]));
	  LCD_goto(2,6);
	  LCD_int(adcdata[2],sizeof(adcdata[2]));
  	  LCD_goto(2,11);
  	  LCD_int(adcdata[3],sizeof(adcdata[3]));

  	  adc_mcutemp = HAL_ADC_GetValue(&hadc1);
  	  mcutemp = ((((3.3/4096)*adc_mcutemp) - 0.76)/(2.5/1000)) + 25;
	  LCD2_goto2(1,0);
	  LCD2_string2("Servo: all");
	  LCD2_goto2(2,1);
	  ftoa(mcutemp,buffer_mcutemp,2);
	  LCD2_string2(buffer_mcutemp);

	  MPU_read_gyro(coordinate);
	  ftoa(coordinate[0],uartbuffer_x,4);
	  ftoa(coordinate[1],uartbuffer_y,4);
	  ftoa(coordinate[2],uartbuffer_z,4);

	  HAL_UART_Transmit(&huart3,"X coord.: ",sizeof("X coord.: "),1000);
	  HAL_UART_Transmit(&huart3,uartbuffer_x,sizeof(uartbuffer_x),1000);
	  HAL_UART_Transmit(&huart3,"\n\r",sizeof("\n\r"),1000);

	  HAL_UART_Transmit(&huart3,"Y coord.: ",sizeof("Y coord.: "),1000);
	  HAL_UART_Transmit(&huart3,uartbuffer_y,sizeof(uartbuffer_x),1000);
	  HAL_UART_Transmit(&huart3,"\n\r",sizeof("\n\r"),1000);

	  HAL_UART_Transmit(&huart3,"Z coord.: ",sizeof("Z coord.: "),1000);
	  HAL_UART_Transmit(&huart3,uartbuffer_z,sizeof(uartbuffer_x),1000);
	  HAL_UART_Transmit(&huart3,"\n\r",sizeof("\n\r"),1000);
	  uart_newrow();

	  if(HAL_GPIO_ReadPin(sensor_barrier_GPIO_Port,sensor_barrier_Pin) == 0){
		  LCD2_goto2(2,11);
		  LCD2_string2("megy");
	  }
	  else{
		  LCD2_goto2(2,11);
		  LCD2_string2("ment");
	  }
	  HAL_Delay(500);

	 // HAL_UART_Transmit(&huart3,uart_buffer_tx,sizeof(uart_buffer_tx),1000);
	 //__HAL_TIM_SetCompare(&htim13,TIM_CHANNEL_1,1);
	// HAL_Delay(1000);
	//__HAL_TIM_SetCompare(&htim13,TIM_CHANNEL_1,3);
	 //HAL_Delay(1000);
	 __HAL_TIM_SetCompare(&htim13,TIM_CHANNEL_1,5);
	  HAL_Delay(1000);
	 __HAL_TIM_SetCompare(&htim13,TIM_CHANNEL_1,25);
	  HAL_Delay(1000);
	// __HAL_TIM_SetCompare(&htim13,TIM_CHANNEL_1,15);
	// HAL_Delay(1000);
	// __HAL_TIM_SetCompare(&htim13,TIM_CHANNEL_1,0);
	// 	 HAL_Delay(1000);

	  /*
	  LCD_goto(1,5);
	  LCD_string("Helo");
	  LCD_goto(2,6);
	  LCD_string("belo");
*/
	  //HAL_Delay(1000);
	  //adc resz
	/*	  HAL_ADC_Start(&hadc1);
		  HAL_ADC_Start(&hadc3);
		  //LCD_command(0x01);
		  adc_data[0] = HAL_ADC_GetValue(&hadc1);
		  adc_data[1] = HAL_ADC_GetValue(&hadc3);

		  LCD_command(0x80);		//elso sor
		  LCD_int(adc_data[0],sizeof(adc_data[0]));

		  //LCD_string("akkor mehet");
		  LCD_command(0xC0);		//masodik sor
		  LCD_int(adc_data[1],sizeof(adc_data[1]));
		  HAL_Delay(2000);*/
/*
		 // LCD_string("talan?");
		  //LCD_data(adc_data[0]);

		  if(adc_data[0] >= 4000){	//elore
			  HAL_GPIO_WritePin(LED_piros_GPIO_Port,LED_piros_Pin,1);
			  HAL_GPIO_WritePin(LED_zold_GPIO_Port,LED_zold_Pin,0);
			  HAL_GPIO_WritePin(LED_kek_GPIO_Port,LED_kek_Pin,0);
		  }

		  if(adc_data[1] >= 4000){	//hatra
			  HAL_GPIO_WritePin(LED_piros_GPIO_Port,LED_piros_Pin,0);
			  HAL_GPIO_WritePin(LED_zold_GPIO_Port,LED_zold_Pin,1);
			  HAL_GPIO_WritePin(LED_kek_GPIO_Port,LED_kek_Pin,0);
		  }

		  if(adc_data[0] <= 800){		//jobbra
			  HAL_GPIO_WritePin(LED_piros_GPIO_Port,LED_piros_Pin,0);
			  HAL_GPIO_WritePin(LED_zold_GPIO_Port,LED_zold_Pin,0);
			  HAL_GPIO_WritePin(LED_kek_GPIO_Port,LED_kek_Pin,1);
		  }

		  if(adc_data[1] <= 800){		//jobbra
			  HAL_GPIO_WritePin(LED_piros_GPIO_Port,LED_piros_Pin,1);
			  HAL_GPIO_WritePin(LED_zold_GPIO_Port,LED_zold_Pin,1);
			  HAL_GPIO_WritePin(LED_kek_GPIO_Port,LED_kek_Pin,1);
		  }
		  if(adc_data[1] < 3200 && adc_data[1] > 3100 && adc_data[0] > 3100 && adc_data[0] < 3200){		//jobbra
			  HAL_GPIO_WritePin(LED_piros_GPIO_Port,LED_piros_Pin,0);
			  HAL_GPIO_WritePin(LED_zold_GPIO_Port,LED_zold_Pin,0);
			  HAL_GPIO_WritePin(LED_kek_GPIO_Port,LED_kek_Pin,0);
		  }

		 // HAL_Delay(1000);
		  if(adc_data[0] >= 4000){	//elore

			  }

			  if(adc_data[1] >= 4000){	//hatra


			  }

			  if(adc_data[0] <= 800){		//jobbra

			  }
			  if(adc_data[1] <= 800){		//jobbra

			  }

*/


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_I2C4;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 4;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00303D5B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00303D5B;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 39999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 399;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1599;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 799;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 99;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 799;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 99;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 1599;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 199;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LCD2_E_Pin|LCD2_DATA4_Pin|LCD2_DATA5_Pin|LCD2_DATA6_Pin 
                          |LCD2_DATA7_Pin|LCD_Rs_Pin|LCD_E_Pin|HCS_trig_Pin 
                          |LCD_DATA4_Pin|LCD_DATA5_Pin|LCD_DATA6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LCD2_Rs_Pin|test_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Stepper_12_GPIO_Port, Stepper_12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_zold_Pin|LCD_DATA7_Pin|Stepper_33_Pin|Stepper_32_Pin 
                          |LED_piros_Pin|Stepper_31_Pin|Stepper_11_Pin|Stepper_13_Pin 
                          |Stepper_10_Pin|LED_kek_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Stepper_30_Pin|Stepper_20_Pin|Stepper_21_Pin|Stepper_22_Pin 
                          |Stepper_23_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Stepper_03_Pin|Stepper_02_Pin|Stepper_01_Pin|Stepper_00_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD2_E_Pin LCD2_DATA4_Pin LCD2_DATA5_Pin LCD2_DATA6_Pin 
                           LCD2_DATA7_Pin LCD_Rs_Pin LCD_E_Pin HCS_trig_Pin 
                           LCD_DATA4_Pin LCD_DATA5_Pin LCD_DATA6_Pin */
  GPIO_InitStruct.Pin = LCD2_E_Pin|LCD2_DATA4_Pin|LCD2_DATA5_Pin|LCD2_DATA6_Pin 
                          |LCD2_DATA7_Pin|LCD_Rs_Pin|LCD_E_Pin|HCS_trig_Pin 
                          |LCD_DATA4_Pin|LCD_DATA5_Pin|LCD_DATA6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USERbutton_IT_Pin */
  GPIO_InitStruct.Pin = USERbutton_IT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USERbutton_IT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD2_Rs_Pin test_Pin */
  GPIO_InitStruct.Pin = LCD2_Rs_Pin|test_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : Stepper_12_Pin */
  GPIO_InitStruct.Pin = Stepper_12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Stepper_12_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_zold_Pin LCD_DATA7_Pin LED_piros_Pin LED_kek_Pin */
  GPIO_InitStruct.Pin = LED_zold_Pin|LCD_DATA7_Pin|LED_piros_Pin|LED_kek_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : sensor_barrier_Pin J1_in_Pin J2_in_Pin */
  GPIO_InitStruct.Pin = sensor_barrier_Pin|J1_in_Pin|J2_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : Stepper_33_Pin Stepper_32_Pin Stepper_31_Pin Stepper_11_Pin 
                           Stepper_13_Pin Stepper_10_Pin */
  GPIO_InitStruct.Pin = Stepper_33_Pin|Stepper_32_Pin|Stepper_31_Pin|Stepper_11_Pin 
                          |Stepper_13_Pin|Stepper_10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Stepper_30_Pin Stepper_20_Pin Stepper_21_Pin Stepper_22_Pin 
                           Stepper_23_Pin */
  GPIO_InitStruct.Pin = Stepper_30_Pin|Stepper_20_Pin|Stepper_21_Pin|Stepper_22_Pin 
                          |Stepper_23_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Stepper_03_Pin Stepper_02_Pin Stepper_01_Pin Stepper_00_Pin */
  GPIO_InitStruct.Pin = Stepper_03_Pin|Stepper_02_Pin|Stepper_01_Pin|Stepper_00_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim -> Instance == TIM2){
		HAL_GPIO_TogglePin(LED_piros_GPIO_Port,LED_piros_Pin);
	}

	if(htim -> Instance == TIM6){
		if(adcdata[2] <= 7000 && adcdata[2] >= 3000){
			flag_00 = 0;
			if(flag_00 == 0 && flag_01 == 0){
					allit_00();
					allit_10();
					flag_01 = 1;
					flag_00 = 1;
			}
			else
				if(flag_00 == 0 && flag_02 == 0){
					allit_01();
					allit_11();
					flag_02 = 1;
					flag_00 = 1;
				}
				else
					if(flag_00 == 0 && flag_03 == 0){
						allit_02();
						allit_12();
						flag_03 = 1;
						flag_00 = 1;
					}
					else
						if(flag_00 == 0){
							allit_03();
							allit_13();
							flag_01 = 0;
							flag_02 = 0;
							flag_03 = 0;
						}
		}
		if(adcdata[2] <= 1500){
			flag_00 = 0;
			if(flag_00 == 0 && flag_01 == 0){
					allit_03();
					allit_13();
					flag_01 = 1;
					flag_00 = 1;
			}
			else
				if(flag_00 == 0 && flag_02 == 0){
					allit_02();
					allit_12();
					flag_02 = 1;
					flag_00 = 1;
				}
				else
					if(flag_00 == 0 && flag_03 == 0){
						allit_01();
						allit_11();
						flag_03 = 1;
						flag_00 = 1;
					}
					else
						if(flag_00 == 0){
							allit_00();
							allit_10();
							flag_01 = 0;
							flag_02 = 0;
							flag_03 = 0;
						}
		}
	}

	if(htim -> Instance == TIM7){
		if(adcdata[3] <= 7000 && adcdata[3] >= 3000){
			flag_10 = 0;
			if(flag_10 == 0 && flag_11 == 0){
					allit_20();
					flag_11 = 1;
					flag_10 = 1;
			}
			else
				if(flag_10 == 0 && flag_12 == 0){
					allit_21();
					flag_12 = 1;
					flag_10 = 1;
				}
				else
					if(flag_10 == 0 && flag_13 == 0){
						allit_22();
						flag_13 = 1;
						flag_10 = 1;
					}
					else
						if(flag_10 == 0){
							allit_23();
							flag_11 = 0;
							flag_12 = 0;
							flag_13 = 0;
						}
		}
		if(adcdata[3] <= 1500){
			flag_10 = 0;
			if(flag_10 == 0 && flag_11 == 0){
				allit_23();
				flag_11 = 1;
				flag_10 = 1;
			}
			else
				if(flag_10 == 0 && flag_12 == 0){
					allit_22();
					flag_12 = 1;
					flag_10 = 1;
				}
				else
					if(flag_10 == 0 && flag_13 == 0){
						allit_21();
						flag_13 = 1;
						flag_10 = 1;
					}
					else
						if(flag_10 == 0){
							allit_20();
							flag_11 = 0;
							flag_12 = 0;
							flag_13 = 0;
						}
		}
	}

	if(htim -> Instance == TIM10){
		if(adcdata[0] <= 7000 && adcdata[0] >= 3000){
			flag_20 = 0;
			if(flag_20 == 0 && flag_21 == 0){
				//allit_10();
				flag_21 = 1;
				flag_20 = 1;
			}
			else
				if(flag_20 == 0 && flag_22 == 0){
					//allit_11();
					flag_22 = 1;
					flag_20 = 1;
				}
				else
					if(flag_20 == 0 && flag_23 == 0){
						//allit_12();
						flag_23 = 1;
						flag_20 = 1;
					}
					else
						if(flag_20 == 0){
							//allit_13();
							flag_21 = 0;
							flag_22 = 0;
							flag_23 = 0;
						}
		}
		if(adcdata[0] <= 1500){
			flag_20 = 0;
			if(flag_20 == 0 && flag_21 == 0){
				//allit_13();
				flag_21 = 1;
				flag_20 = 1;
			}
			else
				if(flag_20 == 0 && flag_22 == 0){
					//allit_12();
					flag_22 = 1;
					flag_20 = 1;
				}
				else
					if(flag_20 == 0 && flag_23 == 0){
						//allit_11();
						flag_23 = 1;
						flag_20 = 1;
					}
					else
						if(flag_20 == 0){
							//allit_10();
							flag_21 = 0;
							flag_22 = 0;
							flag_23 = 0;
						}
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
