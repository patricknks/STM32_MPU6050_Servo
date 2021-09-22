/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdbool.h"
#include "math.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

////////////////////////////////////////GABUNGAN REGISTER//////////////////////////////////////////
#define MPU_ADDRESS 0x68<<1
uint8_t buffer[2];
uint8_t settings_buffer1[2] = {0x1B, 0x00}; // untuk setting gyro  diberi nilai 0x00
uint8_t settings_buffer2[2] = {0x1C, 0x00}; // untuk setting accel diberi nilai 0x00
uint8_t sleep_off[2] = {0x6B, 0x00}; // untuk membangunkan MPU6050
uint8_t mpu_start[1] = {0x3B}; //untuk mulai start baca sensor dari alamt 3B (Accel_Xout_H)
uint8_t mpu_start2[1] = {0x43}; //untuk mulai start baca sensor dari alamt 3B (GYRO_Xout_H)
uint8_t mpu6050_buffer[14]; //array untuk menyimpan data dari sensor
//int16_t shift_x_gyro, shift_y_gyro, shift_z_gyro, shift_x_accel, shift_y_accel, shift_z_accel, shift_temp;
int16_t aX1,aY1,aZ1;
int16_t gX1,gY1,gZ1;
float temp;
//double gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;
float aX,aY,aZ;
float gX,gY,gZ;
float accRoll, accPitch; //sudut dari accel
float gyrRoll, gyrPitch; //sudut dari gyro
float roll, pitch; //value after filter

float dt; //interval waktu (timer)
unsigned long waktu;

const int x1 = -22, x2 = 5000;
int val;
int servo;

///////////////////////////////////////////////////////////////////////////////////////////////////




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t uart_send_buffer[50];
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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_I2C_IsDeviceReady(&hi2c1, MPU_ADDRESS, 2, 100) == HAL_OK) {
  		sprintf((char*) uart_send_buffer, "slave ready!\r\n");
  		HAL_UART_Transmit(&huart3, uart_send_buffer,
  				(uint16_t) strlen((char*) uart_send_buffer), 50);
} else {
  		sprintf((char*) uart_send_buffer, "slave error\r\n");
  		HAL_UART_Transmit(&huart3, uart_send_buffer,
  				(uint16_t) strlen((char*) uart_send_buffer), 50);
}
  //SLEEP MOD OFF
   HAL_I2C_Master_Transmit(&hi2c1, MPU_ADDRESS, sleep_off, 2, 100);
   HAL_I2C_Master_Transmit(&hi2c1, MPU_ADDRESS, settings_buffer1, 2, 100);
   HAL_I2C_Master_Transmit(&hi2c1, MPU_ADDRESS, settings_buffer2, 2, 100);
  //SERVO
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
   htim1.Instance->CCR1 = 3000;
   HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	  while (1)
	  {
		  HAL_I2C_Master_Transmit(&hi2c1, MPU_ADDRESS, mpu_start, 1,100);
		  HAL_I2C_Master_Receive(&hi2c1, MPU_ADDRESS, mpu6050_buffer, 8,100);
		  waktu = HAL_GetTick();
		  aX1 = (mpu6050_buffer[0]<<8 | mpu6050_buffer[1]);
		  aY1 = (mpu6050_buffer[2]<<8 | mpu6050_buffer[3]);
		  aZ1 = (mpu6050_buffer[4]<<8 | mpu6050_buffer[5]);

		  temp = (mpu6050_buffer[6]<<8 | mpu6050_buffer[7]);

		  aX = (float) aX1  / 16384.00;
		  aY = (float) aY1  / 16384.00;
		  aZ = (float) aZ1  / 16384.00;

		  accRoll = (atan2(aX,aZ))*57.2958;
		  accPitch = (atan2(aY,aZ))*57.2958;

		  for (int i = 0; i<10 ; i++)
		    {
			  dt = (HAL_GetTick() - waktu)/1000.00;
			  HAL_I2C_Master_Transmit(&hi2c1, MPU_ADDRESS, mpu_start2, 1,100);
			  HAL_I2C_Master_Receive(&hi2c1, MPU_ADDRESS, mpu6050_buffer, 6,100);

			  gX1 = (mpu6050_buffer[8]<<8 | mpu6050_buffer[9]);
			  gY1 = (mpu6050_buffer[10]<<8 | mpu6050_buffer[11]);
			  gZ1 = (mpu6050_buffer[12]<<8 | mpu6050_buffer[13]);

			  gX = (float) gX1  / 131;
			  gY = (float) gY1  / 131;
			  gZ = (float) gZ1  / 131;

			  gyrRoll += gY*dt;
			  gyrPitch += gX*dt;
			  if(i==100-1)
			    {
			       gyrRoll = (gyrRoll/100)*57.2958;
			       gyrPitch = (gyrPitch/100)*57.2958;
			       break;
			     }
			 }
		  //COMPLEMENTARY FILTER
		  roll = (accRoll * 0.9998 + gyrRoll * 0.0002) / (0.9998+0.0002);
		  pitch = (accPitch * 0.9998 + gyrPitch * 0.0002) / (0.9998+0.0002);

		  sprintf((char*) uart_send_buffer, "roll : %f\r\n",roll);
		  HAL_UART_Transmit(&huart3, uart_send_buffer, (uint16_t) strlen((char*) uart_send_buffer), 50);

		  val = roll;

		  servo = (val* x1) + x2;
		  htim1.Instance->CCR1 = servo;

		  sprintf((char*) uart_send_buffer, "val : %d\r\n",val);
		  HAL_UART_Transmit(&huart3, uart_send_buffer, (uint16_t) strlen((char*) uart_send_buffer), 50);
		  sprintf((char*) uart_send_buffer, "servo.val : %d\r\n",servo);
		  HAL_UART_Transmit(&huart3, uart_send_buffer, (uint16_t) strlen((char*) uart_send_buffer), 50);

		  HAL_Delay(100);
//	sprintf((char*) uart_send_buffer, "nilai aX1 : %d\r\n",aX1);
//	HAL_UART_Transmit(&huart3, uart_send_buffer, (uint16_t) strlen((char*) uart_send_buffer), 50);
//	sprintf((char*) uart_send_buffer, "nilai aY1 : %d\r\n",aY1);
//	HAL_UART_Transmit(&huart3, uart_send_buffer, (uint16_t) strlen((char*) uart_send_buffer), 50);
//	sprintf((char*) uart_send_buffer, "nilai aZ1 : %d\r\n",aZ1);
//	HAL_UART_Transmit(&huart3, uart_send_buffer, (uint16_t) strlen((char*) uart_send_buffer), 50);
//	sprintf((char*) uart_send_buffer, "nilai TEMP : %f\r\n",temp);
//	HAL_UART_Transmit(&huart3, uart_send_buffer, (uint16_t) strlen((char*) uart_send_buffer), 50);
//	sprintf((char*) uart_send_buffer, "nilai gX1 : %d\r\n",gX1);
//	HAL_UART_Transmit(&huart3, uart_send_buffer, (uint16_t) strlen((char*) uart_send_buffer), 50);
//	sprintf((char*) uart_send_buffer, "nilai gY1 : %d\r\n",gY1);
//	HAL_UART_Transmit(&huart3, uart_send_buffer, (uint16_t) strlen((char*) uart_send_buffer), 50);
//	sprintf((char*) uart_send_buffer, "nilai gZ1 : %d\r\n",gZ1);
//	HAL_UART_Transmit(&huart3, uart_send_buffer, (uint16_t) strlen((char*) uart_send_buffer), 50);

	//HAL_Delay(1000);




 }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 35;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
