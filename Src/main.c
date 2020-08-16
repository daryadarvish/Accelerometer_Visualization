/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "cmsis_os.h"

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* Definitions for BlinkLED_1 */
osThreadId_t BlinkLED_1Handle;
const osThreadAttr_t BlinkLED_1_attributes = {
  .name = "BlinkLED_1",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 128 * 4
};
/* Definitions for Kalman_Timer */
osTimerId_t Kalman_TimerHandle;
const osTimerAttr_t Kalman_Timer_attributes = {
  .name = "Kalman_Timer"
};
/* USER CODE BEGIN PV */

static const uint8_t SAMPLE_RATE = 0x19;
static const uint8_t LPF_REG = 0x1A;
static const uint8_t GYRO_CONFIG = 0x1B;
static const uint8_t ACCEL_CONFIG = 0x1C;
static const uint8_t X_DATA = 0x3B;
static const uint8_t Y_DATA = 0x3D;
static const uint8_t Z_DATA = 0x3F;
static const uint8_t TEMP_DATA = 0x41;
static const uint8_t GYRO_X_DATA = 0x43;
static const uint8_t GYRO_Y_DATA = 0x45;
static const uint8_t GYRO_Z_DATA = 0x47;
static const uint8_t PWR_MANAGE_1 = 0x6B;
static const uint8_t PWR_MANAGE_2 = 0x6C;
static const uint8_t ACCEL_ADDR = 0x68 << 1;
static const uint8_t WHO_AM_I = 0x75;

uint8_t ret;
uint8_t buf[512];
int16_t valx, valy, valz, valt, valgx, valgy, valgz;
int32_t phi, rho;
int32_t phi_old[5], rho_old[5] = {0, 0, 0, 0, 0};
int32_t phi_df, rho_df;
int32_t phi_dot, rho_dot;

int32_t Measurement[][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
int32_t state_Priori[][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
int32_t state_Posteriori[][4]= {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
int32_t state_Covariance_Priori[][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
int32_t state_Covariance_Posteriori[][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
int32_t A[][4] = {{1000,1,0,0},{0,1000,0,0},{0,0,1000,1},{0,0,0,1000}};
int32_t A_Transpose[][4] = {{1000,0,0,0},{1,1000,0,0},{0,0,1000,0},{0,0,1,1000}};
int32_t matrix_buf_1[][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
int32_t matrix_buf_2[][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
int32_t matrix_buf_3[][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
int32_t kalman_gain[][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
int32_t identity_matrix[][4] = {{100,0,0,0},{0,100,0,0},{0,0,100,0},{0,0,0,100}};
int32_t sensor_noise_R[][4] = {{10,0,0,0},{0,10,0,0},{0,0,10,0},{0,0,0,10}};
int32_t h_matrix[][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
int32_t h_matrix_T[][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
int32_t process_noise_covariance_Q[][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
int32_t inverse_buf[][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void Start_BlinkLED_1(void *argument);
void Start_Kalman_Filter(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  buf[0] = WHO_AM_I;
  ret = I2C_Read_From_Mem(&hi2c1, buf, ACCEL_ADDR, WHO_AM_I, 1);
  if (buf[0] != 0x68 || ret != HAL_OK)
  {
     sprintf((char*)buf, "Error finding device @ 0x68\r\n");
	 HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
	 return 1;
  }
  buf[0] = PWR_MANAGE_1;
  buf[1] = 0x01;
  ret = I2C_Write_To_Mem(&hi2c1, buf, ACCEL_ADDR, PWR_MANAGE_1, 2);
  buf[0] = PWR_MANAGE_2;
  buf[1] = 0x00;
  ret = I2C_Write_To_Mem(&hi2c1, buf, ACCEL_ADDR, PWR_MANAGE_2, 2);
  buf[0] = SAMPLE_RATE;
  buf[1] = 0x20;
  ret = I2C_Write_To_Mem(&hi2c1, buf, ACCEL_ADDR, SAMPLE_RATE, 2);
  buf[0] = LPF_REG;
  buf[1] = 0x01;
  ret = I2C_Write_To_Mem(&hi2c1, buf, ACCEL_ADDR, LPF_REG, 2);
  buf[0] = ACCEL_CONFIG;
  buf[1] = 0x00;
  ret = I2C_Write_To_Mem(&hi2c1, buf, ACCEL_ADDR, ACCEL_CONFIG, 2);
  buf[0] = GYRO_CONFIG;
  buf[1] = 0x00;
  ret = I2C_Write_To_Mem(&hi2c1, buf, ACCEL_ADDR, GYRO_CONFIG, 2);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of Kalman_Timer */
  Kalman_TimerHandle = osTimerNew(Start_Kalman_Filter, osTimerPeriodic, NULL, &Kalman_Timer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of BlinkLED_1 */
  BlinkLED_1Handle = osThreadNew(Start_BlinkLED_1, NULL, &BlinkLED_1_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  osTimerStart(Kalman_TimerHandle, 1U);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_BlinkLED_1 */
/**
  * @brief  Function implementing the BlinkLED_1 thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_Start_BlinkLED_1 */
void Start_BlinkLED_1(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      osDelay(200);
  }
  /* USER CODE END 5 */
}

/* Start_Kalman_Filter function */
void Start_Kalman_Filter(void *argument)
{
   /* USER CODE BEGIN Start_Kalman_Filter */
   for(;;)
      {
	  // This loop executes every 10ms as triggered by the timer channel output compare
	  // interrupt.  It reads from the accelerometer and uses the kalman filter algorithm
	  // to estimate the accelerometer state.

	  // Read from the MPU6050 Accelerometer
	  buf[0] = X_DATA;
	  ret = I2C_Read_From_Mem(&hi2c1, buf, ACCEL_ADDR, X_DATA, 14);

	  //Combine the bytes
	  valx = (int16_t)(buf[0] << 8) | (buf[1]);
	  valy = (int16_t)(buf[2] << 8) | (buf[3]);
	  valz = (int16_t)(buf[4] << 8) | (buf[5]);
	  //valt = (int16_t)(buf[6] << 8) | (buf[7]);
	  valgx = (int16_t)(buf[8] << 8) | (buf[9]);
	  valgy = (int16_t)(buf[10] << 8) | (buf[11]);
	  valgz = (int16_t)(buf[12] << 8) | (buf[13]);

	  //calculate temperature
	  //valt = valt/340.0 + 36.53;

	  //multiply phi, rho, and theta by 1000 so they can be written as ints
	  phi = atan(valy/(sqrt((valx*valx)+(valz*valz))))*100;
	  rho = -atan(valx/(sqrt((valy*valy)+(valz*valz))))*100;

	  //calculate angular velocities by dividing by 131 to get degrees per second
	  //multiply by 0.01745 to get rad/s
	  //multiply by 100 for scaling and initialize to zero
	  phi_dot = ((valgx)*0.01332)-14;
	  rho_dot = ((valgy)*0.01332)+2;

	  sprintf((char*)buf, "Raw: %d %d \r\n", (int)phi, (int)rho);
	  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

	  phi_old[4] = phi_old[3];
	  rho_old[4] = rho_old[3];
	  phi_old[3] = phi_old[2];
	  rho_old[3] = rho_old[2];
	  phi_old[2] = phi_old[1];
	  rho_old[2] = rho_old[1];
	  phi_old[1] = phi_old[0];
	  rho_old[1] = rho_old[0];
	  phi_old[0] = phi;
	  rho_old[0] = rho;

	  phi_df = (phi_old[0] + phi_old[1] + phi_old[2] + phi_old[3] + phi_old[4])/5;
	  rho_df = (rho_old[0] + rho_old[1] + rho_old[2] + rho_old[3] + phi_old[4])/5;

	  //Kalman Filter Implementation

	  //Step 1: Prediction

	  //compute state_priori based on last state

	  Multiply_Matrix_Scaled(A, state_Posteriori, state_Priori, 4, 4, 1, 1000);

	  //compute state covariance priori
	  Multiply_Matrix_Scaled(state_Covariance_Posteriori, A_Transpose, matrix_buf_1, 4, 4, 4, 1000);
	  Multiply_Matrix_Scaled(A, matrix_buf_1, matrix_buf_2, 4, 4, 4, 1000);
	  Add_Matrix(matrix_buf_2, process_noise_covariance_Q, state_Covariance_Priori, 4, 4);

	  //Step2: Update

	  //compute measurement prediction covariance
	  //compute kalman gain

	  Multiply_Matrix(state_Covariance_Priori, h_matrix_T, matrix_buf_1, 4, 4, 4);
	  Multiply_Matrix(h_matrix, matrix_buf_1, matrix_buf_2, 4, 4, 4);
	  Add_Matrix(matrix_buf_2, sensor_noise_R, matrix_buf_3, 4, 4);
	  inverse(matrix_buf_3, inverse_buf);
	  Multiply_Matrix(matrix_buf_1, inverse_buf, kalman_gain, 4, 4, 4);

	  //compute innovation or measurement residual
	  Measurement[0][0] = phi;
	  Measurement[1][0] = phi_dot;
	  Measurement[2][0] = rho;
	  Measurement[3][0] = rho_dot;

	  Multiply_Matrix(h_matrix, state_Priori, matrix_buf_1, 4, 4, 1);
	  Subtract_Matrix(Measurement, matrix_buf_1, matrix_buf_2, 4, 1);

	  //Compute state_Posteriori
	  Multiply_Matrix_Scaled(kalman_gain, matrix_buf_2, matrix_buf_1, 4, 4, 1, 10000);
	  Add_Matrix(state_Priori, matrix_buf_1, state_Posteriori, 4, 1);

	  //Compute state covariance Posteriori
	  Multiply_Matrix_Scaled(kalman_gain, h_matrix, matrix_buf_1, 4, 4, 4, 10000);
	  Subtract_Matrix(identity_matrix, matrix_buf_1, matrix_buf_2, 4, 4);
	  Multiply_Matrix_Scaled(matrix_buf_2, state_Covariance_Priori,\
			  state_Covariance_Posteriori, 4, 4, 4, 100);

	  sprintf((char*)buf, "Raw: %d %d \r\n", (int)phi, (int)rho);
	  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

	  sprintf((char*)buf, "Df: %d %d \r\n", (int)phi_df, (int)rho_df);
	  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

	  sprintf((char*)buf, "Kalman: %d %d \r\n", (int)state_Posteriori[0][0], (int)state_Posteriori[2][0]);
	  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
	  }
  /* USER CODE END Start_Kalman_Filter */
}

uint8_t I2C_Write_To_Mem(I2C_HandleTypeDef* i2c, uint8_t* buf, uint8_t device_addr,\
		uint8_t register_addr, uint8_t bytes_to_write){
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Master_Transmit(i2c, device_addr, buf, bytes_to_write, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
	  sprintf((char*)buf, "Error writing to device @ %X register %X\r\n", device_addr, register_addr);
	  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
	  return 1;
	}
	return 0;
}

uint8_t I2C_Read_From_Mem(I2C_HandleTypeDef* i2c, uint8_t* buf, uint8_t device_addr,\
		uint8_t register_addr, uint8_t bytes_to_read){
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Master_Transmit(i2c, device_addr, buf, 1, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
	  sprintf((char*)buf, "Error reading from device I2C device @ %X register %X\r\n", device_addr, register_addr);
	  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
	  return 1;
	}
	ret = HAL_I2C_Master_Receive(i2c, device_addr, buf, bytes_to_read, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
	  sprintf((char*)buf, "Error reading from device I2C device @ %X register %X\r\n", device_addr, register_addr);
	  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
	  return 1;
	}
	return 0;
}

void Multiply_Matrix(int32_t Matrix_1[][4], int32_t Matrix_2[][4], int32_t result[][4],\
		int16_t P, int16_t M, int16_t n){
	//Matrix 1 is PxM, Matrix 2 is MxN
	for (int i = 0; i < P; i++){
		for (int j = 0; j < n; j++){
			result[i][j] = 0;
			for (int k = 0; k < M; k++){
				result[i][j] += Matrix_1[i][k]*Matrix_2[k][j];
			}
		}
	}
}

void Multiply_Matrix_Scaled(int32_t Matrix_1[][4], int32_t Matrix_2[][4], int32_t result[][4],\
		int16_t P, int16_t M, int32_t n, int32_t scale){
	//Matrix 1 is PxM, Matrix 2 is MxN
	for (int i = 0; i < P; i++){
		for (int j = 0; j < n; j++){
			result[i][j] = 0;
			for (int k = 0; k < M; k++){
				result[i][j] += Matrix_1[i][k]*Matrix_2[k][j];
			}
		}
	}
	for (int i = 0; i < P; i++){
		for (int j = 0; j < n; j++){
			result[i][j] /= scale;
		}
	}
}

void Add_Matrix(int32_t Matrix_1[][4], int32_t Matrix_2[][4], int32_t result[][4],\
		int16_t M, int16_t n){
	for (int i = 0; i < M; i++){
		for (int j = 0; j < n; j++){
		    result[i][j] = Matrix_1[i][j]+Matrix_2[i][j];
		}
	}
}

void Subtract_Matrix(int32_t Matrix_1[][4], int32_t Matrix_2[][4], int32_t result[][4],\
		int16_t M, int16_t n){
	for (int i = 0; i < M; i++){
		for (int j = 0; j < n; j++){
		    result[i][j] = Matrix_1[i][j]-Matrix_2[i][j];
		}
	}
}


void inverse(int32_t A[N][N], int32_t inverse[N][N])
{
	//Credit: https://www.sanfoundry.com/c-program-find-inverse-matrix/
    // Find determinant of A[][]
	int32_t det = determinant(A, N);

    // Find adjoint
	int32_t adj[N][N];
    adjoint(A, adj);

    // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
    for (int32_t i=0; i<N; i++)
        for (int32_t j=0; j<N; j++)
            inverse[i][j] = 10000*adj[i][j]/det;
}

void getCofactor(int32_t A[N][N], int32_t temp[N][N], int32_t p, int32_t q, int32_t n)
{
	//Credit: https://www.sanfoundry.com/c-program-find-inverse-matrix/
	int32_t i = 0, j = 0;

    // Looping for each element of the matrix
    for (int32_t row = 0; row < n; row++)
    {
        for (int32_t col = 0; col < n; col++)
        {
            //  Copying into temporary matrix only those element
            //  which are not in given row and column
            if (row != p && col != q)
            {
                temp[i][j++] = A[row][col];

                // Row is filled, so increase row index and
                // reset col index
                if (j == n - 1)
                {
                    j = 0;
                    i++;
                }
            }
        }
    }
}

/* Recursive function for finding determinant of matrix.
   n is current dimension of A[][]. */
int32_t determinant(int32_t A[N][N], int32_t n)
{
	//Credit: https://www.sanfoundry.com/c-program-find-inverse-matrix/

	int32_t D = 0; // Initialize result

    //  Base case : if matrix contains single element
    if (n == 1)
        return A[0][0];

    int32_t temp[N][N]; // To store cofactors

    int32_t sign = 1;  // To store sign multiplier

     // Iterate for each element of first row
    for (int32_t f = 0; f < n; f++)
    {
        // Getting Cofactor of A[0][f]
        getCofactor(A, temp, 0, f, n);
        D += sign * A[0][f] * determinant(temp, n - 1);

        // terms are to be added with alternate sign
        sign = -sign;
    }

    return D;
}

// Function to get adjoint of A[N][N] in adj[N][N].
void adjoint(int32_t A[N][N],int32_t adj[N][N])
{
	//Credit: https://www.sanfoundry.com/c-program-find-inverse-matrix/
    if (N == 1)
    {
        adj[0][0] = 1;
        return;
    }

    // temp is used to store cofactors of A[][]
    int32_t sign = 1, temp[N][N];

    for (int32_t i=0; i<N; i++)
    {
        for (int32_t j=0; j<N; j++)
        {
            // Get cofactor of A[i][j]
            getCofactor(A, temp, i, j, N);

            // sign of adj[j][i] positive if sum of row
            // and column indexes is even.
            sign = ((i+j)%2==0)? 1: -1;

            // Interchanging rows and columns to get the
            // transpose of the cofactor matrix
            adj[j][i] = (sign)*(determinant(temp, N-1));
        }
    }
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
