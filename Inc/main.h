/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

#define N 4

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
uint8_t Write_To_Mem(void);
uint8_t I2C_Write_To_Mem(I2C_HandleTypeDef* i2c, uint8_t* buf, uint8_t device_addr,\
		uint8_t register_addr, uint8_t bytes_to_write);
uint8_t I2C_Read_From_Mem(I2C_HandleTypeDef* i2c, uint8_t* buf, uint8_t device_addr,\
		uint8_t register_addr, uint8_t bytes_to_read);
void Multiply_Matrix(int32_t Matrix_1[][4], int32_t Matrix_2[][4], int32_t result[][4],\
		int16_t P, int16_t M, int16_t n);
void Multiply_Matrix_Scaled(int32_t Matrix_1[][4], int32_t Matrix_2[][4], int32_t result[][4],\
		int16_t P, int16_t M, int32_t n, int32_t scale);
void Add_Matrix(int32_t Matrix_1[][4], int32_t Matrix_2[][4], int32_t result[][4],\
		int16_t M, int16_t n);
void Subtract_Matrix(int32_t Matrix_1[][4], int32_t Matrix_2[][4], int32_t result[][4],\
		int16_t M, int16_t n);
void inverse(int32_t A[N][N], int32_t inverse[N][N]);
void getCofactor(int32_t A[N][N], int32_t temp[N][N], int32_t p, int32_t q, int32_t n);
int32_t determinant(int32_t A[N][N], int32_t n);
void adjoint(int32_t A[N][N],int32_t adj[N][N]);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
