/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RED_LED_Pin GPIO_PIN_2
#define RED_LED_GPIO_Port GPIOC
#define GREEN_LED_Pin GPIO_PIN_3
#define GREEN_LED_GPIO_Port GPIOC
#define IA_Pin GPIO_PIN_0
#define IA_GPIO_Port GPIOA
#define IB_Pin GPIO_PIN_1
#define IB_GPIO_Port GPIOA
#define Voltage_Pin GPIO_PIN_3
#define Voltage_GPIO_Port GPIOA
#define RX_COM_ESP_Pin GPIO_PIN_5
#define RX_COM_ESP_GPIO_Port GPIOC
#define TX_COM_ESP_Pin GPIO_PIN_10
#define TX_COM_ESP_GPIO_Port GPIOB
#define TX_COM_BMS_Pin GPIO_PIN_9
#define TX_COM_BMS_GPIO_Port GPIOA
#define RX_COM_BMS_Pin GPIO_PIN_10
#define RX_COM_BMS_GPIO_Port GPIOA
#define TX_COM_DEBUG_Pin GPIO_PIN_12
#define TX_COM_DEBUG_GPIO_Port GPIOC
#define RX_COM_DEBUG_Pin GPIO_PIN_2
#define RX_COM_DEBUG_GPIO_Port GPIOD
#define Disable_Driver_Mos_Pin GPIO_PIN_5
#define Disable_Driver_Mos_GPIO_Port GPIOB
#define HALL_SENSOR_A_Pin GPIO_PIN_6
#define HALL_SENSOR_A_GPIO_Port GPIOB
#define HALL_SENSOR_B_Pin GPIO_PIN_7
#define HALL_SENSOR_B_GPIO_Port GPIOB
#define HALL_SENSOR_C_Pin GPIO_PIN_8
#define HALL_SENSOR_C_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
