/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define DIRECTION4_Pin GPIO_PIN_1
#define DIRECTION4_GPIO_Port GPIOC
#define MD_EN4_Pin GPIO_PIN_2
#define MD_EN4_GPIO_Port GPIOC
#define DIRECTION3_Pin GPIO_PIN_3
#define DIRECTION3_GPIO_Port GPIOC
#define RE_4A_Pin GPIO_PIN_0
#define RE_4A_GPIO_Port GPIOA
#define RE_4B_Pin GPIO_PIN_1
#define RE_4B_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define MD_EN3_Pin GPIO_PIN_4
#define MD_EN3_GPIO_Port GPIOA
#define DIRECTION2_Pin GPIO_PIN_5
#define DIRECTION2_GPIO_Port GPIOA
#define RE_3A_Pin GPIO_PIN_6
#define RE_3A_GPIO_Port GPIOA
#define RE_3B_Pin GPIO_PIN_7
#define RE_3B_GPIO_Port GPIOA
#define MD_EN2_Pin GPIO_PIN_4
#define MD_EN2_GPIO_Port GPIOC
#define DIRECTION1_Pin GPIO_PIN_2
#define DIRECTION1_GPIO_Port GPIOB
#define MD_EN1_Pin GPIO_PIN_10
#define MD_EN1_GPIO_Port GPIOB
#define RE_2B_Pin GPIO_PIN_6
#define RE_2B_GPIO_Port GPIOC
#define RE_2A_Pin GPIO_PIN_7
#define RE_2A_GPIO_Port GPIOC
#define MPWM1_Pin GPIO_PIN_8
#define MPWM1_GPIO_Port GPIOA
#define MPWM2_Pin GPIO_PIN_9
#define MPWM2_GPIO_Port GPIOA
#define MPWM3_Pin GPIO_PIN_10
#define MPWM3_GPIO_Port GPIOA
#define MPWM4_Pin GPIO_PIN_11
#define MPWM4_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define RE_1A_Pin GPIO_PIN_6
#define RE_1A_GPIO_Port GPIOB
#define RE_1B_Pin GPIO_PIN_7
#define RE_1B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
