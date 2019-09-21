/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define EN_B_Pin GPIO_PIN_0
#define EN_B_GPIO_Port GPIOA
#define EN_B_EXTI_IRQn EXTI0_IRQn
#define EN_A_Pin GPIO_PIN_1
#define EN_A_GPIO_Port GPIOA
#define EN_A_EXTI_IRQn EXTI1_IRQn
#define CURRENT_Pin GPIO_PIN_2
#define CURRENT_GPIO_Port GPIOA
#define HALL3_Pin GPIO_PIN_3
#define HALL3_GPIO_Port GPIOA
#define HALL3_EXTI_IRQn EXTI3_IRQn
#define HALL1_Pin GPIO_PIN_4
#define HALL1_GPIO_Port GPIOA
#define HALL1_EXTI_IRQn EXTI4_IRQn
#define HALL2_Pin GPIO_PIN_5
#define HALL2_GPIO_Port GPIOA
#define HALL2_EXTI_IRQn EXTI9_5_IRQn
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_0
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_1
#define LED4_GPIO_Port GPIOB
#define HIGH1_Pin GPIO_PIN_8
#define HIGH1_GPIO_Port GPIOA
#define HIGH2_Pin GPIO_PIN_9
#define HIGH2_GPIO_Port GPIOA
#define HIGH3_Pin GPIO_PIN_10
#define HIGH3_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_15
#define SW1_GPIO_Port GPIOA
#define SELECT2_Pin GPIO_PIN_3
#define SELECT2_GPIO_Port GPIOB
#define SELECT1_Pin GPIO_PIN_4
#define SELECT1_GPIO_Port GPIOB
#define LOW3_Pin GPIO_PIN_5
#define LOW3_GPIO_Port GPIOB
#define LOW2_Pin GPIO_PIN_6
#define LOW2_GPIO_Port GPIOB
#define LOW1_Pin GPIO_PIN_7
#define LOW1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
