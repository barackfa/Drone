/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define IT_MAGN_Pin GPIO_PIN_13
#define IT_MAGN_GPIO_Port GPIOC
#define IT_MAGN_EXTI_IRQn EXTI15_10_IRQn
#define BUTTON_Pin GPIO_PIN_0
#define BUTTON_GPIO_Port GPIOC
#define BUTTON_EXTI_IRQn EXTI0_IRQn
#define ECHO_Pin GPIO_PIN_0
#define ECHO_GPIO_Port GPIOA
#define TRIG_Pin GPIO_PIN_1
#define TRIG_GPIO_Port GPIOA
#define CRNT_Pin GPIO_PIN_5
#define CRNT_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_6
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_7
#define PWM2_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_0
#define PWM3_GPIO_Port GPIOB
#define PWM4_Pin GPIO_PIN_1
#define PWM4_GPIO_Port GPIOB
#define IT_PRESS_Pin GPIO_PIN_2
#define IT_PRESS_GPIO_Port GPIOB
#define IT_PRESS_EXTI_IRQn EXTI2_IRQn
#define PRESS_SCL_Pin GPIO_PIN_10
#define PRESS_SCL_GPIO_Port GPIOB
#define PRESS_SDA_Pin GPIO_PIN_11
#define PRESS_SDA_GPIO_Port GPIOB
#define CS_GYRO_Pin GPIO_PIN_12
#define CS_GYRO_GPIO_Port GPIOB
#define IMU_SCK_Pin GPIO_PIN_13
#define IMU_SCK_GPIO_Port GPIOB
#define IMU_MISO_Pin GPIO_PIN_14
#define IMU_MISO_GPIO_Port GPIOB
#define IMU_MOSI_Pin GPIO_PIN_15
#define IMU_MOSI_GPIO_Port GPIOB
#define CS_ACC_Pin GPIO_PIN_8
#define CS_ACC_GPIO_Port GPIOC
#define RECEIVER_TX_Pin GPIO_PIN_9
#define RECEIVER_TX_GPIO_Port GPIOA
#define RECEIVER_RX_Pin GPIO_PIN_10
#define RECEIVER_RX_GPIO_Port GPIOA
#define IT_GYRO_Pin GPIO_PIN_11
#define IT_GYRO_GPIO_Port GPIOA
#define IT_GYRO_EXTI_IRQn EXTI15_10_IRQn
#define IT_ACC_Pin GPIO_PIN_12
#define IT_ACC_GPIO_Port GPIOA
#define IT_ACC_EXTI_IRQn EXTI15_10_IRQn
#define GNSS_RST_Pin GPIO_PIN_15
#define GNSS_RST_GPIO_Port GPIOA
#define GNSS_TX_Pin GPIO_PIN_10
#define GNSS_TX_GPIO_Port GPIOC
#define GNSS_RX_Pin GPIO_PIN_11
#define GNSS_RX_GPIO_Port GPIOC
#define MAGN_SCL_Pin GPIO_PIN_8
#define MAGN_SCL_GPIO_Port GPIOB
#define MAGN_SDA_Pin GPIO_PIN_9
#define MAGN_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
