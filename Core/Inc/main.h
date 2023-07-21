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
#include "stm32f1xx_hal.h"

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

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Selection_Pin GPIO_PIN_0
#define Selection_GPIO_Port GPIOF
#define IN_1_Pin GPIO_PIN_1
#define IN_1_GPIO_Port GPIOF
#define IN_2_Pin GPIO_PIN_2
#define IN_2_GPIO_Port GPIOF
#define IN_3_Pin GPIO_PIN_3
#define IN_3_GPIO_Port GPIOF
#define IN_4_Pin GPIO_PIN_4
#define IN_4_GPIO_Port GPIOF
#define Warning_Pin GPIO_PIN_1
#define Warning_GPIO_Port GPIOC
#define EB_Pin GPIO_PIN_0
#define EB_GPIO_Port GPIOA
#define EA_Pin GPIO_PIN_1
#define EA_GPIO_Port GPIOA
#define Egine_Pin GPIO_PIN_2
#define Egine_GPIO_Port GPIOA
#define Encoder_R_A_Pin GPIO_PIN_9
#define Encoder_R_A_GPIO_Port GPIOE
#define Encoder_R_B_Pin GPIO_PIN_11
#define Encoder_R_B_GPIO_Port GPIOE
#define CLK_SCL_Pin GPIO_PIN_10
#define CLK_SCL_GPIO_Port GPIOB
#define CLK_SDA_Pin GPIO_PIN_11
#define CLK_SDA_GPIO_Port GPIOB
#define Encoder_L_A_Pin GPIO_PIN_12
#define Encoder_L_A_GPIO_Port GPIOD
#define Encoder_L_B_Pin GPIO_PIN_13
#define Encoder_L_B_GPIO_Port GPIOD
#define Ultra_2_Echo_Pin GPIO_PIN_6
#define Ultra_2_Echo_GPIO_Port GPIOC
#define Ultra_1_Echo_Pin GPIO_PIN_15
#define Ultra_1_Echo_GPIO_Port GPIOA
#define Com_TX_Pin GPIO_PIN_10
#define Com_TX_GPIO_Port GPIOC
#define Com_RX_Pin GPIO_PIN_11
#define Com_RX_GPIO_Port GPIOC
#define Info_TX_Pin GPIO_PIN_12
#define Info_TX_GPIO_Port GPIOC
#define Info_RX_Pin GPIO_PIN_2
#define Info_RX_GPIO_Port GPIOD
#define Ultra_Trig_Pin GPIO_PIN_5
#define Ultra_Trig_GPIO_Port GPIOB
#define MPU_SCL_Pin GPIO_PIN_6
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_7
#define MPU_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
