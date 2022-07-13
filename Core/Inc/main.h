/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define IN2_5_Pin GPIO_PIN_13
#define IN2_5_GPIO_Port GPIOC
#define IN3_5_Pin GPIO_PIN_14
#define IN3_5_GPIO_Port GPIOC
#define IN4_5_Pin GPIO_PIN_15
#define IN4_5_GPIO_Port GPIOC
#define NCS_MEMS_SPI_Pin GPIO_PIN_0
#define NCS_MEMS_SPI_GPIO_Port GPIOC
#define MEMS_INT1_Pin GPIO_PIN_1
#define MEMS_INT1_GPIO_Port GPIOC
#define MEMS_INT2_Pin GPIO_PIN_2
#define MEMS_INT2_GPIO_Port GPIOC
#define IN1_1_Pin GPIO_PIN_0
#define IN1_1_GPIO_Port GPIOA
#define IN2_1_Pin GPIO_PIN_1
#define IN2_1_GPIO_Port GPIOA
#define IN3_1_Pin GPIO_PIN_2
#define IN3_1_GPIO_Port GPIOA
#define IN4_1_Pin GPIO_PIN_3
#define IN4_1_GPIO_Port GPIOA
#define IN1_2_Pin GPIO_PIN_4
#define IN1_2_GPIO_Port GPIOA
#define IN2_2_Pin GPIO_PIN_5
#define IN2_2_GPIO_Port GPIOA
#define IN3_2_Pin GPIO_PIN_6
#define IN3_2_GPIO_Port GPIOA
#define IN4_2_Pin GPIO_PIN_7
#define IN4_2_GPIO_Port GPIOA
#define MAG1_Pin GPIO_PIN_4
#define MAG1_GPIO_Port GPIOC
#define EXT_RESET_Pin GPIO_PIN_5
#define EXT_RESET_GPIO_Port GPIOC
#define IN1_6_Pin GPIO_PIN_0
#define IN1_6_GPIO_Port GPIOB
#define IN2_6_Pin GPIO_PIN_1
#define IN2_6_GPIO_Port GPIOB
#define IN3_6_Pin GPIO_PIN_2
#define IN3_6_GPIO_Port GPIOB
#define I2C2_SCL_Pin GPIO_PIN_10
#define I2C2_SCL_GPIO_Port GPIOB
#define I2C2_SDA_Pin GPIO_PIN_11
#define I2C2_SDA_GPIO_Port GPIOB
#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_MISO_Pin GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_6
#define LD3_GPIO_Port GPIOC
#define LD6_Pin GPIO_PIN_7
#define LD6_GPIO_Port GPIOC
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOC
#define LD5_Pin GPIO_PIN_9
#define LD5_GPIO_Port GPIOC
#define IN1_3_Pin GPIO_PIN_8
#define IN1_3_GPIO_Port GPIOA
#define IN2_3_Pin GPIO_PIN_9
#define IN2_3_GPIO_Port GPIOA
#define IN3_3_Pin GPIO_PIN_10
#define IN3_3_GPIO_Port GPIOA
#define IN4_3_Pin GPIO_PIN_11
#define IN4_3_GPIO_Port GPIOA
#define IN1_4_Pin GPIO_PIN_12
#define IN1_4_GPIO_Port GPIOA
#define IN2_4_Pin GPIO_PIN_13
#define IN2_4_GPIO_Port GPIOA
#define IN3_4_Pin GPIO_PIN_14
#define IN3_4_GPIO_Port GPIOA
#define IN4_4_Pin GPIO_PIN_15
#define IN4_4_GPIO_Port GPIOA
#define IN1_5_Pin GPIO_PIN_12
#define IN1_5_GPIO_Port GPIOC
#define IN4_6_Pin GPIO_PIN_3
#define IN4_6_GPIO_Port GPIOB
#define IN1_7_Pin GPIO_PIN_4
#define IN1_7_GPIO_Port GPIOB
#define IN2_7_Pin GPIO_PIN_5
#define IN2_7_GPIO_Port GPIOB
#define IN3_7_Pin GPIO_PIN_6
#define IN3_7_GPIO_Port GPIOB
#define IN4_7_Pin GPIO_PIN_7
#define IN4_7_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
