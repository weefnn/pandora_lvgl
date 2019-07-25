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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define USB_SPI_DMA 1
#define USE_KEYPAD 0

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
#define WK_UP_Pin GPIO_PIN_13
#define WK_UP_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_7
#define LED_R_GPIO_Port GPIOE
#define LED_G_Pin GPIO_PIN_8
#define LED_G_GPIO_Port GPIOE
#define LED_B_Pin GPIO_PIN_9
#define LED_B_GPIO_Port GPIOE
#define KEY2_Pin GPIO_PIN_8
#define KEY2_GPIO_Port GPIOD
#define KEY1_Pin GPIO_PIN_9
#define KEY1_GPIO_Port GPIOD
#define KEY0_Pin GPIO_PIN_10
#define KEY0_GPIO_Port GPIOD
#define Tx_Pin GPIO_PIN_9
#define Tx_GPIO_Port GPIOA
#define Rx_Pin GPIO_PIN_10
#define Rx_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_7
#define LCD_CS_GPIO_Port GPIOD
#define LCD_DC_Pin GPIO_PIN_4
#define LCD_DC_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_6
#define LCD_RST_GPIO_Port GPIOB
#define LCD_PWR_Pin GPIO_PIN_7
#define LCD_PWR_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
