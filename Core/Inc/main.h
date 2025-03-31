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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DATA0_Pin GPIO_PIN_0
#define DATA0_GPIO_Port GPIOA
#define DATA1_Pin GPIO_PIN_1
#define DATA1_GPIO_Port GPIOA
#define DATA2_Pin GPIO_PIN_2
#define DATA2_GPIO_Port GPIOA
#define DATA3_Pin GPIO_PIN_3
#define DATA3_GPIO_Port GPIOA
#define CDA0_Pin GPIO_PIN_4
#define CDA0_GPIO_Port GPIOA
#define CDA1_Pin GPIO_PIN_5
#define CDA1_GPIO_Port GPIOA
#define INPUT_DATA0_Pin GPIO_PIN_7
#define INPUT_DATA0_GPIO_Port GPIOA
#define LED_RGB0_Pin GPIO_PIN_0
#define LED_RGB0_GPIO_Port GPIOB
#define LED_RGB1_Pin GPIO_PIN_1
#define LED_RGB1_GPIO_Port GPIOB
#define INPUT_DATA1_Pin GPIO_PIN_2
#define INPUT_DATA1_GPIO_Port GPIOB
#define INPUT_DATA2_Pin GPIO_PIN_10
#define INPUT_DATA2_GPIO_Port GPIOB
#define INPUT_DATA3_Pin GPIO_PIN_11
#define INPUT_DATA3_GPIO_Port GPIOB
#define LED_MOD_Pin GPIO_PIN_12
#define LED_MOD_GPIO_Port GPIOB
#define LED_PREG_Pin GPIO_PIN_13
#define LED_PREG_GPIO_Port GPIOB
#define LED_DAT_Pin GPIO_PIN_14
#define LED_DAT_GPIO_Port GPIOB
#define LED_START_Pin GPIO_PIN_15
#define LED_START_GPIO_Port GPIOB
#define LED_RGB2_Pin GPIO_PIN_8
#define LED_RGB2_GPIO_Port GPIOA
#define LED_DATA0_Pin GPIO_PIN_9
#define LED_DATA0_GPIO_Port GPIOA
#define LED_DATA1_Pin GPIO_PIN_10
#define LED_DATA1_GPIO_Port GPIOA
#define LED_DATA2_Pin GPIO_PIN_11
#define LED_DATA2_GPIO_Port GPIOA
#define LED_DATA3_Pin GPIO_PIN_12
#define LED_DATA3_GPIO_Port GPIOA
#define DAT_Pin GPIO_PIN_4
#define DAT_GPIO_Port GPIOB
#define START_Pin GPIO_PIN_5
#define START_GPIO_Port GPIOB
#define ACK_Pin GPIO_PIN_6
#define ACK_GPIO_Port GPIOB
#define CLK_Pin GPIO_PIN_7
#define CLK_GPIO_Port GPIOB
#define MOD_Pin GPIO_PIN_8
#define MOD_GPIO_Port GPIOB
#define PREG_Pin GPIO_PIN_9
#define PREG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void TIM2_CallBack(void);
void Start_Button_Pressed(void);
void Dat_Button_Pressed(void);
void Preg_Button_Pressed(void);
void Mod_Button_Pressed(void);
void sendLED(void);
void updateLedData(uint16_t, uint8_t);
void ARM_WR_CDA(void);
void ARM_WR_DATA(void);
void ARM_RD_DATA(void);
void RD_CDA(void);
void WR_CDA(void);
void SEND_DATA(void);
void readButtonData(void);
void READ_DATA(void);
void setPinMode(uint16_t *, GPIO_TypeDef **, uint8_t , uint32_t);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
