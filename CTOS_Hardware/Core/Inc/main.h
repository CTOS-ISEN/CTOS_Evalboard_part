/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32wbxx_hal.h"

#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ToF_library.h"	//include deja ranging_sensor
#include "SD_library.h"
#include "Config.h"	//include deja MEMS et ranging_sensor
#include "stdio.h"
#include "stdlib.h"
#include "Driver_gnss.h"
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
#define RST_GNSS_Pin GPIO_PIN_13
#define RST_GNSS_GPIO_Port GPIOC
#define SD_CS_Pin GPIO_PIN_0
#define SD_CS_GPIO_Port GPIOA
#define WKP_GNSS_Pin GPIO_PIN_5
#define WKP_GNSS_GPIO_Port GPIOA
#define B1_Pin GPIO_PIN_4
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI4_IRQn
#define LD2_Pin GPIO_PIN_0
#define LD2_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_1
#define LD3_GPIO_Port GPIOB
#define VL53L4CX_XSHUT_Pin GPIO_PIN_10
#define VL53L4CX_XSHUT_GPIO_Port GPIOA
#define JTMS_Pin GPIO_PIN_13
#define JTMS_GPIO_Port GPIOA
#define JTCK_Pin GPIO_PIN_14
#define JTCK_GPIO_Port GPIOA
#define B2_Pin GPIO_PIN_0
#define B2_GPIO_Port GPIOD
#define B2_EXTI_IRQn EXTI0_IRQn
#define B3_Pin GPIO_PIN_1
#define B3_GPIO_Port GPIOD
#define LD1_Pin GPIO_PIN_5
#define LD1_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_6
#define STLINK_RX_GPIO_Port GPIOB
#define STLINK_TX_Pin GPIO_PIN_7
#define STLINK_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef enum {
	STATE_HANDLER,
	STATE_ACK_TOF,
	STATE_ACK_IMU,
	STATE_ACK_GNSS,
	STATE_STORE,
	STATE_SEND
} FSM_States_Enum;

typedef enum {
	NOT_EXECUTED, EXECUTED,
} ExecutionState_Enum;



#define SD_SPI_HANDLE hspi2
extern uint16_t startACK_flag;
extern uint16_t startSEND_flag;
extern uint8_t* txt_to_send;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
