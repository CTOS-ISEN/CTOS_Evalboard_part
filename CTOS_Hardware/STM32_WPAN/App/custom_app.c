/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    App/custom_app.c
 * @author  MCD Application Team
 * @brief   Custom Example Application (Server)
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* start_survey */
  uint8_t               Rv_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TASKID 26
/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* start_survey */
static void Custom_Rv_Update_Char(void);
static void Custom_Rv_Send_Notification(void);

/* USER CODE BEGIN PFP */
void readAndSendData_task();
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */
	// copy data update buffer +BCH
	memcpy(UpdateCharData, pNotification->DataTransfered.pPayload,
			pNotification->DataTransfered.Length);
  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* start_survey */
    case CUSTOM_STM_CV_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CV_WRITE_NO_RESP_EVT */
		//telephone envoie des infos
		Custom_STM_App_Update_Char(CUSTOM_STM_CV, (uint8_t*) UpdateCharData);
      /* USER CODE END CUSTOM_STM_CV_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_RV_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_RV_NOTIFY_ENABLED_EVT */
		startSEND_flag = 1;
		startACK_flag = 0;

		//taks registered at init
		UTIL_SEQ_ResumeTask(1 << TASKID);

      /* USER CODE END CUSTOM_STM_RV_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_RV_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_RV_NOTIFY_DISABLED_EVT */
		startSEND_flag = 0;

		//stop and unregister the task so next time it start from the begining
		UTIL_SEQ_PauseTask(1 << TASKID);
		//UTIL_SEQ_RegTask(1 << TASKID, UTIL_SEQ_RFU, NULL);
      /* USER CODE END CUSTOM_STM_RV_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
	//register a new task that read and send data through notification
	UTIL_SEQ_RegTask(1 << TASKID, UTIL_SEQ_RFU, readAndSendData_task);
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* start_survey */
__USED void Custom_Rv_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Rv_UC_1*/

  /* USER CODE END Rv_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_RV, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Rv_UC_Last*/

  /* USER CODE END Rv_UC_Last*/
  return;
}

void Custom_Rv_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Rv_NS_1*/

	if (startSEND_flag) {
		updateflag = 1;
	}
  /* USER CODE END Rv_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_RV, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Rv_NS_Last*/

  /* USER CODE END Rv_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
void readAndSendData_task(){

	static unsigned int bytesRead = 0;

	if(startSEND_flag == 1 && bytesRead == 0){
		end_fileWriting();
		start_fileReading();
		bytesRead = 20; //nb aleatoire au dessus de 0
	}

	if(bytesRead > 0){
		//bytesRead = readFile_toBuffer((uint8_t *)NotifyCharData);
		sprintf(NotifyCharData, "%d", bytesRead);
		log_printf("%s\r\n", NotifyCharData);
		Custom_Rv_Send_Notification();
		bytesRead--;
	}

	if(bytesRead == 0 && startSEND_flag == 1){
		end_fileReading();
		startSEND_flag = 0;
		log_printf("finished sending data");

		UTIL_SEQ_PauseTask(1 << TASKID);
	}

}


/* USER CODE END FD_LOCAL_FUNCTIONS*/
