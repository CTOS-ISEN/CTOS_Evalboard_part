/*
 * driver_gnss.c
 *
 *  Created on: Mar 30, 2025
 *      Author: theo0
 */
#include <Gnss_library/Driver_gnss.h>

static GNSSParser_Data_t parsed_GNSSData;
const GNSS1A1_GNSS_Msg_t *gnssMsg = NULL;
//GNSSParser_Status_t status, check;



void Gnss_init(){
	if (GNSS1A1_GNSS_Init(GNSS1A1_TESEO_LIV3F) != BSP_ERROR_NONE) {
		Error_Handler();
	}

	if (GNSS_PARSER_Init(&parsed_GNSSData) != GNSS_PARSER_OK) {
		Error_Handler();
	}
}



void Gnss_acquire_data(){
	GNSS1A1_GNSS_BackgroundProcess(GNSS1A1_TESEO_LIV3F);
	gnssMsg = GNSS1A1_GNSS_GetMessage(GNSS1A1_TESEO_LIV3F);
	if (gnssMsg == NULL) {
		log_printf("messssage : empty message\r\n");
	}

	log_printf("messssage : %s\r\n", gnssMsg->buf);
	GNSS1A1_GNSS_ReleaseMessage(GNSS1A1_TESEO_LIV3F, gnssMsg);
}
