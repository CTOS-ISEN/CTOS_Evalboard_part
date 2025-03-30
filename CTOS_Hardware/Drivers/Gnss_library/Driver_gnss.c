/*
 * driver_gnss.c
 *
 *  Created on: Mar 30, 2025
 *      Author: theo0
 */
#include <Gnss_library/Driver_gnss.h>

static GNSSParser_Data_t parsed_GNSSData;



void Gnss_init(){
	if (GNSS1A1_GNSS_Init(GNSS1A1_TESEO_LIV3F) != BSP_ERROR_NONE) {
		Error_Handler();
	}

	if (GNSS_PARSER_Init(&parsed_GNSSData) != GNSS_PARSER_OK) {
		Error_Handler();
	}
}



void Gnss_acquire_data(GNSS1A1_GNSS_Msg_t *mess){
	GNSS1A1_GNSS_Msg_t *gnssMsg = NULL;
	GNSS1A1_GNSS_BackgroundProcess(GNSS1A1_TESEO_LIV3F);
	gnssMsg = GNSS1A1_GNSS_GetMessage(GNSS1A1_TESEO_LIV3F);

	if (gnssMsg == NULL) {
		mess = (GNSS1A1_GNSS_Msg_t *) malloc(sizeof(GNSS1A1_GNSS_Msg_t));
		mess->len = strlen("empty message");
		mess->buf = (uint8_t *) calloc(mess->len, sizeof(uint8_t));
		strcpy(mess->buf, "empty message");
	}
	else{
		*mess = *gnssMsg;
	}


	GNSS1A1_GNSS_ReleaseMessage(GNSS1A1_TESEO_LIV3F, gnssMsg);
}





//plus susceptible aux erreurs
/*
void Gnss_acquire_data(GNSS1A1_GNSS_Msg_t *mess){
	GNSS1A1_GNSS_Msg_t *gnssMsg = NULL;
	GNSS1A1_GNSS_BackgroundProcess(GNSS1A1_TESEO_LIV3F);
	gnssMsg = GNSS1A1_GNSS_GetMessage(GNSS1A1_TESEO_LIV3F);

	if (gnssMsg == NULL) {
		gnssMsg = (GNSS1A1_GNSS_Msg_t *) malloc(sizeof(GNSS1A1_GNSS_Msg_t));
		gnssMsg->len = strlen("empty message");
		gnssMsg->buf = (uint8_t *) calloc(gnssMsg->len, sizeof(uint8_t));
		strcpy(gnssMsg->buf, "empty message");
	}

	*mess = *gnssMsg;

	GNSS1A1_GNSS_ReleaseMessage(GNSS1A1_TESEO_LIV3F, gnssMsg);
}
*/


