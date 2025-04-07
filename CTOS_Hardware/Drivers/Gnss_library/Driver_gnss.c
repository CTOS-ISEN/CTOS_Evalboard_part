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

    if (mess->buf) {
        free(mess->buf);
        mess->buf = NULL;
    }

	if (gnssMsg == NULL) {
		//mess = (GNSS1A1_GNSS_Msg_t *) malloc(sizeof(GNSS1A1_GNSS_Msg_t));
		mess->len = strlen("empty message");
		mess->buf = (uint8_t *) calloc(mess->len, sizeof(uint8_t));
		strcpy(mess->buf, "empty message");
	}
	else if(gnssMsg != NULL && gnssMsg->len > 0){
		//*mess = *gnssMsg;
		mess->len = gnssMsg->len ;
		mess->buf = (uint8_t *) calloc(mess->len , sizeof(uint8_t));
		memcpy(mess->buf, gnssMsg->buf, mess->len -2);	//-2 to delete '\r\n'
	}


	GNSS1A1_GNSS_ReleaseMessage(GNSS1A1_TESEO_LIV3F, gnssMsg);
}


