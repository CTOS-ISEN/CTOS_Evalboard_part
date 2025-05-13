/*
 * SD_lib.c
 *
 *  Created on: Jan 24, 2025
 *      Author: reppl
 */

#include "SD_library.h"
#include "app_fatfs.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "logger.h"

FATFS FatFs;    // FatFS handle
FIL fil;        // File handle
FRESULT fres;   // Result after operations
char *file_name = "mesure.txt";

SD_State sd_state = STATE_CLOSE;


void SD_mount(){
	//mount sd
    fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK) {
        log_printf("f_mount error (%i)\r\n", fres);
        Error_Handler();
    } else {
        log_printf("SD card mounted successfully.\r\n");
    }

}



void SD_unMount(void){

	//unmount sd
    fres = f_mount(NULL, "", 0);
    if (fres == FR_OK) {
        log_printf("SD card unmounted successfully.\r\n");
    } else {
        log_printf("f_mount error during unmount (%i).\r\n", fres);
        Error_Handler();
    }
}


void closeFile(void){
	switch(sd_state){
		case STATE_CLOSE:
			break;
		case STATE_READING:
			end_fileReading();
			break;
		case STATE_WRITING:
			end_fileWriting();
			break;
	}
}


void start_fileWriting(){
	if(sd_state != STATE_CLOSE){
		closeFile();
	}

    //open
    fres = f_open(&fil, file_name, FA_WRITE | FA_CREATE_ALWAYS);
    if (fres != FR_OK) {
        log_printf("f_open error (%i)\r\n", fres);
        Error_Handler();
        return;
    }


    //start a json tab

    char *startBuf = (char*) calloc(2, sizeof(char));
	sprintf(startBuf, "[");

    UINT bytesWritten;
    fres = f_write(&fil, startBuf, 1, &bytesWritten);
    if (fres == FR_OK) {
        log_printf("Wrote %u bytes to '%s'.\r\n", bytesWritten, file_name);
    } else {
        log_printf("f_write error (%i) or partial write.\r\n", fres);
    }

    sd_state = STATE_WRITING;
}

void end_fileWriting(){
    //end the json tab
    char *endBuf = "{\"END\":0}]\r\n";
    UINT bytesWritten;
    fres = f_write(&fil, endBuf, 12, &bytesWritten);
    if (fres == FR_OK) {
        log_printf("Wrote %u bytes to '%s'.\r\n", bytesWritten, file_name);
    } else {
        log_printf("f_write error (%i) or partial write.\r\n", fres);
    }

    //close file
	f_close(&fil);
}






void SD_status(void) {
    DWORD free_clusters, free_sectors, total_sectors;
    FATFS* getFreeFs;

    fres = f_getfree("", &free_clusters, &getFreeFs);
    if (fres != FR_OK) {
        log_printf("f_getfree error (%i)\r\n", fres);
        Error_Handler();
    } else {
        total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
        free_sectors = free_clusters * getFreeFs->csize;

        log_printf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n",
               total_sectors / 2, free_sectors / 2);
    }
}


void write_object(mesure *data) {

	if(sd_state != STATE_WRITING){
		closeFile();
		start_fileWriting();
	}


	char *acc = (char*) calloc(40, sizeof(char));
	sprintf(acc, "\t\"acc\":[%ld, %ld, %ld],\r\n", data->inertialValue.Acc.x, data->inertialValue.Acc.y, data->inertialValue.Acc.z);


	char *quat = (char*) calloc(40, sizeof(char));
	sprintf(quat, "\t\"quat\":[%.2f, %.2f, %.2f, %.2f],\r\n", data->inertialValue.quat[0], data->inertialValue.quat[1], data->inertialValue.quat[2], data->inertialValue.quat[3]);


	char *gyr = (char*) calloc(40, sizeof(char));
	sprintf(gyr, "\t\"gyr\":[%ld, %ld, %ld],\r\n", data->inertialValue.Gyr.x, data->inertialValue.Gyr.y, data->inertialValue.Gyr.z);


	char *mag = (char*) calloc(40, sizeof(char));
	sprintf(mag, "\t\"mag\":[%ld, %ld, %ld],\r\n", data->inertialValue.Mag.x, data->inertialValue.Mag.y, data->inertialValue.Mag.z);




	char *yaw = (char*) calloc(20, sizeof(char));
	sprintf(yaw, "\t\"yaw\":%.2f,\r\n", data->inertialValue.yaw);
	char *pitch = (char*) calloc(20, sizeof(char));
	sprintf(pitch, "\t\"pitch\":%.2f,\r\n", data->inertialValue.pitch);
	char *roll = (char*) calloc(20, sizeof(char));
	sprintf(roll, "\t\"roll\":%.2f,\r\n", data->inertialValue.roll);


	char *zn1 = (char*) calloc(51, sizeof(char));
	sprintf(zn1, "\t\"zone1\":[%ld, %ld, %ld, %ld],\r\n",  data->distance.ZoneResult[0].Distance[0],  data->distance.ZoneResult[0].Distance[1],  data->distance.ZoneResult[0].Distance[2],  data->distance.ZoneResult[0].Distance[3]);


	char *zn2 = (char*) calloc(51, sizeof(char));
	sprintf(zn2, "\t\"zone2\":[%ld, %ld, %ld, %ld],\r\n",  data->distance.ZoneResult[1].Distance[0],  data->distance.ZoneResult[1].Distance[1],  data->distance.ZoneResult[1].Distance[2],  data->distance.ZoneResult[1].Distance[3]);



	char *gnss = (char*) calloc(100, sizeof(char));;
	if(data->gnss_message.len >= 1){
		sprintf(gnss, "\t\"gnss\":\"%s\"\r\n", data->gnss_message.buf);
	}
	else{
		sprintf(gnss, "\t\"gnss\":\"empty message\"\r\n");
	}


    // Create a buffer to store the human-readable data
	//strcat automatically trim empty space
    char *buffer = (char*) calloc(352, sizeof(char));
    strcat(buffer, "{\r\n");
    strcat(buffer, acc);
    strcat(buffer, quat);
    strcat(buffer, gyr);
    strcat(buffer, mag);
    strcat(buffer, yaw);
    strcat(buffer, pitch);
    strcat(buffer, roll);
    strcat(buffer, zn1);
    strcat(buffer, zn2);
    strcat(buffer, gnss);
    strcat(buffer, "},\r\n");

    // Write the formatted data to the file
    UINT bytesWritten;
    fres = f_write(&fil, buffer, strlen(buffer), &bytesWritten);
    if (fres == FR_OK) {
        log_printf("Wrote %u bytes to '%s'.\r\n", bytesWritten, file_name);
    } else {
        log_printf("f_write error (%i) or partial write.\r\n", fres);
    }

    log_printf("%s", buffer);
}


void start_fileReading(){
	if(sd_state != STATE_CLOSE){
		closeFile();
	}

    fres = f_open(&fil, file_name, FA_READ);
    if (fres != FR_OK) {
        log_printf("f_open error (%i)\r\n", fres);
        Error_Handler();
        return;
    }

    sd_state = STATE_READING;
}

void end_fileReading(){
    //close file
	f_close(&fil);
}


unsigned int readFile_toBuffer(uint8_t *notificationBuffer) {
    UINT bytesRead = 0;

    char *buffer = (char*) calloc(200, sizeof(char));
	fres = f_read(&fil, buffer, 200, &bytesRead);


	if (fres != FR_OK) {
		log_printf("f_read error (%i)\r\n", fres);
		f_close(&fil);
		bytesRead = 0;
	}

	log_printf("%s\r\n", buffer);
	memcpy(notificationBuffer, buffer, bytesRead);
	free(buffer);


    return bytesRead;
}

