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



void start_fileWriting(){
    //open
    fres = f_open(&fil, file_name, FA_OPEN_APPEND | FA_WRITE);
    if (fres != FR_OK) {
        log_printf("f_open error (%i)\r\n", fres);
        Error_Handler();
        return;
    }


    //start a json tab

    char *startBuf = "[\n";
    UINT bytesWritten;
    fres = f_write(&fil, startBuf, 3, &bytesWritten);
    if (fres == FR_OK) {
        log_printf("Wrote %u bytes to '%s'.\r\n", bytesWritten, file_name);
    } else {
        log_printf("f_write error (%i) or partial write.\r\n", fres);
    }
}

void end_fileWriting(){
    //end the json tab
    char *endBuf = "{}]\n";
    UINT bytesWritten;
    fres = f_write(&fil, endBuf, 5, &bytesWritten);
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

	//sprintf automatically trim empty space
	char *Acc_x = (char*) calloc(20, sizeof(char));
	sprintf(Acc_x, "\t\"acc_x\":%ld,\r\n", data->inertialValue.Acc.x);
	char *Acc_y = (char*) calloc(20, sizeof(char));
	sprintf(Acc_y, "\t\"acc_y\":%ld,\r\n", data->inertialValue.Acc.y);
	char *Acc_z = (char*) calloc(20, sizeof(char));
	sprintf(Acc_z, "\t\"acc_z\":%ld,\r\n", data->inertialValue.Acc.z);


	char *Gyr_x = (char*) calloc(20, sizeof(char));
	sprintf(Gyr_x, "\t\"gyr_x\":%ld,\r\n", data->inertialValue.Gyr.x);
	char *Gyr_y = (char*) calloc(20, sizeof(char));
	sprintf(Gyr_y, "\t\"gyr_y\":%ld,\r\n", data->inertialValue.Gyr.y);
	char *Gyr_z = (char*) calloc(20, sizeof(char));
	sprintf(Gyr_z, "\t\"gyr_z\":%ld,\r\n", data->inertialValue.Gyr.z);


	char *Mag_x = (char*) calloc(20, sizeof(char));
	sprintf(Mag_x, "\t\"mag_x\":%ld,\r\n", data->inertialValue.Mag.x);
	char *Mag_y = (char*) calloc(20, sizeof(char));
	sprintf(Mag_y, "\t\"mag_y\":%ld,\r\n", data->inertialValue.Mag.y);
	char *Mag_z = (char*) calloc(20, sizeof(char));
	sprintf(Mag_z, "\t\"mag_z\":%ld,\r\n", data->inertialValue.Mag.z);


	char *yaw = (char*) calloc(20, sizeof(char));
	sprintf(yaw, "\t\"yaw\":%.2f,\r\n", data->inertialValue.yaw);
	char *pitch = (char*) calloc(20, sizeof(char));
	sprintf(pitch, "\t\"pitch\":%.2f,\r\n", data->inertialValue.pitch);
	char *roll = (char*) calloc(20, sizeof(char));
	sprintf(roll, "\t\"roll\":%.2f,\r\n", data->inertialValue.roll);


	char *dist1 = (char*) calloc(22, sizeof(char));
	sprintf(dist1, "\t\"dist1\":%ld,\r\n", data->distance.ZoneResult[0].Distance[0]);
	char *dist2 = (char*) calloc(22, sizeof(char));
	sprintf(dist2, "\t\"dist2\":%ld,\r\n", data->distance.ZoneResult[0].Distance[1]);
	char *dist3 = (char*) calloc(22, sizeof(char));
	sprintf(dist3, "\t\"dist3\":%ld,\r\n", data->distance.ZoneResult[0].Distance[2]);
	char *dist4 = (char*) calloc(22, sizeof(char));
	sprintf(dist4, "\t\"dist4\":%ld\r\n", data->distance.ZoneResult[0].Distance[3]);



    // Create a buffer to store the human-readable data
	//strcat automatically trim empty space
    char *buffer = (char*) calloc(350, sizeof(char));
    strcat(buffer, "{\r\n");
    strcat(buffer, Acc_x);
    strcat(buffer, Acc_y);
    strcat(buffer, Acc_z);
    strcat(buffer, Gyr_x);
    strcat(buffer, Gyr_y);
    strcat(buffer, Gyr_z);
    strcat(buffer, Mag_x);
    strcat(buffer, Mag_y);
    strcat(buffer, Mag_z);
    strcat(buffer, yaw);
    strcat(buffer, pitch);
    strcat(buffer, roll);
    strcat(buffer, dist1);
    strcat(buffer, dist2);
    strcat(buffer, dist3);
    strcat(buffer, dist4);
    strcat(buffer, "},\r\n");

    // Write the formatted data to the file
    UINT bytesWritten;
    fres = f_write(&fil, buffer, strlen(buffer), &bytesWritten);
    if (fres == FR_OK) {
        log_printf("Wrote %u bytes to '%s'.\r\n", bytesWritten, file_name);
    } else {
        log_printf("f_write error (%i) or partial write.\r\n", fres);
    }

}


void start_fileReading(){
    fres = f_open(&fil, file_name, FA_READ);
    if (fres != FR_OK) {
        log_printf("f_open error (%i)\r\n", fres);
        Error_Handler();
        return;
    }
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


	strncat(notificationBuffer, buffer, 200);
	free(buffer);


    return bytesRead;
}

