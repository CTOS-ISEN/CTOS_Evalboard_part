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
    char *endBuf = "]\n";
    UINT bytesWritten;
    fres = f_write(&fil, endBuf, 3, &bytesWritten);
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


void write_object() {
    // Create a buffer to store the human-readable data
    char *buffer = (char*) calloc(60, sizeof(char));
    strcat(buffer, "{\r\n");
    strcat(buffer, "\t\"test1\":\"value1\"\r\n");
    strcat(buffer, "\t\"test2\":\"value2\"\r\n");
    strcat(buffer, "}\r\n");

    // Write the formatted data to the file
    UINT bytesWritten;
    fres = f_write(&fil, buffer, 60, &bytesWritten);
    if (fres == FR_OK) {
        log_printf("Wrote %u bytes to '%s'.\r\n", bytesWritten, file_name);
    } else {
        log_printf("f_write error (%i) or partial write.\r\n", fres);
    }

}



void read_file() {

    fres = f_open(&fil, file_name, FA_READ);
    if (fres != FR_OK) {
        log_printf("f_open error (%i)\r\n", fres);
        Error_Handler();
        return;
    }


    UINT bytesRead = 0;


    do{
        char *buffer = (char*) calloc(60, sizeof(char));
    	fres = f_read(&fil, buffer, 60, &bytesRead);
    	//fres = f_read(&fil, buffer, strlen(buffer) - 1, &bytesRead);
    	//fres = f_read(&fil, buffer, 1023, &bytesRead);

		if (fres != FR_OK) {
			log_printf("f_read error (%i)\r\n", fres);
			f_close(&fil);
			//Error_Handler();
			break;
		}

    	log_printf("%s\n", buffer);



		free(buffer);

    }while(bytesRead > 0);


    //close file
	f_close(&fil);
}

