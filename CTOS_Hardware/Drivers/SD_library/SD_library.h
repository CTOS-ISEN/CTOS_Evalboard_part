/*
 * SD_lib.h
 *
 *  Created on: Jan 24, 2025
 *      Author: reppl
 */

#ifndef SD_LIBRARY_SD_LIBRARY_H_
#define SD_LIBRARY_SD_LIBRARY_H_

//#include "main.h" //inutile
#include "Config.h"


void SD_mount();
void SD_unMount(void);
void SD_status(void);
void start_fileWriting();
void end_fileWriting();
void start_fileReading();
void end_fileReading();

void write_object(mesure *data);
unsigned int readFile_toBuffer(uint8_t *notificationBuffer);

#endif /* SD_LIBRARY_SD_LIBRARY_H_ */
