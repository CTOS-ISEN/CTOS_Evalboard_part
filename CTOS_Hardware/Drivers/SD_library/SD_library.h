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


void SD_Init();
void SD_deInit(void);
void SD_status(void);
void SD_write_object();
void SD_read_data();

#endif /* SD_LIBRARY_SD_LIBRARY_H_ */
