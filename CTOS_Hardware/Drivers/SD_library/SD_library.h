/*
 * SD_lib.h
 *
 *  Created on: Jan 24, 2025
 *      Author: reppl
 */

#ifndef SD_LIBRARY_SD_LIBRARY_H_
#define SD_LIBRARY_SD_LIBRARY_H_

#include "main.h"
#include "config.h"
#include "custom_ranging_sensor.h"


void SD_mount(void);
void SD_status(void);
void SD_write_data(const char* file_name, mesure* data);
void SD_read_data(const char* file_name, mesure* data, uint32_t measure_index);
uint32_t SD_count_measures(const char* file_name);
void SD_demount(void);

#endif /* SD_LIBRARY_SD_LIBRARY_H_ */
