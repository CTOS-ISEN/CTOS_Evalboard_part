/*
 * ToF_library.h
 *
 *  Created on: Dec 4, 2024
 *      Author: reppl
 */

#ifndef TOF_LIBRARY_TOF_LIBRARY_H_
#define TOF_LIBRARY_TOF_LIBRARY_H_

#include "main.h"	//sinon warning error_handler
#include "logger.h"
#include "custom_ranging_sensor.h"

#define TIMING_BUDGET (30U) /* 8 ms < TimingBudget < 200 ms */
#define POLLING_PERIOD (250U) /* refresh rate for polling mode (ms, shall be consistent with TimingBudget value) */

void print_result(RANGING_SENSOR_Result_t *Result);
void ToF_init(void);
void ToF_acquire_data(RANGING_SENSOR_Result_t* result);
void logger_print_result(RANGING_SENSOR_Result_t *Result);

#endif /* TOF_LIBRARY_TOF_LIBRARY_H_ */
