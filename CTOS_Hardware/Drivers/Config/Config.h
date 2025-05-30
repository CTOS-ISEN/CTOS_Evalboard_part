/*
 * Config.h
 *
 *  Created on: Jan 26, 2025
 *      Author: reppl
 */

#ifndef CONFIG_CONFIG_H_
#define CONFIG_CONFIG_H_

#include "custom_ranging_sensor.h"
#include "gnss1a1_gnss.h"
#include "MEMS_library.h"

typedef struct{
	IMU_Data inertialValue;
    RANGING_SENSOR_Result_t distance;
    GNSS1A1_GNSS_Msg_t gnss_message;
}mesure;

#endif /* CONFIG_CONFIG_H_ */
