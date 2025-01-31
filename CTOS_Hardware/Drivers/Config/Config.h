/*
 * Config.h
 *
 *  Created on: Jan 26, 2025
 *      Author: reppl
 */

#ifndef CONFIG_CONFIG_H_
#define CONFIG_CONFIG_H_

#include "custom_ranging_sensor.h"

typedef struct {
    RANGING_SENSOR_Result_t distance;
    int32_t AccX;
    int32_t AccY;
    int32_t AccZ;

    int32_t GyroX;
    int32_t GyroY;
    int32_t GyroZ;
} mesure;

#endif /* CONFIG_CONFIG_H_ */
