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
    float posX;
    float posY;
    float posZ;

    float vX;
    float vY;
    float vZ;
} mesure;

#endif /* CONFIG_CONFIG_H_ */
