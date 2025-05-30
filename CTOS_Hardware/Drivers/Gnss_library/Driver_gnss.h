/*
 * Driver_gnss.h
 *
 *  Created on: Mar 30, 2025
 *      Author: theo0
 */

#ifndef GNSS_LIBRARY_DRIVER_GNSS_H_
#define GNSS_LIBRARY_DRIVER_GNSS_H_

#include "main.h"
#include "teseo_liv3f.h"
#include "gnss_parser.h"


void Gnss_init();
void Gnss_acquire_data(GNSS1A1_GNSS_Msg_t *mess);


#endif /* GNSS_LIBRARY_DRIVER_GNSS_H_ */
