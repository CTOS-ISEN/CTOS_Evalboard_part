/*
 * MEMS_library.h
 *
 *  Created on: Dec 9, 2024
 *      Author: reppl
 */

#ifndef MEMS_LIBRARY_MEMS_LIBRARY_H_
#define MEMS_LIBRARY_MEMS_LIBRARY_H_

#include "stdio.h"
#include "stm32wbxx_nucleo_errno.h"
#include "lsm6dso.h"
#include "lis2mdl.h"
#include "stm32wbxx_nucleo_bus.h"

#define LSM6DSO_I2C_BUS 0U
#define LSM6DSO_I2C_ADD_H 0xD7
#define LSM6DSO_I2C_INIT BSP_I2C1_Init
#define LSM6DSO_I2C_DEINIT BSP_I2C1_DeInit
#define LSM6DSO_I2C_READ_REG BSP_I2C1_ReadReg
#define LSM6DSO_I2C_WRITE_REG BSP_I2C1_WriteReg
#define LSM6DSO_GET_TICK BSP_GetTick
#define LSM6DSO_DELAY HAL_Delay

#define LIS2MDL_I2C_BUS 0U
#define LIS2MDL_I2C_ADD_H 0x3DU
#define LIS2MDL_I2C_INIT BSP_I2C1_Init
#define LIS2MDL_I2C_DEINIT BSP_I2C1_DeInit
#define LIS2MDL_I2C_READ_REG BSP_I2C1_ReadReg
#define LIS2MDL_I2C_WRITE_REG BSP_I2C1_WriteReg
#define LIS2MDL_GET_TICK BSP_GetTick
#define LIS2MDL_DELAY HAL_Delay

typedef struct{
	LSM6DSO_Axes_t Acc;
	LSM6DSO_Axes_t Gyr;
	LIS2MDL_Axes_t Mag;
}IMU_Data;

void MyInitLSM6DSO(void);
void MyEnableLSM6DSO(void);
void MyInitLIS2MDL(void);
void MyEnableLIS2MDL(void);

void GettingIMUInfo(IMU_Data *data);


#endif /* MEMS_LIBRARY_MEMS_LIBRARY_H_ */
