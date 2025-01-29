/*
 * MEMS_library.c
 *
 *  Created on: Dec 9, 2024
 *      Author: reppl
 */

#include "MEMS_library.h"

static LSM6DSO_Object_t lsm6dso_obj_0;
static LIS2MDL_Object_t lis2mdl_obj_0;

void MyInitLSM6DSO(void){
	LSM6DSO_IO_t io_ctx;
	io_ctx.BusType     = LSM6DSO_I2C_BUS;
	io_ctx.Address     = LSM6DSO_I2C_ADD_H;
	io_ctx.Init        = LSM6DSO_I2C_INIT;
	io_ctx.DeInit      = LSM6DSO_I2C_DEINIT;
	io_ctx.ReadReg     = LSM6DSO_I2C_READ_REG;
	io_ctx.WriteReg    = LSM6DSO_I2C_WRITE_REG;
	io_ctx.GetTick     = LSM6DSO_GET_TICK;
	io_ctx.Delay       = LSM6DSO_DELAY;

	LSM6DSO_RegisterBusIO(&lsm6dso_obj_0, &io_ctx);
	LSM6DSO_Init(&lsm6dso_obj_0);
}

void MyEnableLSM6DSO(void){
	LSM6DSO_ACC_Enable(&lsm6dso_obj_0);
	LSM6DSO_GYRO_Enable(&lsm6dso_obj_0);
}

void MyGettingLSM6DSO(LSM6DSO_Axes_t *Acc, LSM6DSO_Axes_t *Gyr){
	LSM6DSO_ACC_GetAxes(&lsm6dso_obj_0, Acc);
	LSM6DSO_GYRO_GetAxes(&lsm6dso_obj_0, Gyr);
//	printf("______________________________________________________\n");
//	printf("Gyr X : %ld | Gyr Y : %ld | Gyr Z : %ld\n", Gyr->x, Gyr->y, Gyr->z);
//	printf("Acc X : %ld | Acc Y : %ld | Acc Z : %ld\n", Acc->x, Acc->y, Acc->z);
}

void MyInitLIS2MDL(void){
	LIS2MDL_IO_t io_ctx_bis;
	io_ctx_bis.BusType     = LIS2MDL_I2C_BUS;
	io_ctx_bis.Address     = LIS2MDL_I2C_ADD_H;
	io_ctx_bis.Init        = LIS2MDL_I2C_INIT;
	io_ctx_bis.DeInit      = LIS2MDL_I2C_DEINIT;
	io_ctx_bis.ReadReg     = LIS2MDL_I2C_READ_REG;
	io_ctx_bis.WriteReg    = LIS2MDL_I2C_WRITE_REG;
	io_ctx_bis.GetTick     = LIS2MDL_GET_TICK;
	io_ctx_bis.Delay       = LIS2MDL_DELAY;

	LIS2MDL_RegisterBusIO(&lis2mdl_obj_0, &io_ctx_bis);
	LIS2MDL_Init(&lis2mdl_obj_0);
}

void MyEnableLIS2MDL(void){
	LIS2MDL_MAG_Enable(&lis2mdl_obj_0);
}

void MyGettingLIS2MDL(LIS2MDL_Axes_t *Magn){
//	LIS2MDL_MAG_GetAxes(&lis2mdl_obj_0, Magn);
//	printf("Mag X : %ld | Mag Y : %ld | Mag Z : %ld\n", Magn->x, Magn->y, Magn->z);
}
