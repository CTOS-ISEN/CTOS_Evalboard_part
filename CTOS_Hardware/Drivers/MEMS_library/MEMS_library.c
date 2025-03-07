/*
 * MEMS_library.c
 *
 *  Created on: Dec 9, 2024
 *      Author: reppl
 */

#include "MEMS_library.h"

static LSM6DSO_Object_t lsm6dso_obj_0;
static LIS2MDL_Object_t lis2mdl_obj_0;

float LastTime = 0;
char lib_version[MFX_STR_LENG];
static uint8_t mfxstate[STATE_SIZE];
MFX_knobs_t iKnobs;

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

void GettingIMUInfo(IMU_Data *data){
	LSM6DSO_ACC_GetAxes(&lsm6dso_obj_0,  &(data->Acc));
	LSM6DSO_GYRO_GetAxes(&lsm6dso_obj_0,  &(data->Gyr));
	LIS2MDL_MAG_GetAxes(&lis2mdl_obj_0, &(data->Mag));

	MotionFXCompute(data);
}



void MotionFXInit(void){
	if(STATE_SIZE < MotionFX_GetStateSize()){
		Error_Handler();
	}
	  /* Sensor Fusion API initialization function */
	MotionFX_initialize((MFXState_t *)mfxstate);
	  /* Optional: Get version */
	MotionFX_GetLibVersion(lib_version);
	  /* Modify knobs settings & set the knobs */
	MotionFX_getKnobs(mfxstate, &iKnobs);
	MotionFX_setKnobs(mfxstate, &iKnobs);
	MotionFX_enable_6X(mfxstate, MFX_ENGINE_DISABLE);
	MotionFX_enable_9X(mfxstate, MFX_ENGINE_DISABLE);

	  /* Enable 9-axis sensor fusion */
	if (ENABLE_6X == 1){
		MotionFX_enable_6X(mfxstate, MFX_ENGINE_ENABLE);
	}
	else{
		MotionFX_enable_9X(mfxstate, MFX_ENGINE_ENABLE);
	}
}

void MotionFXCompute(IMU_Data *data){
	MFX_input_t data_in;
	MFX_output_t data_out;
	float dT;
	float CurrentTime;
	float *q;

	data_in.acc[0] = (float)(data->Acc.x)/1000;
	data_in.acc[1] = (float)(data->Acc.y)/1000;
	data_in.acc[2] = (float)(data->Acc.z)/1000;

	data_in.gyro[0] = (float)(data->Gyr.x)/1000;
	data_in.gyro[1] = (float)(data->Gyr.y)/1000;
	data_in.gyro[2] = (float)(data->Gyr.z)/1000;

	data_in.mag[0] = (float)(data->Mag.x)/500;
	data_in.mag[1] = (float)(data->Mag.y)/500;
	data_in.mag[2] = (float)(data->Mag.z)/500;

	CurrentTime = (float)HAL_GetTick();
	dT = (CurrentTime - LastTime)/1000;
	LastTime = CurrentTime;

	MotionFX_propagate(mfxstate, &data_out, &data_in, &dT);
	MotionFX_update(mfxstate, &data_out, &data_in, &dT, NULL);

	q = data_out.quaternion;

	data->yaw = data_out.rotation[0];
	data->pitch = data_out.rotation[1];
	data->roll = data_out.rotation[2];
}
