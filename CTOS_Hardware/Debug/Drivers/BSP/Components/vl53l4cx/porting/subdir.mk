################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform.c \
../Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform_ipp.c \
../Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform_log.c 

OBJS += \
./Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform.o \
./Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform_ipp.o \
./Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform_log.o 

C_DEPS += \
./Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform.d \
./Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform_ipp.d \
./Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform_log.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/vl53l4cx/porting/%.o Drivers/BSP/Components/vl53l4cx/porting/%.su Drivers/BSP/Components/vl53l4cx/porting/%.cyclo: ../Drivers/BSP/Components/vl53l4cx/porting/%.c Drivers/BSP/Components/vl53l4cx/porting/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/vl53l4cx/modules -I../Drivers/BSP/Components/vl53l4cx/porting -I../Drivers/BSP/Components/vl53l4cx -I../TOF/Target -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/adrie/STM32CubeIDE/technical_project/CTOS_Hardware/Drivers/BSP" -I"C:/Users/adrie/STM32CubeIDE/technical_project/CTOS_Hardware/TOF" -I"C:/Users/adrie/STM32CubeIDE/technical_project/CTOS_Hardware/Drivers/ToF_library" -I"C:/Users/adrie/STM32CubeIDE/technical_project/CTOS_Hardware/Drivers/Logger" -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/lsm6dso -I"C:/Users/adrie/STM32CubeIDE/technical_project/CTOS_Hardware/Drivers/MEMS_library" -I../GNSS/Target -I../Middlewares/ST/lib_gnss/LibGNSS/Inc -I../Middlewares/ST/lib_gnss/LibNMEA/Inc -I../Drivers/BSP/Components/teseo_liv3f -I../Drivers/BSP/GNSS1A1 -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Drivers/BSP/Components/lis2mdl -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-vl53l4cx-2f-porting

clean-Drivers-2f-BSP-2f-Components-2f-vl53l4cx-2f-porting:
	-$(RM) ./Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform.cyclo ./Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform.d ./Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform.o ./Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform.su ./Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform_ipp.cyclo ./Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform_ipp.d ./Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform_ipp.o ./Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform_ipp.su ./Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform_log.cyclo ./Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform_log.d ./Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform_log.o ./Drivers/BSP/Components/vl53l4cx/porting/vl53lx_platform_log.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-vl53l4cx-2f-porting

