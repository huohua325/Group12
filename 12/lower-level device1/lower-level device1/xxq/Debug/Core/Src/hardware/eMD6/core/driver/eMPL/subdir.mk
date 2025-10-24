################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/hardware/eMD6/core/driver/eMPL/inv_mpu.c \
../Core/Src/hardware/eMD6/core/driver/eMPL/inv_mpu_dmp_motion_driver.c 

OBJS += \
./Core/Src/hardware/eMD6/core/driver/eMPL/inv_mpu.o \
./Core/Src/hardware/eMD6/core/driver/eMPL/inv_mpu_dmp_motion_driver.o 

C_DEPS += \
./Core/Src/hardware/eMD6/core/driver/eMPL/inv_mpu.d \
./Core/Src/hardware/eMD6/core/driver/eMPL/inv_mpu_dmp_motion_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/hardware/eMD6/core/driver/eMPL/%.o Core/Src/hardware/eMD6/core/driver/eMPL/%.su Core/Src/hardware/eMD6/core/driver/eMPL/%.cyclo: ../Core/Src/hardware/eMD6/core/driver/eMPL/%.c Core/Src/hardware/eMD6/core/driver/eMPL/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-hardware-2f-eMD6-2f-core-2f-driver-2f-eMPL

clean-Core-2f-Src-2f-hardware-2f-eMD6-2f-core-2f-driver-2f-eMPL:
	-$(RM) ./Core/Src/hardware/eMD6/core/driver/eMPL/inv_mpu.cyclo ./Core/Src/hardware/eMD6/core/driver/eMPL/inv_mpu.d ./Core/Src/hardware/eMD6/core/driver/eMPL/inv_mpu.o ./Core/Src/hardware/eMD6/core/driver/eMPL/inv_mpu.su ./Core/Src/hardware/eMD6/core/driver/eMPL/inv_mpu_dmp_motion_driver.cyclo ./Core/Src/hardware/eMD6/core/driver/eMPL/inv_mpu_dmp_motion_driver.d ./Core/Src/hardware/eMD6/core/driver/eMPL/inv_mpu_dmp_motion_driver.o ./Core/Src/hardware/eMD6/core/driver/eMPL/inv_mpu_dmp_motion_driver.su

.PHONY: clean-Core-2f-Src-2f-hardware-2f-eMD6-2f-core-2f-driver-2f-eMPL

