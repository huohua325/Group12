################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/hardware/Encoder.c \
../Core/Src/hardware/Motor.c \
../Core/Src/hardware/pid.c \
../Core/Src/hardware/radar.c 

OBJS += \
./Core/Src/hardware/Encoder.o \
./Core/Src/hardware/Motor.o \
./Core/Src/hardware/pid.o \
./Core/Src/hardware/radar.o 

C_DEPS += \
./Core/Src/hardware/Encoder.d \
./Core/Src/hardware/Motor.d \
./Core/Src/hardware/pid.d \
./Core/Src/hardware/radar.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/hardware/%.o Core/Src/hardware/%.su Core/Src/hardware/%.cyclo: ../Core/Src/hardware/%.c Core/Src/hardware/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-hardware

clean-Core-2f-Src-2f-hardware:
	-$(RM) ./Core/Src/hardware/Encoder.cyclo ./Core/Src/hardware/Encoder.d ./Core/Src/hardware/Encoder.o ./Core/Src/hardware/Encoder.su ./Core/Src/hardware/Motor.cyclo ./Core/Src/hardware/Motor.d ./Core/Src/hardware/Motor.o ./Core/Src/hardware/Motor.su ./Core/Src/hardware/pid.cyclo ./Core/Src/hardware/pid.d ./Core/Src/hardware/pid.o ./Core/Src/hardware/pid.su ./Core/Src/hardware/radar.cyclo ./Core/Src/hardware/radar.d ./Core/Src/hardware/radar.o ./Core/Src/hardware/radar.su

.PHONY: clean-Core-2f-Src-2f-hardware

