################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/hardware/eMD6/core/mllite/data_builder.c \
../Core/Src/hardware/eMD6/core/mllite/hal_outputs.c \
../Core/Src/hardware/eMD6/core/mllite/message_layer.c \
../Core/Src/hardware/eMD6/core/mllite/ml_math_func.c \
../Core/Src/hardware/eMD6/core/mllite/mlmath.c \
../Core/Src/hardware/eMD6/core/mllite/mpl.c \
../Core/Src/hardware/eMD6/core/mllite/results_holder.c \
../Core/Src/hardware/eMD6/core/mllite/start_manager.c \
../Core/Src/hardware/eMD6/core/mllite/storage_manager.c 

OBJS += \
./Core/Src/hardware/eMD6/core/mllite/data_builder.o \
./Core/Src/hardware/eMD6/core/mllite/hal_outputs.o \
./Core/Src/hardware/eMD6/core/mllite/message_layer.o \
./Core/Src/hardware/eMD6/core/mllite/ml_math_func.o \
./Core/Src/hardware/eMD6/core/mllite/mlmath.o \
./Core/Src/hardware/eMD6/core/mllite/mpl.o \
./Core/Src/hardware/eMD6/core/mllite/results_holder.o \
./Core/Src/hardware/eMD6/core/mllite/start_manager.o \
./Core/Src/hardware/eMD6/core/mllite/storage_manager.o 

C_DEPS += \
./Core/Src/hardware/eMD6/core/mllite/data_builder.d \
./Core/Src/hardware/eMD6/core/mllite/hal_outputs.d \
./Core/Src/hardware/eMD6/core/mllite/message_layer.d \
./Core/Src/hardware/eMD6/core/mllite/ml_math_func.d \
./Core/Src/hardware/eMD6/core/mllite/mlmath.d \
./Core/Src/hardware/eMD6/core/mllite/mpl.d \
./Core/Src/hardware/eMD6/core/mllite/results_holder.d \
./Core/Src/hardware/eMD6/core/mllite/start_manager.d \
./Core/Src/hardware/eMD6/core/mllite/storage_manager.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/hardware/eMD6/core/mllite/%.o Core/Src/hardware/eMD6/core/mllite/%.su Core/Src/hardware/eMD6/core/mllite/%.cyclo: ../Core/Src/hardware/eMD6/core/mllite/%.c Core/Src/hardware/eMD6/core/mllite/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-hardware-2f-eMD6-2f-core-2f-mllite

clean-Core-2f-Src-2f-hardware-2f-eMD6-2f-core-2f-mllite:
	-$(RM) ./Core/Src/hardware/eMD6/core/mllite/data_builder.cyclo ./Core/Src/hardware/eMD6/core/mllite/data_builder.d ./Core/Src/hardware/eMD6/core/mllite/data_builder.o ./Core/Src/hardware/eMD6/core/mllite/data_builder.su ./Core/Src/hardware/eMD6/core/mllite/hal_outputs.cyclo ./Core/Src/hardware/eMD6/core/mllite/hal_outputs.d ./Core/Src/hardware/eMD6/core/mllite/hal_outputs.o ./Core/Src/hardware/eMD6/core/mllite/hal_outputs.su ./Core/Src/hardware/eMD6/core/mllite/message_layer.cyclo ./Core/Src/hardware/eMD6/core/mllite/message_layer.d ./Core/Src/hardware/eMD6/core/mllite/message_layer.o ./Core/Src/hardware/eMD6/core/mllite/message_layer.su ./Core/Src/hardware/eMD6/core/mllite/ml_math_func.cyclo ./Core/Src/hardware/eMD6/core/mllite/ml_math_func.d ./Core/Src/hardware/eMD6/core/mllite/ml_math_func.o ./Core/Src/hardware/eMD6/core/mllite/ml_math_func.su ./Core/Src/hardware/eMD6/core/mllite/mlmath.cyclo ./Core/Src/hardware/eMD6/core/mllite/mlmath.d ./Core/Src/hardware/eMD6/core/mllite/mlmath.o ./Core/Src/hardware/eMD6/core/mllite/mlmath.su ./Core/Src/hardware/eMD6/core/mllite/mpl.cyclo ./Core/Src/hardware/eMD6/core/mllite/mpl.d ./Core/Src/hardware/eMD6/core/mllite/mpl.o ./Core/Src/hardware/eMD6/core/mllite/mpl.su ./Core/Src/hardware/eMD6/core/mllite/results_holder.cyclo ./Core/Src/hardware/eMD6/core/mllite/results_holder.d ./Core/Src/hardware/eMD6/core/mllite/results_holder.o ./Core/Src/hardware/eMD6/core/mllite/results_holder.su ./Core/Src/hardware/eMD6/core/mllite/start_manager.cyclo ./Core/Src/hardware/eMD6/core/mllite/start_manager.d ./Core/Src/hardware/eMD6/core/mllite/start_manager.o ./Core/Src/hardware/eMD6/core/mllite/start_manager.su ./Core/Src/hardware/eMD6/core/mllite/storage_manager.cyclo ./Core/Src/hardware/eMD6/core/mllite/storage_manager.d ./Core/Src/hardware/eMD6/core/mllite/storage_manager.o ./Core/Src/hardware/eMD6/core/mllite/storage_manager.su

.PHONY: clean-Core-2f-Src-2f-hardware-2f-eMD6-2f-core-2f-mllite

