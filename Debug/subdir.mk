################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../myMainProgram.c 

C_DEPS += \
./myMainProgram.d 

OBJS += \
./myMainProgram.o 


# Each subdirectory must supply rules for building sources it contributes
%.o %.su: ../%.c subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Z0180064/Vanttec/software/Stm32-project/libs/vanttec_CANLib/src/Vanttec_CANLib" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Z0180064/Vanttec/software/Stm32-project/libs/vanttec_CANLib/src/Vanttec_CANLib_Linux" -I"C:/Users/Z0180064/Vanttec/software/Stm32-project/libs" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean--2e-

clean--2e-:
	-$(RM) ./myMainProgram.d ./myMainProgram.o ./myMainProgram.su

.PHONY: clean--2e-

