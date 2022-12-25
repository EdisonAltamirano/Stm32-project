################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32g431kbux.s 

S_DEPS += \
./Core/Startup/startup_stm32g431kbux.d 

OBJS += \
./Core/Startup/startup_stm32g431kbux.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/Z0180064/Vanttec/newEcuador/Stm32-project/libs/vanttec_CANLib/src/Vanttec_CANLib_Linux" -I"C:/Users/Z0180064/Vanttec/newEcuador/Stm32-project/libs" -I"C:/Users/Z0180064/Vanttec/newEcuador/Stm32-project/libs/vanttec_CANLib/src/Vanttec_CANLib" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g431kbux.d ./Core/Startup/startup_stm32g431kbux.o

.PHONY: clean-Core-2f-Startup

